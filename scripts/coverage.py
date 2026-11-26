#!/usr/bin/env python3
"""Toggle-coverage spike for Icarus FST dumps (no extra system deps).

Covered 0.7.10 cannot parse this RTL (parameter integer, $clog2, $fatal)
and apt Verilator 4.038 rejects the WIDTHTRUNC pragma, so neither gives
trivial line coverage here. This script is the portable fallback: it
converts tb.fst -> VCD via fst2vcd (already on PATH via gtkwave) and
counts per-bit 0->1 / 1->0 toggles with stdlib only.

Usage:
    python3 scripts/coverage.py test/tb.fst --out coverage.json
    python3 scripts/coverage.py test/tb.fst --out coverage.json --top tb

Output: text summary + JSON {totals, modules:{...}}. Exit 0 always
(non-gating; for fuzz-corpus guidance, not a gate).
"""
import argparse
import json
import re
import subprocess
import sys
import tempfile
from pathlib import Path

# hierarchy substring -> module label (first match wins, ordered specific-first)
MODULE_MAP = [
    (".controller.planner.", "motion_engine"),
    (".controller.feedback.", "pi_p_controller"),
    (".controller.", "shared_motion_controller"),
    (".spi.", "stepper_spi"),
    (".encoder.", "quadrature_decoder"),
    (".timebase.", "control_timebase"),
    (".pulses.", "step_dir_generator"),
    ("user_project", "project"),
]


def module_of(hier: str) -> str:
    for sub, mod in MODULE_MAP:
        if sub in hier:
            return mod
    if hier.startswith("tb."):
        return "tb"
    return "other"


def fst_to_vcd(fst: Path, vcd: Path) -> None:
    r = subprocess.run(["fst2vcd", "-f", str(fst), "-o", str(vcd)],
                       capture_output=True, text=True)
    if r.returncode != 0 or not vcd.exists():
        sys.stderr.write(r.stderr[-2000:])
        raise SystemExit(f"fst2vcd failed for {fst}")


VAR_RE = re.compile(r"\$var\s+\S+\s+(\d+)\s+(\S+)\s+(\S+).*\$end")


def parse_vcd(vcd: Path):
    """Stream VCD. Returns dict id -> {hier, width, risen:set, fallen:set, seen0, seen1}."""
    ids = {}
    scope = []
    in_defs = True
    with open(vcd, errors="replace") as f:
        for line in f:
            s = line.strip()
            if in_defs:
                if s.startswith("$scope"):
                    parts = s.split()
                    if len(parts) >= 3:
                        scope.append(parts[2])
                elif s.startswith("$upscope"):
                    if scope:
                        scope.pop()
                elif s.startswith("$var"):
                    m = VAR_RE.match(s)
                    if m:
                        width, code, name = int(m.group(1)), m.group(2), m.group(3)
                        hier = ".".join(scope + [name])
                        ids[code] = {"hier": hier, "width": width,
                                     "risen": set(), "fallen": set(),
                                     "seen0": False, "seen1": False,
                                     "prev": None}
                elif s.startswith("$enddefinitions"):
                    in_defs = False
                continue
            if not s or s.startswith("$") or s.startswith("#"):
                continue
            # value change: scalar "0!" / vector "b1010 <code>"
            if s[0] in "01xzXZ":
                val, code = s[0].lower(), s[1:]
                st = ids.get(code)
                if st is None or st["width"] != 1:
                    continue
                cur = val if val in "01" else None
                if val in "01":
                    if val == "0":
                        st["seen0"] = True
                    else:
                        st["seen1"] = True
                prev = st["prev"]
                if cur is not None and prev is not None and cur != prev:
                    if cur == "1":
                        st["risen"].add(0)
                    else:
                        st["fallen"].add(0)
                if cur is not None:
                    st["prev"] = cur
            elif s[0] in "bB":
                parts = s.split()
                if len(parts) != 2:
                    continue
                bits, code = parts[0][1:].lower(), parts[1]
                st = ids.get(code)
                if st is None:
                    continue
                w = st["width"]
                # VCD MSB-first; pad/truncate to width, LSB = bit 0
                bits = bits.rjust(w, "0")[-w:][::-1]
                prev = st["prev"]
                if prev is None:
                    prev = [None] * w
                for i, ch in enumerate(bits):
                    if ch in "01":
                        if ch == "0":
                            st["seen0"] = True
                        else:
                            st["seen1"] = True
                        if prev[i] is not None and prev[i] != ch:
                            if ch == "1":
                                st["risen"].add(i)
                            else:
                                st["fallen"].add(i)
                        prev[i] = ch
                st["prev"] = prev
    return ids


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("fst", help="input .fst (or .vcd with --vcd)")
    ap.add_argument("--out", default="coverage.json")
    ap.add_argument("--vcd", action="store_true", help="input is already VCD")
    ap.add_argument("--detail", default="", help="optional path for per-signal CSV")
    args = ap.parse_args()

    src = Path(args.fst)
    if args.vcd:
        vcd = src
        tmp = None
    else:
        tmp = tempfile.NamedTemporaryFile(suffix=".vcd", delete=False)
        tmp.close()
        vcd = Path(tmp.name)
        print(f"[coverage] fst2vcd {src} -> {vcd}", flush=True)
        fst_to_vcd(src, vcd)

    print("[coverage] parsing VCD...", flush=True)
    ids = parse_vcd(vcd)
    if tmp is not None:
        try:
            vcd.unlink()
        except OSError:
            pass

    mods = {}
    for st in ids.values():
        mod = module_of(st["hier"])
        m = mods.setdefault(mod, {"signals": 0, "bits": 0, "rose": 0, "fell": 0, "both": 0})
        m["signals"] += 1
        w = st["width"]
        m["bits"] += w
        m["rose"] += len(st["risen"])
        m["fell"] += len(st["fallen"])
        m["both"] += len(st["risen"] & st["fallen"])

    tot_bits = sum(m["bits"] for m in mods.values())
    tot_rose = sum(m["rose"] for m in mods.values())
    tot_fell = sum(m["fell"] for m in mods.values())
    tot_both = sum(m["both"] for m in mods.values())

    print(f"\nToggle coverage from {src} ({len(ids)} signals, {tot_bits} bits)")
    print(f"  bits with 0->1: {tot_rose}/{tot_bits} ({100*tot_rose/max(tot_bits,1):.1f}%)")
    print(f"  bits with 1->0: {tot_fell}/{tot_bits} ({100*tot_fell/max(tot_bits,1):.1f}%)")
    print(f"  bits both ways: {tot_both}/{tot_bits} ({100*tot_both/max(tot_bits,1):.1f}%)")
    print(f"\n{'module':<24}{'signals':>9}{'bits':>8}{'rise%':>8}{'fall%':>8}{'both%':>8}")
    for mod in sorted(mods, key=lambda k: -mods[k]["bits"]):
        m = mods[mod]
        print(f"{mod:<24}{m['signals']:>9}{m['bits']:>8}"
              f"{100*m['rose']/max(m['bits'],1):>7.1f}%"
              f"{100*m['fell']/max(m['bits'],1):>7.1f}%"
              f"{100*m['both']/max(m['bits'],1):>7.1f}%")

    out = {"source": str(src), "signals": len(ids), "bits": tot_bits,
           "rose": tot_rose, "fell": tot_fell, "both": tot_both, "modules": mods}
    Path(args.out).write_text(json.dumps(out, indent=1))
    print(f"\n[coverage] wrote {args.out}")

    if args.detail:
        with open(args.detail, "w") as f:
            f.write("hier,width,rose,fell,both\n")
            for st in sorted(ids.values(), key=lambda s: s["hier"]):
                both = len(st["risen"] & st["fallen"])
                f.write(f"{st['hier']},{st['width']},{len(st['risen'])},{len(st['fallen'])},{both}\n")
        print(f"[coverage] wrote {args.detail}")


if __name__ == "__main__":
    main()
