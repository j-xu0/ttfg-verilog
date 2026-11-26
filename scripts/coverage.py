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

Phantom signals (params, genvars, elab-time constants, by-design-unused)
can never toggle, so they are excluded from the filtered metric by
default (Covered/Verilator do the same). Raw counts are still reported
for transparency.

Usage:
    python3 scripts/coverage.py test/tb.fst --out coverage.json
    python3 scripts/coverage.py e2e.fst spi.fst encoder.fst --out merged.json
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


# Signals that can never toggle: elaboration-time constants Icarus dumps
# into the FST. Covered/Verilator exclude these automatically.
# - ALL_CAPS leaf: parameter/localparam (CLOCK_HZ, VMAX, IDLE, ...)
# - bit_index: genvar loop indices (count_toggle[N].bit_index, ...)
# - constant_gcd./trailing_zero_bits.: constant-function temporaries used
#   to compute PHASE_GCD at elaboration (all 64-bit, never driven)
# - uio_*/_unused: pins/wires unused by design (all uio are inputs, low)
EXCLUDE_SUBSTRINGS = (
    "constant_gcd.",
    "trailing_zero_bits.",
    "bit_index",
)
EXCLUDE_LEAVES = {
    "uio_in", "uio_oe", "uio_out", "_unused",
}
# Single-driver constant wires: `assign vmax = VMAX` (13'd8191), bound=32,
# limit={3'b0,vmax}. They can never toggle by construction; the real
# saturation signal is `total` (kept). Covered marks these untestable too.
TIED_CONSTANT_LEAVES = {
    "vmax", "limit", "bound", "bound_ext",
}


def is_excluded(hier: str) -> bool:
    leaf = hier.split(".")[-1].split("[")[0]
    if leaf in EXCLUDE_LEAVES or leaf in TIED_CONSTANT_LEAVES:
        return True
    if leaf.isupper() and len(leaf) > 1:
        return True
    return any(s in hier for s in EXCLUDE_SUBSTRINGS)


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


def merge_into(agg, ids):
    """Merge one parsed VCD (id-keyed) into hier-keyed aggregate (union)."""
    for st in ids.values():
        h, w = st["hier"], st["width"]
        a = agg.setdefault(h, {"hier": h, "width": w, "risen": set(), "fallen": set()})
        if a["width"] != w:
            continue  # width mismatch across runs: keep first
        a["risen"] |= st["risen"]
        a["fallen"] |= st["fallen"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("fst", nargs="+", help="input .fst files (merged by union; or .vcd with --vcd)")
    ap.add_argument("--out", default="coverage.json")
    ap.add_argument("--vcd", action="store_true", help="inputs are already VCD")
    ap.add_argument("--detail", default="", help="optional path for per-signal CSV")
    ap.add_argument("--no-exclude", action="store_true", help="disable phantom-signal exclusion")
    args = ap.parse_args()

    agg = {}
    for src in args.fst:
        src = Path(src)
        if args.vcd:
            vcd, tmp = src, None
        else:
            tmp = tempfile.NamedTemporaryFile(suffix=".vcd", delete=False)
            tmp.close()
            vcd = Path(tmp.name)
            print(f"[coverage] fst2vcd {src} -> {vcd}", flush=True)
            fst_to_vcd(src, vcd)
        print("[coverage] parsing VCD...", flush=True)
        merge_into(agg, parse_vcd(vcd))
        if tmp is not None:
            try:
                vcd.unlink()
            except OSError:
                pass

    excl = (lambda h: False) if args.no_exclude else is_excluded
    mods, fmods = {}, {}
    for st in agg.values():
        mod = module_of(st["hier"])
        m = mods.setdefault(mod, {"signals": 0, "bits": 0, "rose": 0, "fell": 0, "both": 0})
        m["signals"] += 1
        w = st["width"]
        m["bits"] += w
        m["rose"] += len(st["risen"])
        m["fell"] += len(st["fallen"])
        m["both"] += len(st["risen"] & st["fallen"])
        if not excl(st["hier"]):
            f = fmods.setdefault(mod, {"signals": 0, "bits": 0, "rose": 0, "fell": 0, "both": 0})
            f["signals"] += 1
            f["bits"] += w
            f["rose"] += len(st["risen"])
            f["fell"] += len(st["fallen"])
            f["both"] += len(st["risen"] & st["fallen"])

    def pct(a, b):
        return 100 * a / max(b, 1)

    tot_bits = sum(m["bits"] for m in mods.values())
    tot_both = sum(m["both"] for m in mods.values())
    ftot_bits = sum(m["bits"] for m in fmods.values())
    ftot_both = sum(m["both"] for m in fmods.values())
    srcs = ", ".join(args.fst)

    print(f"\nToggle coverage from {srcs} ({len(agg)} signals, {tot_bits} bits)")
    print(f"  raw bits both ways:      {tot_both}/{tot_bits} ({pct(tot_both, tot_bits):.1f}%)")
    print(f"  filtered bits both ways: {ftot_both}/{ftot_bits} ({pct(ftot_both, ftot_bits):.1f}%)")
    print(f"  excluded {tot_bits - ftot_bits} phantom bits "
          f"(params/genvars/elab-consts/unused)")
    print(f"\n{'module':<24}{'raw both%':>10}{'filt both%':>11}{'filt bits':>11}")
    for mod in sorted(mods, key=lambda k: -mods[k]["bits"]):
        m, f = mods[mod], fmods.get(mod, {"bits": 0, "both": 0})
        print(f"{mod:<24}{pct(m['both'], m['bits']):>9.1f}%"
              f"{pct(f['both'], f['bits']):>10.1f}%{f['bits']:>11}")

    out = {"sources": args.fst, "signals": len(agg),
           "bits": tot_bits, "both": tot_both,
           "filtered_bits": ftot_bits, "filtered_both": ftot_both,
           "modules": mods, "filtered_modules": fmods}
    Path(args.out).write_text(json.dumps(out, indent=1))
    print(f"\n[coverage] wrote {args.out}")

    if args.detail:
        with open(args.detail, "w") as f:
            f.write("hier,width,rose,fell,both,excluded\n")
            for st in sorted(agg.values(), key=lambda s: s["hier"]):
                both = len(st["risen"] & st["fallen"])
                f.write(f"{st['hier']},{st['width']},{len(st['risen'])},"
                        f"{len(st['fallen'])},{both},{int(excl(st['hier']))}\n")
        print(f"[coverage] wrote {args.detail}")


if __name__ == "__main__":
    main()
