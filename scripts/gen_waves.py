#!/usr/bin/env python3
"""Generate FST waveforms from the existing cocotb test suites.

cocotb's Icarus runner honors ``WAVES=1`` (or ``waves=True``), so the existing
tests can dump waveforms without separate testbenches.  Each suite is run in its
own pytest ``--basetemp`` directory; the resulting FSTs are copied to
``test/waves/`` with stable names.

Usage:
    python3 scripts/gen_waves.py                 # blocks + chip-level e2e
    python3 scripts/gen_waves.py --no-e2e        # blocks only
    python3 scripts/gen_waves.py --only spi encoder   # selected suites
    python3 scripts/gen_waves.py --keep-build    # keep pytest build trees

The full run includes the shared-controller feedback cancellation sweep in two
configurations and the chip-level integration suite. The chip-level phase also
reruns `spi_init_short_move` for a short, easy-to-read trace.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "test" / "waves"
BUILD = OUT / ".build"

SUITES = {
    "spi": ["test/test_spi.py"],
    "encoder": ["test/test_encoder.py"],
    "timebase": ["test/test_timebase.py"],
    "step_dir": ["test/test_step_dir.py"],
    "shared_controller": ["test/test_shared_controller.py"],
}

# Directed chip-level traces: stable FST name -> cocotb test filter.  These run
# through the same tb.v as the full integration suite, so they dump the complete
# chip hierarchy; they simply execute only the named testcase.
DIRECTED_E2E = {
    "e2e__spi_init_short_move": r"test\.spi_init_short_move$",
}


def label_for(name: str, item: str) -> str:
    """Return the stable, human-readable label used for generated waveform files."""
    if name in ("spi", "encoder"):
        return name
    if name == "timebase":
        if "10000000_1000" in item:
            return "timebase__10mhz_1khz"
        if "7_1" in item:
            return "timebase__7hz_1hz"
        if "1_1" in item:
            return "timebase__1hz_1hz"
    if name == "step_dir":
        if "False" in item:
            return "step_dir__pulse_default"
        if "True" in item:
            return "step_dir__pulse_one_cycle"
        nco = {
            "produc": "production",
            "coprim": "coprime",
            "one_cy": "one_cycle",
            "blocke": "blocked",
            "unit_t": "unit_threshold",
            "small_": "small_split",
            "long_d": "long_direction_setup",
            "wide_f": "wide_fraction",
        }
        for key, label in nco.items():
            if key in item:
                return f"step_dir__nco_{label}"
    if name == "shared_controller":
        return "shared_controller__stable" if "_1_0" in item else "shared_controller__unstable"
    return f"{name}__{item}"


def run_suite(name: str, files: list[str]) -> bool:
    base = BUILD / name
    base.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["WAVES"] = "1"
    cmd = [sys.executable, "-m", "pytest", *files, "-q", f"--basetemp={base}"]
    print(f"\n== {name}: {' '.join(files)}", flush=True)
    result = subprocess.run(cmd, cwd=ROOT, env=env)
    if result.returncode != 0:
        print(f"   warning: pytest exited {result.returncode}; collecting any FSTs produced", flush=True)

    for old in OUT.glob(f"{name}__*.fst"):
        old.unlink()

    copied = 0
    for fst in sorted(base.rglob("*.fst")):
        rel = fst.relative_to(base)
        item = rel.parts[0] if len(rel.parts) > 1 else fst.stem
        dest = OUT / f"{label_for(name, item)}.fst"
        shutil.copy2(fst, dest)
        print(f"   {dest.relative_to(ROOT)}  ({dest.stat().st_size:,} B)", flush=True)
        copied += 1
    if not copied:
        print("   warning: no FSTs found", flush=True)
    return result.returncode == 0 and copied > 0


def results_passed(path: Path) -> bool:
    """Check cocotb's JUnit test cases, including missing or malformed results."""
    try:
        root = ET.parse(path).getroot()
    except (OSError, ET.ParseError) as exc:
        print(f"   warning: cannot read {path}: {exc}", flush=True)
        return False
    cases = root.findall(".//testcase")
    if not cases or any(case.find("failure") is not None or case.find("error") is not None
                        for case in cases):
        print(f"   warning: {path} has no test cases or contains failures", flush=True)
        return False
    return True


def run_e2e() -> bool:
    print("\n== chip-level integration (make -C test -B)", flush=True)
    ok = True
    results = ROOT / "test" / "results.xml"
    tb_fst = ROOT / "test" / "tb.fst"
    (OUT / "e2e__all_8_tests.fst").unlink(missing_ok=True)
    results.unlink(missing_ok=True)
    tb_fst.unlink(missing_ok=True)
    result = subprocess.run(["make", "-C", "test", "-B"], cwd=ROOT)
    ok &= result.returncode == 0
    if tb_fst.exists():
        dest = OUT / "e2e__all_tests.fst"
        shutil.copy2(tb_fst, dest)
        print(f"   {dest.relative_to(ROOT)}  ({dest.stat().st_size:,} B)", flush=True)
    else:
        print("   warning: test/tb.fst was not produced", flush=True)
        ok = False

    ok &= results_passed(results)

    # Short, directed chip-level traces.  The test filter selects a single
    # cocotb testcase, so the dump is not dominated by the full sweep.
    for name, test_filter in DIRECTED_E2E.items():
        print(f"\n== directed chip-level waveform ({name})", flush=True)
        env = os.environ.copy()
        env["COCOTB_TEST_FILTER"] = test_filter
        results.unlink(missing_ok=True)
        tb_fst.unlink(missing_ok=True)
        directed = subprocess.run(["make", "-C", "test", "-B"], cwd=ROOT, env=env)
        ok &= directed.returncode == 0
        ok &= results_passed(results)
        if tb_fst.exists():
            dest = OUT / f"{name}.fst"
            shutil.copy2(tb_fst, dest)
            print(f"   {dest.relative_to(ROOT)}  ({dest.stat().st_size:,} B)", flush=True)
        else:
            print("   warning: directed test/tb.fst was not produced", flush=True)
            ok = False
    return ok


def write_manifest() -> None:
    lines = ["FST waveform manifest", "=" * 80, ""]
    fsts = sorted(OUT.glob("*.fst"))
    for fst in fsts:
        lines.append(f"{fst.name:48} {fst.stat().st_size:>12,} B")
    lines += [
        "",
        f"{len(fsts)} FST files",
        "Generated by scripts/gen_waves.py.",
        "FSTs contain all signals in the compiled top (module internals included).",
        "Open with: cd test/waves && gtkwave <file>.fst",
    ]
    (OUT / "MANIFEST.txt").write_text("\n".join(lines) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--only", nargs="+", choices=sorted(SUITES),
                        help="run only these suites")
    parser.add_argument("--no-e2e", action="store_true",
                        help="skip the chip-level make/integration run")
    parser.add_argument("--keep-build", action="store_true",
                        help="keep .build/ pytest trees (large)")
    args = parser.parse_args()

    OUT.mkdir(parents=True, exist_ok=True)
    if BUILD.exists() and not args.keep_build:
        shutil.rmtree(BUILD)
    BUILD.mkdir(parents=True, exist_ok=True)

    names = args.only or list(SUITES)
    failures = []
    for name in names:
        if not run_suite(name, SUITES[name]):
            failures.append(name)

    if not args.no_e2e:
        if not run_e2e():
            failures.append("e2e")

    write_manifest()
    saves = subprocess.run([sys.executable, str(ROOT / "scripts" / "gen_wave_saves.py")], cwd=ROOT)
    if saves.returncode != 0:
        failures.append("waveform save files")
    if BUILD.exists() and not args.keep_build:
        shutil.rmtree(BUILD, ignore_errors=True)

    print("\n== summary", flush=True)
    print(f"   output: {OUT}", flush=True)
    if failures:
        print(f"   suites with failures: {', '.join(failures)}", flush=True)
        print("   FSTs produced before failure were still copied.", flush=True)
        return 1
    print("   all selected suites passed", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
