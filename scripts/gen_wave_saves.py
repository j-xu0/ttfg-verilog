#!/usr/bin/env python3
"""Generate a GTKWave save file next to every FST in test/waves/.

Each generated ``<name>.gtkw`` opens the matching ``<name>.fst`` with a
curated, module-specific signal set already inserted.  The signal lists are
filtered against the actual FST, so parameter variants that omit internals
(e.g. parallel PI) do not produce "signal not found" warnings.

Usage:
    python3 scripts/gen_wave_saves.py
"""

from __future__ import annotations

import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WAVES = ROOT / "test" / "waves"

# Top-level module name -> (header, [(group_label, [signal, ...]), ...]).
# A group label of None means "no separator"; the signal is placed directly
# after the previous group.  Signals are filtered against the FST.
TEMPLATES = {
    "stepper_spi": ("SPI block", [
        (None, ["clk", "rst_n"]),
        (None, ["sclk", "cs_n", "mosi", "miso"]),
        ("Frame state", ["sclk_sync", "cs_sync", "mosi_sync", "cs_armed",
                          "active", "bit_count"]),
        ("Shift registers", ["rx_shift", "tx_shift"]),
        ("Committed registers", ["target", "control", "plan"]),
        ("Strobes", ["write_pulse", "target_write_pulse", "reject_pulse"]),
        ("Status captured at CS fall", ["busy", "at_target", "fault"]),
    ]),
    "quadrature_decoder": ("Quadrature encoder", [
        (None, ["clk", "rst_n", "a", "b"]),
        ("Synchronizer / decode", ["ab_meta", "ab_sync", "previous_ab",
                                    "changed", "valid_edge", "decrement",
                                    "startup_cycles"]),
        ("Count", ["position", "toggle", "fault"]),
    ]),
    "control_timebase": ("Control timebase", [
        (None, ["clk", "rst_n", "enable"]),
        ("Control-tick enable", ["count", "control_ce"]),
    ]),
    "motion_planner": ("Motion planner (reference)", [
        (None, ["clk", "rst_n", "enable", "start", "tick"]),
        ("Inputs / outputs", ["start_position", "target_position",
                               "reference_position", "reference_velocity"]),
        ("Lifecycle", ["busy", "done", "preparing", "final_step"]),
        ("Phase / speed", ["phase", "next_phase", "velocity_limit",
                            "velocity_magnitude", "next_speed"]),
        ("Distances / mirror rule", ["traveled", "turn_distance",
                                      "total_distance", "remaining",
                                      "remaining_after", "sped_up", "faster"]),
        ("Serial workspace", ["stepped", "step_ext", "start_q", "span",
                               "projected", "position_ext", "doubled",
                               "advanced", "direction_negative",
                               "start_negative"]),
    ]),
    "compact_motion_planner": ("Compact motion planner", [
        (None, ["clk", "rst_n", "enable", "start", "tick"]),
        ("Inputs / profile", ["start_position", "target_position",
                               "plan_code", "profile", "goal"]),
        ("Outputs", ["reference_position", "reference_velocity",
                      "busy", "done", "calculating"]),
        ("FSM / serial ALU", ["state", "next_state", "bit_index",
                               "operand_a", "operand", "carry", "subtract",
                               "working"]),
        ("Planner internals", ["speed", "stopping", "accumulator", "vmax",
                                "max_stop", "negative", "capped", "finishing",
                                "source_position_sign"]),
        ("Publish strobes", ["write_position", "write_speed", "write_stopping"]),
    ]),
    "pi_controller": ("PI controller", [
        (None, ["clk", "rst_n", "enable", "clear", "tick"]),
        ("Inputs", ["reference_position", "measured_position",
                     "feedforward_velocity", "max_velocity",
                     "kp_select", "ki_select"]),
        ("Outputs", ["velocity_command", "saturated", "integral",
                      "calculating"]),
        ("Parallel datapath", ["error", "proportional", "integral_delta",
                                "command_sum", "integral_sum", "reference_wide",
                                "measured_wide", "feedforward_wide",
                                "integral_wide"]),
        ("Serial sampled arithmetic", ["error_integer", "work", "error_ring",
                                        "error_negative", "negative", "carry"]),
        ("Serial engine", ["state", "bit_count", "sum", "value",
                            "gain", "error_bit", "integral_bit", "ff_bit",
                            "integral_phase"]),
        ("Clamp / anti-windup", ["limit", "overflow", "clipped", "windup",
                                  "comparison", "compare_clipped", "low_sign"]),
    ]),
    "step_dir_generator": ("STEP / DIR generator", [
        ("Clock / abort gates", ["clk", "rst_n", "enable", "fault"]),
        ("Command / status", ["velocity_command", "rate_limited"]),
        ("DDS phase (why the next step comes when it does)", ["phase"]),
        ("Driver pins", ["step", "dir", "driver_enable"]),
        # One row, and only in the traces where the pin timing really does drop
        # a step. Everything else that explains a drop (phase_event, the
        # pulse_left/setup_left timers) is internal wiring and stays out.
        ("Step drops (timing_limited only)", ["timing_limited"],
         lambda changing: "timing_limited" in changing),
    ]),
    "shared_motion_controller": ("Shared planner + PI engine", [
        (None, ["clk", "rst_n", "enable"]),
        ("Chip-facing control", ["start", "tick", "clear_control",
                                  "control_enable", "start_position",
                                  "target_position", "plan_code",
                                  "kp_select", "ki_select"]),
        ("Shared engine", ["state", "next_state", "working", "work",
                            "bit_index", "calculating", "planner_busy",
                            "planner_done", "planner_calculating",
                            "pi_calculating"]),
        ("Planner outputs", ["reference_position", "reference_velocity"]),
        ("PI outputs", ["velocity_command", "integral", "saturated"]),
        ("Serial scratch", ["value", "accumulator", "carry",
                             "integral_phase", "error_negative",
                             "integral_negative", "goal", "goal_extended",
                             "profile", "vmax"]),
    ]),
}

FILE_TOP = [
    ("spi", "stepper_spi"),
    ("encoder", "quadrature_decoder"),
    ("timebase", "control_timebase"),
    ("planner", "motion_planner"),
    ("compact_planner", "compact_motion_planner"),
    ("pi", "pi_controller"),
    ("step_dir", "step_dir_generator"),
    ("shared_controller", "shared_motion_controller"),
]


def top_for(path: Path) -> str | None:
    name = path.stem
    for prefix, top in FILE_TOP:
        if name == prefix or name.startswith(prefix + "__"):
            return top
    return None


def variables(fst: Path, scan_values: bool = False) -> tuple[list[tuple[str, str]], set[str]]:
    """Return (hierarchical_path, leaf_name_with_range) for every signal.

    GTKWave resolves save-file names by exact string match against the
    hierarchy, so a vector must be written as ``top.bus[7:0]``; the bare
    name ``top.bus`` does not match and the signal is silently dropped.

    Also returns, when ``scan_values`` is set, the set of leaf base names whose
    value ever changes in the trace, so a template can omit signals that carry
    no information. Scanning the value section is skipped otherwise because it
    dominates the runtime on the long chip/controller traces.
    """
    entries: list[tuple[str, str]] = []
    changing: set[str] = set()
    scope: list[str] = []
    leaf_by_id: dict[str, str] = {}
    first_value: dict[str, str] = {}
    proc = subprocess.Popen(
        ["fst2vcd", str(fst)],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    assert proc.stdout is not None
    in_values = False
    for line in proc.stdout:
        if line.startswith("$scope"):
            scope.append(line.split()[2])
            continue
        if line.startswith("$upscope"):
            if scope:
                scope.pop()
            continue
        if line.startswith("$enddefinitions"):
            if not scan_values:
                break
            in_values = True
            continue
        if not in_values:
            match = re.match(
                r"\$var\s+\w+\s+\d+\s+\S+\s+(\S+)(?:\s+(\[[^\]]+\]))?", line)
            if match:
                leaf = match.group(1) + (match.group(2) or "")
                entries.append((".".join(scope + [leaf]), leaf))
                leaf_by_id[re.match(r"\$var\s+\w+\s+\d+\s+(\S+)", line).group(1)] = leaf
            continue
        if not line or line[0] == "#":
            continue
        token = line.rstrip("\n")
        if token[0] in "bBrR":
            try:
                value, identifier = token[1:].split()
            except ValueError:
                continue
        elif token[0] in "01xz":
            identifier, value = token[1:], token[0]
        else:
            continue
        # Undefined values (the power-on x state) are not a change of interest:
        # a signal that is x and then 0 is still constant as far as a reader is
        # concerned, and counting it would pull every x-initialised register
        # into the save file.
        if set(value.lower()) & set("xz"):
            continue
        if first_value.setdefault(identifier, value) != value:
            changing.add(leaf_by_id.get(identifier, "").split("[")[0])
    proc.stdout.close()
    proc.terminate()
    try:
        proc.wait(timeout=2)
    except subprocess.TimeoutExpired:
        proc.kill()
    return entries, changing


def resolve(entries: list[tuple[str, str]], spec: str) -> str | None:
    """Map a template entry to the exact save-file path, or None if absent.

    ``spec`` is either a leaf name (``phase``), which may expand to the
    ranged form ``phase[29:0]``, or a scope-relative path (``split_phase.high``)
    for signals that only exist inside a generate block.
    """
    if "." in spec:
        suffix = "." + spec
        matches = [full for full, _ in entries if full.endswith(suffix)]
        return min(matches, key=len) if matches else None
    base = spec.split("[")[0]
    matches = [full for full, leaf in entries
               if leaf == spec or leaf.split("[")[0] == base]
    return min(matches, key=len) if matches else None


def make_save(fst: Path, top: str) -> None:
    header, groups = TEMPLATES[top]
    available, changing = variables(fst, any(len(g) > 2 for g in groups))
    out = [f"[*]\n[*] {header} — generated by scripts/gen_wave_saves.py\n[*]\n",
           f'[dumpfile] "{fst.name}"\n',
           "[size] 1500 800\n",
           f"[treeopen] {top}.\n",
           "[sst_width] 300\n[signals_width] 230\n",
           "[sst_expanded] 1\n[sst_vpaned_height] 180\n"]
    emitted = 0
    for group in groups:
        label, signals = group[0], group[1]
        predicate = group[2] if len(group) > 2 else None
        if predicate is not None and not predicate(changing):
            continue
        present = [path for path in (resolve(available, s) for s in signals)
                   if path]
        present = list(dict.fromkeys(present))
        if not present:
            continue
        if label:
            out.append("@200\n")
            out.append(f"-{label}\n")
        out.append("@24\n" if emitted == 0 else "@22\n")
        out.extend(f"{path}\n" for path in present)
        emitted += len(present)
    out.append("[pattern_trace] 1\n[pattern_trace] 0\n")
    save = fst.with_suffix(".gtkw")
    save.write_text("".join(out))
    # Guard against exactly the bug this generator once had: emitting a name
    # that is not an exact hierarchy entry (missing [msb:lsb] range, or a
    # duplicated top-level scope). GTKWave would drop such a trace silently.
    known = {full for full, _ in available}
    written = [line for line in save.read_text().splitlines()
               if line and line[0] not in "[@-*+" and " " not in line]
    unresolved = [line for line in written if line not in known]
    if unresolved:
        raise SystemExit(f"{save.name}: unresolved save entries: {unresolved[:5]}")
    print(f"{save.name:48} {emitted:3d} signals  <- {fst.name}")


def main() -> int:
    if not WAVES.is_dir():
        print(f"missing {WAVES}; run scripts/gen_waves.py first")
        return 1
    generated = 0
    for fst in sorted(WAVES.glob("*.fst")):
        if fst.stem.startswith("e2e"):
            continue  # e2e.gtkw is hand-curated with hierarchical names
        top = top_for(fst)
        if top is None:
            print(f"skip {fst.name}: no template")
            continue
        make_save(fst, top)
        generated += 1
    print(f"\n{generated} GTKWave save files written in {WAVES}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
