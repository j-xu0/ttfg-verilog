# Waveform guide and agent handoff

This document records the waveform tooling added to the repository and explains
what each waveform contains, so another agent can continue helping the user
read and interpret the traces.

Date: 2026-11-05. Revised after the STEP/DIR deep dive: save files now load
their multi-bit signals at all, the STEP/DIR signal set was cut down to what
each trace actually needs, and the production STEP/DIR trace now opens with a
commanded rate demo. Later revision adds a directed chip-level trace,
`e2e__spi_init_short_move`, for SPI setup plus one tiny accel/decel move.

## TL;DR

The user wants to inspect waveforms for every RTL block and for the full chip.
That already exists locally in `test/waves/`:

```sh
cd test/waves
gtkwave spi.gtkw
gtkwave step_dir__nco_production.gtkw
gtkwave e2e.gtkw
gtkwave e2e__spi_init_short_move.gtkw
```

Every `.fst` has a matching `.gtkw` save file with a curated signal set already
loaded. **Open the `.gtkw`, not the `.fst`.** GTKWave does not auto-insert traces
from an FST; the save file is what prevents the user from adding signals by hand.

There is also a launcher:

```sh
./test/waves/open.sh spi
./test/waves/open.sh e2e
./test/waves/open.sh e2e__spi_init_short_move
```

`open.sh` changes into `test/waves/` first, so the relative `[dumpfile]` entry in
each save file resolves correctly.

The smallest chip-level trace is `e2e__spi_init_short_move.gtkw`: three SPI
setup frames, then a 64-step accel/decel-only move (no cruise), then an
at-target status read. It is 56 ms long instead of 490 ms; start there if the
full-chip e2e is too much at once.

Two things learned the hard way, worth knowing before reading any trace:

1. **A save-file entry for a bus must carry its bit range.** GTKWave resolves
   names by exact match, so `top.velocity_command` does not match the FST's
   `velocity_command[20:0]` and the trace is dropped without an error. This was
   silently true of every generated save until it was fixed; see “Important
   convention” and the generator notes below.
2. **`step_dir__nco_production.gtkw` opens on a demo.** The first ~0.9 ms is a
   slow-command versus fast-command comparison; the remaining 1.2 ms is a
   correctness sweep. The STEP/DIR section explains each stage, so the trace can
   be handed to someone new without narration.

## Baseline and branch warning

The waveforms were generated from the **Q16 shared-engine baseline** at commit
`804a79c` (`nick/serial-motion-planning`, a detached worktree under
`/tmp/ttfg-q16`).

The repository is currently checked out on **`tmp/q8-area-test` at `30d2d6d`**,
which is explicitly marked “TEMP Q8 area experiment (do not merge)” and
intentionally breaks low-acceleration profiles. Do **not** regenerate waveforms
from that branch unless the goal is specifically to inspect the Q8 failure. The
Q8 experiment:

- changes `Q16 -> Q8` in `plan_lut.vh`, `project.v`,
  `shared_motion_controller.v`, and `step_dir_generator.v`;
- fails its `test` workflow on planner/compact-planner and range assertions;
- also fails `test/test_step_dir.py` because that suite assumes Q16 units
  (`pulse_contract` command scaling, and the phase-width assertion in
  `nco_old_algorithm_equivalence`), so the STEP/DIR traces cannot be regenerated
  from this branch at all;
- fails the directed chip test `spi_init_short_move` (it never reaches the
  settled state), which is another reason to generate that trace from Q16;
- still does not fit 1x1 (placement fails at 107.36% utilization).

The only valid local reference is the Q16 baseline.

## What exists in `test/waves/`

`test/waves/` is git-ignored; it holds local artifacts only.

| Artifact | Meaning |
|---|---|
| `*.fst` | 27 FST dumps: 25 module/subsystem traces + 1 full-chip trace + 1 directed chip trace |
| `*.gtkw` | 27 GTKWave save files, one per FST, pre-loaded with key signals |
| `e2e.gtkw` | Hand-curated chip-level save with grouped pins/state/engine signals |
| `e2e__spi_init_short_move.gtkw` | Hand-curated directed chip save: SPI frames, motion state, STEP/DIR |
| `open.sh` | Runs GTKWave from the correct directory for a given name |
| `README.md` | Short user-facing file inventory and generation commands |
| `MANIFEST.txt` | Sizes of all generated FSTs |

There are 25 module save files because the 27 module/subsystem cocotb “runs”
produce 25 FSTs: the two cocotb tests inside `test_shared_controller.py` share
one simulation per `STABLE_CONFIG` (4 runs -> 2 FSTs). All other suites produce
one FST per parameter configuration.

### FST inventory

| FST | Top module | Configuration / scenario | Sim length | Save signals |
|---|---|---|---:|---:|
| `spi.fst` | `stepper_spi` | all SPI checks | 0.057 ms | 23 |
| `encoder.fst` | `quadrature_decoder` | startup/direction/fault/wrap | 0.250 ms | 14 |
| `timebase__10mhz_1khz.fst` | `control_timebase` | production 10 MHz / 1 kHz | 5.002 ms | 5 |
| `timebase__7hz_1hz.fst` | `control_timebase` | 7 Hz / 1 Hz | 0.005 ms | 5 |
| `timebase__1hz_1hz.fst` | `control_timebase` | divide-by-1 | 0.002 ms | 5 |
| `planner__q8_parallel.fst` | `motion_planner` | Q8 reference, parallel | 0.115 ms | 35 |
| `planner__q16_parallel.fst` | `motion_planner` | Q16 reference, parallel | 0.406 ms | 35 |
| `planner__q16_serial.fst` | `motion_planner` | Q16 reference, serial setup | 0.406 ms | 35 |
| `compact_planner__unstable.fst` | `compact_motion_planner` | `STABLE_CONFIG=0` | 48.135 ms | 35 |
| `compact_planner__stable.fst` | `compact_motion_planner` | `STABLE_CONFIG=1` | 48.135 ms | 35 |
| `pi__parallel.fst` | `pi_controller` | `SERIAL=0` | 0.022 ms | 26 |
| `pi__serial.fst` | `pi_controller` | `SERIAL=1` | 1.690 ms | 37 |
| `pi__serial_stable.fst` | `pi_controller` | `SERIAL=1`, `STABLE_CONFIG=1` | 1.690 ms | 37 |
| `step_dir__pulse_default.fst` | `step_dir_generator` | production pulse widths | 0.840 ms | 10 |
| `step_dir__pulse_one_cycle.fst` | `step_dir_generator` | all timing params = 1 | 0.839 ms | 10 |
| `step_dir__nco_*.fst` (8 files) | `step_dir_generator` | production, coprime, one-cycle, blocked, unit-threshold, small-split, long-direction-setup, wide-fraction; each opens with the `rate_demo` low/high comparison | 1.18–4.91 ms | 10 or 11 |
| `shared_controller__unstable.fst` | `shared_motion_controller` | `STABLE_CONFIG=0` | 58.210 ms | 37 |
| `shared_controller__stable.fst` | `shared_motion_controller` | `STABLE_CONFIG=1`, matches chip top | 58.210 ms | 37 |
| `e2e__all_8_tests.fst` | `tb` / `tt_um_j_xu0_stepper` | all 8 chip-level tests, sequential | 489.788 ms | 22 |
| `e2e__spi_init_short_move.fst` | `tb` / `tt_um_j_xu0_stepper` | directed: SPI setup + 64-step accel/decel-only move | 56.342 ms | 29 |

FST timestamps are in picoseconds; the “Sim length” values above are converted
to milliseconds for readability. Signal counts were re-measured from the
generated save files after the STEP/DIR pruning, not carried over.

The 58 ms shared-controller traces and the 48 ms compact-planner traces are long
because the tests sweep many cancellations/profiles. The local full-chip e2e
trace is long because the eight tests it contains run in one simulation. The
directed chip trace is the short alternative: only one testcase runs, so the FST
is 56 ms and the whole profile fits on screen.

## How the waveforms are generated

cocotb's Icarus runner supports waveform dumping through either:

- `WAVES=1` in the environment, or
- `waves=True` passed to `runner.build()` and `runner.test()`.

The existing suites did not need testbench changes; waveform dumping is enabled
by the runner or by the chip-level `tb.v` itself. Each pytest wrapper uses a
`tmp_path` build directory, so the FST lands at:

```text
<pytest basetemp>/<test item>/<build subdir>/<hdl_toplevel>.fst
```

Single-suite example:

```sh
WAVES=1 python3 -m pytest test/test_spi.py -q --basetemp=/tmp/waves-spi
find /tmp/waves-spi -name '*.fst'
# /tmp/waves-spi/test_spi0/spi/stepper_spi.fst
```

The chip-level integration test already dumps unconditionally because
`test/tb.v` contains:

```verilog
initial begin
  $dumpfile("tb.fst");
  $dumpvars(0, tb);
  #1;
end
```

so `make -C test -B` writes `test/tb.fst` and `test/results.xml`. A single
chip-level testcase can be selected without changing the testbench:

```sh
COCOTB_TEST_FILTER='test\.spi_init_short_move$' make -C test -B
cp test/tb.fst test/waves/e2e__spi_init_short_move.fst
```

That is how the directed trace below is produced; the filter runs only the named
cocotb testcase but still dumps the whole chip hierarchy.

### Generator scripts

`scripts/gen_waves.py` re-runs the existing pytest suites with `WAVES=1`, copies
the FSTs into `test/waves/` with stable names, optionally runs the chip-level
`make` plus the directed `spi_init_short_move` testcase, writes `MANIFEST.txt`,
and then calls the save-file generator.

```sh
python3 scripts/gen_waves.py                 # all blocks + e2e, ~7 min
python3 scripts/gen_waves.py --no-e2e        # blocks only
python3 scripts/gen_waves.py --only spi pi   # selected suites
python3 scripts/gen_waves.py --keep-build    # retain .build/ trees
```

`scripts/gen_wave_saves.py` scans every FST, identifies its top module, and
writes `<name>.gtkw` from a curated template. What it has to get right:

- **Exact hierarchy names, with ranges.** It walks `$scope`/`$upscope` so a
  template entry like `phase` is emitted as `step_dir_generator.phase[29:0]`.
  Entries that are absent from a parameter variant are skipped rather than
  written as dangling names, and a scope-relative entry can reach into a
  generate block (`split_phase.high`).
- **A self-check after writing.** If any emitted line is not an exact `$var`
  name, the script aborts with `unresolved save entries`. That is the guard for
  the class of bug described above.
- **Conditional groups.** A template group may carry a predicate over the set of
  signals that actually change in the trace, evaluated by scanning the value
  section. STEP/DIR uses this for its one-row “Step drops” group, so
  `timing_limited` only appears in traces where it fires. Undefined (`x`/`z`)
  transitions are ignored when deciding whether a signal is informative, or the
  power-on `x -> 0` would make every register look live.
- **Cost.** The value scan dominates runtime, so it only runs for templates that
  have predicates; the full 25-save regeneration takes about 7 s, versus 3.5 min
  when every FST body was scanned.

```sh
python3 scripts/gen_wave_saves.py
```

`gen_waves.py` calls this automatically after generating FSTs.

### Why the full generation is slow

- `test_shared_controller.py` runs a 768-point cancellation sweep in two
  configurations: **~2 min 12 s** alone.
- The chip-level integration run is **~4 min**.
- A single combined command can exceed a 3-minute tool-command window, even
  though it is making progress. This happened during the initial generation; the
  missing shared-controller FSTs were produced by a separate dedicated run.

If a future agent needs to regenerate, run the long pieces separately or in the
background.

## How to open and read a waveform

### Opening

```sh
cd test/waves
gtkwave spi.gtkw
# or from the repo root:
./test/waves/open.sh spi
```

If an FST is already open without signals:

```text
GTKWave: File -> Read Save File -> <name>.gtkw
```

The `[dumpfile]` entry inside each save file is a basename such as `"spi.fst"`,
so **GTKWave must be started from `test/waves/`** unless `open.sh` is used.

### Useful GTKWave operations

- Expand the top module in the left hierarchy pane to find signals not already
  loaded.
- Select a bus, then use **Data Format** to switch between binary/hex/decimal;
  use **Signed** for two's-complement signals such as `target`,
  `position`, `velocity_command`, and `integral`.
- Add markers with a right-click on the time axis; use **Edit → Find Next Edge**
  to jump to a transition.
- **Zoom → Zoom Fit** shows the whole simulation; **Zoom → Zoom Best Fit** fits
  the selected signals. For long traces, zoom before adding markers.
- The save files use group labels (`@200` + `-Group`) to keep related signals
  together.
- If a signal appears flat, check the time window and the radix first.

### Important convention

Open the `.gtkw`, not the `.fst`. Opening an FST alone gives a blank wave pane
because GTKWave loads the file but does not automatically add any signals.

Bus names in a save file must carry their bit range. GTKWave resolves each
entry by exact string match against the hierarchy, so
`top.velocity_command` does **not** match the FST's
`velocity_command[20:0]` and the signal is skipped without any error. The
generator now writes ranges; `e2e.gtkw` always did. If a trace is missing from
a loaded save, check for a bare bus name.

## Model facts needed to interpret signals

These recur across modules:

- Position/error/velocity use **Q16** fractional bits unless a module says
  otherwise. `reference_position` is signed Q13.16; `measured_position` is an
  integer; error is `reference - measured*65536`.
- `target` and `position` are signed 13-bit values: `0x0000..0x0FFF` are
  positive `0..4095`; `0x1000..0x1FFF` are negative `-4096..-1`.
- PI gain codes select `/8`, `/4`, `/2`, `/1` for both Kp and Ki.
- PLAN codes pack velocity in the low 3 bits and acceleration in the high 3
  bits.
- SPI status word on MISO is `{1'b0, busy, at_target, fault, 12'b0}`:
  - `0x0000` idle
  - `0x1000` fault
  - `0x2000` at target
  - `0x3000` at target + fault
  - `0x4000` busy
- The chip aborts motion on disable, encoder fault, or range fault; a new TARGET
  is required to restart.
- STEP/DIR `velocity_command` is signed Q5.16 **steps per control tick**: 65536
  is one step/tick, and the clamp is `65536 * MAX_STEP_HZ / CONTROL_HZ`
  (655360, i.e. 10 steps/tick, in production). Step rate = `increment / 65.536`
  steps/s at 10 MHz / 1 kHz.
- `dir` is 1 for positive velocity and 0 for negative. `step` and
  `driver_enable` are combinational, so they drop in the same delta as an abort;
  `dir` and the internal phase are registered and clear on the next edge.

## Waveform-by-waveform walkthrough

### SPI — `spi.fst` / `spi.gtkw`

Top: `stepper_spi`. Simulation length 57.191 µs. This is `test_spi.py`, not the
whole chip.

What the test does, in order:

1. Reset, then five signed-payload writes:
   `0x8000`, `0x8001`, `0x8FFF`, `0x9000`, `0x9FFF` (target `0, 1, 4095,
   -4096, -1`).
2. CONTROL write `0xA000 | 0x1FFA`, PLAN write `0xC000 | 0x1FD5`.
3. Reserved-address write, every truncated length 1..15, overlong frames
   17/32/48/80, and an empty frame — all rejected.
4. Read/status snapshot test: set `at_target` and `fault`, start the frame,
   clear both, send `0x7FFF`, expect MISO `0x3000`.
5. Busy-poison and busy-at-start/completion cases.
6. Minimum-half-period frames at phase offsets 1/3/6/9 ns, checking MISO
   `0x3000`.
7. Reset while CS is low, and reset aborting an in-flight frame.

Key signals:

- `clk`, `rst_n` — 10 MHz system clock.
- `sclk`, `cs_n`, `mosi`, `miso` — physical pins.
- `sclk_sync`, `cs_sync`, `mosi_sync` — two-flop synchronized versions.
- `cs_armed` — only a post-reset high CS sample permits a new frame.
- `active`, `bit_count` — frame state and received bit count.
- `rx_shift` — receive scratch shift register; **not cleared at frame start**.
- `tx_shift` — status word shifted out on MISO.
- `target`, `control`, `plan` — committed registers.
- `write_pulse`, `target_write_pulse`, `reject_pulse` — one-cycle strobes.
- `busy`, `at_target`, `fault` — status inputs captured when CS falls.

Frame format:

```text
rx_shift[15]    = write
rx_shift[14:13] = address: 0 target, 1 control, 2 plan, 3 reserved
rx_shift[12:0]  = payload
```

First five normal writes:

| Frame | CS-low interval | MOSI | Committed target |
|---|---:|---:|---:|
| 1 | 91–1461 ns | `0x8000` | 0 |
| 2 | 1541–2911 ns | `0x8001` | 1 |
| 3 | 2991–4361 ns | `0x8FFF` | 4095 |
| 4 | 4441–5811 ns | `0x9000` | −4096 |
| 5 | 5891–7261 ns | `0x9FFF` | −1 |

`0x8FFF` is `1 00 0 111111111111`: the three zeros after the write bit are the
two address bits plus payload bit 12.

Notable later frames:

- ~22.47–23.76 µs: a 15-bit truncated all-ones frame (`0x7FFF`); rejected, and
  `rx_shift` holds scratch data.
- ~38.85–40.22 µs: the read/status frame `0x7FFF`; write bit is 0, so no
  register changes, but MISO returns the status snapshot.

Gotchas already encountered by the user:

- `target` is the target-position register, not the SPI address. The address is
  `rx_shift[14:13]`.
- MISO is full-duplex and driven on **every** frame, write or read. The write
  bit only gates register commits; it does not enable MISO.
- `rx_shift` can show stale data during short or rejected frames. It is only
  meaningful after exactly 16 shifted bits at CS rising.
- A blank GTKWave window means no signals were added; open `spi.gtkw`.

### Encoder — `encoder.fst` / `encoder.gtkw`

Top: `quadrature_decoder`. Simulation length 250.16 µs.

What the test does: all four stationary startup phases, forward/reverse
rotation, holding a phase, all invalid two-bit transitions, sticky fault,
counting after fault, signed wrap in both directions, and reset recovery.

Key signals:

- `a`, `b` — raw encoder pins.
- `ab_meta`, `ab_sync` — two-flop synchronizer.
- `previous_ab`, `changed` — previous/current phase difference.
- `valid_edge = changed[1] ^ changed[0]` — exactly one bit changed (legal x4 edge).
- `decrement = previous_ab[0] ^ ab_sync[1]` — direction decode.
- `position` — signed 13-bit count, modulo 8192.
- `toggle` — shared increment/decrement carry chain.
- `fault` — sticky illegal-transition flag.

What good looks like: AB walks `00 -> 01 -> 11 -> 10` (or reverse), `valid_edge`
pulses, `position` increments/decrements by one, and a diagonal transition
(both bits change) sets `fault` without changing position.

### Timebase — `timebase__*.fst` / `.gtkw`

Top: `control_timebase`.

- `timebase__10mhz_1khz.fst`: production divider, `count` 0..9999, one-cycle
  `control_ce` every 10,000 clocks. This is the clearest trace.
- `timebase__7hz_1hz.fst`: divisor 7.
- `timebase__1hz_1hz.fst`: divide-by-1, `control_ce` high every enabled cycle.

Signals: `clk`, `rst_n`, `enable`, `count`, `control_ce`. Notice that disabling
freezes the divider phase and clears the pulse; re-enabling resumes the remaining
interval.

### Reference planner — `planner__*.fst` / `.gtkw`

Top: `motion_planner`. **Not instantiated by the chip top**; this is the
standalone mirror-rule reference planner. The chip uses
`shared_motion_controller`.

Variants:

- `planner__q8_parallel` — Q8/parallel reference.
- `planner__q16_parallel` — Q16/parallel.
- `planner__q16_serial` — Q16 with serial setup.

Signals:

- Inputs: `clk`, `rst_n`, `enable`, `start`, `tick`,
  `start_position`, `target_position`.
- Outputs: `reference_position` (signed Q13.16),
  `reference_velocity` (signed Q5.16), `busy`, `done`.
- `phase`, `next_phase` — planner lifecycle.
- `velocity_limit`, `velocity_magnitude`, `next_speed` — speed selection.
- `traveled`, `turn_distance`, `total_distance`, `remaining`,
  `remaining_after`, `sped_up`, `faster` — mirror-rule distance accounting.
- `stepped`, `step_ext`, `start_q`, `span`, `projected`, `position_ext`,
  `doubled`, `advanced`, `direction_negative`, `start_negative` — serial
  workspace and edge-case internals.

The important behavior is that position advances per tick, speed accelerates /
cruises / decelerates, and the mirror rule avoids the runtime divide/multiply
engine. Use the `busy`/`done` handshake and `reference_position` as the primary
view; the distance signals explain why a speed decision was made.

### Compact planner — `compact_planner__*.fst` / `.gtkw`

Top: `compact_motion_planner`. **Not instantiated by the chip top**; it is the
compact standalone planner with the generated 64-profile LUT in `plan_lut.vh`.

Variants: `unstable` (`STABLE_CONFIG=0`, latches host inputs) and `stable`
(`STABLE_CONFIG=1`, owner must hold inputs while busy).

Signals:

- `start_position`, `target_position`, `plan_code`, `profile`, `goal`.
- `reference_position`, `reference_velocity`, `busy`, `done`, `calculating`.
- `state`, `next_state`, `bit_index`, `operand_a`, `operand`, `carry`,
  `subtract`, `working` — serial FSM/ALU.
- `speed`, `stopping`, `accumulator`, `vmax`, `max_stop`, `negative`,
  `capped`, `finishing`, `source_position_sign` — planner state.
- `write_position`, `write_speed`, `write_stopping` — publish strobes.

The 48 ms length comes from sweeping all 64 profiles and both stable
configurations. For a first look, zoom into one `start`/`tick` sequence and
watch `state` walk through `BEGIN_DIST -> DIST -> ... -> POSITION/VELOCITY`, with
`reference_position` updating only when calculation completes.

### PI — `pi__*.fst` / `.gtkw`

Top: `pi_controller`. **Not instantiated by the chip top**; the shared engine has
its own serial PI phases. This is the standalone reference implementation.

Variants:

- `pi__parallel` — `SERIAL=0`, 32-bit combinational datapath.
- `pi__serial` — `SERIAL=1`, bit-serial engine.
- `pi__serial_stable` — `SERIAL=1`, `STABLE_CONFIG=1` (gains/limit held).

Signals:

- `tick`, `enable`, `clear`, `rst_n`.
- `reference_position`, `measured_position`, `feedforward_velocity`,
  `max_velocity`, `kp_select`, `ki_select`.
- `velocity_command`, `saturated`, `integral`, `calculating`.
- Parallel internals: `error` (Q16), `proportional`, `integral_delta`,
  `command_sum`, `integral_sum`, `reference_wide`, `measured_wide`,
  `feedforward_wide`, `integral_wide`, `limit`, `windup`.
- Serial internals: `state`, `bit_count`, `sum`, `value`, `gain`,
  `error_bit`, `integral_bit`, `ff_bit`, `integral_phase`, `error_ring`,
  `work`, `carry`, `overflow`, `clipped`, `comparison`,
  `compare_clipped`, `low_sign`, `error_negative`, `integral_negative`,
  `negative`.

Contract: on tick, `command = clamp(feedforward + Kp*error + OLD integral)`;
then integrate `Ki*error`, clamped, unless saturated in the error direction.
The newly integrated term affects the next tick. `calculating` rises during the
serial computation; outputs stay at the previous value until publish.

In serial traces, watch `bit_count` sweep, `error_ring`/`work` rotate, and
`velocity_command` update only at publish. In parallel traces, the full
`command_sum`/`error` buses update on the tick edge directly.

### STEP/DIR — `step_dir__*.fst` / `.gtkw`

Top: `step_dir_generator`. The production NCO trace is the best entry point:
`step_dir__nco_production.fst`.

**The trace opens with a rate demo.** `test_step_dir.rate_demo` runs before the
equivalence sweep, so the first ~0.9 ms of the production trace is the
comparison the block exists for: a low command, a pause, then the maximum
command over windows of equal length.

| Segment | `velocity_command` | Window | Pulses | Spacing |
|---|---:|---:|---:|---:|
| low | 81920 (0.625 steps/tick, 625 steps/s) | 0.011–481 µs | 6 | 80 µs (8000 clocks) |
| gap | 0 | 481–521 µs | 0 | — |
| high | 655360 (10 steps/tick, 10 ksteps/s) | 521–862 µs | 34 | 10 µs (1000 clocks) |

Both phases count pulses over an identical 320 µs window (four low-rate
spacings), so the contrast is **4 versus 32**: the "few steps in an interval"
versus "a lot of steps in the same interval" relationship, with the same 500 ns
pulse width therefore a 0.6% versus a 5% duty cycle. The segment totals are 6
and 34 because each phase first spends two steps measuring the spacing. Zoom to
0–900 µs (or 500–900 µs for the dense train) rather than zooming to fit the
whole trace, where both segments collapse into two combs.

The tiny `velocity_command` blip at t=0 is the test measuring the DUT's
implemented clamp (`bounded_magnitude` probed combinationally, no clock edge, no
pulses) so the demo also runs on the Q8 variant; the demo itself starts at
0.011 µs.

The demo is a real assertion, not decoration: it measures the rise spacing and
checks the pulse count against it (so it stays correct in configurations where
the pulse contract stretches the DDS period), and it asserts that the high
command produces more pulses than the low one in the same window. It is skipped
when `MAX_STEP_HZ` is too small to produce a contrast, which is the case in the
degenerate `unit_threshold` configuration.

**After the demo (902–2117 µs) the trace is the equivalence sweep**, which
is a correctness test, not a second demo: eight fixed 45 µs windows, then 100
pseudo-random input vectors, then three deliberate aborts.

Fixed windows, each 4500 clocks, 902–1262 µs:

| Window (µs) | `velocity_command` (signed) | Steps | Spacing | Meaning |
|---|---:|---:|---:|---|
| 902–947 | 98305 (1.5/tick) | 0 | – | period 66.7 µs > window: nothing is due yet |
| 947–992 | 700001 → clamped | 5 | 10 µs | over-range, `rate_limited` high |
| 992–1037 | 1 | 0 | – | period ≈ 6.5 ms |
| 1037–1082 | 655359 | 4 | 10 µs | just under the clamp |
| 1082–1127 | 0 | 0 | – | `phase` pinned at 0 |
| 1127–1172 | −98305 | 0 | – | `dir` flips to 0 |
| 1172–1217 | −1048576 → clamped | 5 | 10 µs | over-range negative |
| 1217–1262 | 0 | 0 | – | |

Two traps here. "No steps at 98305" is correct behaviour, not a stall: the window
is shorter than the DDS period, so `phase` is still climbing when the command
changes. And `phase` is not reset by a magnitude change while `dir` holds, so the
first step of a window can arrive early. Set `velocity_command` to Signed:
1998847 is −98305 and 1048576 is −1048576.

Then ~1262–2070 µs of random vectors: commands drawn from
{0, ±1, ±65536, 98305, ±max} with `enable`, `fault` and `rst_n` dropped at
random (1-in-25, 1-in-9, 1-in-11). Expect frequent asynchronous aborts — `step`
and `driver_enable` fall in the same delta as the aborting input, `dir` goes low,
`phase` resets — plus long stretches where `dir` toggles but no pulse appears,
because a ±1 command cannot accumulate a step within the window.

Finally three deliberate aborts at 2095.2, 2106.2 and 2117.2 µs (`enable`,
`fault`, `rst_n`). Each is preceded by ~10 µs at 655360 so a pulse is in flight:
`step` is high for about **5 ns instead of 500 ns**, truncated mid pulse, and the
trace ends on the reset case.

Variants:

- `pulse_default` / `pulse_one_cycle` — pulse and setup timing configurations.
- `nco_production`, `nco_coprime`, `nco_one_cycle`, `nco_blocked`,
  `nco_unit_threshold`, `nco_small_split`, `nco_long_direction_setup`,
  `nco_wide_fraction` — exact-NCO parameter sets.

What the save file loads (production trace: 10 traces in 4 groups). Every row
here earns its place; the criterion was measured, not guessed:

- `clk`, `rst_n`, `enable`, `fault` — the abort gates. `active` is deliberately
  not loaded: it is exactly `rst_n && enable && !fault`.
- `velocity_command` (signed Q5.16) — the demand; `rate_limited` — the status
  bit that says the demand was clipped or could not be serviced.
- `phase` — the only state that cannot be derived from inputs and outputs; it
  explains why the first step after a magnitude change can arrive early and why
  nothing moves while setup is pending. It is the hardest signal here to read,
  so it gets its own explanation below.
- `step`, `dir`, `driver_enable` — the pins.

**What `phase` is.** A 30-bit accumulator measuring how far through the current
step interval we are, scaled so one step is `PHASE_LIMIT = 65536 * CLOCK_HZ /
CONTROL_HZ` (production: 655,360,000). Each clock adds `increment`:

```verilog
if (velocity == 0 || dir wrong || setup_left != 0) phase <= 0;
else if (phase + increment >= LIMIT) { phase_event; phase <= sum - LIMIT; }  // one step due
else phase <= phase + increment;
```

So `phase / PHASE_LIMIT` is the fraction of a step already elapsed, and
`increment / 65.536` is steps per second (655360 -> 10 ksteps/s). The RTL divides
both constants by `gcd(65536*CLOCK_HZ, CONTROL_HZ)` before synthesis, so do not
recompute the modulus from parameters without the same reduction: production
wraps at 655.36e6 (30 bits), `nco_coprime` needs 33. The bit split exists because
`2^20 * 625 = PHASE_LIMIT`: `phase[19:0]` is the fraction and `phase[29:20]` is a
coarse word counting 0..624, which is why the compare is 10 bits and not 30.

Reading it: use Hex and treat it as a sawtooth (or Data Format -> Analog). The
value right after a wrap is the overrun, not zero, and that leftover is what
keeps the average rate exact. A flat 0 means "not counting". It is deliberately
not reset by a magnitude change — only by a zero command, a direction change, or
a pending setup — which is why a step can arrive early after the command rises.

Deliberately omitted from the default set, with the measurement that justified
it on the 211,713-clock production trace:

| Signal | Measurement | Why it is not worth a row |
|---|---|---|
| `increment` | always `min(\|velocity_command\|, 655360)` | pure function of the row above it; the clamp shows up in `rate_limited` |
| `phase_next` | combinational | duplicate of `phase` |
| `magnitude`, `bounded_magnitude`, `command_limited` | combinational | same information as `increment`/`rate_limited` |
| `pulse` | differs from `step` for **2 cycles in 211,713** | only the abort delta; `driver_enable` already shows the abort |
| `pulse_left` | non-zero **3%** of cycles | dead weight unless the timing actually limits |
| `setup_left` | non-zero 9% of cycles | the same delay is measurable as the `dir`-to-`step` gap |
| `phase_event` | **65 events / 65 step rises** | 1:1 with `step` unless a step is dropped |
| `timing_limited` | **0 asserts** | flat line in production by design |

Of that list, only `timing_limited` is added back, as a one-row "Step drops"
group, in the three traces where the pin timing really does drop a step
(`nco_blocked`, `nco_long_direction_setup`, `nco_unit_threshold`). There the DDS
asks for more pulses than the pins can deliver, so a single "this request was
discarded" row is the whole story; `phase_event` and the `pulse_left` /
`setup_left` timers stay out even there, because they are internal wiring that
only reconstructs the same fact:

| Trace | clocks | `phase_event` | step rises | `timing_limited` asserts |
|---|---:|---:|---:|---:|
| `nco_production` | 211,713 | 65 | 65 | 0 |
| `nco_blocked` | 119,173 | 3,157 | 1,294 | 1,863 |
| `nco_long_direction_setup` | 119,228 | 3,145 | 1,288 | 1,855 |
| `nco_coprime` | 119,183 | 3,147 | 3,144 | 0 |

If more detail is needed, add signals from the GUI: `increment` for the exact
effective rate, `pulse` to see the register behind the `step` gate, or the
reduced-accumulator internals `split_phase.high` / `split_phase.fractional_sum`
(production, wide_fraction, small_split only).

What to observe:

- Step spacing is set by `velocity_command`; a discarded request shows as
  `timing_limited` (in the traces that load it) while `step` stays low.
- `step` high time and low time obey `STEP_HIGH_CYCLES`/`STEP_LOW_CYCLES`.
- DIR changes only while `step` is low, and only after the direction hold/setup
  timers; ENABLE has its own setup delay.
- Abort/fault must force `step`, `dir`, and `driver_enable` low in the same
  delta.

### Shared controller — `shared_controller__*.fst` / `.gtkw`

Top: `shared_motion_controller`. This is the chip’s actual planner + PI engine
(instantiated with `STABLE_CONFIG=1` by `project.v`). Use
`shared_controller__stable.fst` to match the chip.

The 58 ms trace runs both cocotb tests in the suite, including the 768-point
cancellation sweep. It is useful for correctness but long; zoom into a launch /
tick interval.

Signals:

- `clk`, `rst_n`, `enable`.
- `start`, `tick`, `clear_control`, `control_enable`,
  `start_position`, `target_position`, `plan_code`, `kp_select`,
  `ki_select`.
- `state`, `next_state`, `working`, `work`, `bit_index`, `calculating`.
- `planner_busy`, `planner_done`, `planner_calculating`, `pi_calculating`.
- `reference_position`, `reference_velocity` — planner output.
- `velocity_command`, `integral`, `saturated` — PI output.
- `value`, `accumulator`, `carry`, `integral_phase`, `error_negative`,
  `integral_negative`, `goal`, `goal_extended`, `profile`, `vmax` —
  shared serial workspace.

What to observe:

- A `start` pulse initializes the planner; `tick` launches one control interval.
- PI runs first (sample, error subtraction, PI sum, clamp, publish), then the
  planner advances the reference. `pi_calculating` and `planner_calculating`
  show the two phases.
- `velocity_command` changes only when PI publishes.
- `reference_position` changes after the planner completes.
- The shared `work`/`accumulator`/`error_ring` registers are reused between PI
  and planner; that is why the tests include cancellation and workspace-restore
  cases.

### Full chip — `e2e__all_8_tests.fst` / `e2e.gtkw`

Top: `tb`, containing `tt_um_j_xu0_stepper`. This is one simulation containing all
8 integration tests sequentially. Total length 489.788 ms.

The starter save groups:

- clock/reset/enable;
- SPI/pins: `ui_in`, `uo_out`, `uio_out`;
- servo state: `position`, `target`, `active`, `armed`, `settled`, `busy`,
  `at_target`, `fault`;
- shared engine: `controller.reference_position`,
  `controller.velocity_command`, `controller.planner_busy`,
  `controller.planner_calculating`, `controller.pi_calculating`;
- STEP/DIR/ENABLE output bits.

Test time ranges, useful for jumping:

| Test | Start | End |
|---|---:|---:|
| `ports_status_and_reset` | 0 | 0.255767 ms |
| `rtl_register_and_encoder_integration` | 0.255768 ms | 0.383498 ms |
| `closed_loop_moves_and_disturbance` | 0.383499 ms | 82.668229 ms |
| `motion_abort_and_fault` | 82.668230 ms | 92.849970 ms |
| `absolute_encoder_range_fault` | 92.849971 ms | 102.116701 ms |
| `full_speed_and_default_limits` | 102.116702 ms | 472.258432 ms |
| `serial_reference_sampling` | 472.258433 ms | 477.382133 ms |
| `shared_engine_abort_phases` | 477.382134 ms | 489.787884 ms |

Best first look: the closed-loop test from ~0.383 ms to a few ms after, where a
target move starts, a step is dropped, and the loop corrects it. Then look at the
abort/fault segment around 82.7–92.8 ms.

Gotcha: the e2e FST is one trace, not one trace per test. Zoom or use markers;
the early reset phase occupies only a tiny fraction of the 490 ms axis.

A fresh `make -C test -B` also runs the directed `spi_init_short_move` testcase
at the end (after the eight tests above), so a regenerated full-chip trace is
~546 ms long. Use the dedicated trace below when only that scenario is wanted.

### Directed e2e — `e2e__spi_init_short_move.fst` / `.gtkw`

This is the short, hand-readable counterpart to the full-chip trace. The test is
`test.py::spi_init_short_move`, run alone through the same `tb.v`, so the FST
still contains the whole chip hierarchy. Total length **56.342 ms**.

What it does, in order:

1. Reset, then three ordinary 16-bit SPI frames while motion is disabled.
2. Assert the motion-enable input (`ui_in[5]`); the queued TARGET launches.
3. The loop runs a 64-step move and settles at target.
4. One status read returns `0x2000` (at target, not busy/fault).

SPI frames (CS-low intervals are 34 us because the test drives a 2 us bit
period):

| Frame | CS low | MOSI | Meaning |
|---|---:|---:|---|
| 1 | 1.731–35.731 us | `0xA004` | CONTROL: Kp = 1/4, Ki = 1/8 (code 1/0) |
| 2 | 36.731–70.731 us | `0xC03F` | PLAN: velocity code 7, acceleration code 7 |
| 3 | 71.731–105.731 us | `0x8040` | TARGET: +64 steps; queued while disabled |
| 4 | 56.306731–56.340731 ms | `0x0000` | status read: MISO returns `0x2000` |

The TARGET commit strobe is at 106 us and the first control tick after enable is
at 1.207 ms. The first nonzero velocity publish is at 2.2214 ms, and the first
STEP rise is at 5.7014 ms.

Profile (Q16 raw, `velocity_command` sampled at each 1 kHz publish; the table
shows selected ticks, not every publish):

| Tick | Time | `velocity_command` | Steps/tick | Phase |
|---:|---:|---:|---:|---|
| 1 | 2.2214 ms | 6554 | 0.10 | accel start |
| 5 | 6.2214 ms | 40962 | 0.63 | accel |
| 10 | 11.2214 ms | 65547 | 1.00 | accel |
| 15 | 16.2214 ms | 86049 | 1.31 | accel |
| 20 | 21.2214 ms | 122957 | 1.88 | accel |
| 25 | 26.2214 ms | 172178 | 2.63 | accel |
| 26 | 27.2214 ms | 173833 | 2.65 | peak |
| 30 | 31.2214 ms | 123121 | 1.88 | decel |
| 35 | 36.2214 ms | 102759 | 1.57 | decel |
| 40 | 41.2214 ms | 79959 | 1.22 | decel |
| 45 | 46.2214 ms | 35052 | 0.53 | decel |
| 50 | 51.2214 ms | 2440 | 0.04 | decel end |
| clear | 52.2341 ms | 0 | 0.00 | `target_reached` clears |

There is no cruise: the peak is ~2.65 steps/tick against a 10 steps/tick PLAN
limit, so the profile never reaches the velocity plateau. The closed loop adds
small PI corrections on the way up and down (the publish sequence is not
perfectly monotonic), but the triangle is unambiguous. `target_reached` clears
the command at 52.234 ms.

STEP rises and spacing (the physical profile; 64 pulses, selected rows):

| Step | Time | Gap from previous |
|---:|---:|---:|
| 1 | 5.7014 ms | – |
| 2 | 7.2955 ms | 1.5941 ms |
| 5 | 10.5806 ms | 1.0430 ms |
| 10 | 15.2486 ms | 0.8297 ms |
| 20 | 21.7184 ms | 0.5347 ms |
| 30 | 26.0724 ms | 0.3843 ms |
| 34 | 27.5928 ms | 0.3771 ms |
| 35 | 27.9698 ms | 0.3770 ms |
| 40 | 30.0787 ms | 0.4591 ms |
| 50 | 35.6179 ms | 0.6307 ms |
| 60 | 43.0921 ms | 1.0245 ms |
| 64 | 49.5801 ms | 2.2156 ms |

The spacing falls to a minimum at step 34-35 and then rises to the end: that is
accel then decel, with no constant-spacing cruise run. Small PI corrections make
individual gaps wobble slightly. `position` reaches 64 at 49.581 ms, `at_target`
rises at 56.207 ms, and the final status frame follows.

Reading it in GTKWave: zoom to 0-0.12 ms for the three init frames, then to
5-50 ms for the motion. The save loads `spi.rx_shift` so each frame can be
decoded in place (`rx_shift[15]` = write, `[14:13]` = address), and
`controller.velocity_command` is the triangle. `reference_position` is valid at
the end of each tick but rotates through the serial workspace during the
calculation, so its brief dense transitions between ticks are not motion. The
STEP row has 64 pulses; each is 500 ns wide, so zoom further to see a pulse.

## What has been verified

- All 25 generated module save files were checked programmatically: every
  signal listed in each `.gtkw` matches an exact `$var` name (including the
  `[msb:lsb]` range) in the corresponding FST. The generator now aborts if any
  save entry is not an exact hierarchy name; the earlier check compared bare
  leaf names, which is why it passed while GTKWave was silently dropping every
  multi-bit trace. Symptom to recognise: a save loads, the one-bit signals
  appear, and every bus is missing — for example `velocity_command` but not
  `velocity_command[20:0]`.
- The STEP/DIR signal set was re-derived from measurements on the trace instead
  of kept as inherited: 16 signals became 10, plus one (`timing_limited`) only in
  the traces that drop steps. Each omission carries its number in the STEP/DIR
  section.
- `rate_demo` was validated by reading the FST back, not by trusting the log:
  6 pulses at 80 us spacing then 34 at 10 us, over the same 320 us counting
  window (4 versus 32). `nco_blocked` / `nco_long_direction_setup` show 5-6
  versus 11-12, because the pulse contract stretches the DDS period there, and
  `nco_unit_threshold` skips the demo (no possible contrast).
- Save generation is idempotent: rerunning `gen_wave_saves.py` rewrites the same
  25 module files and leaves the hand-written `e2e__*.gtkw` saves alone.
- `e2e.gtkw` hierarchical signal paths were checked against the FST hierarchy.
- The directed `e2e__spi_init_short_move` trace was measured back from its FST:
  frames at 1.731/36.731/71.731 us and the status read at 56.307 ms; 64 STEP
  rises from 5.701 to 49.580 ms; spacing falls from 1.594 ms to 0.377 ms at
  step 34-35 then rises to 2.216 ms; `velocity_command` peaks at 173833
  (2.65 steps/tick, far below the 655360 limit) and ends at 0. All 29 save
  entries resolve against the FST hierarchy.
- `gtkwave -x spi.gtkw` loads without errors and exits cleanly.
- FST generation was validated with a single-suite run and a dedicated
  shared-controller run.

No attempt was made to visually verify every trace; there is no X display in the
container. The user is doing the visual inspection.

## Next-agent playbook

When the user says “I’m looking at X”:

1. Identify the block from the filename, then open the matching `.gtkw` (not the
   `.fst`). Use the per-waveform section above to explain the loaded signals.
2. Establish the time scale before interpreting anything. The dump precision is
   1 ps, and GTKWave changes the displayed unit as you zoom. A signal that looks
   flat may simply be outside the current zoom window.
3. For SPI, always separate three things: the **frame bits** (`rx_shift`), the
   **committed register** (`target`/`control`/`plan`), and the **status response**
   (`tx_shift`/MISO). A write frame returns status too; a read frame only returns
   status.
4. For closed-loop traces, establish which phase is active
   (`pi_calculating` vs `planner_calculating`) and compare
   `reference_position`/`velocity_command` against `position`/`target`.
5. If the user wants a different signal set, edit that `.gtkw` in GTKWave and use
   **File -> Write Save File**, or edit the template in
   `scripts/gen_wave_saves.py` and rerun it.
6. If the user wants a shorter/new trace, add a directed cocotb test or filter
   the existing suite; do not regenerate the whole 7-minute set for one block.
   `WAVES=1 python3 -m pytest test/test_spi.py -q --basetemp=/tmp/w`
   produces the FST quickly; larger suites take longer.
7. Never use the `tmp/q8-area-test` branch as a waveform baseline.
8. If a signal the user expects is missing from a loaded save, check its name
   before anything else: a bus needs `[msb:lsb]` and the range must match the
   FST exactly (`change[1:0]`, not `change[0:1]`). The generator self-check
   catches this on write, but a hand-written save will not.
9. For STEP/DIR, don't read `phase` digits — it is a sawtooth. Judge it by
   wrap cadence vs `increment`, and read the demo first (0-0.9 ms), then the
   fixed command windows (0.9-1.26 ms), then the truncated pulses at the end.

## Known limitations / next steps

1. **Full-suite traces are long and contain many scenarios.** STEP/DIR now
   opens with a directed demo (`rate_demo`), which is the pattern to copy. The
   directed chip trace `e2e__spi_init_short_move` covers the “SPI setup + one
   tiny move” case. Still missing, in usefulness order:
   - one abort during PI and one during planner;
   - one compact-planner profile.
   The STEP/DIR trace is still two tests in one file (demo, then the equivalence
   sweep); splitting them into separate FSTs would make the demo trace ~0.9 ms
   instead of 2.1 ms.
2. **The long e2e trace is one 490 ms FST.** `gen_waves.py` now runs one
   directed testcase (`spi_init_short_move`) separately through
   `COCOTB_TEST_FILTER`, producing a 56 ms trace. The full-suite trace could be
   split the same way if more scenarios need their own files.
3. **No markers/annotations.** The save files group signals but do not mark
   “frame 3 commit” or “dropped step” in the time axis. GTKWave marker syntax
   could be added if desired.
4. **Only Q16 baseline.** Regenerate on `804a79c` or a clean worktree, not on
   `tmp/q8-area-test`.
5. **`test/waves/` is git-ignored.** The doc here is tracked; the FST/Gtkw
   artifacts are local. If the user wants to keep a particular waveform,
   regenerate or copy it elsewhere.
6. **`test/tb.gtkw` is stale** from the Tiny Tapeout template and contains an
   absolute path under `/home/uri/...`. Use `test/waves/e2e.gtkw` instead.
7. **Save files carry no radix or signedness.** GTKWave's save format has no
   stable per-signal format field in the version here, so `velocity_command`
   (Q5.16 two's complement) shows as unsigned hex and `phase` shows as a wide
   binary by default. Set Data Format -> Hex/Signed/Analog in the GUI per
   session, or add the signal as a second copy in the trace to keep two formats.
   Do not try to smuggle `[msb:lsb]` off-by-one tricks in for this; the range in
   the name must match the FST exactly.

## Command reference

```sh
# Open a block or the full chip
cd test/waves
gtkwave spi.gtkw
gtkwave e2e.gtkw
gtkwave e2e__spi_init_short_move.gtkw
./test/waves/open.sh spi

# Regenerate all waveforms (slow)
python3 scripts/gen_waves.py

# Regenerate only block waveforms
python3 scripts/gen_waves.py --no-e2e

# Regenerate selected blocks
python3 scripts/gen_waves.py --only spi encoder pi step_dir

# Regenerate GTKWave save files from existing FSTs (~7 s)
python3 scripts/gen_wave_saves.py

# Regenerate one block's FSTs from the Q16 baseline worktree, then re-save
cd /tmp/ttfg-q16                     # git worktree at 804a79c, Q16
cp -a /home/north/ttfg-verilog/test/test_step_dir.py test/   # if the test changed
cp -a /home/north/ttfg-verilog/scripts/gen_wave*.py scripts/ # untracked in Q8 tree
python3 scripts/gen_waves.py --only step_dir --no-e2e
cp test/waves/step_dir__*.fst /home/north/ttfg-verilog/test/waves/
cd /home/north/ttfg-verilog && python3 scripts/gen_wave_saves.py

# Raw cocotb waveform for one suite
WAVES=1 python3 -m pytest test/test_spi.py -q --basetemp=/tmp/waves-spi

# Chip-level waveform
make -C test -B

# One directed chip-level testcase (short trace)
COCOTB_TEST_FILTER='test\.spi_init_short_move$' make -C test -B
cp test/tb.fst test/waves/e2e__spi_init_short_move.fst
```
