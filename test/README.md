# Verification

Install Icarus Verilog and make, then run from the repository root:

```sh
python3 -m venv .venv
. .venv/bin/activate
pip install -r test/requirements.txt
python -m pytest test/test_spi.py test/test_encoder.py test/test_timebase.py test/test_planner.py test/test_pi.py test/test_step_dir.py test/test_shared_controller.py -q
make -C test -B
```

The pytest runners compile each standalone module and run cocotb assertions.
Each suite contains many directed cases:

- Timebase and planner: standalone motion-building-block tests described in
  [the motion-planning contract](../docs/motion-planning.md), including Q8/Q16
  equivalence and mirror-rule trapezoid/triangle coverage. Generated ROM
  reproducibility is checked alongside the shared controller tests.
- Shared controller: both captured/stable configurations, all acceleration profiles,
  sampled input isolation, fixed-Kp feedback without feedforward, reserved gain
  codes, cancellation and completion within 2,000 clocks.
- PI: all gains, signed endpoints, rounding, saturation/anti-windup, feedforward,
  clear/disable/reset, and seeded randomized arithmetic.
- Pulse generator: signed/fractional rate counts, rate cap, high/low widths,
  DIR setup/hold, enable setup, reversals, zero rate, and safety aborts. An
  independent unreduced phase model checks every cycle across default, coprime,
  split-phase, one-cycle timing and boundary parameter configurations.
- SPI: all registers and signed endpoints, truncation/masking, atomic commit,
  every short frame length, overlong/empty frames, reserved address, status
  snapshot, busy at start/during/commit, reset while selected, physical rising-edge
  MISO sampling, and minimum supported SCLK timing at varied clock phases.
- Encoder: all four stationary startup phases, both directions, holding a phase,
  all invalid two-bit transitions, sticky fault, counting after fault, signed
  wrap in both directions, and reset recovery.

The Makefile compiles `tt_um_j_xu0_stepper` through `tb.v` and runs nine integration
tests at 10 MHz clk / 500 kHz SPI: pins/status/reset, RTL wiring, short moves with
a lost step and shaft disturbance, abort/fault recovery, both absolute signed
boundaries, and a long move reaching the 4 kHz pulse ceiling followed by the
lowest PLAN codes. The seventh test checks RTL serial-reference sampling before
workspace reuse; the eighth checks raw disable/reset during both shared-engine
phases and prevents restart. The ninth is the directed waveform scenario: three
SPI setup writes while disabled, then a 64-step accel/decel-only move with no
cruise, ending in an at-target status read. Shared module tests sweep transient
feedback cancellations and compare against independent trajectory and fixed-Kp P
models. Motor-model tests use external pins with one legal encoder
transition per STEP edge. They verify digital feedback
behavior, not a real motor's mechanics or tuning. Results are `test/results.xml`
and `test/tb.fst`. Planner reset must span a rising clock edge.

For gate-level verification, first harden with the matching GF180 Tiny Tapeout
flow. Place its powered netlist at `test/gate_level_netlist.v` and point
`PDK_ROOT` at a PDK installation containing the `gf180mcuD` directory:

```sh
make -C test -B GATES=yes
```

The hierarchy-dependent wiring, serial-sampling and phase-abort tests are skipped with `GATES=yes`;
all external pin tests still run. The retained GF180 GDS workflow separately
handles netlist and PDK setup in GitHub Actions. No gate-level or physical result
is implied by the local RTL tests.

## Waveforms

GTKWave's `fst2vcd` must be on PATH to generate curated save files.
Generate block and chip traces with `python3 scripts/gen_waves.py`, or select a
block with `python3 scripts/gen_waves.py --only shared_controller --no-e2e`.
Outputs go to ignored `test/waves/`; `MANIFEST.txt` lists generated traces.
Each FST has a matching GTKWave save file with curated signals. Open it with
`gtkwave test/waves/shared_controller__stable.gtkw` (use the name in the manifest),
or open an FST directly in Surfer. Regenerate save files after changing templates
with `python3 scripts/gen_wave_saves.py`.

View the integration waveform using `gtkwave test/tb.fst test/tb.gtkw` or `surfer test/tb.fst`.

The test workflow also checks full-design synthesis and asserts no variable
divider/modulo cells are present. Generic synthesis with Yosys (ABC available)
can be reproduced with:

```sh
yosys -p 'read_verilog -I src src/*.v; synth -top tt_um_j_xu0_stepper; check; stat'
```

For GF180-mapped area measurements, use the filtered Liberty library from the
matching hardening run (or the corresponding PDK library), with Yosys and ABC:

```sh
python3 scripts/measure_gf180_area.py --lib /path/to/gf180.lib --output /tmp/stepper-area
```

The script stores its synthesis commands, log, mapped netlist and area JSON;
results include the Liberty and source hashes. It does not perform placement,
routing, STA, or signoff. The current tip measures 36.3 k um² local and fits 1x1
(42,913 um² final placement); see [the area budget](../docs/area.md) for the
ledger and the measurement caveats that make small deltas meaningless.
