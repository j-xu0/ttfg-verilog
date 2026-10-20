# Verification

Install Icarus Verilog and make, then run from the repository root:

```sh
python3 -m venv .venv
. .venv/bin/activate
pip install -r test/requirements.txt
python -m pytest test/test_spi.py test/test_encoder.py test/test_timebase.py test/test_planner.py test/test_compact_planner.py test/test_pi.py test/test_step_dir.py -q
make -C test -B
```

The pytest runners compile each standalone module and run cocotb assertions.
Each suite contains many directed cases:

- Timebase and planner: standalone motion-building-block tests described in
  [the motion-planning contract](../docs/motion-planning.md), including Q8/Q16
  equivalence and serial command preparation. The integrated compact planner
  separately checks all 64profiles, sentinel bounds, stable-config mode, transient
  workspace sampling, mid-calculation aborts, and generated ROM reproducibility.
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

The Makefile compiles `tt_um_j_xu0_stepper` through `tb.v` and runs seven integration
tests at 10 MHz clk / 500 kHz SPI: pins/status/reset, RTL wiring, short moves with
a lost step and shaft disturbance, abort/fault recovery, both absolute signed
boundaries, and a long move reaching the 10 kHz pulse ceiling followed by the
lowest PLAN codes. The seventh test checks RTL serial-reference sampling before
workspace reuse. Motor-model tests use external pins with one legal encoder
transition per STEP edge. They verify digital feedback
behavior, not a real motor's mechanics or tuning. Results are `test/results.xml`
and `test/tb.fst`. Planner reset must span a rising clock edge.

For gate-level verification, first harden with the matching GF180 Tiny Tapeout
flow. Place its powered netlist at `test/gate_level_netlist.v` and point
`PDK_ROOT` at a PDK installation containing the `gf180mcuD` directory:

```sh
make -C test -B GATES=yes
```

The hierarchy-dependent wiring and serial-sampling tests are skipped with `GATES=yes`;
all external pin tests still run. The retained GF180 GDS workflow separately
handles netlist and PDK setup in GitHub Actions. No gate-level or physical result
is implied by the local RTL tests.

View waveforms using `gtkwave test/tb.fst test/tb.gtkw` or `surfer test/tb.fst`.

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
routing, STA, or signoff. Current controlled full-top area is 71,420.832 um² and
still exceeds 1x1; see `docs/area-optimization.md`.
