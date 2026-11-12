# Verification

For a self-contained RTL test environment, build and run the container from the
repository root. Python 3.11, Icarus Verilog, make, and the pinned Python test
dependencies are included; nothing needs to be installed on the host besides
Docker:

```sh
docker build -f test/Dockerfile -t ttfg-tests .
docker run --rm -it ttfg-tests
```

Pass suite names after the image name for a shorter run:

```sh
docker run --rm -it ttfg-tests spi encoder timebase
```

The `-it` enables colored badges and live progress. Omit it in CI. Tests run
as an unprivileged user inside the image, without writing caches to the host.
Rebuild after source changes. Logs and waveforms stay inside the container;
to keep them, name the container and copy results out before removing it:

```sh
docker run --name ttfg-test-results ttfg-tests spi encoder timebase
docker cp ttfg-test-results:/work/test/output ./test-output
docker rm ttfg-test-results
```

For a native setup, install Icarus Verilog and make, then run from the repository root:

```sh
python3 -m venv .venv
. .venv/bin/activate
pip install -r test/requirements.txt
python scripts/run_tests.py
```

The runner shows a live elapsed-time line in a terminal, a PASS/FAIL result for
each suite, and a final count and duration. Redirected output and CI use plain,
append-only lines; set `NO_COLOR=1` to disable terminal colors. Successful
compiler/simulator output is saved under `test/output/runner/`. The final report
shows every suite with its counts, timing, and log path for failures. Add
`--verbose` to print the last 60 lines of failed logs before the compact report. All selected suites run even if
an earlier suite fails, and any failed command, missing/invalid report, or XML
failure/error makes the runner exit nonzero. Ctrl-C stops the active suite and
exits with status 130.

Run selected suites with `python scripts/run_tests.py spi encoder`, or use
`python scripts/run_tests.py integration` for top-level tests only. Use
`--log-dir /path/to/logs` to choose a log directory. The final test count combines
pytest cases (including parameterized cases) for module suites with cocotb cases
for integration; it does not count assertions inside a case. Unit JUnit reports
are stored alongside their logs; integration keeps `test/results.xml` and
`test/tb.fst`. The original pytest and make commands remain available for raw
simulator output.

The pytest runners compile each standalone module and run cocotb assertions.
Each suite contains many directed cases:

- Timebase and planner: standalone motion-building-block tests described in
  [the motion-planning contract](../docs/motion-planning.md), including Q8/Q16
  equivalence and mirror-rule trapezoid/triangle coverage. The standalone compact planner
  separately checks all 64 profiles, sentinel bounds, stable-config mode, transient
  workspace sampling, mid-calculation aborts, and generated ROM reproducibility.
- Shared controller: both captured/stable configurations, all profiles and gain
  pairs, sampled input isolation, PI-only correction, cancellation, shared-ring
  restoration and completion within 2,000 clocks.
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
boundaries, and a long move reaching the 10 kHz pulse ceiling followed by the
lowest PLAN codes. The seventh test checks RTL serial-reference sampling before
workspace reuse; the eighth checks raw disable/reset during both shared-engine
phases and prevents restart. The ninth is the directed waveform scenario: three
SPI setup writes while disabled, then a 64-step accel/decel-only move with no
cruise, ending in an at-target status read. Shared module tests also sweep 768 transient PI
cancellations and retain exact trajectory/PI oracles. Motor-model tests use external pins with one legal encoder
transition per STEP edge. They verify digital feedback
behavior, not a real motor's mechanics or tuning. Results are `test/results.xml`
and `test/tb.fst`. Planner reset must span a rising clock edge.

For gate-level verification, first harden with the matching GF180 Tiny Tapeout
flow. Place its powered netlist at `test/gate_level_netlist.v` and point
`PDK_ROOT` at a PDK installation containing the `gf180mcuD` directory:

```sh
python scripts/run_tests.py --gates
```

The hierarchy-dependent wiring, serial-sampling and phase-abort tests are skipped with `GATES=yes`;
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
routing, STA, or signoff. Current controlled full-top area is 58,174.995 um² and
still exceeds 1x1; see `docs/area-optimization.md`.
