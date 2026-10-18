# Verification

Install Icarus Verilog and make, then run from the repository root:

```sh
python3 -m venv .venv
. .venv/bin/activate
pip install -r test/requirements.txt
python -m pytest test/test_spi.py test/test_encoder.py -q
make -C test -B
```

The pytest runners compile each standalone module and run cocotb assertions.
Each suite contains many directed cases:

- SPI: all registers and signed endpoints, truncation/masking, atomic commit,
  every short frame length, overlong/empty frames, reserved address, status
  snapshot, busy at start/during/commit, reset while selected, physical rising-edge
  MISO sampling, and minimum supported SCLK timing at varied clock phases.
- Encoder: all four stationary startup phases, both directions, holding a phase,
  all invalid two-bit transitions, sticky fault, counting after fault, signed
  wrap in both directions, and reset recovery.

The Makefile compiles `tt_um_j_xu0_stepper` through `tb.v` and runs two integration
tests: external pins/status/reset at 10 MHz clk and 500 kHz SPI, and RTL register
and encoder wiring. It writes `test/results.xml` and `test/tb.fst`.

For gate-level verification, first harden with the matching GF180 Tiny Tapeout
flow. Place its powered netlist at `test/gate_level_netlist.v` and point
`PDK_ROOT` at a PDK installation containing the `gf180mcuD` directory:

```sh
make -C test -B GATES=yes
```

The hierarchy-dependent wiring test is explicitly skipped with `GATES=yes`;
the external pin test still runs. The retained GF180 GDS workflow separately
handles netlist and PDK setup in GitHub Actions. No gate-level or physical result
is implied by the local RTL tests.

View waveforms using `gtkwave test/tb.fst test/tb.gtkw` or `surfer test/tb.fst`.
