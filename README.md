![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg)

# Tiny Tapeout GF180 Stepper Motor Controller

A staged implementation of the ECE298A stepper-driver proposal. The intended
controller receives signed target positions over SPI, plans trapezoidal motion,
and uses quadrature feedback and a PI controller to drive an external STEP/DIR
motor driver.

**Stage 1 is implemented:** mode-0 SPI, TARGET/CONTROL/PLAN registers, synchronized
quadrature decoding, fault reporting, and Tiny Tapeout integration. Motor outputs
remain inactive. This is an interface milestone, not a complete motor controller
or a tapeout-ready design.

The next milestone adds standalone, configurable timebase and trajectory-planner
modules with unit tests. They are not connected to the command registers or motor
outputs yet; see the roadmap for the remaining integration decisions.

- [Interface, pinout, and timing](docs/info.md)
- [Implementation steps and unresolved proposal details](docs/implementation.md)
- [Running the tests](test/README.md)

The top module is `tt_um_j_xu0_stepper`. The GF180 build uses `gf180mcuD` and the
repository's existing `ttgf26a` workflow. The system clock target is **10 MHz**;
`info.yaml` and the 100 ns constraint in `src/config.json` agree. The 1x1 tile
allocation is provisional until the complete design is hardened.

With Python, Icarus Verilog, and make installed, run from the repository root:

```sh
python3 -m venv .venv
. .venv/bin/activate
pip install -r test/requirements.txt
python -m pytest test/test_spi.py test/test_encoder.py test/test_timebase.py test/test_planner.py -q
make -C test -B
```

The test workflow runs both unit and integration simulations. The separate GDS
workflow performs GF180 hardening, precheck, and gate-level simulation when run
on GitHub. Local RTL simulation does not establish post-layout timing or physical
verification results.
