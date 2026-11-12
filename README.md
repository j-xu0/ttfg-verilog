![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg)

# Tiny Tapeout GF180 Stepper Motor Controller

A staged implementation of the ECE298A stepper-driver proposal. The intended
controller receives signed target positions over SPI, plans trapezoidal motion,
and uses quadrature feedback and a PI controller to drive an external STEP/DIR
motor driver.

**Closed-loop RTL is implemented:** SPI command registers, synchronized encoder
feedback, a Q16 trapezoidal planner, PI control with anti-windup, and timed
STEP/DIR/ENABLE outputs. TARGET commands now drive motion in simulation, correct
missed steps and disturbances, and report busy/settled/fault status.

The numerical and driver-interface choices are documented below. GF180 tile fit,
10 MHz physical timing, gate-level verification, and hardware tuning are pending.

- [Interface, pinout, and timing](docs/info.md)
- [Implementation steps and unresolved proposal details](docs/implementation.md)
- [Running the tests](test/README.md)

The top module is `tt_um_j_xu0_stepper`. The GF180 build uses `gf180mcuD` and the
repository's existing `ttgf26a` workflow. The system clock target is **10 MHz**;
`info.yaml` and the 100 ns constraint in `src/config.json` agree.

**The mandatory allocation is 1x1.** The area-optimized controller still exceeds
that budget: controlled GF180 synthesis measures 58,175 um² versus the original
140,824 um². It is development RTL until a 1x1 hardening run succeeds. See
[the optimization results](docs/area-optimization.md) and
[the hardening history](docs/hardening.md).

With Python, Icarus Verilog, and make installed, run from the repository root:

```sh
python3 -m venv .venv
. .venv/bin/activate
pip install -r test/requirements.txt
python -m pytest test/test_spi.py test/test_encoder.py test/test_timebase.py test/test_planner.py test/test_compact_planner.py test/test_pi.py test/test_step_dir.py -q
make -C test -B
```

The test workflow runs both unit and integration simulations. The separate GDS
workflow performs GF180 hardening, precheck, and gate-level simulation when run
on GitHub. Local RTL simulation does not establish post-layout timing or physical
verification results.
