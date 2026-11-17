![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg)

# Tiny Tapeout GF180 Stepper Motor Controller

**Codename: TinyStep.** "TTS" was a placeholder; the chip's working name is now
TinyStep. The module name and `info.yaml` title are unchanged for now, so the
rename is a documentation-only change.

A staged implementation of the ECE298A stepper-driver proposal. The controller
receives signed target positions over SPI, plans trapezoidal motion, and uses
quadrature feedback and a fixed-gain proportional (P) controller to drive an
external STEP/DIR motor driver.

**Closed-loop RTL is implemented:** SPI command registers, synchronized encoder
feedback, a Q11 trapezoidal planner, a fixed-gain P stage, and timed
STEP/DIR/ENABLE outputs. TARGET commands drive motion in simulation, correct
missed steps and disturbances, and report busy/settled/fault status.

**The 1x1 tapeout build now hardens and passes precheck** (CI run
`44132561693`: 0 DRT violations, minimum feasible density 0.8300). Post-layout
timing signoff, gate-level harness X handling (`2fa46ad` stops the X at source in
the planner bit index; the bench still needs a guard), and hardware tuning remain.

- [Interface, pinout, and timing](docs/info.md)
- [Implementation steps and unresolved proposal details](docs/implementation.md)
- [The motion-control contract](docs/motion-planning.md)
- [Area budget, measurement discipline, and rejected alternatives](docs/area.md)
- [Running the tests](test/README.md)
- [Generating and viewing waveforms](test/README.md#waveforms)

The top module is `tt_um_j_xu0_stepper`. The GF180 build uses `gf180mcuD` and the
repository's existing `ttgf26a` workflow. The system clock target is **10 MHz**;
`info.yaml` and the 100 ns constraint in `src/config.json` agree.

**The mandatory allocation is 1x1 and the current tip meets it.** The fit uses
the P-only fixed-Kp restriction set: no velocity feedforward, fixed Kp (former
code 1), and a fixed 8191 velocity ceiling. The earlier 58,175 um² design needed
those cuts; the full ledger, measurement caveats and the list of approaches that
were measured and rejected are in [the area budget](docs/area.md).

The open PR stack is not merge-ready: #8 is a draft, the tip branch has no PR, and
only the tip may claim 1x1 fit. Trim the area-history PR last.

With Python, Icarus Verilog, and make installed, run from the repository root:

```sh
python3 -m venv .venv
. .venv/bin/activate
pip install -r test/requirements.txt
python -m pytest test/test_spi.py test/test_encoder.py test/test_timebase.py test/test_step_dir.py test/test_shared_controller.py -q
make -C test -B
```

The test workflow runs both unit and integration simulations. The separate GDS
workflow performs GF180 hardening, precheck, and gate-level simulation when run
on GitHub. Local RTL simulation does not establish post-layout timing or physical
verification results.
