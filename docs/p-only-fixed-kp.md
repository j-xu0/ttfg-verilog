# P-only fixed-Kp experiment

Branch `exp/p-only-fixed-kp`, based on `1547004` (the fixed-`vmax` experiment).
This keeps the closed loop but drops the two remaining P-stage features:

* **no velocity feedforward** — `velocity_command` is now purely `Kp·error`;
* **fixed Kp = former code 1** — `error` is clamped to ±32 and scaled by `2^9`
  (Q11), i.e. 250 steps/s per encoder count at the 1 kHz control rate.

`CONTROL[3:2]` is retained in the SPI frame but reserved. The planner, step
generator, encoder and timebase are unchanged; `vmax` selection is still the
fixed 8191 of the underlying experiment.

## Local area (same flow as `scripts/measure_gf180_area.py`)

| piece | before (`1547004`) | after | delta |
|---|---:|---:|---:|
| full top | 40,231.4 | **36,302.0** | **−3,929.4 (−9.8 %)** |
| `shared_motion_controller` (STABLE=1) | 21,633.7 | 18,997.3 | −2,636.4 |
| `pi_p_controller` | 7,540.5 | 4,664.8 | −2,875.7 |
| sequential area (top) | 18,178.5 | 17,583.6 | −594.9 |

Using the q13 CI factor (`total/local = 1.18694`, core 51,966.97 µm²) this is
**82.9 % utilisation**, down from 91.9 %. The 80 % line is 35,026 µm², so about
**1.3 k** is still missing.

## Control impact

Feedforward is a tracking term, not a loop-dynamics term: dropping it does not
change the error-loop poles. The plant is a velocity command into a position
integrator, so P-only is a type-1 loop (zero steady-state error to a position
step). The cost is a steady cruise lag `e_ss = v_ref/Kp` — up to 16 counts at the
4 steps/tick ceiling for Kp=1 — and weaker rejection of a constant disturbance.
The fixed Kp=1 gives a discrete loop gain `Kp·T = 0.25`, which is comfortably
damped; the retired `kp=3` code was deadbeat and should not be reintroduced.

## Measured follow-ups (local, not committed)

| variant | top | Δ vs fixed-vmax base | est. util |
|---|---:|---:|---:|
| this branch | 36,302 | −3,929 | 82.9 % |
| + fixed acceleration (`profile_acceleration(7)`) | **35,773** | −4,458 | 81.7 % |
| + fixed acceleration + cheap 14-bit clamp | 35,817 | −4,414 | 81.7 % |
| halve velocity ceiling (no width narrowing) | 36,025 | −4,206 | 82.2 % |

Fixing acceleration makes `PLAN` entirely unused and reserves the last profile
field. The cheap clamp is a wash because the fixed `vmax` already constant-folds
the comparators. Halving the velocity ceiling without narrowing `speed`,
`reference_velocity` and the command path is slightly *worse*, so that descope
needs the width changes to pay off.

## Tests

* `python3 -m pytest test/test_shared_controller.py -q` — **2/2** pass.
  The oracle was updated for the fixed `vmax = 8191` experiment and the P-only
  fixed-Kp law.
* `make -C test -B` — **9/9** integration tests pass.
  `serial_reference_sampling` and the shared-controller oracle were updated for
  the new command law.

Local synthesis is not a fit decision. Push this branch and read the CI `gds`
run (`GPL-0018`/`GPL-0301`, `DPL`, `DRT`) before trusting the 82.9 % number.

## CI result — run 44132561693 (commit 2ed4e39)

The flow hardened and routed cleanly for the first time on 1x1 (`gds` job passed
in 5m37s, `precheck` passed):

| metric | value |
|---|---:|
| GPL original cell area | 41,578.41 µm² |
| GPL final placement area (+3.21 % routability inflation) | 42,913.48 µm² |
| available free area | 50,439.11 µm² |
| minimum feasible density | **0.8300** |
| placement target density | 0.8505 |
| DRT violation trajectory | 495 → 177 → 130 → 4 → 1 → 1 → **0** |
| GDS / precheck | **pass** |

For comparison, the previous fixed-`vmax` submission never finished DRT (it was
cancelled at the 6 h job limit after plateauing around 2.6–3.0 k violations),
and the spec-preserving P-only run was at 97.5 % utilisation with `DPL-0036`.

`gl_test` still fails, but in the testbench rather than the netlist: gate-level
`uo_out` can briefly carry X and `test/test.py` calls `int(dut.uo_out.value)`
(`MotorModel.run`). The RTL suites pass; this is a GL harness X-handling issue.
The separate `test` workflow is red only on the pre-existing Q16-era
`compact_planner`/`step_dir` tests (14 passed, 12 failed).
