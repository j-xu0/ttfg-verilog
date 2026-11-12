# Q13 handoff: P-only split, corrected planner area, and the 1x1 result

Date: 2026-11-12. Branch `tmp/q8-area-test`. Supersedes the PI-area conclusion
in [`q12-handoff.md`](q12-handoff.md) §7; read that note for the exact-Q11 brake
algorithm and the Q12 test map.

## 1. Result

The integrator and anti-windup path are gone, and the proportional controller
is a separate single-cycle module. The serial planner no longer contains any PI
state.

**Local** (stock GF180 lib, the eight files now in `PROJECT_SOURCES`):
**42,718.6 µm²** (sequential 18,402.4, 1,691 cells).
Baseline on the same flow was 47,613.9 (21,161.7 sequential, 1,833 cells), so
this is **−4,895.3 µm² (−10.3 %)**.

| block (standalone, `STABLE_CONFIG=1`) | area |
|---|---:|
| `motion_engine` (serial planner only) | 18,128.0 (seq 6,939.0, 797 cells) |
| `pi_p_controller` (dedicated narrow P) | ~7,479 (seq 954.9, 385 cells) |
| `shared_motion_controller` wrapper, flattened top contribution | remainder |
| `step_dir_generator` | 6,831.5 |
| `stepper_spi` | 6,767.8 |
| `quadrature_decoder` | 2,794.5 |
| `control_timebase` | 1,870.3 |

Using the Q12 CI/local relation (`total ≈ 1.1656·local + 1,527.9`) gives
**≈98.8 % utilisation**, i.e. under the GPL limit but not at the 90 % headroom
target. The CI number on this commit is the authority.

## 2. The Q12 planner number was not synthesizable

`q12-handoff.md` reports "planner only (PI structurally deleted) 12,934.1
(seq 5,220.2, 552 cells)" and derives a 15,981 µm² PI marginal from it. That
ablation put reset assignments on the combinational `operand`/`operand_a`
signals. Yosys responded by constant-folding both operand muxes to zero (the
netlist has `operand = 4'b0000`), which is why the cell count is suspiciously
small and cannot work on silicon.

Measured correctly (no reset on the combinational operand mux), the planner is
**18,128.0 STABLE=1**, so the old PI's true marginal was about **10.7 k**, not
16 k. That is the main correction for future area planning: do not trust a
planner-only or PI-only delta unless the operand mux survives synthesis untouched.

## 3. What changed in the RTL

* `src/motion_engine.v` is the planner-only serial engine. It keeps the exact
  Q11 brake sum, serial position advance and 1-bit shared ALU.
* `src/pi_p_controller.v` is a dedicated P stage. Error is integer counts, the
  P term is `error << (8 + Kp)`, and the sum with feedforward is clamped to the
  profile `vmax`. Any `|error| > 64 >> Kp` is pre-clamped because its Q11 P term
  exceeds 16,384 and therefore saturates for every legal feedforward/limit, so
  the whole datapath fits in 16 bits.
* `src/shared_motion_controller.v` is now a thin wrapper: on a tick it lets the
  P stage register one command from the pre-advance reference, then advances the
  serial planner. A registered one-cycle `pi_calculating` strobe replaces the old
  multi-cycle PI pass window. Clear/disable zero the command without losing the
  pending planner advance; `enable=0`/reset abort both.
* `reference_position`, `remaining` and `stopping` are **24 bits**, not 25. The
  legal integer position is exactly −4 096…+4 095, which is 13 signed integer
  bits plus 11 fraction bits. The accumulator is 25 bits. This is worth about
  0.75 k and does not change the delivered numeric contract.
* `CONTROL[1:0]` (the old Ki field) is now reserved/ignored. The SPI register is
  unchanged for host compatibility.

## 4. Tests

* `python3 -m pytest test/test_shared_controller.py -q` — both `STABLE_CONFIG`
  values pass with the P-only oracle. The 96-offset PI cancellation sweep was
  replaced by a one-clock clear sweep across the planner pass; the P stage has
  no in-place rotation to protect.
* `make -C test -B` — **9/9** integration cases pass. `full_speed_and_default_limits`
  was updated from the retired 10 kHz assumption to the current 4 kHz / Q11
  ceiling. `serial_reference_sampling` now checks the P publication edge and
  `shared_engine_abort_phases` still passes.
* `test_planner`, `test_compact_planner`, `test_pi`, `test_step_dir` remain
  Q16-era reference tests for modules outside `PROJECT_SOURCES`.

## 5. If 90 % headroom is still required

The remaining ~4 k cannot come from the P stage alone: it is already about as
small as a correct 16-bit P datapath gets, and moving its clamp into the step
generator measured *worse*. The measured next steps, in order:

1. **Fixed velocity/acceleration profile.** Replacing the two 8-entry `plan_lut`
   functions with constants measures **−1.45 k** on the current tree (top
   ≈41.2 k, ≈94 %). This removes PLAN profile selection, a visible spec change.
2. **Numeric descope** as in `q12-handoff.md` §8.3: halve the velocity ceiling,
   dropping one bit from `speed`/`vmax`/command/position integer math. Estimated
   1–1.5 k. Also a spec change.
3. **Non-engine blocks** (SPI 6.8 k, step generator 6.8 k, encoder 2.8 k,
   timebase 1.9 k). The safe, measured wins here are only hundreds of µm²; the
   SPI frame staging and step-generator phase accumulator are the only large
   candidates left and both change a documented contract.

Do not retry the Q12 "planner-only" ablation, a parallel full PI, or a serial
P-only inside the shared engine: the serial P-only variant measured 26,750.7 for
the whole engine (only ~0.2 k better than planner + parallel P) because the
clamp/compare schedule still costs the same as the parallel logic.
