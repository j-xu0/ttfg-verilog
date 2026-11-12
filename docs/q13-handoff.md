# Q13 handoff: P-only split, measured area ledger, and the 1x1 decision

Date: 2026-11-12. Branch `tmp/q8-area-test`. Supersedes the PI-area conclusion
in [`q12-handoff.md`](q12-handoff.md) §7. Read Q12 for the exact-Q11 brake
algorithm and the Q12 test map.

## 1. Executive summary

The integrator, Ki field and anti-windup path are gone. The proportional
controller is a separate single-cycle module and the serial planner contains no
PI state.

**Spec-preserving local area: 42,718.6 µm²** (sequential 18,402.4, 1,691
cells), down from **47,613.9** (21,161.7 sequential, 1,833 cells) —
**−4,895.3 µm² (−10.3 %)**.

CI run `43310580360` (commit `5420187`):

```
Movable instances area:        43309.101 um^2
Pin density area adjust:        5867.302 um^2
Movable instances area:        49176.403 um^2  (GPL-0018)
Utilization:                      97.497 %    (GPL-0018, under 100 %)
```

GPL passes, but **detailed placement fails** with `DPL-0036` on 42
repair-design instances. This is a real density problem: changing
`PL_TARGET_DENSITY_PCT` 60 → 80 → 40 changed nothing (movable area stayed
43,309.101 µm² and DPL failed identically).

The main conclusion for the next agent: **P-only is not enough by itself.**
The remaining P stage is already close to its feature-preserving floor, and the
cheap feature/numeric cuts below are what actually move the area. The Q12
"planner-only 12.9 k" number was not synthesizable, so the old PI marginal was
overstated by ~5 k.

## 2. Current local area ledger (STABLE_CONFIG=1)

| piece | area | cells / seq |
|---|---:|---|
| `motion_engine` (planner only) | **16,723.0** | 715 / 6,748.0 |
| `pi_p_controller` (P only) | **7,404.4** | 374 / 954.9 |
| open-loop top (P stage deleted) | 36,282.3 | 1,381 / 17,361.8 |
| **full top** | **42,718.6** | 1,691 / 18,402.4 |
| `step_dir_generator` | ~6,831.5 | — |
| `stepper_spi` | ~6,767.8 | — |
| `quadrature_decoder` | ~2,794.5 | — |
| `control_timebase` | ~1,870.3 | — |

The P stage's marginal cost at the top is therefore about **6.4 k**, not the
~16 k the Q12 note implied for the old PI.

## 3. What changed in the RTL

* `src/motion_engine.v` is the planner-only serial engine. It keeps the exact
  Q11 brake sum, serial position advance and the 1-bit shared ALU.
* `src/pi_p_controller.v` is a dedicated P stage. Error is integer counts, the
  P term is `error << (8 + Kp)`, and the sum with feedforward is clamped to the
  profile `vmax`. Any `|error| > 64 >> Kp` is pre-clamped because its Q11 P
  term exceeds 16,384 and therefore saturates for every legal
  feedforward/limit, so the whole datapath fits in 16 bits.
* `src/shared_motion_controller.v` is a thin wrapper: on a tick it lets the P
  stage register one command from the pre-advance reference, then advances the
  serial planner. A registered one-cycle `pi_calculating` strobe replaces the
  old multi-cycle PI pass window. Clear/disable zero the command without losing
  the pending planner advance; `enable=0`/reset abort both.
* `reference_position`, `remaining` and `stopping` are **24 bits**, not 25. The
  legal integer position is exactly −4 096…+4 095, which is 13 signed integer
  bits plus 11 fraction bits. The serial accumulator is 25 bits. This saved
  about 0.75 k without changing the delivered numeric contract.
* `CONTROL[1:0]` (the old Ki field) is now reserved/ignored. The SPI register is
  unchanged for host compatibility.
* The Q12 handoff's planner-only measurement reset the combinational
  `operand`/`operand_a` signals. Yosys then constant-folded them to zero
  (`operand = 4'b0000`), producing a non-functional 12.9 k planner. That is why
  its PI marginal was 16 k instead of the true ~10.7 k. **Never reset a signal
  that is also driven by `always @*`; verify the netlist operand mux before
  trusting a planner/PI-only area delta.**

## 4. P-only redesign: how much is really left?

Measured standalone P variants (same flow, `kp_select`/clamp features removed
one at a time):

| P variant | area | delta |
|---|---:|---:|
| full P-only | 7,404 | — |
| fixed Kp = 1 (gain mux deleted) | 6,509 | −0.9 k |
| no `vmax` clamp (raw 16-bit output) | 4,891 | −2.5 k |
| fixed gain + no clamp | **3,734** | **−3.7 k** |

Those are the ceilings *with features deleted*. The clamp cannot simply
disappear: it has to be reproduced in the step generator, and the first
cross-module attempt measured **worse** (+0.35 k on the 25-bit revision). The
gain mux is the documented `CONTROL[3:2]` feature.

A serial P integrated into the planner ALU was also tried: the whole engine
measured **26,750.7 µm²**, versus **~26.9 k** for planner + parallel P on that
revision. Sharing the adder buys ~0.2 k because the clamp/compare schedule, not
the ALU, is the cost.

**Conclusion: a feature-preserving P redesign can plausibly recover 1.5–2 k,
not the ~3.4 k needed. P micro-optimization is no longer the lever.**

## 5. Feature/numeric cuts, measured or estimated

| cut | local top | notes |
|---|---:|---|
| **Fix `vmax = 8191`** | **40,082.2** | keeps 8 acceleration codes; `PLAN[2:0]` reserved. DPL passed on this experiment (it reached DRT). |
| Fix both `vmax` and `accel` | **39,728.7** | removes all PLAN profile selection; only −0.35 k beyond fixed vmax. |
| Fix Kp to one code | ~−0.9 k | `CONTROL[3:2]` reserved. |
| Move P clamp into step generator | measured worse in first attempt | needs a careful rework before it is worth retrying. |
| Halve velocity ceiling (max 2000 steps/s) | ~−1 to −1.5 k estimated | keeps all 64 profile codes, but changes the delivered speed table and lets widths drop. Not yet built on this tree. |
| SPI/stepgen/timebase contraction | ~1–2 k total | SPI frame staging or the stepgen 25-bit phase accumulator; both change documented contracts. |

The 90 % headroom target corresponds to roughly **39.3 k local** using the
observed CI ratios. Fixed vmax alone is ~40.1 k; fixed both is ~39.7 k. So even
the profile cuts are only just at the line. A numeric descope or a second cut
is likely needed if the current DRT experiment does not converge.

## 6. CI history on this branch

| commit | change | CI result |
|---|---|---|
| `5420187` | spec-preserving P-only split, 24-bit position | GPL **97.497 %**, DPL-0036 fail (42 instances) |
| `5a1a897` | `PL_TARGET_DENSITY_PCT=80` | identical movable area and DPL fail |
| `8d8fb97` | `PL_TARGET_DENSITY_PCT=40` | identical movable area and DPL fail |
| `ad78ad6` | **experiment:** fixed `vmax=8191` | DPL passed; reached detailed routing. At time of writing run `43312425240` was still in DRT, with 2,631 violations after the 9th iteration (1,205 Metal2 shorts, 790 Metal3 shorts, 322 Metal4 shorts). Not yet a fit. |

The `ad78ad6` experiment is **not the final intended design**. It demonstrates
that detailed placement was density-limited and that the velocity LUT is the
biggest cheap area lever, but its DRT congestion shows 40.1 k is still tight.

## 7. Branch state / what to do next

Current `HEAD` is `ad78ad6`, the fixed-`vmax` experiment. The last
spec-preserving commit is `d1acbd8`.

* If the experiment eventually routes cleanly: decide whether dropping
  `PLAN[2:0]` velocity selection is acceptable, then update `info.md`,
  `motion-planning.md` and the PLAN register description before merging.
* If it fails in DRT: revert the experiment with
  `git revert ad78ad6` (or restore `src/motion_engine.v` from `d1acbd8`) and
  pick from the table in §5. The next cheapest combination is fixed `vmax` plus
  one of: fixed Kp (~0.9 k), fixed acceleration (~0.6 k), or the numeric
  velocity descope (~1–1.5 k).
* Do not wait on local area alone. The CI GDS job is the fit authority, and
  DPL/DRT have both now failed at different points.

## 8. Tests

* `python3 -m pytest test/test_shared_controller.py -q` — both
  `STABLE_CONFIG` values pass with the P-only oracle. The 96-offset PI
  cancellation sweep was replaced by a one-clock clear sweep across the planner
  pass; the P stage has no in-place rotation to protect.
* `make -C test -B` — **9/9** integration cases pass.
  `full_speed_and_default_limits` was updated from the retired 10 kHz
  assumption to the current 4 kHz / Q11 ceiling. `serial_reference_sampling`
  checks the P publication edge; `shared_engine_abort_phases` passes.
* `test_planner`, `test_compact_planner`, `test_pi`, `test_step_dir` remain
  Q16-era reference tests for modules outside `PROJECT_SOURCES`; the CI module
  test job still fails on them. They are unrelated to the current P-only engine.

## 9. Do not retry

* The Q12 "planner-only" ablation without checking the synthesized operand mux.
* A parallel full PI.
* A serial P-only inside the shared engine (≈0.2 k better, not worth the
  scheduling complexity).
* Moving the P clamp into the step generator without a fresh design; the first
  attempt was worse.
* `PL_TARGET_DENSITY_PCT` as a fix for DPL when the design is already at ~97 %
  utilization — it did nothing.
