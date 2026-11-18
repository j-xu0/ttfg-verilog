# Area budget: how the design fits 1x1

The allocation is 1x1: core **51,966.970 um²**. The last measured tip
(`exp/p-only-fixed-kp`) fits it. CI run
[`44132561693`](https://github.com/j-xu0/ttfg-verilog/actions/runs/44132561693)
on commit `2ed4e39` (this measurement predates the SPI status and interrupt
change; see [the status-port cost](#the-status-port-cost) below):

| Metric | Value |
| --- | ---: |
| GPL original cell area | 41,578.41 um² |
| Final placement area (+3.21 % routability inflation) | 42,913.48 um² |
| Available free area | 50,439.11 um² |
| Minimum feasible density | **0.8300** (target 0.8505) |
| DRT violations | 495 -> 177 -> 130 -> 4 -> 1 -> 1 -> **0** |
| GDS / precheck | pass / pass (gds job 5m37s) |

The restriction set that closed the gap: P-only loop, no velocity feedforward,
Kp fixed at the former code 1 (error clamp ±32, `<< 9`), `vmax` fixed at 8191.
Acceleration is the 3-bit PLAN code. Reserved: SPI addr 1 (ex-CONTROL gain) — rejected.
[the control contract](motion-planning.md#integrated-q11-numeric-format) for the
numeric consequences.

## Ledger (local mapped top, one flow)

| Stage | Top area | Change |
| --- | ---: | ---: |
| Independent PI + planner engines, after the module audit | 61,318.5 | −14.1 % vs 71,420.8 |
| Shared planner/PI engine, one FSM and serial adder | 58,175.0 | −5.1 % |
| Q11 serial/LUT arithmetic + dedicated single-cycle P stage | 42,718.6 | −26.6 % |
| Fixed `vmax = 8191` | 40,231.4 | −5.9 % |
| P-only fixed Kp (tip) | 36,302.0 | −9.8 % |
| SPI status word + INT pin, transmit register removed, accel operand sized | 38,400.6 | −2.4 % vs its own parent (below) |

The q13 CI factor is `total/local = 1.18694`; the 80 % local line is 35,026 um².
Only same-flow deltas are meaningful. Two earlier 1x1 attempts are informative:
the first full-controller GDS run (PR #5) failed in global placement (`GPL-0301`)
at 334.765 % utilisation with no `PL_TARGET_DENSITY_PCT` value able to help; a
diagnostic 3x2 allocation passed, but 2x2 estimates 75.5 % utilisation, so no
smaller tile worked — the design had to shrink instead. The spec-preserving
P-only run reached 97.5 % utilisation and failed detailed placement (`DPL-0036`).

## The status-port cost

The SPI status word and INT pin (`8ead1ac`) landed **on the cliff, not next to
it**. Two CI `gds` runs, one commit apart, with the same flow and the same
Liberty:

| Metric | parent `4264a78` | `8ead1ac` (+ status/INT) |
| --- | ---: | ---: |
| Post-synthesis cells (seq) | 39,024.07 (19,921.44) | 39,693.61 (20,149.74) |
| Post-place cells | 43,048.79 | 43,919.49 (+2.02 %) |
| GPL auto target density | 0.8600 -> 0.86064 | 0.8800 -> 0.88911 |
| DRT iteration 0 violations | **1,223** | **5,544** |
| DRT outcome | 0 in 10 iterations | never converged, killed at the 6 h cap |
| `gds` job | pass, 15m48s | cancelled, 6h00m |

Runs `44232341277` (pass) and `44240273510` (timeout). Nothing marginal failed a
rule: +2 % of placed area crossed the density at which TritonRoute stops
converging, and it bounced between 3k and 20k violations for six hours. The
parent's own 1,223 starting violations are the warning sign — the design was
already at the edge, so a small feature is not free. Treat any change that adds
more than ~0.5 % of placed area as a routing risk and re-harden.

Two exact rewrites paid for the feature with room to spare (local mapped top,
same flow, cell-for-cell):

| Change | Local area | Note |
| --- | ---: | --- |
| 16-bit MISO shift register -> 4-bit snapshot + position decoder | −873.7 | −13 flops (74.64 um² each); MISO is now decode-combinational, still stable >=4 clocks before the host's rising-edge sample |
| Accel operand sized to the table (`11` -> `8` bits) | −164.7 | max entry is 205; the width is derived in `generate_plan_lut.py` |
| Total vs `8ead1ac` | **−1,038.4** | seq 20,149.7 -> 19,179.5, cells 1,429 -> 1,397 |

That is 928.6 um² (-2.36 %) *below the branch that routed*, so the projected CI
placed area is ~42.0 k and the GPL density returns below 0.86. Sequential area
matches CI exactly in every measurement, so the flop part of the delta carries
over one-for-one; the combinational part does not (ABC moved the same 110 um²
local delta by 441 um² in CI), which is why the margin matters more than the
local total.

## Do not retry

Measured, not guessed. Numbers are local mapped area unless noted.

* **Mirror-rule planner.** −34.8 % standalone (55,196 vs 84,640) but **+7.2 %**
  inside the shared engine (63,641 vs 59,393): the serial design was already
  co-optimised for it, so the deleted LUT cost more than the extra state.
  Rule of thumb: parallel→shrink arithmetic, serial→shrink state.
* **Descoping PI features to close the gap.** Deleting PI entirely (open loop)
  is −25.5 % to 44,146 and was the only thing that fit on its own; the
  integrator is −3,420, anti-windup −1,006, feedforward −933, integral clamp
  −250. Nothing else gets near the gap. PI area is mostly write muxing and
  enable/select logic on 21–30-bit rings, not flag bits.
* **A parallel full PI.** 16,429 standalone vs a 15,981 serial marginal: no
  saving, and it re-serialises nothing.
* **P-only serial inside the shared engine.** ≈0.2 k better; not worth the
  scheduling complexity.
* **Fixing one gain pair or one profile.** 44–206 um² (gains); full top 54,660
  (profile 63) or 54,924 (profile 0) — still over the core.
* **Narrowing the stopping sum.** Accepted sums approach 2^29, so simple width
  reduction loses legal moves.
* **Timebase rewrites.** Binary down-counter 1,993 vs 1,866; shortened LFSR
  1,811 saves 55 um² for a default-only special case.
* **Integral-init flag instead of ring reset.** +215 um².
* **Alternative ROM bit-plane encodings.** Both worse; keep the generated table.
* **Moving the P clamp into the step generator.** First attempt worse.
* **`PL_TARGET_DENSITY_PCT` as a DPL fix.** Nothing at ~97 % utilisation.
* **Follow-ups not taken** (all ≈wash): fixed acceleration 35,773, cheap 14-bit
  clamp 35,817, halving `vmax` without narrowing widths 36,025.
* **Deciding the P clamp on the clamped error instead of the shifted sum.**
  Provably equivalent (`over` is `clamped >= floor(vmax/512)+1`) and nine bits
  instead of sixteen, but measured **+29 um²**: ABC already maps the 16-bit
  compare against the constant ceiling cheaply. Watch the width if you try it —
  `floor(vmax/512)+1` needs six signed bits at `vmax = 8191`, and a five-bit
  version silently wraps to −16, which is why the first two measurements of this
  idea looked like an 800 um² win.

## Measurement discipline

* Use `~/tools/oss-cad-suite/bin/yosys` (0.70) + `yosys-abc`. The apt 0.9 yosys
  lacks `stat -json` and cannot run the script.
* GF180 PDK via volare: family name is `gf180mcu`, **not** `gf180mcuD`. Liberty
  used throughout:
  `~/.volare/volare/gf180mcu/versions/c6d73a35.../gf180mcuD/libs.ref/gf180mcu_fd_sc_mcu7t5v0/lib/gf180mcu_fd_sc_mcu7t5v0__tt_025C_3v30.lib`
  (stock PDK typical corner; deltas within one flow are the signal, absolute
  values skew a few % against signoff corners).
* Measure **only** the `PROJECT_SOURCES` list from `test/Makefile` — the numbers
  above were taken with six files, before `motion_engine.v` and
  `pi_p_controller.v` were split out; it is eight now. Leaving
  the reference modules in the source dir moves the baseline 59,244 → 58,256.
* Yosys+ABC is **not monotone** in logic size: an algebraically identical adder
  rewrite moved the top +310 um², and removing the PI gain muxes moved it
  **+1,750**. Treat deltas under ~1 k as noise and re-measure a surprising
  result with an independently phrased variant.
* `scripts/measure_gf180_area.py` writes commands, log, netlist and area JSON to
  the output dir and records source/Liberty hashes. It does not place, route,
  time or sign off.

## Open items

* Gate-level simulation: `uo_out` can briefly carry X and `test/test.py` calls
  `int(dut.uo_out.value)` in `MotorModel.run`. `2fa46ad` resets the planner bit
  index to stop the X at source; the harness still needs a guard either way.
* Post-route timing signoff and hardware tuning are outstanding.
* At `2ed4e39` the `test` workflow was red only on pre-existing Q16-era
  `compact_planner`/`step_dir` cases (14 passed, 12 failed). The compact-planner
  suite has since been removed from the tree, so re-read the count on the
  rebased tip rather than trusting that split.
* The open PR stack must not claim 1x1 fit below the tip; only this branch does.
