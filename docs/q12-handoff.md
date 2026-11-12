# Q12 handoff: exact Q11 brake is in, here is what is left

Date: 2026-11-12. Branch `tmp/q8-area-test`. Author of this note: the agent that
did the exact-brake rewrite. Read this together with
[`q11-handoff.md`](q11-handoff.md) (CI history, rejected experiments),
[`numeric-spec.md`](numeric-spec.md) (current numeric contract) and
[`pi-vs-planner.md`](pi-vs-planner.md) (measured PI ablations on the older
revision).

Goal unchanged: 1x1 tile, core **51,966.970 µm²**, and "fits with headroom"
means **90 % → (movable + pin-adjust + fixed) ≤ 46,770**, where fixed is
1,527.859. Fit is decided by the CI `gds` workflow (`GPL-0301`, utilization),
never by local synthesis.

## 1. Where we are

**Local** (stock GF180 lib, the six `PROJECT_SOURCES` files, see §5):
**47,613.9 µm²** (sequential 21,161.7, 1,833 cells).
Before this revision: 51,315.0 (sequential 20,206.8, 2,096 cells) → **−7.2 %**.

| block (standalone, `STABLE_CONFIG=1` where applicable) | area |
|---|---:|
| `shared_motion_controller` (whole engine) | 28,915.2 (seq 10,376.7, 1,310 cells) |
| ├ planner only (PI structurally deleted) | 12,934.1 (seq 5,220.2, 552 cells) |
| └ **PI marginal** | **15,981** (55 % of the engine) |
| `step_dir_generator` | 6,831.5 |
| `stepper_spi` | 6,767.8 |
| `quadrature_decoder` | 2,794.5 |
| `control_timebase` | 1,870.3 |
| top glue + flattening effects | ~1.4 k |

**CI** (run 43302465166, commit `8f7189c`):

```
Core area:            51,966.970 um^2
Movable instances:    60,966.251 -> 48,678.560 um^2   (-20.2%)
Total instances:      62,494.110 -> 50,206.419 um^2
Pin density adjust:    8,053.150 ->  6,817.985 um^2
Utilization:         120.871    -> 110.027 %          GPL-0301, still over
Effective util:        1.018    ->   0.937
Movable instances:     2,183    ->   1,943   Nets: 2,202 -> 1,962
```

So 10.9 utilization points were recovered, and **~8.8 k local area is still
needed for 90 %** (see §4 for the exact algebra).

## 2. What this revision changed (commits `36a5a5f`, `8f7189c`)

### 2.1 Root cause of the "high-speed decel never fires" bug

`source_bits` was declared `wire [10:0]` but the concatenation had **10**
entries. Verilog silently zero-extends on the left, so every operand from index
9 up was shifted by one: `ACCUMULATOR` (index 9) read `speed_int_bit` and
`SPEED_INT` (index 10) read constant 0. `TEST_NEW` therefore compared
`speed >> 11` against the remaining distance instead of the trial stopping
distance. Icarus does not warn about this. **Check concat arity against the
declared width whenever you touch that mux.** The wire now has 9 entries
(`[8:0]`), exactly matching.

### 2.2 The 64-entry stopping ROM is gone (was ~2.3 k on its own)

The table only existed because the integer (floor/ceil) brake sum was too coarse
to be trusted incrementally (the per-term truncation error accumulates over the
whole decel ramp, which is what caused the runaway). The engine now tracks the
**exact Q11 brake sum** and compares it with the **exact Q11 remaining
distance**:

```
S(s)      = s + S(s-a)                  (distance needed to stop, Q11)
accept s' iff S(s') <= remaining        (exact, both sides Q11)
S(s') = S(s) + s'   when the speed grows
S(s)  = S(s)        when the speed is held at the profile limit
S(s-a)= S(s) - s    when decelerating
```

consequences: no table, no sentinel, no `ceil`/`floor` mismatch, **the reference
mathematically cannot overshoot**, and unreachable profile pairs are simply
never accepted. `plan_lut.vh` now only feeds `vmax`/`accel` (8 entries each);
`profile_stopping` is kept solely for the standalone reference planners
(`motion_planner.v`, `compact_motion_planner.v`, not in `PROJECT_SOURCES`).

### 2.3 Position advance is serial again (−3.3 k measured standing alone)

The Q11 retarget had replaced the serial pass with a **parallel 22-bit Q13.8 add
plus a 14-bit negate** — the largest arithmetic block left in the design
(stubbing it out measured −3,879). `reference_position` is now **Q11, 25 bits**
(14 integer + 11 fraction), so `pos += vel` is exact and runs as a 25-cycle pass
through the shared 1-bit adder. Bonus: the "low 3 velocity bits truncated" wart
is gone.

### 2.4 Other changes in the same commit

* `reference_velocity` is combinational (`negative ? -speed : speed`); the
  26-cycle output pass is gone.
* Within one step of the target the reference **snaps exactly onto it**
  (`POSITION` restores `goal + 0` instead of `goal ∓ residue`), so the resting
  error is exactly zero.
* `stopping` grew 14 → 25 bits (+11 FF ≈ +0.76 k), `ACCEL` gained a
  `speed < vmax` compare latched as `grow` (1 FF + ~30 gates) so that holding at
  the limit does not re-add the cruise term.
* `test/test_shared_controller.py` now contains a **stateful** oracle
  (position, speed, brake sum) instead of recomputing a floor sum per candidate.
* `test/test.py::serial_reference_sampling` ported to Q11 engine units
  (`error = (reference//2048 - feedback)*2048`, clamp ±4096 for plan code 4).

## 3. How the engine works now (map for surgery)

`src/shared_motion_controller.v`, 366 lines. Everything is one clock domain and
one bit-serial ALU; time is free (a tick uses ~210 planner + ~110 PI cycles of
the 10,000-clock period), area is not.

* Registers: `reference_position[24:0]` (Q11, doubles as remaining-distance
  scratch), `stopping[24:0]` (exact Q11 brake sum), `speed[12:0]` (magnitude),
  `accumulator[25:0]`, `integral[13:0]`, `work[13:0]`, `velocity_command[13:0]`,
  `state[5:0]` (`fsm_encoding="none"`), `working`, `bit_index[4:0]`,
  `operand_a[3:0]`, `operand[3:0]`, `carry[1:0]`, `subtract`, `negative`,
  `source_position_sign`, `capped`, `grow`, `finishing`, `planner_busy`,
  `planner_done`, PI flags (`error_negative`, `integral_negative`,
  `pi_cancelled`, `integral_phase`, `overflow`, `low_sign`, `pi_negative`,
  `clipped`, `pending_saturated`, `comparison_nonzero`), and
  `latched_profile`/`latched_goal` (compiled out when `STABLE_CONFIG=1`).
* Operand mux: `ZERO=0, SPEED=1, ACCELERATION=2, POSITION_SIGNED=3, DISTANCE=4,
  GOAL=5, LIMIT=6, STOPPING=7, ACCUMULATOR=8`; `source_bits[8:0]`.
* A "pass" = `operation(sub, next_state)` sets `carry[0]=sub`, `bit_index=0`,
  `state=next_state`, `working=1`; the pass then runs 26 cycles and ends at
  `bit_index==25`. During a pass, registers whose operand is selected **rotate in
  place** (speed 13×, position 25×, stopping 25×), the ALU result is shifted into
  their MSB, and the write-back is gated by `bit_index < width`. Therefore a
  source tap must always be `reg[0]` (the post-rotation LSB), never
  `reg[bit_index]` — getting this wrong produces plausible-looking garbage.
* Planner FSM: `ADVANCE → BEGIN_DIST/DIST → BEGIN_ACCEL/ACCEL → CAP →
  (LOAD_LIMIT) → TRIAL → TEST_NEW → [CAP_SPEED|ACCEPT_SPEED] → ACCEPT_ADD →
  BEGIN_POS/POSITION` or `TEST_NEW → HOLD_TEST → (DEC_STOP/DEC_SPEED) →
  BEGIN_POS/POSITION`. `DIST` writes `|goal - pos|` into the position register;
  `POSITION` restores it (or snaps to the goal when finishing).
* PI FSM: `PI_ERROR (error = (pos_int-measured)<<11) → PI_SUM (kp·e + ff +
  integral) → PI_CLAMP/COMPARE/FILL (command and integral clamps, anti-windup)
  → PI_PUBLISH`. `pi_calculating = IDLE < state < 32`,
  `planner_calculating = state >= 32 || working`.
* Cancel contract: `clear_control || !control_enable` clears the PI and (after
  the in-place rotation is restored) still lets the planner advance;
  `!enable` aborts everything. `test_shared_controller.py` sweeps a
  one-clock cancel across all 96 offsets of the initial PI passes — do not
  break this.

## 4. The exact utilisation algebra (recompute if the flow drifts)

From the current pair of measurements (same commit):
`CI movable / local = 48,678.560 / 47,613.9 = 1.0224` and
`pin adjust / movable = 6,817.985 / 48,678.560 = 0.14006`, so

```
util(local m) = (1.1656 · m + 1,527.9) / 51,966.97
```

| target | needs local area |
|---|---:|
| 100 % (just passes GPL) | ≤ **43.3 k** (−4.3 k from here) |
| 90 % (working headroom target) | ≤ **38.8 k** (−8.8 k from here) |

Historical warning: this CI/local factor is **not** a constant across commits
(1.00 at `b6c5a90`, 1.19 at `3227876`, 1.022 now). Recompute it from the CI log
of whatever commit you measure locally; do not reuse 1.1656 blindly.

## 5. How to measure (nothing else counts)

```sh
YS=$HOME/tools/oss/oss-cad-suite/bin/yosys
ABC=$HOME/tools/oss/oss-cad-suite/bin/yosys-abc
LIB=/home/north/.volare/volare/gf180mcu/versions/c6d73a35f524070e85faff4a6a9eef49553ebc2b/gf180mcuD/libs.ref/gf180mcu_fd_sc_mcu7t5v0/lib/gf180mcu_fd_sc_mcu7t5v0__tt_025C_3v30.lib
# copy ONLY the six PROJECT_SOURCES + plan_lut.vh into a scratch dir, then
python3 scripts/measure_gf180_area.py --lib $LIB --yosys $YS --abc $ABC \
  --source-dir /tmp/scratch --output /tmp/area-x
# module splits: --top <module> [--parameter STABLE_CONFIG=1]
```

Always measure the same six files: `project stepper_spi quadrature_decoder
control_timebase shared_motion_controller step_dir_generator`. Deltas under
~1 k are mapping noise (the flow is not monotone: algebraically identical
rewrites have moved the top by ±2 k before). Re-check any surprise with a second
phrasing.

Tests:

```sh
python3 -m pytest test/test_shared_controller.py -q      # 2 cases, ~60 s, THE gate
python3 -m pytest test/test_spi.py test/test_encoder.py test/test_timebase.py \
                  test/test_pi.py -q                     # all pass
make -C test -B                                          # 8/9 integration pass
```

Known pre-existing failures (all Q16-era oracles, not caused by this revision):
`test_planner`, `test_compact_planner` (they `#include "plan_lut.vh"` and expect
Q16), all 10 `test_step_dir` cases (Q16-scale velocities, 10 kHz assumptions;
RTL is Q11 / 4 kHz), and integration `full_speed_and_default_limits` (asserts the
retired 10 kHz ceiling). `test.py::serial_reference_sampling` and
`shared_engine_abort_phases` pass.

CI: any push runs `gds`, `test`, `docs`. Only `tmp/q8-area-test` (this branch)
may be pushed without asking; `origin/jason/stepper-shared-engine` is the
friend's read-only base. Read the floorplan lines
(`GPL-0036 Movable`, `GPL-0035 pin density`, `GPL-0301 Utilization`) out of the
`gds` job log.

## 6. Measured ablations on this revision (use these, do not re-derive)

| Variant | Area | Δ |
|---|---:|---:|
| current tree | 47,613.9 | — |
| PI structurally deleted, planner intact (open loop, `velocity_command = reference_velocity`) | 31,130.1 | **−16,483.8** |
| plan LUT → constants (one fixed profile) | 45,469.2 | −2,144.7 |
| P-only (integrator/anti-windup dead) on the shared engine | 46,586.5 | −1,027.4 |
| P-only + fixed profile | 43,796.4 | −3,817.5 |
| step-gen `command_limited` → 0 (provably dead: `MAX_COMMAND`=8192 covers the whole Q11 command range) | 47,572.2 | −41.7 |
| + `rate_limited`/`timing_limited` removed (unused at the top) | 47,453.6 | −160.3 |

Earlier revisions, still valid conclusions: PI `<<11` tap removal −171 (the
1-bit muxes are nearly free; the bits/FFs are what cost), Q13.0 position −1,159
(not viable: sub-500 step/s motion stalls), dead `TAIL_*` state deletion **+299**,
Yosys-chosen FSM encoding **+1,076**, one-hot **+2,790**, SPI `tx_shift` de-dup
**+215** (only 4 of 16 bits survive constant folding), a hand-written parallel
planner 21.5 k standalone versus 21.8 k for the bit-serial one (un-serialising
the *planner* buys nothing).

## 7. The decisive new finding: the PI is ~16 k whatever you do

I measured the PI's marginal cost directly (planner-only engine 12,934 versus
full engine 28,915 → **15,981**). To test whether that is serialisation
overhead, I wrote a fresh dedicated **parallel** PI with every current feature
(integer error, 4 kp/ki codes, feedforward, command clamp + `saturated`,
integral clamp, anti-windup freeze; prototype in `/tmp/pi/pi_parallel.v`,
reproduce it in ~50 lines):

| implementation | area | cells |
|---|---:|---:|
| current serial PI (marginal) | 15,981 | — |
| dedicated parallel, full features | **16,767** | 761 |
| dedicated parallel, P-only (no integrator/anti-windup) | **7,999** | 392 |
| calibration: one 14-bit add + clamp | 4,645 | 198 |

So sharing the ALU is *not* the cost — the gain shifts, clamps, comparisons and
the 14-bit datapath are. **A full PI costs ~16 k in this PDK no matter how it is
implemented**, which puts a hard floor under the whole design:

| configuration | local | CI util |
|---|---:|---:|
| current | 47.6 k | 109.7 % |
| full PI + every PI-internal cleanup (`work`, 26→16-bit PI accumulator, ~−2 k est.) | ~45.6 k | ~105 % |
| + plan table → one profile (−2.1 k) | ~43.5 k | **~100.5 %** |
| P-only on the shared engine (measured) | 46.6 k | 107.4 % |
| P-only on the shared engine + fixed profile | 43.8 k | 101.2 % |
| open loop (PI deleted, measured) | 31.1 k | **72.8 %** |
| planner serial (12.9 k) + **dedicated P-only PI** (8.0 k) | ~39.9 k | **~92.5 %** |
| that + fixed profile (−2.1 k) | ~37.8 k | ~88 % |

Conclusion: **full PI cannot fit 1x1 in this PDK.** The only closed-loop
configuration that clears 100 % is P-only with an *un-shared* PI (because the
shared serial engine hides the integrator's cost: P-only there saves only 1.0 k,
since the clamp/fill/compare schedule stays). Dropping the 64-profile table is
worth only 2.1 k now — the big LUT win is already banked.

## 8. Suggested next experiments, in order

1. **(cheap, decisive) Prototype the P-only dedicated PI in situ.** Take
   `/tmp/pi/pi_p_only.v`'s approach: a sibling `pi_parallel` module driven from
   the same tick, feeding `velocity_command` and a one-cycle `pi_calculating`
   strobe, with the planner's serial engine no longer containing PI states
   (i.e. start from the planner-only shape that measures 12.9 k). Expected
   ~39.9 k local ≈ **92 %**, profiles intact. This changes the PI contract:
   update `test/test_shared_controller.py` (the oracle drops the integrator; the
   96-offset cancellation sweep becomes trivial because there is no in-place
   rotation), and note in `docs/implementation.md` that anti-windup is gone.
2. **If full PI is non-negotiable**, the only way to the 100 % line is ~4.3 k out
   of the non-PI blocks (~18.3 k total: step_dir 6.8, SPI 6.8, encoder 2.8,
   timebase 1.9). Untried ideas: share the step generator's 25-bit phase
   high-word comparator with its pulse timers; drop the SPI frame staging
   (`rx_shift` 16 FF) by validating length before committing field-by-field
   (spec change: non-atomic commit); merge the timebase divider into the step
   generator's setup timers. Honest expectation: 1–2 k, i.e. ~103 %, still short.
3. **If a further numeric descope is allowed**, shrinking the velocity ceiling
   from 4000 to 2000 steps/s drops one bit from `speed`/`vmax`/`command`/
   position integer math (est. −1…1.5 k) and makes `MAX_STEP_HZ`/phase geometry
   smaller; it changes the delivered spec table.
4. Do **not** retry: parallel planner, parallel full PI, SPI tx de-dup, FSM
   re-encoding, dead-state deletion, `command_limited` removal (all measured
   above).

## 9. Gotchas that cost time this session

* **Concat arity bugs are silent** (§2.1). Add an elaboration-time assert or a
  lint pass if you touch operand lists.
* **Rotating registers**: taps must read bit 0 of the rotating field (§3).
* Cancelling the PI mid-pass must not lose the pending planner advance; the
  test sweeps all 96 offsets. If you re-architect the PI, rewrite that sweep for
  the new (simpler) contract rather than deleting the coverage.
* The measured oracle in `test_shared_controller.py` asserts the **exact**
  position and velocity every tick, and injects a mid-calculation `tick` plus a
  changed `start_position` ("disturb"). That is why the PI must latch the
  measured sample at the acceptance edge — that is the only reason `work`
  exists as a *latch*; do not replace it with the live `position` input without
  reworking the disturbance contract.
* `src/shared_motion_controller.v` and `test/test_shared_controller.py` are
  **CRLF** files; keep the endings when you rewrite them.
* `src/plan_lut.vh` is generated by `scripts/generate_plan_lut.py`; the stopping
  table in it is now only for the standalone reference planners.
* The step generator is parameter-driven (`MAX_STEP_HZ 4000` → `MAX_COMMAND
  8192`, `PHASE_BITS 25` with the split-phase scheme); its `command_limited` is
  provably dead at the default parameters and worth ~40 µm².

## 10. Branch / commit state

* Pushed: `8f7189c` "Port integration PI oracle to Q11 engine units", on top of
  `36a5a5f` "Exact Q11 brake and serial position advance for 1x1 area".
  CI: gds fail (110.03 %), test fail (pre-existing Q16 oracles), docs pass.
* Local only, unpushed, docs-only: `943ce64` (CI record appended to
  `q11-handoff.md`), `4748e60` (90 % algebra correction). Push them with the
  next code change.
* Branch map unchanged from `q11-handoff.md`: only `tmp/q8-area-test` is ours.
