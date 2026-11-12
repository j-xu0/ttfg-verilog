# Q11 handoff: where we are, what to try next

Date: 2026-11-12. Branch: `tmp/q8-area-test`, commit `3227876`
"Q11 numeric-spec retarget for 1x1 area" — pushed to `origin/tmp/q8-area-test`.
Spec: `docs/numeric-spec.md` (committed in the same commit).

Goal: 1x1 tile, core 51,966.970 µm². Fit needs raw cells comfortably under
100% (routing/clock-tree need headroom); working target is **90% = 46,770**.

## Where we stand (CI, commit 3227876)

All three workflows ran (~8 min after push):

* **GDS: FAILED, does not fit.** `GPL-0301 Utilization 120.871% exceeds`
  (run 43274622817). Floorplan effective utilization was already 1.018.
  Numbers: movable/standard-cell area 60,966.251, fixed 1,527.859,
  2,531 instances (2,183 movable), 2,202 nets, pin-density adjust 8,053.150.
  For comparison the old base (`b6c5a90`) was 165.87%, so Q11 bought
  roughly a third of the overflow. Gap to 90%: **−14,196 µm² (−23%)**.
* **Tests: FAILED (expected fallout, partly real bug).** Run 43274622951.
  `test_compact_planner` + standalone planner oracles still expect Q16
  tables (they `#include "plan_lut.vh"`, now Q11) and `test_step_dir`
  `pulse_contract` still drives Q16-scale velocities at 10 kHz max
  (RTL is Q11 / 4 kHz now). Plus one **real** shared-engine mismatch,
  see below.
* **Docs: passed.**

## What Q11 changed (locked in, 12 files in 3227876)

* `scripts/generate_plan_lut.py` + `src/plan_lut.vh` (regenerated):
  `VEL=(205,410,1024,2048,4096,5120,6144,8191)` (~100…3999 steps/s, 4000
  capped to 8191), `ACC=(1,2,4,10,20,41,102,205)` (~500…100k steps/s²,
  all non-zero — this is what the broken Q8 experiment got wrong with
  `0,0,…`). Widths: velocity `[12:0]`, accel `[10:0]`, stopping `[13:0]`
  integer steps, sentinel `0x3FFF`. Stopping = `ceil(sum/2048)`, capped at
  8191 else sentinel.
* `src/shared_motion_controller.v` (rewrite, CRLF preserved):
  `reference_position[21:0]` = Q13.8 `{pos_int[13:0], pos_frac[7:0]}`,
  `reference_velocity` / `velocity_command` `[13:0]` signed Q11,
  `speed[12:0]` magnitude + `negative`, `stopping[13:0]` integer,
  `accumulator[25:0]`, `work`/`integral` `[13:0]`, `vmax[12:0]`,
  `accel[10:0]`, `max_stop[13:0]`. `pos += vel>>3` is a **parallel**
  Q13.8 add (fixed 3-bit shift; low 3 velocity bits truncated).
  `DIST`/`POSITION` are integer-only (`pos_int` doubles as `remaining`
  scratch, `pos_frac` held — **not** zeroed; an earlier revision zeroed it
  and dropped sub-step accumulation). `TAIL_*` states bypassed straight to
  `BEGIN_ACCEL` (integer brake has no tail phase; the ops still execute,
  wasting 26 cycles each — harmless, budget is ~600/10000). `CAP` is
  **combinational** (`accumulator[13:0] >= vmax`, a serial-CAP tap bug was
  dodged this way; saves 26 cycles). `STOP`/`ACCEPT_STOP` add `ZERO`
  because `accel>>11 == 0` for every profile accel (0…2047); stopping
  recurrence uses `SPEED_INT = speed>>11` (**floor**, 0…3, wiring only).
  `POSITION_SIGNED`/`DISTANCE` tap order: index 4 = unsigned DISTANCE,
  index 3 = signed POSITION_SIGNED (this was once swapped — do not
  re-swap; see "gotchas").
* `src/step_dir_generator.v`: Q11 `velocity_command[13:0]`,
  `RAW = 2048*CLOCK_HZ`, `MAX_STEP_HZ 4000` (was 10000),
  `MAX_COMMAND = 8192`, phase 25b / frac 15b / `HIGH_LIMIT 625`,
  `MAX_MAGNITUDE 8192`.
* `src/project.v`: `reference_position[21:0]`, velocities/commands
  `[13:0]`, `target_reached` zero constant widened.
* `test/test_shared_controller.py`: oracle ported (Q13.8 pos with `//256`
  ints and `//8` advance, integer `remaining`, integer finish
  `pos_int==goal_int` accepting <1-step frac error, truncated stopping
  `sum(s>>11)` to match RTL floor). 20000-tick timeout retained.
* Incidental (were already in the tree, committed together):
  `test/test.py` `spi_init_short_move`, `test/test_step_dir.py`
  `rate_demo`, `test/README.md`, `README.md`, `.gitignore`
  (`test/waves/`).

## Open correctness issue (fix before claiming area)

**High-speed decel never fires.** Repro: `test_shared_controller`,
code 63, `old (1032739,8191,1)` → RTL gives `(1033762,8191,1)`,
oracle expects `(1033762,7986,1)` (decel by one accel step).
Mechanism: uncapped trial stopping is floor-truncated (`speed>>11` = 0
for all speeds < 2048, and 3 for 8191), while the capped path uses the
LUT ceil (e.g. 3 for code 16, 82 for code 63). Near the target the
uncapped trial says "fits" (0 ≤ remaining) where the exact answer says
it doesn't, so the engine stays at max, overshoots (observed
`pos 4713 > target 4095` in a long-run probe), and runs away positive.
Low speeds are accidentally safe (pos_int advances ≤1/tick, can't skip
the integer goal); high speeds (`vel≥2048`, pos jumps 2–4 steps/tick)
can skip it. Debug recipe that worked: minimal icarus bench driving
`shared_motion_controller #(.STABLE_CONFIG(1))`, launch, then
`tick`/`wait(calc==0)` loops with `$display` on
`state/working/bit_index/accumulator/carry/speed/stopping`
(see `TEST_NEW` at state 45 vs `TEST_OLD` at 46 to watch the accept path).

## Real options (nothing here is run yet — pick one)

Correctness first (not savings, blocks everything):

1. **Decel guard.** Force decel when `remaining < max_stop_LUT`
   (exact ceil upper bound, already in the LUT). ~+0.2k, kills the
   runaway, keeps floor's low-speed liveness. Do this regardless.

Area, feature-preserving (combined ceiling ≈ −5k → ~108%, i.e. still
overfull — see bottom line):

2. **Delete `work` (~−1.5–2.5k).** Build the PI command in `accumulator`,
   park the 13-bit measured sample in `stopping` (dead during PI). Costs
   one extra ~26-cycle pass (budget free). Touches PI schedule +
   cancel-restore + the 768-case cancellation sweep. Biggest remaining
   item per `docs/pi-vs-planner.md`.
3. **SPI shift sharing (~−1–1.5k).** `src/stepper_spi.v` holds 16b RX +
   16b TX for 3 meaningful status bits. Share the ring or serialize TX
   from a counter+mux. Isolated, low risk; `test/test_spi.py` covers it.
4. **State/input folding (~−0.5–1k).** Merge `clear_control` /
   `control_enable` into one cancel; delete dead `TAIL_*` states;
   fold `PI_COMPARE_START`/`PI_FILL_START`. `shared_motion_controller.v`
   only.

Descopes (measured, with receipts in `docs/pi-vs-planner.md` /
`docs/area-audit.md`):

5. **P-only (drop integrator+clamp/anti-windup): −3.4k (−5.8%).**
   Removes an explicit deliverable; lands ~102% — still over.
6. **Fix/drop profiles: ~−3.5k** (profile 0/63 diagnostic points).
   Loses the 64-profile deliverable; ~95% raw, no routing headroom.
7. **Open-loop (delete PI): −15.1k (−25.5%), ~44k (~80%).** The only
   measured variant that fits. Kills closed-loop — i.e. the project.

Rejected (do not retry): mirror rule (+6–10% now — LUT shrank under Q11
while mirror state didn't; see previous analysis), uniform Q8 (breaks low
accel), one gain pair (44–206 µm², noise), timebase games (±55 µm²),
placement density (can't fix >100% overflow).

Bottom line math: 61.0k − (2.5+1.5+1.0) ≈ 56k ≈ **108%**. No
feature-preserving combination in evidence reaches 90%. Realistic
endings: (a) P-only and still miss (~102%), (b) open-loop and fit
(~80%), or (c) keep closed-loop and admit 1x1 needs deeper surgery than
serial-mux shuffling.

## How to measure (don't trust anything else)

* Local synth (no sudo): `YS=$HOME/tools/oss/oss-cad-suite/bin/yosys`,
  `ABC=$HOME/tools/oss/oss-cad-suite/bin/yosys-abc`,
  PDK `gf180mcu` via volare (see `docs/handoff.md` for the exact lib
  path — stock lib reads ~+2% vs the friend's filtered lib, so only
  compare deltas within one flow).
  `python3 scripts/measure_gf180_area.py --lib <LIB> --yosys $YS
  --abc $ABC --output /tmp/area-X` (add `--top MOD --source-dir DIR`
  for module splits).
* Always measure the 6 `PROJECT_SOURCES` files (`test/Makefile`):
  `project stepper_spi quadrature_decoder control_timebase
  shared_motion_controller step_dir_generator`. Deltas <1k are mapping
  noise (documented identical-rewrite ±2k swings) — re-check surprises
  with a second phrasing.
* Fit is decided by CI GDS (`gds` workflow, `GPL-0301` + utilization),
  not local synth. Push to `tmp/q8-area-test` and read the floorplan
  lines: `Core BBox`/`Core area: 51966.970`, `Movable …`,
  `Utilization: … %`, `Effective utilization: …`.
* Tests: `python3 -m pytest test/test_spi.py test/test_encoder.py
  test/test_timebase.py test/test_planner.py test/test_compact_planner.py
  test/test_pi.py test/test_step_dir.py test/test_shared_controller.py
  -q` then `make -C test -B` (8 integration tests, ~5 min).
  Note: compact/motion-planner + pi_controller are *reference only*
  (excluded from `PROJECT_SOURCES`) but share `plan_lut.vh` — their
  oracles still expect Q16 and currently fail; either port them to Q11
  or decouple their LUT.

## Branch map (don't push anywhere else without asking)

* `origin/jason/stepper-shared-engine` — friend's optimized base. Read-only.
* `nick/serial-motion-planning` — shared-engine base + mirror-rule
  standalone rewrite + oracle/docs. Chip top untouched.
* `nick/mirror-shared-engine` — parked mirror-in-shared-ALU, verified,
  **+7.2%**. Do not build on without a new state-cost idea.
* `tmp/q8-area-test` (ours) — Q8 probe (do-not-merge) + this Q11
  retarget (3227876, pushed). CI workflows: `gds`, `test`, `docs`.
