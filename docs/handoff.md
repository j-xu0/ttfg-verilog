# Handoff: 1x1 area investigation (TinyTapeout GF180 stepper controller)

Date: 2026-11-05. Goal: fit the closed-loop stepper servo in a **1x1 tile
(core 51,966.97 µm²)**. Current best (shared engine) is ~59.4k. Gap ≈ 7.4k
(−12.5%).

## Branch map

- `origin/jason/stepper-shared-engine` — friend's optimized base. Everything
  below builds on it. Do not push here without asking.
- `nick/serial-motion-planning` (this branch) — shared-engine base **plus**:
  standalone mirror-rule `src/motion_planner.v` rewrite + updated
  `test/test_planner.py` oracle + docs. Chip top untouched.
- `nick/mirror-shared-engine` — experiment: mirror rule ported *into* the
  shared engine. Fully verified, but measures **+7.2%**. Parked, do not build
  on without reading "Dead end" below.

## How to measure (no sudo on this box)

Local tools (no sudo needed, already installed):
- `~/tools/oss-cad-suite/bin/yosys` (0.70) + `yosys-abc`. **Use this.**
  The apt 0.9 yosys in `~/tools/yosys-apt` lacks `stat -json` and cannot run
  the area script.
- GF180 PDK via volare: family name is `gf180mcu` (not `gf180mcuD`).
  Liberty used throughout:
  `/home/north/.volare/volare/gf180mcu/versions/c6d73a35f524070e85faff4a6a9eef49553ebc2b/gf180mcuD/libs.ref/gf180mcu_fd_sc_mcu7t5v0/lib/gf180mcu_fd_sc_mcu7t5v0__tt_025C_3v30.lib`
  (stock PDK, hash differs from friend's filtered lib — see Caveats).

```sh
YS=$HOME/tools/oss/oss-cad-suite/bin/yosys; ABC=$HOME/tools/oss/oss-cad-suite/bin/yosys-abc
python3 scripts/measure_gf180_area.py --lib <LIB> --yosys $YS --abc $ABC \
  --output /tmp/area-X [--top MOD --source-dir DIR --parameter NAME=INT ...]
```

Tests: `python3 -m pytest test/test_spi.py test/test_encoder.py test/test_timebase.py test/test_planner.py test/test_compact_planner.py test/test_pi.py test/test_step_dir.py test/test_shared_controller.py -q` then `make -C test -B` (8 integration tests, ~5 min). Shared-engine tests live in `test/test_shared_controller.py`.

## Measured numbers (GF180-mapped, same lib+flow throughout)

Full top (shared engine): **59,393 µm², 2,398 cells, 353 FFs**.

| Module | Area | Share | Note |
|---|---|---|---|
| `shared_motion_controller` (STABLE=1) | 38,460 | 64.8% | the whole game |
| `step_dir_generator` | 8,267 | 13.9% | 40-bit phase acc |
| `stepper_spi` | 6,752 | 11.4% | state-dominated |
| `quadrature_decoder` | 2,794 | 4.7% | |
| `control_timebase` | 1,870 | 3.1% | |
| top glue + opt effects | ~1,250 | 2.1% | residual |
| *`pi_controller` parallel (ref, not in chip)* | *28,305* | — | *serial PI replaces it* |
| *`motion_planner` mirror standalone (ref)* | *55,196* | — | *vs 84,640 stopping serial* |
| *`compact_motion_planner` (ref)* | *24,108* | — | |

By function (whole top): state 353 FFs / 24,018 (40.4%), plain comb
1,716 cells / 28,768 (48.4%), muxes 184 / 5,334 (9.0%), clock bufs 2.1%.
Small blocks are state-dominated (SPI 68% FFs); big blocks comb-dominated.
ABC maps adders to gates (0 dedicated adder cells).

## What we learned

1. **Mirror rule** (speed up half the distance, mirror it slowing down; per
   tick only `2*(traveled+faster) <= total` + guards). Standalone parallel:
   55,196 vs 84,640 µm² (**−34.8%**) — deletes the divide/multiply engine.
2. **Serial mirror in the shared engine: +7.2%** (63,641 vs 59,393 same-flow;
   standalone engines 46,993 vs 44,657). Parked on
   `nick/mirror-shared-engine`. Reason: the stopping design was already
   co-optimized for serial (recompute-from-reference, LUT at build time, one
   shared 1-bit ALU). Mirror's win (no dividers) had nothing left to delete;
   its travelled/total/turn state (+32 FFs) and extra serial states cost more
   than the LUT removal saved. Rule of thumb: parallel→shrink arithmetic;
   serial→shrink state.
3. **LUT mechanics** (`plan_lut.vh`): 64 entries = S(vmax) for the 64 host
   profiles only — not arbitrary speeds. Below vmax a running sum is kept
   exactly (`S(v+a) = S(v) + (v+a)`); capped case loads the table. Sentinel
   2²⁹−1 exceeds any legal move (max 536,854,528), so over-range caps fail
   safe. Runtime uses zero division (only build-time Python divides).
4. **Serial-op discipline** (bit us twice): every op is setup + 30 working
   cycles; the carry seed **must equal the op's subtract flag** (seed 0 for
   adds, 1 for subs — a wrong seed silently shifts results by 1); test ops
   destroy the accumulator, so values needed later must be recomputed or
   parked; test oracle must use identical post-move semantics
   (move first, then decide) or boundaries diverge by a tick.
5. **Gotchas in this repo**: `shared_motion_controller.v` has CRLF endings
   (edit via python with `\r\n`); `test_shared_controller.py` asserts
   *exact* oracle match (no accel-limit invariant — mirror's rare
   1–2-unit landing overshoot is inherited, not a regression); integration
   tests check behavior (settle/correction), not traces.

## Caveats on the numbers

- Stock PDK typical-corner lib, not friend's filtered lib or signoff
  corners; expect a few % skew (our flow reproduces their 58,175 top as
  59,393, i.e. +2%). Deltas within one flow are the reliable signal.
- Standalone module areas don't sum exactly to top (cross-boundary opt).
- No routing/congestion/clock-tree in these numbers; hardening decides fit.

## Suggested next threads (engine needs ~20%: 38.5k → ~30.7k)

- Profile the engine's state vs comb: 202 FFs, which rings are widest and
  least-shared? PI work/error rings and 29–30-bit planner paths are suspects.
- Narrow PI datapath to match actual precision needs (32-bit sums sized for
  Q16; error/accumulator widths).
- `step_dir` phase accumulator (40-bit vs needed rate resolution).
- Revisit `docs/area-audit.md` rejected experiments with fresh eyes.
- Do NOT re-try mirror-in-shared-ALU without a new idea for the state cost;
  the measurement is conclusive for the current formulation.
