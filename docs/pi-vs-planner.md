# Where the shared engine's area and time go

Question being answered: inside `shared_motion_controller` (38.5k of the 59.4k
top), how much is the PI and how much is the planner state machine, and is
there PI-exclusive logic or capability we do not need?

Short answer: **PI ≈ 25% of the top (~15.1k) and ≈ 25% of the per-tick serial
cycles (~132 of ~530).** Almost none of it is unused feature surface: the only
sizeable optional piece is the integrator (≈3.4k, a required deliverable), and
everything else is the error/command datapath. Descoping PI *features* cannot
close the 7.4k gap; only deleting closed-loop control entirely would (that
variant is 44.1k, i.e. it fits).

All numbers below are GF180 mapped cell area with the stock PDK lib and the
checked-in measurement script, using **only the six sources in
`test/Makefile` `PROJECT_SOURCES`** (baseline 59,244; the handoff's 59,393).
Read the measurement caveat at the end before trusting small deltas.

## Measured ablations (full top)

| Ablation (RTL structurally removed) | Area | Δ vs 59,244 |
| --- | ---: | ---: |
| Baseline `shared_motion_controller` STABLE=1 | 59,244 | — |
| **PI deleted**, planner velocity drives STEP/DIR (open loop) | 44,146 | **−15,098 (−25.5%)** |
| Integrator removed (P-only, Ki pass dead) | 55,824 | −3,420 (−5.8%) |
| Anti-windup freeze removed (integral still clamped) | 58,239 | −1,006 (−1.7%) |
| Integral clamp comparison removed | 58,994 | −250 (−0.4%) |
| PI feedforward term removed from the sum | 58,311 | −933 (−1.6%) |
| `saturated` output not produced | 59,323 | +79 (noise; top already prunes it) |

Standalone engine, same flow, other unused modules excluded (the "all-src"
convention that reproduces the handoff's 38,460): 38,460 baseline, 23,706 with
PI deleted → PI marginal 14,754. FF count drops 202 → ~132, i.e. **73 PI FFs,
84 planner FFs, 45 shared FFs**.

## Serial cycles per accepted tick (measured, cocotb)

Full 0→1000-step move, profile 63, unity gains, 1 kHz ticks:

| | clocks/tick | share |
| --- | ---: | ---: |
| PI (`pi_calculating`) | 112–144 (avg 132) | ~25% |
| Planner (`planner_calculating`) | 126–437 (worst 437) | ~75% |
| Whole tick, worst case | **581** | 5.8% of the 10,000-clock period |

So the engine is area-bound, not time-bound: there are ~9,400 unused clocks per
control tick. Any redesign that trades cycles for area is nearly free.

## What is actually PI-exclusive

Registers: `integral[20:0]`, `work[20:0]` (sampled encoder, then command),
`velocity_command[20:0]`, `saturated`, `pending_saturated`, `clipped`,
`comparison_nonzero`, `integral_phase`, `overflow`, `low_sign`, `pi_negative`,
`error_negative`, `integral_negative`, `pi_cancelled` (73 FFs).
States: `PI_ERROR, PI_READY, PI_SUM, PI_SUM_DONE, PI_CLAMP, PI_COMPARE_START,
PI_COMPARE, PI_FILL_START, PI_FILL, PI_CLAMP_DONE, PI_PUBLISH` (11 of 36;
`IDLE` is shared dispatch, and the declared `P_IDLE=32` value is never used).
Combinational: error/feedforward/integral tap muxes, the gain-shift taps, the
21-bit magnitude comparison used for the integral clamp, the windup test, the
dual cancel path, and the extra `alu_a/alu_b/alu_c` select terms on the shared
adder.

Planner-exclusive registers: `reference_position[28:0]`, `speed[19:0]`,
`stopping[28:0]`, `negative`, `source_position_sign`, `capped`, `finishing`,
`planner_busy`, `planner_done` (84 FFs) plus 24 states. Shared: `state`,
`working`, `bit_index`, `accumulator`, `carry` (45 FFs) and the adder itself.

## PI features that are not needed

- `saturated` and `pi_calculating` outputs: nothing consumes them. `project.v`
  drops them (and `engine_calculating`, `planner_done`) into the `_unused`
  reduction, and the SPI status frame only carries busy/at_target/fault.
  Measured cost after top-level pruning: zero. Deleting them is cleanup, not
  area.
- Two cancel inputs: the top drives `clear_control = launch || target_reached`
  and `control_enable = active`, which are near-complements. One `cancel` input
  would delete `pi_cancelled` and one term in three states (order 100–300 µm²).
- `STABLE_CONFIG=0` latches (`saved_kp/ki`, `latched_goal/profile`, 23 FFs) are
  already compiled out at the top.

Everything else is load-bearing: 30-bit error ring (13-bit position range +
16 fractional bits cannot be narrower), 21-bit command and integral (velocity
limit is 20 bits), 4 gain codes (cheap; old audit measured 44–206 µm² for
fixing one pair), feedforward, command clamp, integral clamp/anti-windup.

## Consequences

1. **No PI descope gets near 7.4k.** Dropping the integrator and all its clamp
   logic is ≈3.5k and removes "PI with anti-windup", an explicit deliverable
   (implementation step 5). Keeping the loop but shrinking the PI datapath is
   the only useful direction.
2. **The PI's area is mostly combinational, not storage.** 73 FFs ≈ 4.6k;
   the other ~10k is write-muxing and enable/select logic on 21–30-bit rings
   (`work` has four write sources, `integral` four, plus the publish mux).
   That is where a real saving lives — not in the flag bits.
3. **Concrete candidate (keeps every feature):** `work` exists only because the
   command must be assembled somewhere while the error sits in the accumulator.
   `stopping[28:0]` is dead during PI, and the shared 30-bit accumulator is
   already PI-owned. Build the command in the accumulator, park the 13-bit
   measured position in `stopping`, and delete `work` (21 FFs + its multi-source
   mux). Costs one extra 31-cycle pass (budget is 5.8% used); expected saving
   ~1.5–2.5k. Needs the same schedule/cancellation discipline as the existing
   serial design.
4. **The planner side remains the bigger pot** (~23.7k including the shared
   adder and 84 FFs), but the mirror-rule experiment already showed that
   serialization eats that class of win. Prefer mux/storage-structure changes
   over algorithm changes.

## Measurement caveat (important for the next agent)

The local Yosys+ABC flow is not monotone in logic size. With the measurement
source set held fixed, an algebraically identical reformulation of the shared
adder moved the top by +310 µm², and *removing* the PI gain muxes ("fix one
gain pair") moved it **+1,750 µm²**. With the unused reference modules
(`motion_planner.v`, `compact_motion_planner.v`, `pi_controller.v`) left in the
source dir, the same baseline top measures 58,256 instead of 59,244, and the
neutral rewrite moves it by +2,138.

Therefore: always measure the same source set (use the six `PROJECT_SOURCES`
files, matching the handoff's 59.4k), treat deltas below ~1k as unmapped noise,
and re-check any surprising result with a second, independently phrased variant.
Note that the ablation table above was produced by structural deletion, and the
PI total (15.1k) and integrator (3.4k) are far outside the noise floor.
