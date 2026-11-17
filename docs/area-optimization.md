# Mandatory 1x1 area optimization

The allocation is restored to **1x1**. No host settings or control features are
removed, but **the optimized design still does not fit**. These changes are an
area-reduction milestone, not a completed one-tile implementation.

## Controlled GF180 mapping

The baseline commit 215ec81 and optimized sources were mapped with the same
Yosys 0.66/ABC commands and the same filtered GF180 Liberty file recovered from
the failing hardening run. This local flow differs slightly from CI's synthesis
strategy, so compare its baseline and optimized result with each other.

| Metric | Baseline | Optimized |
| --- | ---: | ---: |
| Mapped cell area (um²) | 140,824.2752 | 58,174.9952 |
| Sequential cell area (um²) | 39,823.1232 | 24,017.6832 |
| Mapped cells | 6,288 | 2,372 |
| Flip-flops | 586 | 353 |

Cell area falls **58.7%**. The failed 1x1 run's core area was 51,966.970 um²;
optimized cells alone still occupy 111.9% of that, without fixed-cell, pin-density,
routing or clock-tree headroom. Raising placement density cannot resolve this
capacity mismatch. The earlier passing 3x2 result does not satisfy the requirement.

The small [measurement record](area-measurement.json) includes source and Liberty
hashes. `scripts/measure_gf180_area.py` reproduces the synthesis procedure and
writes detailed logs/netlists outside the source checkout. It accepts `--top`
and `--parameter NAME=value` for module comparisons.

The follow-up audit reduced 71,420.8320 to 61,318.5216 um² (**14.1%**).
The new shared engine reduces that by another **5.1%**, preserving features and
Q16 arithmetic. See the [shared-controller design](shared-controller.md) and the
[module audit and tradeoffs](area-audit.md) for the measured changes and rejected
experiments. CI on the previous `b6c5a90` source confirmed 165.870% placement
utilization (`GPL-0301`); these new local measurements are not a physical fit.

## Implemented changes

1. **64-profile stopping lookup.** The integrated planner no longer calculates
   cap stopping distances through a general divider and serial multipliers.
   `scripts/generate_plan_lut.py` generates exact finite sums from the existing
   eight velocities and eight accelerations. Out-of-range sums use a sentinel
   that is provably larger than every legal remaining move and is never accepted.
2. **Shared serial planner arithmetic.** One bit adder and one 30-bit accumulator
   serve position advancement, braking comparisons and velocity updates. The
   position output doubles as temporary distance storage. Reachability bounds
   reduce stopping storage to 29 bits. The SPI busy interlock permits live stable
   target/profile inputs instead of redundant latches.
3. **Serial PI calculation.** Error/feedforward are sampled once per update;
   gain shifts, signed sums, clamping and conditional anti-windup are streamed
   through small arithmetic logic. Work registers rotate in place. The same Q16
   precision, all 16 gain pairs, rounding and saturation rules remain supported.
4. **Exact NCO normalization.** A compile-time greatest-common-divisor reduction
   cancels 1000 at the default clock rates. The phase accumulator shrinks 40→30 bits
   and loses its constant multiplier. A fractional/coarse phase split further
   reduces modulo logic; unsupported parameter geometries use the general exact
   fallback. Identical input command streams produce identical pulse events.
5. **Frontend bounds.** Signed wrap detection stores only the previous sign plus
   the new endpoint because encoder updates are at most one count. Configuration
   lookup definitions are shared with the generated planner constants.

6. **Shared PI/planner engine.** A single FSM, 30-bit accumulator/error ring,
   counter, carry and adder execute both calculations. PI error is calculated
   serially; its work register first captures feedback and later holds the next
   command. Persistent rings restore before a cancelled PI yields to planning.

The combined calculation completes within 2,000 clocks; the control period is
10,000 clocks at the unchanged 10 MHz target. Reference outputs are invalid while
calculating, including serial error subtraction. PI samples the prior completed
reference, publishes its command atomically, then the planner advances. Numeric
sample ordering is preserved; sub-period timing changes mean the full controller
is not claimed cycle-identical to the independent engines.

## Verification and remaining work

Module tests cover all 64 profiles, both directions and signed endpoints, exact
finite-sum reference models, generated ROM reproducibility, reachable-range
sentinels, all PI gains and randomized arithmetic, input stability contracts,
ignored in-flight ticks and aborts. STEP tests compare each cycle with an
independent original-unit phase model across default, coprime, one-cycle timing,
split-phase and boundary configurations. External motor-model tests retain
missed-step correction, reverse commands, shaft disturbances, fault/disable/reset,
range faults, settling, the 10 kHz ceiling and lowest motion settings.

Shared storage/arithmetic is now implemented, but it does not close the 1x1 gap.
The remaining persistent registers have overlapping lifetimes, limiting further
direct sharing. A diagnostic one-constant replacement for the stopping
ROM saved only about 2,737 um² in the earlier planner, so removing this table alone
cannot close the gap. No larger tile, disabled check, reduced clock or feature
removal is used to make this milestone appear to fit. Full 1x1 hardening must
ultimately pass before the implementation is complete.
