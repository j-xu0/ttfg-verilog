# Shared planner/PI controller

## Before and after

The previous integrated design used independently scheduled planner and PI
engines. Its GF180 mapped area was 61,318.5216 um² with 392 flip-flops.

`shared_motion_controller.v` now executes PI and the planner with one FSM,
30-bit arithmetic/error register, bit counter, carry state and serial adder.
Initial position subtraction also uses that adder. The integrated mapped area
is **58,174.9952 um² with 353 flip-flops**, a **5.1% area reduction**. Selection
and scheduling logic offset part of the storage saving; standalone block sums
are not a substitute for this full-top measurement.

**The design still does not fit 1x1.** Raw cells occupy 111.9% of the
51,966.970 um² core before physical headroom. No profile, gain, position range,
Q16 precision, driver timing or pin function was removed. STEP generation,
encoder counting and SPI remain independent of the scheduled engine.

## Schedule and register lifetimes

1. An accepted control tick captures integer encoder feedback in the low 13
   bits of the work register. The previous completed reference and speed still
   describe the input sample; no planner advancement has occurred.
2. A 30-clock subtraction rotates the reference in place and writes exact Q16
   position error into the shared accumulator. The 29-bit reference ring is
   restored before this phase completes.
3. PI consumes that error, the old integral and the old signed speed. The speed
   ring supplies feedforward directly and restores after 20 rotations. The
   work register becomes the next command, preserving the previously published
   command until command/saturation publish together.
4. The planner advances the reference using the same counter, carry, adder and
   accumulator. Persistent stopping distance, speed, integral and output state
   remain separate because their lifetimes overlap.
5. Completion restores valid reference position/velocity. A PI-only correction
   after trajectory completion restores zero feedforward before returning idle.

| Storage | Roles |
| --- | --- |
| 30-bit accumulator | Idle reference velocity, PI error ring, planner arithmetic |
| 21-bit work | Sampled encoder count, then command calculation/clamp result |
| 29-bit reference | Completed reference, serial subtraction ring, planner distance workspace |
| 20-bit speed | Persistent planner speed and rotating PI feedforward source |
| Counter/carry/adder | Error subtraction, PI sums/comparison, planner operations |

Ticks received while calculating are ignored, not queued. The interface allows
2,000 clocks between accepted ticks; the unchanged 10 MHz/1 kHz integration
provides 10,000 clocks. Gains/profile/target stay stable through work because
SPI rejects busy writes. The standalone captured-input mode latches target and
profile at start and gains at the accepted tick. `start_position` carries live
encoder feedback at each tick as well as the measured origin at launch.

## Implementation warnings

- Reference outputs are workspace while `calculating=1`, including during PI
  subtraction. Consumers must wait for the combined engine to become idle.
  Published command/saturation remain stable until PI publishes or is cleared.
- `planner_busy`, `planner_calculating`, `pi_calculating` and combined
  `calculating` have different meanings. Planner busy spans an entire move;
  planner calculation marks only planner operations. This permits final-target
  feedback to cancel a PI-only correction promptly.
- PI clear or low controller-enable clears integral, command and saturation on
  the next clock. An in-progress PI error/sum phase may take up to 31 clocks to
  restore persistent rings before the planner can proceed. Cancellation is
  remembered even if clear lasts one clock; cancelled results cannot publish.
- Global disable aborts both calculations on the next clock. Reset must span a
  rising edge. The top-level raw enable/reset/fault gates still inhibit motion
  pins immediately, independently of engine cleanup. A new TARGET is required
  after an aborted move; its launch reinitializes discarded planner workspace.
- PI signed sums need a two-bit carry and sign extension beyond the 21-bit
  output. Planner operations initialize their carry independently. Preserve
  negative gain rounding and anti-windup when changing either path.
- Numerical samples are preserved, but sub-period command/trajectory timing
  differs from the old concurrent engines. A matching command stream still has
  the same STEP-generator behavior; the complete controllers are not claimed
  cycle-identical. Physical timing and 1x1 fit require the actual GF180 flow.

## Verification

The retained standalone suites plus two shared-controller configurations give
26 module cases. Shared tests use independent trajectory and PI arithmetic
models across all 64 profiles, both directions, full position endpoints, all 16
gain pairs, feedback disturbances and ignored ticks. A cancellation sweep tests
one-clock clear/controller-disable pulses at every offset 0 through 95 during
PI work, with both an active planner and a completed trajectory, in both
configuration modes: 768 scenarios. It checks immediate command clearing,
eventual reference/speed restoration, no stale publication and a correct fresh
sample after cancellation.

Eight full-top motor/port tests retain missed-step correction, disturbances,
faults, signed-range checks, rate limits and sampling. The additional phase-abort
test asserts raw disable/reset between clock edges during PI and planner work,
checks immediate pin inhibition, and verifies no automatic restart.

Full-design synthesis checks and controlled GF180 mapping are separate from
placement, routing, STA, gate-level simulation and physical signoff. The 1x1
allocation and its hardening check remain enabled; the current area shortfall
must not be bypassed.
