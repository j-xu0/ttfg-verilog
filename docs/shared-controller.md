# Shared planner/PI controller

> **Update 2026-11-17:** this document records the shared-engine milestone. The
> later P-only fixed-Kp restriction set hardens on 1x1; see [the area budget](area.md).
>
> **Shipped decomposition.** `shared_motion_controller.v` is now a thin wrapper
> composing `motion_engine.v` (bit-serial planner, owns the whole serial ALU) and
> `pi_p_controller.v` (single-cycle fixed-gain P). The schedule and register
> lifetimes below describe the *PI-era* engine where both calculations shared the
> accumulator; that is the 58,175 um² milestone, not the current datapath. What
> still holds for the shipped design: reference/velocity are workspace while
> `calculating=1`, ticks during work are ignored, and the cancellation/abort
> contract is unchanged.

## Before and after

The previous integrated design used independently scheduled planner and PI
engines (61,318.5 um², 392 flip-flops). `shared_motion_controller.v` executes
both with one FSM, 30-bit arithmetic/error register, bit counter, carry state
and serial adder; initial position subtraction uses that adder too. Result:
**58,175.0 um², 353 flip-flops, −5.1 %**. Selection and scheduling logic offset
part of the storage saving; standalone block sums are not a substitute for a
full-top measurement. Full ledger: [area budget](area.md).

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

## Alternatives considered

A mirror-rule planner was prototyped on this shared ALU (branch
`nick/mirror-shared-engine`), reusing the PI schedule, handshake and abort
semantics unchanged. It is functionally verified and deletes the 64-entry
stopping LUT, but costs **+7.2 %** (63,641 vs 59,393 um² same flow): the
travelled/total/turn state and extra serial states outweigh the LUT. It keeps
its ~−34 % advantage as a standalone parallel planner. Do not retry; details in
[the area budget](area.md#do-not-retry).

## Engineering gotchas

These cost real time; they are properties of this code, not history.

* **`src/shared_motion_controller.v`, `src/motion_engine.v`,
  `src/pi_p_controller.v` and `test/test_shared_controller.py` are CRLF files.**
  Keep the line endings when rewriting them.
* **Concat arity bugs are silent.** A mismatched operand list in a sum is
  legal Verilog and shifts a bit position. Add an elaboration-time assert or
  run a lint pass if you touch operand lists.
* **Rotating-register taps must read bit 0** of the rotating field.
* **`work` is a latch in the reference `pi_controller.v`, and must stay one.**
  The oracle injects a mid-calculation `tick` and a changed `start_position`, so
  the measured sample has to be latched at the acceptance edge. Replacing it
  with the live `position` input breaks the disturbance contract.
* **`src/plan_lut.vh` is generated** by `scripts/generate_plan_lut.py`; its
  stopping table serves only the standalone reference planners.
* The step generator is parameter-driven (`MAX_STEP_HZ 4000` → `MAX_COMMAND
  8192`, `PHASE_BITS 25` with the split-phase scheme). Its `command_limited`
  output is provably dead at the default parameters, worth ~40 um².
* Cancelling PI mid-pass must not lose the pending planner advance; the test
  sweeps all 96 offsets. Re-architecting PI means rewriting that sweep for the
  new contract, not deleting the coverage.

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
allocation and its hardening check remain enabled and pass; do not bypass them.
