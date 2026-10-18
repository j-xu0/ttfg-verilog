# Implementation steps

The two checked-in proposals share the command/feedback architecture but differ
on numerical details. Stage 1 implements their common foundation and records
choices needed to make that foundation testable.

| Step | Deliverable | Verification | Status |
| --- | --- | --- | --- |
| 1 | Mode-0 SPI and TARGET/CONTROL/PLAN register bank | Framing, atomic writes, masking, status, busy rejection, reset | Implemented |
| 2 | Synchronized x4 quadrature decoder | Both directions, startup phases, illegal transitions, signed wrap, reset | Implemented |
| 3 | GF180 top-level interface | 10 MHz/500 kHz integration, pin directions, fault/status/reset, inactive motion pins | Implemented in RTL |
| 4 | Timebase and trapezoidal planner | Reference model for acceleration/cruise/deceleration, reverse/short moves, range limits | Standalone modules; top-level integration pending |
| 5 | Fixed-point PI and anti-windup | Signed arithmetic, all gains, saturation, integrator limits | Pending |
| 6 | STEP/DIR/ENABLE generator | Pulse width/spacing, DIR setup/hold, disable and fault interlocks | Pending |
| 7 | Closed-loop integration | Motor/encoder model, forward/reverse targets, disturbances, settling | Pending |
| 8 | GF180 submission checks | Full-design synthesis, tile fit, STA, gate-level tests, precheck and physical verification | Pending |

Two subagents independently implemented steps 1 and 2 and their unit tests, then
reviewed each other's modules. The main agent integrated the design and added
port-level and RTL wiring tests. Review identified a reset-with-CS-low SPI bug;
the implementation and regression now ensure only a fresh post-reset transaction
can write.

## Decisions before motion implementation

The second PR adds standalone building blocks described in
[`motion-planning.md`](motion-planning.md). Their configurable numerical contract
allows testing without assigning undocumented meanings to the host's PLAN codes.
They are not instantiated in `project.v`; the submitted chip still implements
the stage-1 interface and keeps motion outputs inactive. Synthesis of that top
therefore cannot establish the planner's area or timing.

- **Control period:** the shorter proposal says 10 MHz / 8192 (1220.703125 Hz);
  the detailed proposal says 1 kHz. The standalone timebase defaults to 1 kHz
  at 10 MHz and is parameterized. Final system selection remains pending.
- **Acceleration:** the shorter proposal gives 100-10,000 steps/s²; the detailed
  proposal gives 500-100,000 steps/s². Neither defines the six intermediate
  acceleration values. Velocity has endpoints 100-10,000 steps/s but likewise
  lacks the intermediate lookup entries.
- **Position units/range:** define the ratio of encoder x4 ticks to motor steps,
  the homing/reference procedure, and how moves across signed wrap are handled.
  The implemented feedback counter wraps; the planner must not accidentally
  interpret wrap as a large physical position error.
- **Output architecture:** both proposals mention 8-bit PWM near 39 kHz while
  the detailed pinout specifies STEP/DIR into an external driver. Define how PI
  output commands step rate, and whether PWM is internal or a separate output.
- **Numerical scaling:** choose planner fractional bits, PI error/integrator
  widths, saturation, anti-windup, and behavior when TARGET changes while moving.
- **Driver timing:** choose the external IC and derive STEP minimum pulse width,
  DIR setup/hold, ENABLE polarity and setup, and fault stop behavior from it.
- **Host diagnostics:** define persistent dropped-write reporting, interrupt
  behavior, fault recovery, settled tolerance/duration, and readback needs.

Stage 1 choices are documented in `info.md`: MISO uses dedicated output 6;
output 7 is unused; status is a 16-bit frame beginning `0,busy,at_target,fault`;
CS deassertion commits writes; reset clears all command registers. These are
explicit implementation choices where the proposal is silent, not confirmed
requirements of the original proposal.

The existing `ttgf26a` flow is retained for repository consistency. Its official
[shuttle configuration](https://github.com/TinyTapeout/tinytapeout-gf-26a/blob/main/config.yaml)
lists a June 22, 2026 end date; selecting an open shuttle
and its matching template/flow is a separate requirement before submission.
