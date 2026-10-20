# Implementation steps

The two checked-in proposals share the command/feedback architecture but differ
on numerical details. Stage 1 implements their common foundation and records
choices needed to make that foundation testable.

| Step | Deliverable | Verification | Status |
| --- | --- | --- | --- |
| 1 | Mode-0 SPI and TARGET/CONTROL/PLAN register bank | Framing, atomic writes, masking, status, busy rejection, reset | Implemented |
| 2 | Synchronized x4 quadrature decoder | Both directions, startup phases, illegal transitions, signed wrap, reset | Implemented |
| 3 | GF180 top-level interface | 10 MHz/500 kHz integration, pin directions, fault/status/reset, motion pins | Implemented in RTL |
| 4 | Timebase and trapezoidal planner | Reference model for acceleration/cruise/deceleration, reverse/short moves, range limits | Integrated bounded serial planner with profile LUT |
| 5 | Fixed-point PI and anti-windup | Signed arithmetic, all gains, saturation, integrator limits | Implemented in RTL |
| 6 | STEP/DIR/ENABLE generator | Pulse width/spacing, DIR setup/hold, disable and fault interlocks | Implemented in RTL |
| 7 | Closed-loop integration | Motor/encoder model, forward/reverse targets, disturbances, settling | Implemented in RTL |
| 8 | GF180 submission checks | Full-design synthesis, tile fit, STA, gate-level tests, precheck and physical verification | 56.5% mapped area reduction; 1x1 still over budget |

Two subagents independently implemented steps 1 and 2 and their unit tests, then
reviewed each other's modules. The main agent integrated the design and added
port-level and RTL wiring tests. Review identified a reset-with-CS-low SPI bug;
the implementation and regression now ensure only a fresh post-reset transaction
can write.

## Implemented integration choices

The third implementation layer connects all functional blocks. The detailed
proposal's 1 kHz loop and acceleration endpoints are used. Exact lookup tables,
Q16 scaling, PI ordering, STEP timing, and command/fault/settling semantics are
specified in [`motion-planning.md`](motion-planning.md) and `info.md`.

Three subagents implemented PI, pulse timing, and planner precision/serial
arithmetic, with independent cross-review. Integration review found a new-TARGET
race after a settled move; explicit commit priority and a regression fix it.
The main agent added external motor/encoder-model tests for moves, missed steps,
disturbances, aborts, and absolute-range faults.

These are implementation choices where the proposals differ or omit details.
Remaining hardware decisions are the actual driver IC/electrical timing,
encoder-to-step conversion if not 1:1, homing, physical gain tuning, and whether
idle holding torque is required. PWM is replaced by signed step-rate control
for the detailed proposal's external STEP/DIR architecture. Persistent rejection
readback, an interrupt, and hardware homing are not implemented; output 7 remains
zero. Faults clear only with reset. Busy writes are rejected internally and must
be retried by the host; status readback is always available.

Generic full-design synthesis passes with no structural check failures. The
integrated serial planner uses bounded shift/add setup and stopping recurrences;
no variable divider or variable-by-variable multiplier remains. GF180 tile fit,
10 MHz timing, gate-level tests, and physical signoff remain to be established.

The existing `ttgf26a` flow is retained for repository consistency. Its official
[shuttle configuration](https://github.com/TinyTapeout/tinytapeout-gf-26a/blob/main/config.yaml)
lists a June 22, 2026 end date; selecting an open shuttle
and its matching template/flow is a separate requirement before submission.

## Area-optimized layer

The mandatory 1x1 constraint is restored. `compact_motion_planner.v` replaces
the integrated general-purpose planner with a generated 64-profile stopping LUT,
reachable-range bounds, and bit-serial arithmetic. PI gains, saturation and
anti-windup are calculated serially. Exact phase-factor cancellation and
split-phase accumulation shrink STEP generation. Host-visible registers, Q16
precision, all 64 profiles, driver timings, faults and settling remain supported.

Controlled local GF180 mapping reduces cell area from 140,824 to 61,319 um²
(56.5%), but this still exceeds the 51,967 um² 1x1 core even before routing
headroom. [Area results](area-optimization.md) distinguish these synthesis
measurements from physical verification. Do not enlarge the tile to bypass it.
