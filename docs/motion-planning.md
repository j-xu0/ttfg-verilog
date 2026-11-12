# Motion-control contract

The top level now connects SPI, quadrature feedback, timebase, planner, PI, and
STEP/DIR generation. All logic uses the 10 MHz `clk`; the 1 kHz registered control
enable is not a derived clock. `src/config.json` retains its 100 ns target.

## Position and commands

One signed integer position unit is one motor step and one x4 encoder transition.
Reset establishes position zero; there is no homing or encoder-ratio converter.
Valid absolute coordinates are -4096 through 4095. The decoder's modulo wrap is
caught by the top-level sticky range fault, so crossing either signed boundary
stops motion rather than causing a full-range correction.

Configure CONTROL and PLAN, then write TARGET. Every accepted TARGET commit is
a command, including a repeated value. A command written while motion is disabled
is queued until enable. TARGET starts a trajectory at the measured position.
All register writes are rejected if busy occurs at any point during their SPI
frame; status reads remain available. Busy includes planning, feedback correction,
and settling. A stall without feedback leaves busy asserted until disable/reset.

Motion enable assertion is synchronized. Raw low, `ena=0`, reset, or a sticky
fault inhibits STEP/ENABLE immediately. Disable aborts, clears controller state,
and discards the active command; re-enabling requires a new TARGET write. A target
written while faulted cannot cause motion. Fault recovery requires reset.
Hold reset across at least one rising clock edge because the shared planner/PI
engine resets synchronously. Motion-pin suppression is immediate and does not
wait for engine state to clear.

A completed trajectory must have exact encoder feedback at the target for four
control samples before At target asserts and Busy clears. The PI is cleared at
the final reference/feedback match. The pulse generator receives zero velocity,
finishes its normal active pulse, and ENABLE drops after settling. There is no
idle holding torque. Feedback drift while enabled clears At target and reactivates
correction against the retained final reference without a new trajectory.

## Numerical format and lookup tables

The shared controller uses fixed Q16 arithmetic:

- Position reference: signed 29-bit integer position with 16 fractional bits.
- Reference and commanded velocity: signed 21-bit steps/control-tick with 16
  fractional bits; speed limits are unsigned 20-bit values.
- Acceleration: unsigned 13-bit raw steps/control-tick squared, 16 fractional bits.
- Gains: two-bit codes 00, 01, 10, 11 select 1/8, 1/4, 1/2, 1. CONTROL[3:2]
  selects Kp and CONTROL[1:0] selects Ki.

The detailed proposal's 1 kHz loop and acceleration endpoints are used. Intermediate
lookup entries are explicit implementation choices; the proposals omit them.
Raw limits are rounded to nearest Q16 units. Actual rates follow the quantized
values, not exact decimal endpoints.

| Code | Velocity steps/s | Raw Q16 velocity | Acceleration steps/s² | Raw Q16 acceleration |
| --- | ---: | ---: | ---: | ---: |
| 0 | 100 | 6554 | 500 | 33 |
| 1 | 200 | 13107 | 1000 | 66 |
| 2 | 500 | 32768 | 2000 | 131 |
| 3 | 1000 | 65536 | 5000 | 328 |
| 4 | 2000 | 131072 | 10000 | 655 |
| 5 | 4000 | 262144 | 20000 | 1311 |
| 6 | 7000 | 458752 | 50000 | 3277 |
| 7 | 10000 | 655360 | 100000 | 6554 |

PLAN[2:0] selects velocity, PLAN[5:3] acceleration. At 1 kHz, raw acceleration 33
is about 503.54 steps/s². Q16 resolves 15.26 steps/s², so the low acceleration
range is representable. The shorter proposal's /8192 loop and 100 steps/s² lower
endpoint are not selected. No separate PWM output is generated; PI commands a
signed step rate for an external STEP/DIR power driver.

## Timebase and trajectory

`control_timebase` defaults to one pulse per 10,000 system clocks. Its parameters
must be positive with an exact integer ratio. Disable clears its output and freezes
divider phase; reset starts a full interval. Sequential consumers use the registered
enable on the next edge and independently gate aborts. Divisor 1 enables every cycle.

The planner latches positions/limits on `start` while idle; commands while busy
are ignored. Only accepted ticks advance the reference. Each tick advances position
by the previous reference velocity, then selects the next velocity. The first tick
accelerates without displacement. Exact discrete stopping distance determines
acceleration, cruise and braking. Final fractional displacement lands exactly on
the target; the landing edge sets velocity zero and emits one-clock `done`.

The integrated `shared_motion_controller` uses a generated 64-entry lookup table
for the exact discrete stopping sums of the supported velocity/acceleration
pairs. Values larger than every legal move are represented by an always-rejected
sentinel; accepted stopping sums remain exact. Reachability bounds reduce stored
stopping distance to 29 bits and the shared bit-serial accumulator to 30 bits.

Each accepted tick starts PI sampling/calculation followed by planner advancement,
completing within 2,000 clocks before the next 10,000-clock control tick. Position
and velocity are temporary arithmetic workspace while combined `calculating=1`
and may only be sampled while idle. PI uses the previously completed reference
and sampled encoder value. Target/settling logic does not consume workspace;
it uses planner busy/calculation status and integer target/feedback equality.
Planner inputs remain stable while busy because SPI rejects writes. This permits
`STABLE_CONFIG=1` to omit redundant target/profile registers. Disable/reset
cancels an unfinished calculation; the next command starts from measured position.

The old `motion_planner.v`, `compact_motion_planner.v` and `pi_controller.v` remain
as independently tested references, excluded from the physical source list.
No speed, acceleration,
gain, position-range, or Q16 fractional-resolution setting is removed.

## PI arithmetic

On each enabled control tick, error is the full signed difference between the
Q16 reference and measured integer position shifted left 16; it never uses modulo
subtraction. Command is `feedforward + Kp*error + old_integral`, clamped to the
selected velocity limit. Then `Ki*error` is added to the integral and clamped to
that same magnitude. Integration freezes if the command is beyond a limit and
the error would push it farther into saturation. Reduced limits still clamp the
stored integral. The serial signed sum streams include 31 bits, enough for the
defined position range without overflow. Gain shifts round negative values toward
minus infinity.

The top level selects shared-engine `STABLE_CONFIG=1`. An idle tick captures
feedback before serial error calculation; the old reference and speed are
consumed before planner advancement. Gains/limit are read live and stay stable
through the busy/write interlock. Command and saturation publish together before
planner work starts. Internal integral storage is workspace while calculating.
Ticks during work are ignored. Standalone legacy PI modes remain in reference
tests; see [shared scheduling and register lifetimes](shared-controller.md).

A launch, disable, or final reference/feedback match clears PI state independent
of tick. PI-only cancellation may finish restoring a shared ring after command
and integral clear; it cannot publish a cancelled result. The gains are discrete
gains per 1 kHz sample, not gains per second.
Planner acceleration bounds the reference; PI correction may change the actual
step rate faster while remaining under the selected speed limit. Hardware tuning
must account for this distinction and the real plant dynamics.

## STEP/DIR/ENABLE timing

`step_dir_generator` divides its phase threshold and increment by their exact
constant greatest common divisor. At the default clock rates this cancels 1000,
shrinks phase storage 40→30 bits and removes multiplication by 1000. A split
fractional/coarse accumulator additionally simplifies the default modulo logic;
other parameter combinations retain a general carry/borrow fallback. This preserves
every pulse-generator event cycle for an identical command stream, not merely
average frequency. Command magnitude is clamped to 10,000
steps/s. Positive motion uses DIR=1; ENABLE is logical active high. Defaults:

| Requirement | System clocks | At 10 MHz |
| --- | ---: | ---: |
| STEP high | 50 | 5 us |
| STEP low minimum | 50 | 5 us |
| DIR setup | 50 | 5 us |
| DIR hold after STEP falls | 50 | 5 us |
| ENABLE setup before STEP | 100 | 10 us |

Timing values are positive configurable parameters. MAX_STEP_HZ must not exceed
CLOCK_HZ. The unspecified external driver must be checked against this contract;
these values are not a compatibility claim for a particular IC. Reversal finishes
the old pulse, waits hold time, changes DIR, then waits setup. Zero velocity
finishes an active pulse and clears fractional phase. Reset/disable/fault may
truncate a pulse for immediate inhibition. Unserviceable rate events are discarded
and internally flagged rather than queued into later bursts.

## Verification and physical work

Module tests compare arithmetic and trajectories to independent Python models and
check displacement, acceleration, saturation, exact landing, timing and rate counts.
Port-level motor tests model legal encoder feedback, dropped steps and disturbances.
This idealized digital model does not establish real motor stability or tuning.

Controlled full-top GF180 standard-cell synthesis measures 58,174.995 um²,
2,372 cells and 353 flip-flops, a 58.7% area reduction from the original controller
under the same local mapping flow. The mandatory 1x1 core is only 51,966.970 um²,
so this is still over budget. No 1x1 fit, post-route timing or physical-signoff
claim follows from these results. See `area-optimization.md`.
