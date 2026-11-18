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

Configure PLAN, then write TARGET. Every accepted TARGET commit is
a command, including a repeated value. A command written while motion is disabled
is queued until enable. TARGET starts a trajectory at the measured position.
All register writes are dropped when the status word returned by their own SPI
frame reports BUSY; status reads remain available, and motion starts are held
for the duration of a frame so that bit is an exact verdict. Busy includes
planning, feedback correction,
and settling. A stall without feedback leaves busy asserted until disable/reset.

Motion enable assertion is synchronized. Raw low, `ena=0`, reset, or a sticky
fault inhibits STEP/ENABLE immediately. Disable aborts, clears controller state,
and discards the active command; re-enabling requires a new TARGET write. A target
written while faulted cannot cause motion. Fault recovery uses a complete explicit SPI read; see `fault-recovery.md`.
Hold reset across at least one rising clock edge because the planner and P
controller reset synchronously. Motion-pin suppression is immediate and does not
wait for engine state to clear.

A completed trajectory must have exact encoder feedback at the target for four
control samples before the at-target status bit asserts and busy clears. The P command is cleared at
the final reference/feedback match. The pulse generator receives zero velocity,
finishes its normal active pulse, and ENABLE drops after settling. There is no
idle holding torque. Feedback drift while enabled clears At target and reactivates
correction against the retained final reference without a new trajectory.

## Numerical format and lookup tables

The integrated controller uses fixed **Q11** arithmetic; see
[Integrated Q11 numeric format](#integrated-q11-numeric-format) below. The table
below is proposal background in Q16 units. The RTL uses Q11 only; the
standalone Q16 references were deleted on `nick/bug-fixes`.

Proposal Q16 units (background only):

- Position reference: signed 29-bit integer position with 16 fractional bits.
- Reference and commanded velocity: signed 21-bit steps/control-tick with 16
  fractional bits; speed limits are unsigned 20-bit values.
- Acceleration: unsigned 13-bit raw steps/control-tick squared, 16 fractional bits.
- Reference PI gains: two-bit codes 00, 01, 10, 11 select 1/8, 1/4, 1/2, 1.
  The integrated fixed-Kp design reserves all CONTROL gain bits.

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

## Integrated Q11 numeric format

This is the physical design's format. Control tick is 1 kHz; rate units are
steps/tick, where one step is one STEP pulse and one x4 encoder transition.
Velocity ceiling is 4,000 steps/s; position range is ±4,095 steps.

| Field | Signed | Bits | Int | Frac | Raw range | Physical range | LSB |
| --- | --- | ---: | ---: | ---: | --- | --- | --- |
| `reference_velocity` | yes | `[13:0]` | 2 | 11 | −8,191 … +8,191 | ±3.9995 steps/tick = ±3,999 steps/s | 0.4883 steps/s |
| `speed_limit` | no | `[12:0]` | 2 | 11 | 0 … 8,191 | 0 … 3.9995 steps/tick | 0.4883 steps/s |
| `acceleration` | no | `[10:0]` | 0 | 11 | 0 … 2,047 | 0 … 0.99951 steps/tick² = 999,512 steps/s² | 488.3 steps/s² |
| `target` / `start_position` | yes | `[12:0]` | 12 | 0 | −4,096 … +4,095 | ±4,095 steps | 1 step |
| `reference_position`, `remaining`, `stopping` | yes / no | `[23:0]` | 13 | 11 | ±8,388,608 | covers ±4,096 targets | 1/2048 step |

All three rate fields share the Q11 scale, so `v += a` is a direct add and
`pos += v` is an exact Q11 add: no shift, no truncation. Acceleration has no
integer bits, valid only while the ceiling stays below ~8,600 steps/s (a
16,000 steps/s ceiling would need `[11:0]`). The 13-bit signed integer part of
the 24-bit reference covers the whole legal position range and doubles as
scratch for the signed remaining distance during the brake decision.

**Current 1x1 restriction.** `vmax` is fixed at 8191; acceleration is the 3-bit PLAN
code; SPI addr 1 (ex-CONTROL) is reserved and silently ignored.

The brake sum is maintained **exactly** in Q11 as `S(s) = s + S(s-a)`:
accelerating to `s'` adds `s'`, holding at `s` leaves `S` unchanged, decelerating
subtracts the current `s`. A candidate speed is accepted only when its exact `S`
fits inside the exact remaining distance, so the reference cannot overshoot and
there is no stopping lookup table or sentinel in the integrated engine —
unreachable profile pairs are simply never accepted. (The generated 8-entry
`plan_lut.vh` accel table feeds `motion_engine`.) Within
one step of the target the reference snaps exactly onto it, so resting error is
zero: sub-step residue is discarded, not accumulated.

Engine storage: reference position 24, brake sum 24, speed 13, accumulator 25,
sampled measured/P command 14 bits.

Feedback is **P-only with a fixed gain**: error is `reference[23:11] - measured`
(integer counts), clamped to ±32, scaled by `2^9` (Kp = 0.25 steps/tick per
count = 250 steps/s per count). No feedforward, no runtime Kp register, no
integrator, no Ki, no anti-windup. The command is clamped to the fixed 8191
ceiling.

Control consequence: feedforward is a tracking term, not a loop-dynamics term.
The plant is a velocity command into a position integrator, so this is a type-1
loop with zero steady-state error to a position step. The cost is a steady
cruise lag `e_ss = v_ref / Kp` — up to 16 counts at the 4 steps/tick ceiling —
and weaker rejection of a constant disturbance. The fixed Kp gives a discrete
loop gain `Kp·T = 0.25`, comfortably damped; the retired `kp=3` code was
deadbeat and should not be reintroduced.

## Timebase and trajectory

`control_timebase` defaults to one pulse per 10,000 system clocks. Its parameters
must be positive with an exact integer ratio. Disable clears its output and freezes
divider phase; reset starts a full interval. Sequential consumers use the registered
enable on the next edge and independently gate aborts. Divisor 1 enables every cycle.

The planner latches positions/limits on `start` while idle; commands while busy
are ignored. Only accepted ticks advance the reference. Each tick advances position
by the previous reference velocity, then selects the next velocity. The first tick
accelerates without displacement. Speed-up and slow-down use the same rate, so
every move is symmetric: the distance spent speeding up is mirrored when slowing
down. Each tick only checks whether there is still room to mirror a faster step
(two additions and a comparison on the travelled distance). No division or
multiplication is used anywhere; there is no setup delay and motion can begin on
the first tick. A short final step lands exactly on the target; the landing edge
sets velocity zero and emits one-clock `done`. `SERIAL_SETUP` is retained as an
ignored compatibility parameter; all configurations use the same mirror rule.

The integrated planner uses a generated eight-entry Q11 acceleration lookup.
Maximum velocity is fixed at 8191 raw Q11 units. Stopping distance is maintained
incrementally by `motion_engine`; no stopping-distance ROM is used. Its serial
accumulator is 25 bits and its stopping-distance register is 24 bits.

Each accepted tick publishes the fixed-Kp command from the completed reference,
then starts planner advancement, completing within 2,000 clocks before the next
10,000-clock control tick. Reference position is arithmetic workspace while
`calculating=1` and may only be sampled while idle. Feedback uses the completed
reference and sampled encoder value, without velocity feedforward or an integral.
Target/settling logic does not consume workspace;
it uses planner busy/calculation status and integer target/feedback equality.
Planner inputs remain stable while busy because a BUSY frame drops its write. This permits
`STABLE_CONFIG=1` to omit redundant target/profile registers. Disable/reset
cancels an unfinished calculation; the next command starts from measured position.

The standalone `motion_planner.v` and `pi_controller.v` references were deleted on
`nick/bug-fixes`; `test_shared_controller.py` covers the active serial planner.
The integrated design uses Q11, fixed vmax and fixed Kp.

## Feedback arithmetic

> **Current integrated engine (Q11, 1x1 area revision).** The physical design is
> P-only with a fixed gain: `reference_position[23:0]` is Q11, error is
> `reference[23:11] - measured` clamped to ±32 and shifted left 9 (Kp = 0.25
> steps/tick per count). There is no feedforward and no runtime Kp select; the
> command is clamped to the fixed `vmax = 8191`. The integrator, Ki register and
> anti-windup path were removed with the standalone references.

## PI arithmetic (historical — standalone Q16 PI deleted)

> The section below describes the deleted Q16 PI. The ship design is P-only fixed-Kp.

On each enabled control tick, error was the full signed difference between the
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
Ticks during work are ignored. The shipped P-only stage has no integral storage;
see [shared scheduling and register lifetimes](shared-controller.md).

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

The final P-only fixed-Kp variant **hardens on the 1x1 tile and passes precheck**
(CI run `44132561693`: 0 DRT violations, minimum feasible density 0.8300; final
placement area 42,913.48 um² against the 51,966.970 um² core). Post-route timing
signoff and the gate-level test harness still need work. See
[the area budget](area.md).
