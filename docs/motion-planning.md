# Standalone motion-planning contract

This layer adds independently tested building blocks. Neither module is
instantiated in `project.v` yet. SPI behavior, the GF180 pin contract, and the
10 MHz / 100 ns system clock remain as implemented in the foundation layer.

## Clock enable

`control_timebase` uses `clk` for every register and emits `control_ce` as a
registered clock enable. `CLOCK_HZ` and `CONTROL_HZ` must be positive, with an
exact integer ratio. Defaults are 10 MHz and 1 kHz (10,000 clock cycles). The
1 kHz default follows the detailed proposal; the other proposal's /8192 rate
remains an unresolved system-level choice.

Reset is asynchronous active low and starts a full interval. Disable suppresses
the pulse and freezes divider phase; re-enable resumes the remaining interval.
A divisor of one enables every active cycle. Sequential consumers see the
registered enable on the following clock edge and must separately gate immediate
abort/disable behavior.

## Trajectory reference

`motion_planner` accepts `start` on any enabled clock edge while idle. It latches
signed 13-bit integer start/target positions and unsigned 16-bit velocity and
acceleration limits. Commands during `busy` are ignored. Hold `start` for one
clock: a continuously asserted start can launch another command after completion.

- `reference_position`: signed 21-bit position with eight fractional bits.
- `reference_velocity`: signed 17-bit position units per control tick, with eight
  fractional bits.
- `max_velocity`: unsigned Q8 position units per tick.
- `acceleration`: unsigned Q8 position units per tick squared.

For a 1 kHz tick, a raw velocity value of 256 means 1,000 position units/s;
a raw acceleration value of 256 means 1,000,000 position units/s². The smallest
nonzero Q8 acceleration at that rate is 3,906.25 position units/s². This initial
format cannot represent the proposals' smallest accelerations if one position
unit means one motor step. The final format, control rate, and encoder-to-step
ratio must be reconciled before host integration. No PLAN-code table is invented
by this layer.

Only `tick` advances an accepted trajectory. Each tick advances position by the
previous velocity, then computes the velocity for the next interval. The first
tick accelerates from rest without displacement. The planner accelerates up to the
latched speed limit, cruises when further acceleration cannot stop within the
remaining distance, and decelerates using a discrete stopping-distance sum. It
handles both directions and the entire signed position span without modulo
wrap. A fractional final displacement lands exactly on the target. Velocity
becomes zero on the landing edge; each position increment equals the pre-edge
velocity, including the last displacement. `done` lasts one system clock.

A zero-distance command completes immediately. Zero velocity or acceleration
rejects motion and pulses `done`, with the reference at the supplied start
position. Callers must validate limits because `done` does not distinguish these
cases. Disable aborts on the next system clock, holds the current reference,
and clears velocity and busy without pulsing done. Planner reset is synchronous
active low and clears all state.

## Integration and physical verification still required

The exact stopping calculation contains combinational variable division and
products. RTL simulation verifies arithmetic and trajectory behavior; it does
not establish 1x1 GF180 tile fit or operation at 10 MHz. Before connecting this
module, measure synthesis area and timing and, if necessary, replace that
calculation with a bounded multi-cycle implementation using the clocks between
control ticks.

Remaining work includes the final numerical format and host lookup values,
position/reference and encoder scaling, fault/enable interlocks, PI controller,
STEP/DIR timing, top-level integration, and GF180 hardening/signoff. The current
top-level hardening flow excludes these standalone modules and cannot measure
their cost.
