# Numeric format specification

Control tick = 1 kHz. Rate units are steps/tick and steps/tick², where one step is
one STEP pulse and one x4 encoder transition. Velocity ceiling is 4 000 steps/s;
position range is ±4 095 steps.

## Rate and control fields

| Register | Signedness | Bits | Int | Frac | Total | Raw range | Physical range | LSB @1 kHz |
|---|---|---|---:|---:|---:|---|---|---|
| `reference_velocity` | signed | `[13:0]` | 2 | 11 | 14 | −8 191 … +8 191 | ±3.9995 steps/tick = **±3 999 steps/s** | 0.4883 steps/s |
| `speed_limit` | unsigned | `[12:0]` | 2 | 11 | 13 | 0 … 8 191 | 0 … 3.9995 steps/tick = 3 999 steps/s | 0.4883 steps/s |
| `acceleration` | unsigned | `[10:0]` | 0 | 11 | 11 | 0 … 2 047 | 0 … 0.99951 steps/tick² = **999 512 steps/s²** | 488.3 steps/s² |

All three rate fields share the fractional scale Q11, so `v += a` is a direct add
and `pos += v` is an exact Q11 add (no shift, no truncation). Acceleration has no
integer bits, which is valid only while the velocity ceiling stays below
~8 600 steps/s; a 16 000 steps/s ceiling would require `[11:0]` (12 b).

## Position fields

| Register | Signedness | Bits | Int | Frac | Total | Raw range | Physical range | LSB |
|---|---|---|---:|---:|---:|---|---|---|
| `target` / `latched_goal` | signed | `[12:0]` | 12 | 0 | 13 | −4 096 … +4 095 | ±4 095 steps | 1 step |
| `start_position` | signed | `[12:0]` | 12 | 0 | 13 | −4 096 … +4 095 | ±4 095 steps | 1 step |
| `reference_position` | signed | `[23:0]` | 13 | 11 | 24 | −8 388 608 … +8 386 560 | reaches ±4 096/4 095 targets | 1/2048 step |
| `remaining` (scratch) | unsigned | `[23:0]` | 13 | 11 | 24 | 0 … 16 777 215 | 0 … 8 191 steps | 1/2048 step |
| `stopping` (brake sum) | unsigned | `[23:0]` | 13 | 11 | 24 | 0 … 16 777 215 | 0 … 8 191 steps | 1/2048 step |
| plan LUT (`vmax`, `accel`) | — | 13 / 11 | — | — | 24 per profile | from `plan_lut.vh` | — | — |

`reference_position` and `reference_velocity` share the Q11 scale, so each control
tick is an exact `pos += v`. The 13-bit signed integer part holds exactly −4 096 … +4 095, so the 24-bit
register covers the whole legal position range and doubles as scratch for the
signed remaining distance during the brake decision.

The brake sum is maintained **exactly** in Q11 as `S(s) = s + S(s-a)`:

* accelerating to `s'` adds `s'`,
* holding at `s` leaves `S` unchanged,
* decelerating subtracts the current `s`.

A candidate speed is accepted only when its exact `S` fits inside the exact
remaining distance, so the reference can never overshoot. There is no stopping
lookup table and no sentinel: unreachable profile pairs are simply never
accepted. Within one step of the target the reference snaps exactly onto it, so
the resting error is zero (sub-step residue is discarded, not accumulated).

Storage total for the engine registers: position 24, brake sum 24, speed 13,
accumulator 25, sampled measured/P command 14. Feedback is **P-only**: error is
`(reference[23:11] - measured) << 11`, the Kp code selects 1/8…1, and the
command `feedforward + Kp*error` is clamped to the profile velocity limit.
There is no integrator, no Ki register, and no anti-windup path.
