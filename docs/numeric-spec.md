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
and `pos += v` is a fixed 3-bit shift. Acceleration has no integer bits, which is
valid only while the velocity ceiling stays below ~8 600 steps/s; a 16 000 steps/s
ceiling would require `[11:0]` (12 b).

## Position fields

| Register | Signedness | Bits | Int | Frac | Total | Raw range | Physical range | LSB |
|---|---|---|---:|---:|---:|---|---|---|
| `target` / `latched_goal` | signed | `[12:0]` | 12 | 0 | 13 | −4 096 … +4 095 | ±4 095 steps | 1 step |
| `start_position` | signed | `[12:0]` | 12 | 0 | 13 | −4 096 … +4 095 | ±4 095 steps | 1 step |
| `pos_int` | signed | `[13:0]` | 13 | 0 | 14 | −8 192 … +8 191 | ±8 191 steps | 1 step |
| `pos_frac` | unsigned | `[7:0]` | 0 | 8 | 8 | 0 … 255 | 0 … 255/256 step | 1/256 step |
| `stopping` (LUT entry) | unsigned | `[13:0]` | 14 | 0 | 14 | 0 … 16 383; `0x3FFF` = sentinel | 0 … 8 191 legal | 1 step |

`pos_int` needs 14 bits to hold ±4 095 plus up to 4 steps of per-tick velocity
overshoot, and because it doubles as scratch for `remaining = goal - pos_int`
(±8 191). `pos_int` and `pos_frac` are one logical Q13.8 accumulator split so the
fraction never enters the arithmetic; the brake test `remaining <= stopping` is
integer-only and at most 1 step (250 µs) early or late. The `stopping` sentinel
`0x3FFF` exceeds every legal remaining distance, so unreachable profile pairs are
rejected rather than accepted.

Storage total for the registers listed above: 74 bits.
