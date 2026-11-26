# TinyStep (`tt_um_j_xu0_stepper`) Chip Overview

## What the Chip Does

TinyStep (`tt_um_j_xu0_stepper`) is a closed-loop stepper motor servo controller designed for a single 1x1 Tiny Tapeout (GF180) tile running at a **10 MHz** system clock.

An external host microcontroller commands absolute target positions over **SPI (Mode 0)** and selects an acceleration profile. The chip reads real-time shaft position from an incremental **quadrature encoder (x4 decoding)**, plans a trapezoidal velocity trajectory at **1 kHz**, applies closed-loop **proportional (P) position feedback**, and outputs **STEP / DIR / ENABLE** pulses to an external stepper motor driver IC.

Key capabilities:
- **Position range:** Signed 13-bit absolute coordinates (`-4096` to `+4095` steps), where 1 step = 1 STEP pulse = 1 x4 quadrature count.
- **Velocity & acceleration:** Fixed maximum speed of ~4,000 steps/s (Q11 `8191` = 3.9995 steps/ms) with 8 selectable acceleration profiles (`PLAN[2:0]`).
- **Closed-loop feedback:** 1 kHz control loop with fixed-gain proportional control (`Kp = 0.25` steps/tick per count = 250 steps/s per count) to correct missed steps and external disturbances.
- **Safety & fault handling:** Hardware motion-enable inhibit (`ui_in[5]`), invalid quadrature transition detection, signed-range overflow protection, and a level-sensitive interrupt output (`uo_out[3]`).

---

## Block Diagram

```text
                                 +-------------------------------------------------------------------+
                                 |                   tt_um_j_xu0_stepper (10 MHz)                    |
                                 |                                                                   |
 SPI SCLK  (ui_in[0]) ---------->|  +------------------+                                             |
 SPI CS_n  (ui_in[1]) ---------->|  |   stepper_spi    |                                             |
 SPI MOSI  (ui_in[2]) ---------->|  | (Mode-0 16-bit)  |                                             |
 SPI MISO  (uo_out[6]) <---------|  |                  |                                             |
 Interrupt (uo_out[3]) <---------|  +------------------+                                             |
                                 |     ^           |                                                 |
                                 |     | status/   | target[12:0], plan[2:0],                        |
                                 |     | events    | target_write/fault_clear/zero_position pulses   |
                                 |     |           v                                                 |
                                 |  +----------------------------------------------+                 |
 Motion En (ui_in[5]) ---------->|  |   Top-Level Motion State & Safety Interlock  |                 |
                                 |  |  (enable sync, range fault, settle counter)  |                 |
                                 |  +----------------------------------------------+                 |
                                 |     |            ^             |              |                   |
                                 |     | run_       | position,   | active,      | run_allowed,      |
                                 |     | allowed    | enc_fault   | launch,      | active, fault     |
                                 |     v            |             | target, plan |                   |
                                 |  +------------+  |             v              |                   |
                                 |  |  control   |  |  +----------------------+  |                   |
                                 |  |  timebase  |--+->| shared_motion_ctrl   |  |                   |
                                 |  | (10M->1kHz)|  |  | +------------------+ |  |                   |
                                 |  +------------+  |  | |  motion_engine   | |  |                   |
                                 |                  |  | | (bit-serial Q11  | |  |                   |
                                 |  +------------+  |  | |  trapezoidal)    | |  |                   |
 Encoder A (ui_in[3]) ---------->|  | quadrature |--+->| +--------+---------+ |  |                   |
 Encoder B (ui_in[4]) ---------->|  |  decoder   |     |          | ref_pos   |  |                   |
                                 |  | (x4, 13b)  |     |          v           |  |                   |
                                 |  +------------+     | +------------------+ |  |                   |
                                 |                     | | pi_p_controller  | |  |                   |
                                 |                     | | (fixed-Kp P)     | |  |                   |
                                 |                     | +--------+---------+ |  |                   |
                                 |                     +----------|-----------+  |                   |
                                 |                                | velocity_cmd |                   |
                                 |                                v              v                   |
                                 |                     +----------------------------+                |
                                 |                     |     step_dir_generator     |---> STEP  (uo[0])
                                 |                     | (DDS pulse gen + timing    |---> DIR   (uo[1])
                                 |                     |  setup/hold interlocks)    |---> EN    (uo[2])
                                 |                     +----------------------------+                |
                                 +-------------------------------------------------------------------+
```

---

## Pinout

All unused output pins are tied low (`0`), and all bidirectional pins (`uio[7:0]`) are unused inputs (`uio_oe = 8'h00`, `uio_out = 8'h00`).

### System Pins

| Pin | Direction | Description |
| --- | --- | --- |
| `clk` | Input | 10 MHz system clock |
| `rst_n` | Input | Active-low reset (hold low across at least one rising `clk` edge) |
| `ena` | Input | Tiny Tapeout project select / enable |

### Dedicated Inputs (`ui_in`)

| Pin | Signal Name | Description |
| --- | --- | --- |
| `ui_in[0]` | `SPI SCLK` | SPI clock (Mode 0: CPOL=0, CPHA=0; oversampled at 10 MHz) |
| `ui_in[1]` | `SPI CS_n` | Active-low SPI chip select |
| `ui_in[2]` | `SPI MOSI` | SPI master-out, slave-in (16-bit MSB-first frames) |
| `ui_in[3]` | `Encoder A` | Quadrature encoder channel A |
| `ui_in[4]` | `Encoder B` | Quadrature encoder channel B |
| `ui_in[5]` | `Motion enable` | Active-high hardware motion enable; pulling low immediately inhibits motor outputs |
| `ui_in[6]` | *(unused)* | — |
| `ui_in[7]` | *(unused)* | — |

### Dedicated Outputs (`uo_out`)

| Pin | Signal Name | Description |
| --- | --- | --- |
| `uo_out[0]` | `STEP` | Step pulse output to external stepper driver |
| `uo_out[1]` | `DIR` | Direction output (`1` = positive, `0` = negative) |
| `uo_out[2]` | `ENABLE` | Logical active-high driver enable output |
| `uo_out[3]` | `Interrupt` | Level-sensitive interrupt (asserts on move end or fault; cleared by a complete SPI read) |
| `uo_out[4]` | *(unused)* | Driven `0` |
| `uo_out[5]` | *(unused)* | Driven `0` |
| `uo_out[6]` | `SPI MISO` | SPI master-in, slave-out (driven `0` when idle or deselected) |
| `uo_out[7]` | *(unused)* | Driven `0` |

### Bidirectional Pins (`uio`)

| Pins | Direction | Description |
| --- | --- | --- |
| `uio[7:0]` | Input (unused) | All `uio_oe` bits are `0`; all `uio_out` bits are `0` |

---

## High-Level How the Chip Works

### 1. Host Interface (`stepper_spi`)
The host MCU communicates with the chip using 16-bit, MSB-first **SPI Mode 0** frames (`{write, addr[1:0], payload[12:0]}`), oversampled and majority-voted in the 10 MHz clock domain:
- **Registers:**
  - `addr 0`: Signed 13-bit **TARGET** position (`-4096` to `+4095`). Writing `TARGET` arms a move.
  - `addr 1`: Reserved (ignored).
  - `addr 2`: 3-bit **PLAN** acceleration code (`0`–`7`).
  - Special write `0xE000`: **ZERO_POSITION**, which resets encoder position and `TARGET` to `0`. It is accepted only when `BUSY = 0` and voted `Motion enable` (`ui_in[5]`) stays low from CS assertion through commit (drive `ui_in[5]` low at least 4 system clocks before asserting `CS_n` and hold it low for at least 5 system clocks after deasserting `CS_n`).
- **Status & Write Verdict:** Every SPI frame shifts out a 16-bit status snapshot captured at `CS_n` assertion: `{0, BUSY, AT_TARGET, 11'b0, RANGE_FAULT, ENCODER_FAULT}`. If `BUSY` is `1`, any write in that frame is dropped so configuration stays stable during motion.
- **Interrupts & Fault Clearing:** `uo_out[3]` latches high whenever a move finishes/aborts or a fault occurs. A complete 16-bit SPI read (`write = 0`) returns the status word and clears both latched faults and the interrupt when `CS_n` rises (unless a new event arrives during the frame).

### 2. Position Feedback & Fault Detection (`quadrature_decoder` + Top Level)
- `quadrature_decoder` synchronizes and majority-votes `Encoder A` and `Encoder B`, counting every valid edge (**x4 decoding**) into a signed 13-bit position counter (`-4096` to `+4095`).
- **Encoder Fault:** If both `A` and `B` change between consecutive filtered (synchronized and majority-voted) states—an illegal two-bit quadrature transition (raw edges need not occur at the exact same instant)—`encoder_fault` latches.
- **Range Fault:** The top level watches for signed wrap across `-4096`/`+4095` and latches `range_fault` so the controller halts instead of executing a full-range wrap-around correction.
- Either fault, a low `Motion enable` (`ui_in[5]`), deselection (`ena = 0`), or reset immediately inhibits motor outputs and aborts the active move. A fresh `TARGET` write is required to move again after recovery.

### 3. 1 kHz Control Loop (`control_timebase` + `shared_motion_controller`)
`control_timebase` divides the 10 MHz clock by 10,000 to produce a single-cycle **1 kHz** clock enable (`control_ce`). On each accepted 1 kHz tick, `shared_motion_controller` runs two stages in Q11 fixed-point format:
1. **Proportional Feedback (`pi_p_controller`):** Computes integer position error (`reference_position[23:11] - measured_position`), clamps it to `±32` counts, scales by `Kp = 0.25` steps/tick per count (`<< 9` in Q11), and clamps the resulting `velocity_command` to `±8191` (`±4,000` steps/s).
2. **Bit-Serial Trapezoidal Planner (`motion_engine`):** Advances `reference_position` by the current velocity and tests a capped candidate speed `min(s + a, vmax)` using the selected acceleration from `plan_lut.vh`. Using a compact bit-serial ALU (no hardware multiplier or divider), it maintains an incremental brake sum—adding the candidate speed when accelerating, leaving it unchanged whenever speed holds (both at `vmax` and below `vmax` when the current stopping sum fits but the faster candidate does not), and subtracting the current speed when decelerating—so the reference never overshoots. Once remaining distance is less than one step of `TARGET`, the planner snaps `reference_position` directly onto `TARGET` and clears speed to zero.

### 4. Step Pulse Generation (`step_dir_generator`)
`step_dir_generator` is a single-clock DDS (direct digital synthesis) pulse generator driven by the signed Q11 `velocity_command`. It converts the commanded step rate into `STEP`, `DIR`, and `ENABLE` signals while enforcing conservative hardware timing contracts at 10 MHz:
- `STEP` high / low minimum: **5 µs** (50 cycles)
- `DIR` setup / hold: **5 µs** (50 cycles)
- `ENABLE` setup before `STEP`: **10 µs** (100 cycles)

When reversing direction, the generator finishes any active pulse, waits `DIR` hold time, toggles `DIR`, and waits `DIR` setup time before stepping again. Note that immediate shutdown conditions—reset (`rst_n = 0`), disable (`ui_in[5] = 0`), a latched fault, or project deselection (`ena = 0`)—override these timing minima, immediately suppressing `ENABLE`, gating `DIR` low, and potentially truncating an active `STEP` pulse.

### 5. Settling & Disturbance Recovery
Once the planner finishes at `TARGET`, the top-level state machine requires exact encoder equality (`position == target`) for **4 consecutive 1 kHz ticks** before clearing `BUSY`, asserting `AT_TARGET`, and dropping `ENABLE` (there is no idle holding torque). While still armed and enabled, if an external disturbance pushes the shaft away from `TARGET`, `AT_TARGET` clears and the P controller automatically reactivates to pull the motor back to the target.

### 6. Host Fault Handling & Recovery
- **Latched Faults:** Status bits `[1:0]` report latched faults (there is no separate summary fault bit; both can be set at once):
  - `ENC_FAULT` (status bit `0`): Latched on a two-bit transition between consecutive filtered `A`/`B` states. The count does not step on that cycle and may no longer match the physical shaft angle.
  - `RANGE_FAULT` (status bit `1`): Latched when the 13-bit signed position wraps across `-4096`/`+4095`. The counter has wrapped modulo 8192.
- **Immediate Effect:** Either fault immediately inhibits motor outputs (`STEP`, `DIR`, `ENABLE`), aborts the active/armed move, and asserts the level-sensitive interrupt (`uo_out[3]`).
- **Acknowledging Faults & IRQ:** A complete 16-bit SPI status read (`write = 0`, while selected through `CS_n` rising) returns the status snapshot and clears both fault latches and `uo_out[3]`. If any new event (fault or move end) occurs during the read frame, nothing is cleared, `uo_out[3]` remains high, and the host must read again.
- **Restoring Coordinates vs. Clearing Faults:** Acknowledging a fault only clears the latch—it preserves the (potentially inaccurate or wrapped) encoder count and does **not** resume the aborted move. To restore a trusted position, the host must use its own external reference/homing procedure, pull `Motion enable` (`ui_in[5]`) low, keep the shaft stationary while sending `ZERO_POSITION` (`0xE000`, accepted only with `BUSY = 0` and voted disable from CS assertion through commit), and keep `Motion enable` low for at least 5 system clocks after `CS_n` deassertion. Once fault latches are acknowledged, the host can then write a fresh `TARGET` while motion is still disabled and reassert `Motion enable` to start the queued move. Note that `ZERO_POSITION` itself does not clear latched faults.
