## How it works

This SPI-controlled stepper servo plans a trapezoidal reference, compares it to
quadrature feedback, and drives STEP/DIR through a saturated proportional (P)
controller. The
system uses a 10 MHz clock and a 1 kHz control enable. One x4 encoder count is
one motor-step position unit; reset establishes zero and no homing is performed.
See `motion-planning.md` for numerical choices, command lifecycle, and timing.
ENABLE is logical active high; external drivers with an active-low enable need
appropriate inversion. No external power-driver IC has been selected.

The top module uses the Tiny Tapeout GF180 digital interface. All output buses
are assigned. Unused dedicated outputs drive zero. All bidirectional pins are
unused inputs (`uio_oe=0`, `uio_out=0`); there are no internal tri-state nets.
MISO is a dedicated output, so it drives zero while idle rather than becoming
high impedance. Do not wire it directly onto a shared MISO bus without an
external buffer or separate selection arrangement. Busy, at-target and fault
are not driven on dedicated pins; every SPI frame returns them as a status word
(see below), and `uo_out[3]` tells the host when to poll.

### Pins

| Pin | Function |
| --- | --- |
| `ui_in[0]` | SPI SCLK, mode 0 |
| `ui_in[1]` | SPI CS, active low |
| `ui_in[2]` | SPI MOSI |
| `ui_in[3]` | Encoder A |
| `ui_in[4]` | Encoder B |
| `ui_in[5]` | Motion enable; low aborts, rising alone does not restart |
| `ui_in[7:6]` | Unused |
| `uo_out[0]` | STEP, rising-edge step command |
| `uo_out[1]` | DIR, high for positive motion |
| `uo_out[2]` | ENABLE, logical active high while active |
| `uo_out[3]` | Interrupt, active high, held until a status read |
| `uo_out[4]` | Unused, zero |
| `uo_out[5]` | Unused, zero |
| `uo_out[6]` | SPI MISO |
| `uo_out[7]` | Unused, zero |
| `uio[7:0]` | Unused, input mode |
| `clk` | External 10 MHz system clock target |
| `rst_n` | Active-low reset; hold across a clock edge |

MISO on output 6 is an implementation choice: the detailed proposal assigns
outputs 0-5 but omits MISO. Outputs 3-5 replace the proposal's busy, at-target
and fault pins with a single interrupt; status itself is only readable over
SPI, so a host needs six wires (`SCLK`, `CS`, `MOSI`, `MISO`, `INT`, ground).
`ena=0` suppresses dedicated outputs; normal operation assumes the project is
selected and `ena=1`.

### SPI registers

SPI is CPOL=0, CPHA=0, MSB first, up to the proposal's 500 kHz target with a
10 MHz system clock. Inputs pass through two-flop synchronizers and all logic
runs from `clk`; SCLK is a sampled input, not a generated or secondary RTL clock.

A write is exactly 16 bits: `{1'b1, address[1:0], payload[12:0]}`. Registers update
atomically after synchronized CS deassertion, never during a partial transfer.

| Address | Name | Width | Reset | Interpretation |
| --- | --- | --- | --- | --- |
| 0 | TARGET | 13 | 0 | Signed two's-complement position, -4096 to 4095 |
| 1 | Reserved | - | - | Silently ignored (was CONTROL: fixed-Kp/Ki, removed for 1x1) |
| 2 | PLAN | 3 | 0 | Acceleration code 0-7 only; velocity fixed at 8191 |
| 3 | ZERO_POSITION | - | - | Payload must be zero. Zeros position and TARGET with motion disabled per [host timing](fault-recovery.md). |

CONTROL was removed: addr 1 is reserved and silently ignored. Kp was fixed at the former code 1,
i.e. 0.25 steps/tick per encoder count (250 steps/s at 1 kHz). The PLAN velocity field was removed
because `vmax` is fixed at 8191 (3.9995 steps/tick); only payload bits [2:0] (accel code) are stored.
Upper payload bits are ignored. Configure PLAN before writing TARGET. A valid
TARGET commit arms a move, even if its value matches the previous target.
Short, overlong, reserved-address and illegal frames are silent no-ops; an
empty selection is a no-op.

### Status and interrupt

Every frame returns a 16-bit status snapshot captured at synchronized CS
assertion:

| Bits | Meaning |
| --- | --- |
| 15 | Always zero |
| 14 | `BUSY`: planning, feedback correction or settling in progress |
| 13 | `AT_TARGET`: settled with exact feedback at the written target |
| 12:2 | Zero |
| 1 | `RANGE_FAULT`: signed-range wrap |
| 0 | `ENC_FAULT`: invalid encoder transition |

There is no separate summary fault bit: `FAULT` is
`RANGE_FAULT | ENC_FAULT`, i.e. mask `0x0003`. Busy is `0x4000` and at-target is
`0x2000`. A 16-bit command whose first bit is zero reads this word and, after
the complete frame, acknowledges latched faults and the interrupt. Writes never
acknowledge. Reads are allowed while busy.

The returned `BUSY` bit is also the verdict on that same write. Motion starts
are held for as long as a frame is in flight (until its first bit proves the
frame is a read), so a queued command cannot begin during a frame. A write is
accepted exactly when the word returned by its own frame has `BUSY` clear; a
frame that returns `BUSY=1` drops its write without side effects and the host
must retry. A write in flight when motion enable rises wins the race: it
replaces the queued target instead of being rejected.

`uo_out[3]` is an interrupt: it goes high when a move ends (completion or
abort) or a fault latches, and stays high until a complete read acknowledges
it. It is a level, not a pulse. An MCU that only reacts to edges must keep
reading while the pin is still high, because an event that arrives during a
read leaves the interrupt asserted and the read acknowledges nothing. Status
bits are recovered by polling, so a missed edge never loses state; only the
number of coalesced completions is lost.

Because motion is held while CS is low, do not park CS asserted between
frames. Keep each SCLK half-period and the CS setup/hold/idle intervals at least four
system clocks (400 ns at 10 MHz). Keep MOSI stable at least two clocks before
rising SCLK and through falling SCLK. At 500 kHz, the SCLK half-period is 1 us.
MISO is selected from the captured status by frame position, so a bit settles a
couple of clocks after the rising edge that advances the frame; the four-clock
half-period is what keeps it stable well before the host's next rising-edge
sample. Do not shorten it.
After reset, hold CS high for at least four clocks before selecting the slave.
A frame held selected through reset is ignored until CS returns high and a new
transaction begins. Hold SCLK low outside transactions.

### Encoder

A and B pass through two-flop synchronizers. The sequence
`00 -> 01 -> 11 -> 10 -> 00` counts +4; the reverse sequence counts -4. Position
is a signed 13-bit x4 count. The decoder itself wraps modulo 8192, but the
top level latches a range fault on either signed wrap and aborts motion. The
controller requires one encoder count per motor step; other ratios require a
scaling change. A simultaneous two-bit jump does not change position and latches fault until a complete explicit SPI read. The next valid edge can count
again; the fault stays latched until acknowledged. See [fault recovery](fault-recovery.md) for codes, races, and the disabled-only `0xE000` position-zero command.

Hold the encoder stationary through reset and at least four system-clock edges
after release so the initial phase can be established without a false count.
During operation each AB phase must last at least four system-clock periods;
short transitions can be missed. Reset zeroes the relative position; there is no
absolute homing operation yet.

### GF180 build contract

The [official GF Verilog template](https://github.com/TinyTapeout/ttgf-verilog-template/blob/main/src/project.v)
defines the port widths, active-high bidirectional output enables, and requirement
to assign every output. The project follows that contract. The build retains the
existing `TinyTapeout/tt-gds-action@ttgf26a` flow with `pdk: gf180mcuD`.

The 10 MHz clock is a project target, recorded as `clock_hz: 10000000` and
`CLOCK_PERIOD: 100` ns. The [GF180 support-tool configuration](https://github.com/TinyTapeout/tt-support-tools/blob/main/tech.py)
selects the 3.3 V GF180 libraries and process/voltage/temperature timing corners.
The public [GPIO specification](https://tinytapeout.com/specs/gpio/) quotes
SKY130 pad limits; those numbers are not GF180 signoff limits. Maximum safe clock
and output rates must be established with the GF180 flow and the actual board
and output loading. RTL tests alone do not prove them.

All functional blocks now feed observable motion/status outputs. Status is read
over SPI; `uo_out[7:3]` no longer carries busy/at-target/fault, and the added
interrupt plus launch gate are expected to be area-neutral (one dropped strobe
register against one added latch), but CI has not yet re-measured them. The last
hardened tip **passes precheck** (CI run
`44132561693`: 0 DRT violations, minimum feasible density 0.8300) and measured a
cell area of 41,578 um² before routability inflation, 42,913 um² final, against
the 51,967 um² core. See [the area budget](area.md) for the restriction set,
ledger and rejected alternatives.

## How to test

Run the commands in `test/README.md`. Module tests cover arithmetic, trajectories,
pulse timing, command framing, and feedback decoding. Port-level integration tests
exercise a simulated motor with missed steps, reverse commands, shaft disturbance,
settling, busy write drops, abort/reset, and both signed-range boundaries. The
register-wiring test uses RTL hierarchy and is skipped for flattened gate-level
simulation; the port-level tests remain enabled.

## External hardware

Use an SPI master and a quadrature encoder or logic source for interface tests.
The final motor requires an external power driver for winding commutation and
current limiting. Tiny Tapeout output pins carry logic signals only. Confirm the
selected GF180 board's electrical limits and external-driver polarity before
connecting hardware. Verify the configurable timing and encoder scaling against
the chosen hardware before operating a motor. Driver ENABLE drops after settling,
so this implementation does not maintain holding torque while idle.
