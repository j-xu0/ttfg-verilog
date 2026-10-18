## How it works

This SPI-controlled stepper servo plans a trapezoidal reference, compares it to
quadrature feedback, and drives STEP/DIR through a saturated PI controller. The
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
external buffer or separate selection arrangement.

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
| `uo_out[3]` | Busy, motion or settling in progress |
| `uo_out[4]` | At target, exact feedback for four control samples |
| `uo_out[5]` | Sticky encoder transition or signed-range fault |
| `uo_out[6]` | SPI MISO |
| `uo_out[7]` | Unused, zero |
| `uio[7:0]` | Unused, input mode |
| `clk` | External 10 MHz system clock target |
| `rst_n` | Active-low reset; hold across a clock edge |

MISO on output 6 is an implementation choice: the detailed proposal assigns
outputs 0-5 but omits MISO. Output 7 remains available for a future interrupt
once its behavior is defined. `ena=0` suppresses dedicated outputs; normal
operation assumes the project is selected and `ena=1`.

### SPI registers

SPI is CPOL=0, CPHA=0, MSB first, up to the proposal's 500 kHz target with a
10 MHz system clock. Inputs pass through two-flop synchronizers and all logic
runs from `clk`; SCLK is a sampled input, not a generated or secondary RTL clock.

A write is exactly 16 bits: `{1'b1, address[1:0], payload[12:0]}`. Registers update
atomically after synchronized CS deassertion, never during a partial transfer.

| Address | Name | Width | Reset | Interpretation |
| --- | --- | --- | --- | --- |
| 0 | TARGET | 13 | 0 | Signed two's-complement position, -4096 to 4095 |
| 1 | CONTROL | 4 | 0 | Upper two bits select kP, lower two bits select kI |
| 2 | PLAN | 6 | 0 | Upper three bits select acceleration, lower three select velocity |
| 3 | Reserved | - | - | Writes rejected |

CONTROL and PLAN discard upper payload bits. Gain codes 00, 01, 10, 11 correspond
to 1/8, 1/4, 1/2, 1. Configure CONTROL and PLAN before writing TARGET. A valid
TARGET commit arms a move, even if its value matches the previous target.
Short and overlong frames are rejected; an empty selection is a no-op. Busy at any time during a transaction
rejects its write, even if busy clears before CS rises. Reads still return status
while busy.

Every frame returns a status snapshot captured at synchronized CS assertion:
`{1'b0, busy, at_target, fault, 12'b0}`. The first four returned bits are therefore
`0, busy, at_target, fault`. A 16-bit command with its first bit zero reads this
status without modifying registers. This read-command behavior and bit ordering
resolve details left unspecified in the proposal. Busy is `0x4000`, settled is
`0x2000`, and fault is `0x1000`. Malformed/reserved writes produce an internal
one-cycle rejection pulse; a persistent host-visible rejection code is pending.

Keep each SCLK half-period and the CS setup/hold/idle intervals at least four
system clocks (400 ns at 10 MHz). Keep MOSI stable at least two clocks before
rising SCLK and through falling SCLK. At 500 kHz, the SCLK half-period is 1 us.
After reset, hold CS high for at least four clocks before selecting the slave.
A frame held selected through reset is ignored until CS returns high and a new
transaction begins. Hold SCLK low outside transactions.

### Encoder

A and B pass through two-flop synchronizers. The sequence
`00 -> 01 -> 11 -> 10 -> 00` counts +4; the reverse sequence counts -4. Position
is a signed 13-bit x4 count. The decoder itself wraps modulo 8192, but the
top level latches a range fault on either signed wrap and aborts motion. The
controller requires one encoder count per motor step; other ratios require a
scaling change. A simultaneous two-bit jump does not change position and latches fault until reset. The next valid edge can count
again; the fault stays latched.

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

All functional blocks now feed observable motion/status outputs. Generic Yosys
synthesis passes, but tile fit, GF180 STA, DRC/LVS, and gate-level simulation are
still unverified. The 1x1 tile allocation remains provisional.

## How to test

Run the commands in `test/README.md`. Module tests cover arithmetic, trajectories,
pulse timing, command framing, and feedback decoding. Port-level integration tests
exercise a simulated motor with missed steps, reverse commands, shaft disturbance,
settling, busy-write rejection, abort/reset, and both signed-range boundaries. The
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
