# Fault reporting and host recovery

Every SPI frame returns a frame-start status snapshot. Bit 14 is busy and bit 13
is at-target. Bits 1/0 identify signed-range overflow/invalid encoder transition
respectively; both can be set. There is no separate summary fault bit: any fault
is `bits[1:0] != 0`. Other bits are zero.

`uo_out[3]` is the interrupt. It asserts when the fault code latches and when a
move ends (completion or abort), and stays asserted until acknowledged. It is a
level, not a pulse, and it is not a per-event queue: several events before one
read collapse into one assertion.

Any complete 16-bit read (MOSI bit 15 = 0, other bits ignored) returns status
then acknowledges the faults and the interrupt when CS rises. Reads are allowed
while busy. A read is destructive for all latched state, so inspect the whole
word rather than only the bit that caused the poll. Reads require the project to
stay selected through frame completion; writes, partial/overlong frames, and
frames aborted by reset or deselection never acknowledge. If any new event
(fault latch or move completion) occurs during the read, nothing is
acknowledged, the interrupt stays high, and the host must read again. New events
take priority over acknowledgement.

Faults immediately inhibit motor outputs and abort the old target. Acknowledging
preserves the encoder count and does not resume that target. A fresh TARGET is
required. The count may be inaccurate after an invalid transition or wrapped
after a range fault: acknowledgement is permission for host-directed recovery,
not evidence that coordinates are correct. Further faults still stop motion.

Write **0xE000** to establish position zero. Drive motion enable (`ui_in[5]`)
low at least four system-clock periods (400 ns at 10 MHz) before asserting CS.
Hold it low throughout the frame and for at least four system-clock periods
after deasserting CS so the command can commit and the encoder can consume it.
The SPI receiver requires synchronized disable from frame start through commit,
and a status word that does not report BUSY. Nonzero payloads are ignored.

Keep the shaft stationary through this command. It zeros the encoder and TARGET,
cancels pending/armed/settled motion state, and disables controller work. It does
not acknowledge faults or change PLAN. A fresh TARGET is required after zeroing;
asserting enable alone cannot start movement. Because position itself is not
readable over SPI, the only confirmation that zeroing took effect is that the
command was accepted and subsequent at-target reads agree with the host's own
bookkeeping.

The MCU can acknowledge a fault, perform homing using its own independent
reference, disable motion, zero position, then issue a fresh target. It must
account for uncertain/wrapped coordinates when choosing homing targets; there
is no dedicated relative/jog command or on-chip homing sensor in this change.
