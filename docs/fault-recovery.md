# Fault reporting and host recovery

Every SPI frame returns a frame-start status snapshot. Bits 14/13/12 remain
busy/at-target/any-fault. Bits 1/0 identify signed-range overflow/invalid encoder
transition respectively; both can be set. Other bits are zero.

Any complete 16-bit explicit read (MOSI bit 15 = 0, other bits ignored) returns
status then acknowledges faults when CS rises. Reads are allowed while busy.
Writes, partial/overlong frames, and reset-aborted frames never acknowledge.
If any new fault event occurs during the read, neither fault is acknowledged;
the host must read again. New events take priority over acknowledgement.

Faults immediately inhibit motor outputs and abort the old target. Acknowledging
preserves the encoder count and does not resume that target. A fresh TARGET is
required. The count may be inaccurate after an invalid transition or wrapped
after a range fault: acknowledgement is permission for host-directed recovery,
not evidence that coordinates are correct. Further faults still stop motion.

Write **0xE000** to establish position zero. This command is accepted only with
motion-enable input ui_in[5] low and no busy state during the frame. Nonzero
payloads are rejected. Keep the shaft stationary through this command. It zeros
the encoder and TARGET, cancels pending/armed/settled motion state, and disables
controller work. It does not acknowledge faults or change PLAN. A fresh TARGET
is required after zeroing; asserting enable alone cannot start movement.

The MCU can acknowledge a fault, perform homing using its own independent
reference, disable motion, zero position, then issue a fresh target. It must
account for uncertain/wrapped coordinates when choosing homing targets; there
is no dedicated relative/jog command or on-chip homing sensor in this change.
