## How it works

This design is an 8-bit programmable binary up-counter. Pulling `rst_n` low
clears the counter immediately, without waiting for a clock edge. On each rising
edge of `clk`, the counter loads `uio_in[7:0]` when `ui_in[0]` is high;
otherwise, it increments modulo 256.

The counter value is connected to `uio[7:0]`. Tiny Tapeout uses a separate
output-enable signal for each bidirectional pin, so `ui_in[1]` controls all
eight counter outputs. When it is low, all `uio` pins are released (high
impedance); when high, they drive the current counter value.

## How to test

1. Pull `rst_n` low to asynchronously reset the count to zero, then release it.
2. With `ui_in[1]` low, put a value on `uio_in[7:0]`, set `ui_in[0]` high, and
   pulse `clk` to load it.
3. Set `ui_in[0]` low. Each rising edge of `clk` increments the count.
4. Set `ui_in[1]` high to read the count from `uio[7:0]`, or low to release the
   pins.

The automated cocotb test verifies asynchronous reset, synchronous load,
incrementing, wraparound from 255 to 0, and output-enable behavior.

## External hardware

No external hardware is required.
