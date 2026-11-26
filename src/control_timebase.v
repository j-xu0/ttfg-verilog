/*
 * Copyright (c) 2026 Jason Xu
 * SPDX-License-Identifier: Apache-2.0
 */
`default_nettype none

// Generate a registered clock enable; all consumers must still use clk.
// CLOCK_HZ must be a positive exact multiple of positive CONTROL_HZ.
// Default: one enabled-cycle pulse per 10000 cycles of the 10 MHz clock.
// Reset clears the pulse and starts a full interval. Disabling clears the
// pulse and freezes divider phase; re-enabling resumes the remaining interval.
// With divisor 1, control_ce is high on every enabled cycle.
module control_timebase #(
    parameter integer CLOCK_HZ = 10000000,
    parameter integer CONTROL_HZ = 1000
) (
    input wire clk,
    input wire rst_n,
    input wire enable,
    output reg control_ce
);
`ifndef SYNTHESIS
    initial begin
        if (CLOCK_HZ <= 0 || CONTROL_HZ <= 0 ||
            CLOCK_HZ % CONTROL_HZ != 0)
            $fatal(1, "TIMEBASE requires positive CLOCK_HZ exact multiple of CONTROL_HZ");
    end
`endif
    localparam integer DIVISOR = CLOCK_HZ / CONTROL_HZ;
    localparam integer COUNT_WIDTH = (DIVISOR > 1) ? $clog2(DIVISOR) : 1;
    // DIVISOR-1 is a 32-bit integer expression truncated to COUNT_WIDTH;
    // value is exact (DIVISOR <= 2^COUNT_WIDTH). Waiver, not a cast:
    // CI yosys runs Verilog-2005, which rejects SystemVerilog casts.
    // verilator lint_off WIDTHTRUNC
    localparam [COUNT_WIDTH-1:0] LAST_COUNT = DIVISOR - 1;
    // verilator lint_on WIDTHTRUNC
    reg [COUNT_WIDTH-1:0] count;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            count <= {COUNT_WIDTH{1'b0}};
            control_ce <= 1'b0;
        end else begin
            control_ce <= 1'b0;
            if (enable) begin
                if (count == LAST_COUNT) begin
                    count <= {COUNT_WIDTH{1'b0}};
                    control_ce <= 1'b1;
                end else begin
                    count <= count + 1'b1;
                end
            end
        end
    end
endmodule

`default_nettype wire
