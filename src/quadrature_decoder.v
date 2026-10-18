/*
 * Copyright (c) 2026 Jason Xu
 * SPDX-License-Identifier: Apache-2.0
 */
`default_nettype none

// Counts every quadrature edge (x4). AB: 00 -> 01 -> 11 -> 10 is positive.
// Position is a signed modulo-8192 count: 4095 + 1 = -4096.
// Inputs must remain stable long enough to pass the two-flop synchronizer.
module quadrature_decoder (
    input wire clk,
    input wire rst_n,
    input wire a,
    input wire b,
    output reg signed [12:0] position,
    output reg fault
);
    (* async_reg = "true" *) reg [1:0] ab_meta;
    (* async_reg = "true" *) reg [1:0] ab_sync;
    reg [1:0] previous_ab;
    reg [1:0] startup_cycles;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ab_meta <= 2'b00;
            ab_sync <= 2'b00;
            previous_ab <= 2'b00;
            startup_cycles <= 2'd0;
            position <= 13'sd0;
            fault <= 1'b0;
        end else begin
            ab_meta <= {a, b};
            ab_sync <= ab_meta;
            // Establish the initial phase after the synchronizer settles so
            // reset release at any stationary AB value never adds a count.
            if (startup_cycles != 2'd3) begin
                startup_cycles <= startup_cycles + 2'd1;
                previous_ab <= ab_sync;
            end else begin
                previous_ab <= ab_sync;
                case ({previous_ab, ab_sync})
                    4'b0001, 4'b0111, 4'b1110, 4'b1000:
                        position <= position + 13'sd1;
                    4'b0010, 4'b1011, 4'b1101, 4'b0100:
                        position <= position - 13'sd1;
                    4'b0011, 4'b1100, 4'b0110, 4'b1001:
                        fault <= 1'b1;
                    default: begin end // Holding the same phase is valid.
                endcase
            end
        end
    end
endmodule

`default_nettype wire
