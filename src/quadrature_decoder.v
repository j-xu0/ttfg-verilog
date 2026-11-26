/*
 * Copyright (c) 2026 Jason Xu
 * SPDX-License-Identifier: Apache-2.0
 */
`default_nettype none

// Counts every quadrature edge (x4). AB: 00 -> 01 -> 11 -> 10 is positive.
// Position is a signed modulo-8192 count: 4095 + 1 = -4096.
// Keep each AB phase stable for at least three system-clock cycles.
module quadrature_decoder (
    input wire clk,
    input wire rst_n,
    input wire a,
    input wire b,
    input wire clear_fault, zero_position,
    output wire fault_event,
    output reg signed [12:0] position,
    output reg fault
);
    (* async_reg = "true" *) reg [1:0] ab_meta;
    (* async_reg = "true" *) reg [1:0] ab_sync;
    reg [1:0] ab_delay_one, ab_delay_two;
    wire [1:0] ab_voted = (ab_sync & ab_delay_one) |
                          ((ab_sync | ab_delay_one) & ab_delay_two);
    reg [1:0] previous_ab;
    reg [4:0] input_valid;
    wire [1:0] changed = previous_ab ^ ab_voted;
    wire valid_edge = changed[1] ^ changed[0];
    wire decrement = previous_ab[0] ^ ab_voted[1];
    // Increment and decrement share the same toggle chain. A carry propagates
    // through ones for +1 and through zeros for -1; only the direction bit
    // differs. This avoids two independent 13-bit arithmetic paths.
    wire [12:0] toggle;
    assign toggle[0] = 1'b1;
    genvar bit_index;
    generate for (bit_index = 1; bit_index < 13; bit_index = bit_index + 1) begin : count_toggle
        assign toggle[bit_index] = toggle[bit_index-1] &
                                   (position[bit_index-1] ^ decrement);
    end endgenerate

    assign fault_event = input_valid[4] && (&changed) && !zero_position;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ab_meta <= 2'b00;
            ab_sync <= 2'b00;
            previous_ab <= 2'b00;
            ab_delay_one <= 2'b00;
            ab_delay_two <= 2'b00;
            input_valid <= 5'b00000;
            position <= 13'sd0;
            fault <= 1'b0;
        end else begin
            ab_meta <= {a, b};
            ab_sync <= ab_meta;
            ab_delay_one <= ab_sync;
            ab_delay_two <= ab_delay_one;
            input_valid <= {input_valid[3:0], 1'b1};
            if (clear_fault) fault <= 1'b0;
            // Establish the initial phase after the voted samples settle so
            // reset release at any stationary AB value never adds a count.
            previous_ab <= ab_voted;
            if (zero_position) begin
                position <= 0;
            end else if (input_valid[4]) begin
                if (valid_edge)
                    position <= position ^ toggle;
                else if (&changed)
                    fault <= 1'b1;
            end
        end
    end
endmodule

`default_nettype wire
