/*
 * Copyright (c) 2026 Jason Xu
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_j_xu0_counter (
    input  wire [7:0] ui_in,    // ui_in[0]: load, ui_in[1]: output enable
    output wire [7:0] uo_out,   // Unused dedicated outputs
    input  wire [7:0] uio_in,   // Parallel load value
    output wire [7:0] uio_out,  // Counter output value
    output wire [7:0] uio_oe,   // Active-high output enables
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n     // Asynchronous active-low reset
);

  reg [7:0] count;

  // Reset takes effect immediately. Loading and counting occur on rising edges.
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      count <= 8'h00;
    else if (ui_in[0])
      count <= uio_in;
    else
      count <= count + 8'h01;
  end

  // Tiny Tapeout implements tri-state bidirectional pins with separate data and
  // output-enable buses. All eight pins are high impedance when ui_in[1] is 0.
  assign uio_out = count;
  assign uio_oe  = {8{ui_in[1]}};

  assign uo_out = 8'h00;

  // Prevent warnings for intentionally unused inputs.
  wire _unused = &{ena, ui_in[7:2], 1'b0};

endmodule

`default_nettype wire
