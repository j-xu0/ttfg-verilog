/*
 * Copyright (c) 2026 Jason Xu
 * SPDX-License-Identifier: Apache-2.0
 */
`default_nettype none

// Stage 1: command and feedback interfaces. Motion outputs stay inactive until
// the planner, controller, and pulse generator have been implemented and tested.
module tt_um_j_xu0_stepper (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);
  wire signed [12:0] target;
  wire signed [12:0] position;
  wire [3:0] control;
  wire [5:0] plan;
  wire miso, encoder_fault, write_pulse, reject_pulse;

  stepper_spi spi (
      .clk(clk), .rst_n(rst_n),
      .sclk(ui_in[0]), .cs_n(ui_in[1]), .mosi(ui_in[2]),
      .busy(1'b0), .fault(encoder_fault), .at_target(1'b0),
      .miso(miso), .target(target), .control(control), .plan(plan),
      .write_pulse(write_pulse), .reject_pulse(reject_pulse)
  );

  quadrature_decoder encoder (
      .clk(clk), .rst_n(rst_n), .a(ui_in[3]), .b(ui_in[4]),
      .position(position), .fault(encoder_fault)
  );

  // [6] MISO fills an unspecified output assignment in the proposal.
  // [5] Fault is observable; busy/settled and all motor signals remain inactive.
  assign uo_out = {1'b0, (ena && rst_n && miso),
                   (ena && rst_n && encoder_fault), 5'b0};
  assign uio_out = 8'h00;
  assign uio_oe = 8'h00;

  wire _unused = &{ui_in[7:5], uio_in, target, position, control, plan,
                   write_pulse, reject_pulse, 1'b0};
endmodule

`default_nettype wire
