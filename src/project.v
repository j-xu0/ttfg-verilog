/*
 * Copyright (c) 2026 Jason Xu
 * SPDX-License-Identifier: Apache-2.0
 */
`default_nettype none

// 10 MHz, 1 kHz controller. Absolute signed encoder counts are motor-step units.
// TARGET commits arm a move; disable/fault aborts and requires a fresh TARGET.
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
  wire signed [12:0] target, position;
  wire [3:0] control;
  wire [5:0] plan;
  wire miso, encoder_fault, write_pulse, reject_pulse, target_write_pulse;
  (* async_reg = "true" *) reg enable_meta, enable_sync;
  reg signed [12:0] previous_position;
  reg range_fault;
  reg command_pending, active, armed, settled;
  reg [1:0] settle_count;
  wire fault = encoder_fault || range_fault;
  // Raw low immediately inhibits pins; assertion still passes the synchronizer.
  wire run_allowed = rst_n && ena && ui_in[5] && enable_sync && !fault;
  wire busy = active || (command_pending && run_allowed);
  wire at_target = settled && !active && !command_pending && run_allowed
                   && position == target;
  wire launch = run_allowed && command_pending && !active;
  wire control_ce, planner_busy, planner_done;
  wire signed [28:0] reference_position;
  wire signed [20:0] reference_velocity, velocity_command;
  wire target_reached = active && !planner_busy
      && reference_position == $signed({target, 16'b0}) && position == target;
  reg [19:0] velocity_limit, acceleration_limit;

  // Explicit implementation lookup choices; Q16 steps/tick and steps/tick^2.
  // Rounded from 100,200,500,1000,2000,4000,7000,10000 steps/s and
  // 500,1000,2000,5000,10000,20000,50000,100000 steps/s^2 at 1 kHz.
  always @* begin
    case (plan[2:0])
      0: velocity_limit = 20'd6554;
      1: velocity_limit = 20'd13107;
      2: velocity_limit = 20'd32768;
      3: velocity_limit = 20'd65536;
      4: velocity_limit = 20'd131072;
      5: velocity_limit = 20'd262144;
      6: velocity_limit = 20'd458752;
      default: velocity_limit = 20'd655360;
    endcase
    case (plan[5:3])
      0: acceleration_limit = 20'd33;
      1: acceleration_limit = 20'd66;
      2: acceleration_limit = 20'd131;
      3: acceleration_limit = 20'd328;
      4: acceleration_limit = 20'd655;
      5: acceleration_limit = 20'd1311;
      6: acceleration_limit = 20'd3277;
      default: acceleration_limit = 20'd6554;
    endcase
  end

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      enable_meta <= 0;
      enable_sync <= 0;
      previous_position <= 0;
      range_fault <= 0;
      command_pending <= 0;
      active <= 0;
      armed <= 0;
      settled <= 0;
      settle_count <= 0;
    end else begin
      enable_meta <= ui_in[5];
      enable_sync <= enable_meta;
      previous_position <= position;
      // Refuse wrap in the absolute coordinate system instead of making a
      // nearly full-range correction. Reset establishes a new zero reference.
      if ((previous_position == 13'sd4095 && position == -13'sd4096) ||
          (previous_position == -13'sd4096 && position == 13'sd4095))
        range_fault <= 1;
      if (target_write_pulse)
        command_pending <= 1;
      if (!run_allowed) begin
        active <= 0;
        armed <= 0;
        settled <= 0;
        settle_count <= 0;
        // An idle TARGET written while disabled can arm the next enable.
        // Aborted moves and faults never restart automatically.
        if (active || armed || fault)
          command_pending <= 0;
      end else if (target_write_pulse) begin
        // A fresh host command wins over disturbance recovery. The register
        // commits one clock before this strobe is consumed, so comparing the
        // new target against feedback must not reactivate the old reference.
        active <= 0;
        settled <= 0;
        settle_count <= 0;
      end else if (launch) begin
        command_pending <= 0;
        active <= 1;
        armed <= 1;
        settled <= 0;
        settle_count <= 0;
      end else if (armed && !active && position != target) begin
        active <= 1;
        settled <= 0;
        settle_count <= 0;
      end else if (active && control_ce) begin
        if (target_reached) begin
          if (settle_count == 3) begin
            active <= 0;
            settled <= 1;
          end else settle_count <= settle_count + 1'b1;
        end else begin
          settle_count <= 0;
          settled <= 0;
        end
      end
    end
  end

  stepper_spi spi (
      .clk(clk), .rst_n(rst_n),
      .sclk(ui_in[0]), .cs_n(ui_in[1]), .mosi(ui_in[2]),
      .busy(busy), .fault(fault), .at_target(at_target),
      .miso(miso), .target(target), .control(control), .plan(plan),
      .write_pulse(write_pulse), .reject_pulse(reject_pulse),
      .target_write_pulse(target_write_pulse)
  );
  quadrature_decoder encoder (
      .clk(clk), .rst_n(rst_n), .a(ui_in[3]), .b(ui_in[4]),
      .position(position), .fault(encoder_fault)
  );
  control_timebase timebase (
      .clk(clk), .rst_n(rst_n), .enable(run_allowed), .control_ce(control_ce)
  );
  motion_planner #(.FRACTION_BITS(16), .LIMIT_WIDTH(20), .SERIAL_SETUP(1)) planner (
      .clk(clk), .rst_n(rst_n), .tick(control_ce), .enable(run_allowed),
      .start(launch), .start_position(position), .target_position(target),
      .max_velocity(velocity_limit), .acceleration(acceleration_limit),
      .reference_position(reference_position), .reference_velocity(reference_velocity),
      .busy(planner_busy), .done(planner_done)
  );
  wire pi_saturated, rate_limited;
  pi_controller controller (
      .clk(clk), .rst_n(rst_n), .enable(run_allowed && active),
      .clear(launch || target_reached), .tick(control_ce),
      .reference_position(reference_position), .measured_position(position),
      .feedforward_velocity(reference_velocity), .max_velocity(velocity_limit),
      .kp_select(control[3:2]), .ki_select(control[1:0]),
      .velocity_command(velocity_command), .saturated(pi_saturated)
  );
  wire step, dir, driver_enable;
  step_dir_generator pulses (
      .clk(clk), .rst_n(rst_n), .enable(run_allowed && active),
      .fault(fault), .velocity_command(target_reached ? 21'sd0 : velocity_command),
      .step(step), .dir(dir), .driver_enable(driver_enable), .rate_limited(rate_limited)
  );
  assign uo_out = {1'b0, (ena && rst_n && miso),
      (ena && rst_n && fault), at_target,
      (ena && rst_n && busy), driver_enable, (run_allowed && dir), step};
  assign uio_out = 8'h00;
  assign uio_oe = 8'h00;
  wire _unused = &{ui_in[7:6], uio_in, write_pulse, reject_pulse, planner_done,
                   pi_saturated, rate_limited, 1'b0};
endmodule
`default_nettype wire
