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
  reg previous_sign;
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
  wire control_ce, planner_busy, planner_done, planner_calculating, pi_calculating;
  wire engine_calculating;
  wire signed [28:0] reference_position;
  wire signed [20:0] reference_velocity, velocity_command;
  // An idle planner for an armed move has completed at the latched TARGET.
  // Workspace outputs are deliberately invalid during serial calculations.
  wire target_reached = active && !planner_busy && !planner_calculating
      && position == target;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      enable_meta <= 0;
      enable_sync <= 0;
      previous_sign <= 0;
      range_fault <= 0;
      command_pending <= 0;
      active <= 0;
      armed <= 0;
      settled <= 0;
      settle_count <= 0;
    end else begin
      enable_meta <= ui_in[5];
      enable_sync <= enable_meta;
      previous_sign <= position[12];
      // Each encoder update is at most one count, so sign plus the new
      // endpoint identifies either signed wrap without a second 13-bit counter.
      // Refuse wrap in the absolute coordinate system instead of making a
      // nearly full-range correction. Reset establishes a new zero reference.
      if ((!previous_sign && position == -13'sd4096) ||
          (previous_sign && position == 13'sd4095))
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
  wire pi_saturated, rate_limited;
  // One engine samples the completed reference and feedback, evaluates PI,
  // then advances the planner using the same scratch register and bit ALU.
  // Busy SPI writes are rejected, keeping configuration stable during work.
  shared_motion_controller #(.STABLE_CONFIG(1)) controller (
      .clk(clk), .rst_n(rst_n), .tick(control_ce), .enable(run_allowed),
      .control_enable(active), .clear_control(launch || target_reached),
      .start(launch), .start_position(position), .target_position(target),
      .plan_code(plan), .kp_select(control[3:2]), .ki_select(control[1:0]),
      .reference_position(reference_position), .reference_velocity(reference_velocity),
      .planner_busy(planner_busy), .planner_done(planner_done),
      .planner_calculating(planner_calculating), .pi_calculating(pi_calculating),
      .velocity_command(velocity_command), .saturated(pi_saturated),
      .calculating(engine_calculating)
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
                   pi_saturated, rate_limited, pi_calculating, engine_calculating, 1'b0};
endmodule
`default_nettype wire
