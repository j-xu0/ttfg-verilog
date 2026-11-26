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
  wire [2:0] plan;
  wire miso, encoder_fault, write_pulse, target_write_pulse;
  wire int_pending, frame_holds_motion, motion_event;
  (* async_reg = "true" *) reg enable_meta, enable_sync;
  (* async_reg = "true" *) reg pin_enable_meta, pin_enable_sync;
  reg pin_enable_delay_one, pin_enable_delay_two;
  wire pin_enable_voted = (pin_enable_sync && pin_enable_delay_one) ||
      ((pin_enable_sync || pin_enable_delay_one) && pin_enable_delay_two);
  wire enable_reset_n = rst_n && ui_in[5];
  reg previous_sign, busy_last;
  reg range_fault;
  wire fault_clear_pulse, zero_position_pulse, encoder_event;
  wire range_event = !zero_position_pulse &&
      ((!previous_sign && position == -13'sd4096) ||
       (previous_sign && position == 13'sd4095));
  reg command_pending, active, armed, settled;
  reg [1:0] settle_count;
  wire fault = encoder_fault || range_fault;
  // Raw low inhibits the pins through the enable synchronizer's asynchronous
  // reset, so the inhibit does not wait for a clock edge.
  wire run_allowed = rst_n && ena && enable_sync && !fault && !zero_position_pulse;
  // A frame in flight freezes the launch instant so the BUSY bit returned by
  // that same frame is the exact write verdict. busy_core stays ungated for
  // edge detection, otherwise gating a queued command would look like a
  // completion.
  wire busy_core = active || (command_pending && run_allowed);
  wire busy = active || (command_pending && run_allowed && !frame_holds_motion);
  wire at_target = settled && !active && !command_pending && run_allowed
                   && position == target;
  wire launch = run_allowed && command_pending && !active && !frame_holds_motion;
  wire control_ce, planner_busy, planner_done, planner_calculating, pi_calculating;
  wire engine_calculating;
  wire signed [23:0] reference_position;
  wire signed [13:0] reference_velocity, velocity_command;
  // An idle planner for an armed move has completed at the latched TARGET.
  // Workspace outputs are deliberately invalid during serial calculations.
  wire target_reached = active && !planner_busy && !planner_calculating
      && position == target;

  // A falling enable immediately inhibits pins and holds them off through
  // clocked cleanup, even if the entire low pulse occurs between clocks.
  always @(posedge clk or negedge enable_reset_n) begin
    if (!enable_reset_n) begin
      enable_meta <= 0;
      enable_sync <= 0;
    end else begin
      enable_meta <= 1;
      enable_sync <= enable_meta;
    end
  end

  // ZERO_POSITION uses a voted sample of the host pin in both
  // directions. The motion inhibit above also covers re-enable latency.
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pin_enable_meta <= 1;
      pin_enable_sync <= 1;
      pin_enable_delay_one <= 1;
      pin_enable_delay_two <= 1;
    end else begin
      pin_enable_meta <= ui_in[5];
      pin_enable_sync <= pin_enable_meta;
      pin_enable_delay_one <= pin_enable_sync;
      pin_enable_delay_two <= pin_enable_delay_one;
    end
  end

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      previous_sign <= 0;
      busy_last <= 0;
      range_fault <= 0;
      command_pending <= 0;
      active <= 0;
      armed <= 0;
      settled <= 0;
      settle_count <= 0;
    end else begin
      previous_sign <= position[12];
      busy_last <= busy_core;
      // Each encoder update is at most one count, so sign plus the new
      // endpoint identifies either signed wrap without a second 13-bit counter.
      // Refuse wrap in the absolute coordinate system instead of making a
      // nearly full-range correction. Reset establishes a new zero reference.
      if (fault_clear_pulse) range_fault <= 0;
      if (range_event) range_fault <= 1;
      if (zero_position_pulse) previous_sign <= 0;
      if (target_write_pulse)
        command_pending <= 1;
      if (!run_allowed) begin
        active <= 0;
        armed <= 0;
        settled <= 0;
        settle_count <= 0;
        // An idle TARGET written while disabled can arm the next enable.
        // Aborted moves and faults never restart automatically.
        if (active || armed || fault || zero_position_pulse)
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
      end else if (armed && !active && position != target
                   && !frame_holds_motion) begin
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
      .clk(clk), .rst_n(rst_n), .selected(ena),
      .sclk(ui_in[0]), .cs_n(ui_in[1]), .mosi(ui_in[2]),
      .busy(busy), .at_target(at_target),
      .fault_code({range_fault, encoder_fault}),
      .fault_event(encoder_event || range_event),
      .motion_event(motion_event), .motion_disabled(!pin_enable_voted),
      .fault_clear_pulse(fault_clear_pulse), .zero_position_pulse(zero_position_pulse),
      .miso(miso), .frame_holds_motion(frame_holds_motion),
      .int_pending(int_pending), .target(target), .plan(plan),
      .write_pulse(write_pulse), .target_write_pulse(target_write_pulse)
  );
  quadrature_decoder encoder (
      .clk(clk), .rst_n(rst_n), .a(ui_in[3]), .b(ui_in[4]),
      .position(position), .fault(encoder_fault),
      .clear_fault(fault_clear_pulse), .zero_position(zero_position_pulse),
      .fault_event(encoder_event)
  );
  control_timebase timebase (
      .clk(clk), .rst_n(rst_n), .enable(run_allowed), .control_ce(control_ce)
  );
  wire pi_saturated, rate_limited;
  // The planner advances the reference bit-serially; a dedicated single-cycle
  // P stage samples the completed reference and feedback and publishes the
  // clamped command before planner work starts.
  // Busy SPI writes are rejected, keeping configuration stable during work.
  shared_motion_controller #(.STABLE_CONFIG(1)) controller (
      .clk(clk), .rst_n(rst_n), .tick(control_ce), .enable(run_allowed),
      .control_enable(active), .clear_control(launch || target_reached),
      .start(launch), .start_position(position), .target_position(target),
      .plan_code(plan), .kp_select(2'b00),
      .reference_position(reference_position), .reference_velocity(reference_velocity),
      .planner_busy(planner_busy), .planner_done(planner_done),
      .planner_calculating(planner_calculating), .pi_calculating(pi_calculating),
      .velocity_command(velocity_command), .saturated(pi_saturated),
      .calculating(engine_calculating)
  );
  wire step, dir, driver_enable;
  step_dir_generator pulses (
      .clk(clk), .rst_n(rst_n), .enable(run_allowed && active),
      .fault(fault), .velocity_command(target_reached ? 14'sd0 : velocity_command),
      .step(step), .dir(dir), .driver_enable(driver_enable), .rate_limited(rate_limited)
  );
  // The interrupt is a level held until a status read acknowledges it, so a
  // slow or late MCU cannot miss an event. Status is read over SPI only.
  assign motion_event = busy_last && !busy_core;
  // Deselected frames cannot drive MISO or acknowledge a fault, so the SPI
  // receiver gates MISO itself and the project-select term is not repeated.
  assign uo_out = {1'b0, (rst_n && miso), 1'b0, 1'b0,
      (ena && rst_n && int_pending), driver_enable,
      (run_allowed && dir), step};
  assign uio_out = 8'h00;
  assign uio_oe = 8'h00;
  wire _unused = &{ui_in[7:6], uio_in, write_pulse, planner_done,
                   pi_saturated, rate_limited, pi_calculating, engine_calculating, 1'b0};
endmodule
`default_nettype wire
