`timescale 1ns/1ps
module tb;
  reg clk=0, rst_n=0, enable=1, control_enable=1, clear_control=0, start=0, tick=0;
  reg signed [12:0] start_position=0, target_position=0;
  reg [5:0] plan_code=0; reg [1:0] kp=0, ki=0;
  wire signed [24:0] reference_position; wire signed [13:0] reference_velocity;
  wire busy, done, pcalc, picalc, calc; wire signed [13:0] vcmd; wire sat;
  shared_motion_controller dut(.clk(clk),.rst_n(rst_n),.enable(enable),.control_enable(control_enable),
    .clear_control(clear_control),.start(start),.tick(tick),.start_position(start_position),
    .target_position(target_position),.plan_code(plan_code),.kp_select(kp),.ki_select(ki),
    .reference_position(reference_position),.reference_velocity(reference_velocity),
    .planner_busy(busy),.planner_done(done),.planner_calculating(pcalc),.pi_calculating(picalc),
    .calculating(calc),.velocity_command(vcmd),.saturated(sat));
  always #5 clk=~clk;
  task nege; @(negedge clk); endtask
  integer t, k;
  initial begin

    if (!$value$plusargs("goal=%d", target_position)) target_position = 300;
    if (!$value$plusargs("code=%d", plan_code)) plan_code = 63;
    repeat(3) nege; rst_n=1; nege;
    start_position=0; start=1; nege; start=0; nege; nege;
    for (t=0; t<400; t=t+1) begin
      tick=1; nege; tick=0;
      k=0;
      while (calc !== 0 && k < 4000) begin @(negedge clk); k=k+1; end
      $display("T %0d pos=%0d rv=%0d spd=%0d E=%0d busy=%b", t, reference_position, reference_velocity, dut.speed, dut.stopping, busy);
      if (!busy) t=999;
      nege;
    end
    $finish;
  end
endmodule
