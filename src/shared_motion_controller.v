// SPDX-License-Identifier: Apache-2.0
`default_nettype none
// Closed-loop controller: a bit-serial trapezoidal planner (motion_engine) plus
// a dedicated single-cycle proportional controller (pi_p_controller).
// The PI engine's integrator and anti-windup machinery were removed to fit 1x1;
// only the P path remains. Every tick publishes one clamped command from the
// completed reference and then advances the planner. Ticks during planner work
// are ignored. clear_control or a low control_enable holds the command at zero
// without cancelling planner work; global enable=0 aborts both.
module shared_motion_controller #(parameter integer STABLE_CONFIG=0) (
    input wire clk, rst_n, enable, control_enable, clear_control, start, tick,
    input wire signed [12:0] start_position, target_position,
    input wire [5:0] plan_code,
    input wire [1:0] kp_select,
    output wire signed [23:0] reference_position,
    output wire signed [13:0] reference_velocity,
    output wire planner_busy, planner_done,
    output wire planner_calculating, pi_calculating, calculating,
    output wire signed [13:0] velocity_command,
    output wire saturated
);
    wire [12:0] vmax;
    wire engine_calculating;
    wire accept = tick && !engine_calculating && control_enable && !clear_control;
    reg pi_strobe;
    always @(posedge clk) begin
        if (!rst_n) pi_strobe <= 1'b0;
        else pi_strobe <= accept;
    end
    motion_engine #(.STABLE_CONFIG(STABLE_CONFIG)) planner (
        .clk(clk), .rst_n(rst_n), .enable(enable), .start(start), .tick(tick),
        .start_position(start_position), .target_position(target_position),
        .plan_code(plan_code), .reference_position(reference_position),
        .reference_velocity(reference_velocity), .planner_busy(planner_busy),
        .planner_done(planner_done), .planner_calculating(planner_calculating),
        .calculating(engine_calculating), .vmax(vmax)
    );
    pi_p_controller feedback (
        .clk(clk), .rst_n(rst_n), .enable(enable),
        .clear(start || clear_control || !control_enable || !enable), .accept(accept),
        .reference_position(reference_position), .measured(start_position),
        .feedforward(reference_velocity), .vmax(vmax), .kp_select(kp_select),
        .velocity_command(velocity_command), .saturated(saturated)
    );
    assign calculating = engine_calculating;
    // The P controller is single-cycle; this registered publish strobe replaces
    // the old multi-cycle PI pass window.
    assign pi_calculating = pi_strobe;
endmodule
`default_nettype wire
