/*
 * Copyright (c) 2026 Jason Xu
 * SPDX-License-Identifier: Apache-2.0
 */
`default_nettype none

// Position error and all arithmetic use Q16 fractional bits. Measured position
// is an integer; reference is signed Q13.16 and velocity is signed Q5.16.
// Gains 00,01,10,11 select 1/8,1/4,1/2,1. Arithmetic shifts round toward
// negative infinity (including sub-LSB negative errors).
// On tick: command = clamp(feedforward + Kp*error + OLD integral).
// Then integrate Ki*error, clamped to the velocity limit, unless the command
// sum is beyond a limit and error would drive it further into saturation.
// Thus the newly integrated contribution first affects the NEXT tick.
module pi_controller (
    input wire clk,
    input wire rst_n,
    input wire enable,
    input wire clear,
    input wire tick,
    input wire signed [28:0] reference_position,
    input wire signed [12:0] measured_position,
    input wire signed [20:0] feedforward_velocity,
    input wire [19:0] max_velocity,
    input wire [1:0] kp_select,
    input wire [1:0] ki_select,
    output reg signed [20:0] velocity_command,
    output reg saturated
);
    reg signed [20:0] integral;
    wire signed [31:0] reference_wide = {{3{reference_position[28]}}, reference_position};
    wire signed [31:0] measured_wide = {{3{measured_position[12]}}, measured_position, 16'b0};
    wire signed [31:0] error = reference_wide - measured_wide;
    wire signed [31:0] proportional = error >>> (2'd3 - kp_select);
    wire signed [31:0] integral_delta = error >>> (2'd3 - ki_select);
    wire signed [31:0] integral_wide = {{11{integral[20]}}, integral};
    wire signed [31:0] feedforward_wide = {{11{feedforward_velocity[20]}}, feedforward_velocity};
    wire signed [31:0] limit = {12'b0, max_velocity};
    wire signed [31:0] command_sum = feedforward_wide + proportional + integral_wide;
    wire signed [31:0] integral_sum = integral_wide + integral_delta;
    wire windup = ((command_sum > limit) && (error > 0)) ||
                  ((command_sum < -limit) && (error < 0));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            integral <= 21'sd0;
            velocity_command <= 21'sd0;
            saturated <= 1'b0;
        end else if (clear || !enable) begin
            integral <= 21'sd0;
            velocity_command <= 21'sd0;
            saturated <= 1'b0;
        end else if (tick) begin
            saturated <= (command_sum > limit) || (command_sum < -limit);
            if (command_sum > limit)
                velocity_command <= limit[20:0];
            else if (command_sum < -limit)
                velocity_command <= -$signed({1'b0, max_velocity});
            else
                velocity_command <= command_sum[20:0];

            // Always respect a newly reduced limit, even if integration freezes.
            if (windup) begin
                if (integral_wide > limit)
                    integral <= limit[20:0];
                else if (integral_wide < -limit)
                    integral <= -$signed({1'b0, max_velocity});
            end else if (integral_sum > limit)
                integral <= limit[20:0];
            else if (integral_sum < -limit)
                integral <= -$signed({1'b0, max_velocity});
            else
                integral <= integral_sum[20:0];
        end
    end
endmodule

`default_nettype wire
