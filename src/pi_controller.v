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
// SERIAL=0 publishes on the tick edge. SERIAL=1 captures all inputs on an idle
// tick, raises calculating, and publishes command/saturated together
// within 450 clocks. Ticks while calculating are ignored. Clear/disable/reset
// cancel in-flight work and clear outputs immediately as in parallel mode.
// STABLE_CONFIG=1 omits configuration latches: max_velocity and both gains
// must remain unchanged from the accepted tick until calculating falls.
module pi_controller #(
    parameter integer SERIAL = 0,
    parameter integer STABLE_CONFIG = 0
) (
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
    output reg saturated,
    output wire calculating
);
    reg signed [20:0] integral;
    generate if (SERIAL == 0) begin : parallel
    assign calculating = 1'b0;
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
    end else begin : serial
        // Capture one exact position subtraction; all following arithmetic is
        // streamed through a single three-input bit adder. Error and integral
        // rotate in place. The sampled feedforward register becomes the command
        // result as its input bits are consumed. No wide sum scratch is needed.
        localparam [3:0] IDLE=0, SUM=1, SUM_DONE=2, CLAMP=3,
            FILL_LIMIT=4, COMPARE_START=5, COMPARE=6, FILL_START=7,
            CLAMP_DONE=8, SAMPLE_READY=9, PUBLISH=10;
        // Keep the measured smaller binary GF180 control implementation.
        (* fsm_encoding = "none" *) reg [3:0] state;
        reg [4:0] bit_count;
        reg [1:0] carry, comparison;
        reg [29:0] error_ring;
        reg [20:0] work;
        reg [19:0] saved_limit;
        reg [1:0] saved_kp, saved_ki;
        reg error_negative, integral_negative, feedforward_negative;
        reg integral_phase, overflow, low_sign, negative, clipped;
        reg pending_saturated;
        wire [19:0] limit = STABLE_CONFIG ? max_velocity : saved_limit;
        wire [1:0] kp = STABLE_CONFIG ? kp_select : saved_kp;
        wire [1:0] ki = STABLE_CONFIG ? ki_select : saved_ki;
        wire [1:0] gain = integral_phase ? ki : kp;
        wire error_tap = gain == 0 ? error_ring[3] :
                         gain == 1 ? error_ring[2] :
                         gain == 2 ? error_ring[1] : error_ring[0];
        wire [4:0] last_error_bit = gain == 0 ? 5'd26 :
                                   gain == 1 ? 5'd27 :
                                   gain == 2 ? 5'd28 : 5'd29;
        wire error_bit = bit_count <= last_error_bit ? error_tap : error_negative;
        wire integral_bit = bit_count < 21 ? integral[0] : integral_negative;
        wire ff_bit = integral_phase ? 1'b0 :
                      (bit_count < 21 ? work[0] : feedforward_negative);
        wire [2:0] sum = {2'b0, error_bit} + {2'b0, integral_bit} +
                         {2'b0, ff_bit} + {1'b0, carry};
        wire [20:0] value = integral_phase ? integral : work;
        wire limit_bit = bit_count == 20 ? 1'b0 : limit[bit_count];
        // Compare |value| against limit by subtracting their bit streams.
        // Two's-complement absolute value carries locally; value itself only
        // rotates, preserving its original signed representation.
        wire magnitude_bit = negative ? (~value[0] ^ carry[1]) : value[0];
        wire [1:0] difference = {1'b0, magnitude_bit} + {1'b0, ~limit_bit}
                                + {1'b0, carry[0]};
        wire limit_signed_bit = negative ? (~limit_bit ^ carry[0]) : limit_bit;
        wire signed [13:0] error_integer =
            $signed({reference_position[28], reference_position[28:16]}) -
            $signed({measured_position[12], measured_position});
        assign calculating = state != IDLE;
        wire sample = state == IDLE && tick;
        wire windup = clipped && negative == error_negative;
        wire compare_clipped = difference[1] && (comparison[0] || difference[0]);

        // Only architecturally visible state requires reset or abort gating.
        // Scratch values can continue on an abort edge; the next accepted sample
        // overwrites them before use. Keeping this separate also avoids a reset
        // and enable mux on every bit of the sampled error and work rings.
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                integral <= 0;
                velocity_command <= 0;
                saturated <= 0;
                state <= IDLE;
            end else if (clear || !enable) begin
                integral <= 0;
                velocity_command <= 0;
                saturated <= 0;
                state <= IDLE;
            end else begin
                case (state)
                    IDLE: if (tick) state <= SAMPLE_READY;
                    SAMPLE_READY: state <= SUM;
                    SUM: if (bit_count == 30) state <= SUM_DONE;
                    SUM_DONE: state <= CLAMP;
                    CLAMP: state <= overflow ? FILL_START : COMPARE_START;
                    COMPARE_START: state <= COMPARE;
                    COMPARE: if (bit_count == 20)
                        state <= compare_clipped ? FILL_START : CLAMP_DONE;
                    FILL_START: state <= FILL_LIMIT;
                    FILL_LIMIT: if (bit_count == 20) state <= CLAMP_DONE;
                    CLAMP_DONE: if (!integral_phase)
                        state <= windup ? CLAMP : SUM;
                    else state <= PUBLISH;
                    PUBLISH: begin
                        velocity_command <= work;
                        saturated <= pending_saturated;
                        state <= IDLE;
                    end
                    default: state <= IDLE;
                endcase
                if (state == SUM && bit_count < 21)
                    integral <= {integral_phase ? sum[0] : integral[0], integral[20:1]};
                else if (state == COMPARE && integral_phase)
                    integral <= {integral[0], integral[20:1]};
                else if (state == FILL_LIMIT && integral_phase)
                    integral <= {limit_signed_bit, integral[20:1]};
            end
        end

        always @(posedge clk) begin
            if (sample) begin
                error_ring <= {error_integer, reference_position[15:0]};
                work <= feedforward_velocity;
                saved_limit <= max_velocity;
                saved_kp <= kp_select;
                saved_ki <= ki_select;
                integral_negative <= integral[20];
                feedforward_negative <= feedforward_velocity[20];
                integral_phase <= 0;
                carry <= 0;
                bit_count <= 0;
                overflow <= 0;
            end else begin
                if (state == SAMPLE_READY) error_negative <= error_ring[29];
                if (state == SUM) begin
                    if (bit_count < 30)
                        error_ring <= {error_ring[0], error_ring[29:1]};
                    if (bit_count < 21 && !integral_phase)
                        work <= {sum[0], work[20:1]};
                    if (bit_count == 20) low_sign <= sum[0];
                    if (bit_count > 20 && sum[0] != low_sign) overflow <= 1;
                    carry <= sum[2:1];
                    bit_count <= bit_count + 1'b1;
                    if (bit_count == 30) negative <= sum[0];
                end
                if (state == CLAMP) clipped <= overflow;
                if (state == COMPARE_START) begin
                    comparison <= 0;
                    bit_count <= 0;
                    carry <= 2'b11;
                end
                if (state == COMPARE) begin
                    if (!integral_phase) work <= {work[0], work[20:1]};
                    carry <= {negative && (~value[0] & carry[1]), difference[1]};
                    comparison <= {1'b0, comparison[0] | difference[0]};
                    bit_count <= bit_count + 1'b1;
                    if (bit_count == 20) clipped <= compare_clipped;
                end
                if (state == FILL_START) begin
                    carry <= 1;
                    bit_count <= 0;
                end
                if (state == FILL_LIMIT) begin
                    if (!integral_phase) work <= {limit_signed_bit, work[20:1]};
                    carry <= {1'b0, (~limit_bit & carry[0])};
                    bit_count <= bit_count + 1'b1;
                end
                if (state == CLAMP_DONE && !integral_phase) begin
                    pending_saturated <= clipped;
                    integral_phase <= 1;
                    integral_negative <= integral[20];
                    overflow <= 0;
                    if (windup) negative <= integral[20];
                    else begin
                        carry <= 0;
                        bit_count <= 0;
                    end
                end
            end
        end
    end endgenerate
endmodule

`default_nettype wire
