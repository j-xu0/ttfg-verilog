/*
 * Copyright (c) 2026 Jason Xu
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 * Copyright (c) 2026 Jason Xu
 * SPDX-License-Identifier: Apache-2.0
 */
`default_nettype none

// Standalone discrete trapezoidal reference generator; no motor pin connection.
// start is sampled on ANY clock edge when enable && !busy (not only on tick).
// The host must pulse start; holding it high can start another move after done.
// Commands latch signed integer 13-bit start/target positions and unsigned Q8
// limits: max_velocity / 256 position units per tick, acceleration / 256
// position units per tick squared. The encoder-to-position ratio is external.
// reference_position is signed Q13.8 (21 bits), reference_velocity signed Q9.8
// (17 bits); limits range 1/256 .. 255+255/256. Only tick advances a move.
// Disable aborts immediately, holds position, clears busy/velocity, no done.
// Zero limits reject a command by pulsing done without moving from start_position.
// Zero distance completes on acceptance. done is one clk pulse, including when
// tick is low. Busy commands and changing limits/positions are ignored.
// Velocity describes the NEXT tick interval: each tick first advances by the
// previous reference_velocity, then chooses the next speed. The first tick
// therefore accelerates without displacement. Completion lands at zero velocity.
// Position never wraps.
// Exact discrete stopping arithmetic uses variable division/products: synthesizable
// but area/timing require separate GF180 assessment before top-level integration.
module motion_planner (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick,
    input  wire               enable,
    input  wire               start,
    input  wire signed [12:0] start_position,
    input  wire signed [12:0] target_position,
    input  wire        [15:0] max_velocity,
    input  wire        [15:0] acceleration,
    output reg  signed [20:0] reference_position,
    output wire signed [16:0] reference_velocity,
    output reg                busy,
    output reg                done
);
    reg signed [20:0] target_q;
    reg direction_negative;
    reg [15:0] velocity_magnitude;
    reg [15:0] velocity_limit;
    reg [15:0] acceleration_limit;

    wire signed [21:0] position_wide = {reference_position[20], reference_position};
    wire signed [21:0] target_wide = {target_q[20], target_q};
    wire [21:0] remaining = direction_negative
        ? position_wide - target_wide : target_wide - position_wide;
    wire [21:0] remaining_after_step = remaining - {6'b0, velocity_magnitude};
    assign reference_velocity = direction_negative
        ? -$signed({1'b0, velocity_magnitude})
        :  $signed({1'b0, velocity_magnitude});

    // Sum v, max(v-a,0), ... exactly. ceil(v/a) terms are nonzero.
    // 48-bit products bound every intermediate even at the full 16-bit limits;
    // the actual stopping distance fits 32 bits. Guard division when idle/reset.
    function [47:0] stopping_distance;
        input [15:0] speed;
        input [15:0] accel;
        reg [16:0] numerator;
        reg [47:0] count;
        reg [47:0] speed_wide;
        reg [47:0] accel_wide;
        begin
            if (accel == 0 || speed == 0) begin
                stopping_distance = 0;
            end else begin
                numerator = {1'b0, speed} + {1'b0, accel} - 17'd1;
                count = numerator / accel;
                speed_wide = {32'b0, speed};
                accel_wide = {32'b0, accel};
                stopping_distance = count * speed_wide
                    - ((accel_wide * count * (count - 48'd1)) >> 1);
            end
        end
    endfunction

    reg [16:0] accelerated;
    reg [15:0] candidate;
    reg [15:0] next_speed;
    always @* begin
        accelerated = {1'b0, velocity_magnitude} + {1'b0, acceleration_limit};
        candidate = accelerated > {1'b0, velocity_limit}
            ? velocity_limit : accelerated[15:0];
        // Accelerate only if that speed can still stop at/before the target.
        // Otherwise cruise if safe, or decelerate by the configured limit.
        if (stopping_distance(candidate, acceleration_limit) <= {26'b0, remaining_after_step})
            next_speed = candidate;
        else if (stopping_distance(velocity_magnitude, acceleration_limit)
                 <= {26'b0, remaining_after_step})
            next_speed = velocity_magnitude;
        else if (velocity_magnitude > acceleration_limit)
            next_speed = velocity_magnitude - acceleration_limit;
        else
            next_speed = 0;

        // At low speed, a fractional final step fits within the acceleration
        // bound and lands exactly; this also handles moves shorter than a tick.
        if (velocity_magnitude <= acceleration_limit &&
            remaining_after_step <= {6'b0, acceleration_limit} &&
            remaining_after_step <= {6'b0, velocity_limit})
            next_speed = remaining_after_step[15:0];
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            reference_position <= 0;
            target_q <= 0;
            direction_negative <= 0;
            velocity_magnitude <= 0;
            velocity_limit <= 0;
            acceleration_limit <= 0;
            busy <= 0;
            done <= 0;
        end else begin
            done <= 0;
            if (!enable) begin
                busy <= 0;
                velocity_magnitude <= 0;
            end else if (!busy) begin
                if (start) begin
                    reference_position <= {start_position, 8'b0};
                    target_q <= {target_position, 8'b0};
                    direction_negative <= target_position < start_position;
                    velocity_magnitude <= 0;
                    velocity_limit <= max_velocity;
                    acceleration_limit <= acceleration;
                    if (start_position == target_position ||
                        max_velocity == 0 || acceleration == 0) begin
                        busy <= 0;
                        done <= 1;
                    end else begin
                        busy <= 1;
                    end
                end
            end else if (tick) begin
                if (direction_negative)
                    reference_position <= position_wide - $signed({6'b0, velocity_magnitude});
                else
                    reference_position <= position_wide + $signed({6'b0, velocity_magnitude});
                if ({6'b0, velocity_magnitude} == remaining) begin
                    velocity_magnitude <= 0;
                    busy <= 0;
                    done <= 1;
                end else begin
                    velocity_magnitude <= next_speed;
                end
            end
        end
    end
endmodule
`default_nettype wire
