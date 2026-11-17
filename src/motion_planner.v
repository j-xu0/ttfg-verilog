/*
 * Copyright (c) 2026 Jason Xu
 * SPDX-License-Identifier: Apache-2.0
 */
`default_nettype none

// Simple mirror-rule reference generator; motor pulse generation is external.
// start is sampled on ANY clock edge when enable && !busy (not only on tick).
// The host must pulse start; holding it high can start another move after done.
// Commands latch signed integer 13-bit positions and unsigned fixed-point
// limits with FRACTION_BITS fractional bits. Position width is 13+FRACTION_BITS;
// velocity has LIMIT_WIDTH magnitude bits plus sign. Default parameters retain
// the original Q8 interface; the integrated controller uses a separate Q11 engine.
// Disable aborts immediately, holds position, clears busy/velocity, no done.
// Zero limits reject a command by pulsing done without moving from start_position.
// Zero distance completes on acceptance. done is one clk pulse, including when
// tick is low. Busy commands and changing limits/positions are ignored.
// Velocity describes the NEXT tick interval: each tick first advances by the
// previous reference_velocity, then chooses the next speed. The first tick
// therefore accelerates without displacement. Completion lands at zero velocity.
// Position never wraps.
//
// Speed-up and slow-down use the same rate, so every move is symmetric: the
// distance spent speeding up is mirrored when slowing down. Each tick the
// planner only checks whether there is still room to mirror a faster step
// (two additions and a comparison on the travelled distance). No division or
// multiplication is needed anywhere; stopping-distance math was removed.
// SERIAL_SETUP is retained for interface compatibility and ignored.
module motion_planner #(
    parameter integer FRACTION_BITS = 8,
    parameter integer LIMIT_WIDTH = 16,
    parameter integer SERIAL_SETUP = 0
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick,
    input  wire               enable,
    input  wire               start,
    input  wire signed [12:0] start_position,
    input  wire signed [12:0] target_position,
    input  wire        [LIMIT_WIDTH-1:0] max_velocity,
    input  wire        [LIMIT_WIDTH-1:0] acceleration,
    output reg  signed [12+FRACTION_BITS:0] reference_position,
    output wire signed [LIMIT_WIDTH:0] reference_velocity,
    output reg                busy,
    output reg                done
);
    localparam integer POSITION_WIDTH = 13 + FRACTION_BITS;
    localparam integer DIST_WIDTH = (POSITION_WIDTH + 1 > LIMIT_WIDTH + 1)
        ? POSITION_WIDTH + 1 : LIMIT_WIDTH + 1;

    // No multi-cycle setup; kept at zero for test compatibility.
    wire preparing = 1'b0;

    reg direction_negative;
    reg [LIMIT_WIDTH-1:0] velocity_magnitude;
    reg [LIMIT_WIDTH-1:0] velocity_limit;
    reg [LIMIT_WIDTH-1:0] acceleration_limit;
    reg [DIST_WIDTH-1:0] total_distance;
    reg [DIST_WIDTH-1:0] traveled;
    reg [DIST_WIDTH-1:0] turn_distance;
    reg [1:0] phase;
    localparam [1:0] ACCEL = 0, CRUISE = 1, DECEL = 2;

    // Pre-move distance left; the tick lands exactly when speed covers it.
    wire [DIST_WIDTH-1:0] remaining = total_distance - traveled;
    // Post-move values: every choice below is based on where this tick lands,
    // matching move-then-update semantics (position moves first, then speed).
    wire [DIST_WIDTH-1:0] advanced = traveled
        + {{(DIST_WIDTH-LIMIT_WIDTH){1'b0}}, velocity_magnitude};
    wire [DIST_WIDTH-1:0] remaining_after = total_distance - advanced;
    assign reference_velocity = direction_negative
        ? -$signed({1'b0, velocity_magnitude})
        :  $signed({1'b0, velocity_magnitude});

    // Faster-step candidate, capped at the latched speed limit.
    wire [LIMIT_WIDTH:0] sped_up = {1'b0, velocity_magnitude}
        + {1'b0, acceleration_limit};
    wire [LIMIT_WIDTH-1:0] faster =
        sped_up > {1'b0, velocity_limit}
            ? velocity_limit : sped_up[LIMIT_WIDTH-1:0];

    // Room-to-mirror check: after taking the faster step, the mirrored
    // slow-down must still fit. Only additions and comparisons.
    wire [DIST_WIDTH:0] projected = advanced
        + {{(DIST_WIDTH+1-LIMIT_WIDTH){1'b0}}, faster};
    wire [DIST_WIDTH+1:0] doubled = {1'b0, projected} + {1'b0, projected};
    wire room_for_faster = (projected <= {1'b0, total_distance})
        && (doubled <= {{2{1'b0}}, total_distance});

    wire [DIST_WIDTH-1:0] accel_wide =
        {{(DIST_WIDTH-LIMIT_WIDTH){1'b0}}, acceleration_limit};
    wire [DIST_WIDTH-1:0] limit_wide =
        {{(DIST_WIDTH-LIMIT_WIDTH){1'b0}}, velocity_limit};
    wire final_step = (velocity_magnitude <= acceleration_limit)
        && (remaining_after <= accel_wide) && (remaining_after <= limit_wide);

    reg [LIMIT_WIDTH-1:0] next_speed;
    reg [DIST_WIDTH-1:0] next_turn;
    reg [1:0] next_phase;
    always @* begin
        next_speed = velocity_magnitude;
        next_turn = turn_distance;
        next_phase = phase;
        if (phase == ACCEL) begin
            if (velocity_magnitude == velocity_limit) begin
                // Top speed reached: remember how far speeding up took.
                next_turn = advanced;
                next_phase = CRUISE;
            end else if (room_for_faster) begin
                next_speed = faster;
            end else begin
                // Past halfway: mirror the distance covered so far.
                next_turn = advanced;
                next_phase = DECEL;
            end
        end
        if (next_phase == CRUISE) begin
            if (advanced >= total_distance - next_turn)
                next_phase = DECEL;
            else
                next_speed = velocity_limit;
        end
        if (next_phase == DECEL) begin
            if (final_step)
                next_speed = remaining_after[LIMIT_WIDTH-1:0];
            else if (velocity_magnitude > acceleration_limit)
                next_speed = velocity_magnitude - acceleration_limit;
            // Otherwise hold the low speed; the guards below land exactly.
        end
        // Guards: always make progress, never step past the target.
        // (Slices of remaining_after are exact here: either guard implies
        // it fits in LIMIT_WIDTH bits.)
        if (next_speed == {LIMIT_WIDTH{1'b0}} && remaining_after != 0) begin
            next_speed = acceleration_limit;
            if (next_speed > velocity_limit)
                next_speed = velocity_limit;
            if ({{(DIST_WIDTH-LIMIT_WIDTH){1'b0}}, next_speed} > remaining_after)
                next_speed = remaining_after[LIMIT_WIDTH-1:0];
        end else if ({{(DIST_WIDTH-LIMIT_WIDTH){1'b0}}, next_speed} > remaining_after) begin
            next_speed = remaining_after[LIMIT_WIDTH-1:0];
        end
    end

    wire [POSITION_WIDTH:0] position_ext =
        {reference_position[POSITION_WIDTH-1], reference_position};
    wire [POSITION_WIDTH:0] step_ext =
        {{(POSITION_WIDTH+1-LIMIT_WIDTH){1'b0}}, velocity_magnitude};
    wire [POSITION_WIDTH:0] stepped = direction_negative
        ? position_ext - step_ext : position_ext + step_ext;

    // Shifted like the reference: one integer step is 2**FRACTION_BITS units.
    wire signed [POSITION_WIDTH:0] start_q =
        {start_position[12], start_position, {FRACTION_BITS{1'b0}}};
    wire signed [POSITION_WIDTH:0] goal_q =
        {target_position[12], target_position, {FRACTION_BITS{1'b0}}};
    // The command's own direction (not the registered one, which still
    // holds the previous move while start is sampled).
    wire start_negative = target_position < start_position;
    wire [DIST_WIDTH-1:0] span = start_negative
        ? $unsigned(start_q - goal_q) : $unsigned(goal_q - start_q);

    always @(posedge clk) begin
        if (!rst_n) begin
            reference_position <= 0;
            direction_negative <= 0;
            velocity_magnitude <= 0;
            velocity_limit <= 0;
            acceleration_limit <= 0;
            total_distance <= 0;
            traveled <= 0;
            turn_distance <= 0;
            phase <= ACCEL;
            busy <= 0;
            done <= 0;
        end else begin
            done <= 0;
            if (!enable) begin
                busy <= 0;
                velocity_magnitude <= 0;
                traveled <= 0;
                turn_distance <= 0;
                phase <= ACCEL;
            end else if (!busy) begin
                if (start) begin
                    reference_position <= {start_position, {FRACTION_BITS{1'b0}}};
                    direction_negative <= start_negative;
                    velocity_magnitude <= 0;
                    traveled <= 0;
                    turn_distance <= 0;
                    phase <= ACCEL;
                    velocity_limit <= max_velocity;
                    acceleration_limit <= acceleration;
                    total_distance <= span;
                    if (start_position == target_position ||
                        max_velocity == 0 || acceleration == 0) begin
                        busy <= 0;
                        done <= 1;
                    end else begin
                        busy <= 1;
                    end
                end
            end else if (tick) begin
                reference_position <= stepped[POSITION_WIDTH-1:0];
                traveled <= advanced;
                if (velocity_magnitude == remaining) begin
                    velocity_magnitude <= 0;
                    busy <= 0;
                    done <= 1;
                end else begin
                    velocity_magnitude <= next_speed;
                    turn_distance <= next_turn;
                    phase <= next_phase;
                end
            end
        end
    end
endmodule
`default_nettype wire
