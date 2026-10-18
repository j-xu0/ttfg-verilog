/*
 * Copyright (c) 2026 Jason Xu
 * SPDX-License-Identifier: Apache-2.0
 */
`default_nettype none

// Discrete trapezoidal reference generator; motor pulse generation is external.
// start is sampled on ANY clock edge when enable && !busy (not only on tick).
// The host must pulse start; holding it high can start another move after done.
// Commands latch signed integer 13-bit positions and unsigned fixed-point
// limits with FRACTION_BITS fractional bits. Position width is 13+FRACTION_BITS;
// velocity has LIMIT_WIDTH magnitude bits plus sign. Default parameters retain
// the original Q8 interface; the integrated controller uses Q16 / 20-bit limits.
// Disable aborts immediately, holds position, clears busy/velocity, no done.
// Zero limits reject a command by pulsing done without moving from start_position.
// Zero distance completes on acceptance. done is one clk pulse, including when
// tick is low. Busy commands and changing limits/positions are ignored.
// Velocity describes the NEXT tick interval: each tick first advances by the
// previous reference_velocity, then chooses the next speed. The first tick
// therefore accelerates without displacement. Completion lands at zero velocity.
// Position never wraps.
// SERIAL_SETUP=0 computes exact stopping distances combinationally with variable
// division/products. SERIAL_SETUP=1 uses bounded serial command setup, then exact
// per-tick stopping recurrences. Both configurations require GF180 area/timing
// assessment; simulation does not establish physical implementation feasibility.
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
    // A sum of up to 2**LIMIT_WIDTH terms, each less than 2**LIMIT_WIDTH.
    // The three-factor intermediate a*n*(n-1) needs three limit widths.
    localparam integer STOP_WIDTH = 3 * LIMIT_WIDTH + 2;
    localparam integer DIST_WIDTH = (POSITION_WIDTH + 1 > LIMIT_WIDTH + 1)
        ? POSITION_WIDTH + 1 : LIMIT_WIDTH + 1;

    reg signed [12+FRACTION_BITS:0] target_q;
    reg direction_negative;
    reg [LIMIT_WIDTH-1:0] velocity_magnitude;
    reg [LIMIT_WIDTH-1:0] velocity_limit;
    reg [LIMIT_WIDTH-1:0] acceleration_limit;

    wire signed [DIST_WIDTH-1:0] position_wide =
        {{(DIST_WIDTH-POSITION_WIDTH){reference_position[POSITION_WIDTH-1]}}, reference_position};
    wire signed [DIST_WIDTH-1:0] target_wide =
        {{(DIST_WIDTH-POSITION_WIDTH){target_q[POSITION_WIDTH-1]}}, target_q};
    wire [DIST_WIDTH-1:0] speed_wide =
        {{(DIST_WIDTH-LIMIT_WIDTH){1'b0}}, velocity_magnitude};
    wire [DIST_WIDTH-1:0] acceleration_wide =
        {{(DIST_WIDTH-LIMIT_WIDTH){1'b0}}, acceleration_limit};
    wire [DIST_WIDTH-1:0] limit_wide =
        {{(DIST_WIDTH-LIMIT_WIDTH){1'b0}}, velocity_limit};
    wire [DIST_WIDTH-1:0] remaining = direction_negative
        ? position_wide - target_wide : target_wide - position_wide;
    wire [DIST_WIDTH-1:0] remaining_after_step = remaining - speed_wide;
    assign reference_velocity = direction_negative
        ? -$signed({1'b0, velocity_magnitude})
        :  $signed({1'b0, velocity_magnitude});

    // Sum v, max(v-a,0), ... exactly. ceil(v/a) terms are nonzero.
    // Wide products preserve all bits through the triple product. Guard zero divisors.
    function [STOP_WIDTH-1:0] stopping_distance;
        input [LIMIT_WIDTH-1:0] speed;
        input [LIMIT_WIDTH-1:0] accel;
        reg [LIMIT_WIDTH:0] numerator;
        reg [STOP_WIDTH-1:0] count;
        reg [STOP_WIDTH-1:0] speed_wide;
        reg [STOP_WIDTH-1:0] accel_wide;
        begin
            if (accel == 0 || speed == 0) begin
                stopping_distance = 0;
            end else begin
                numerator = {1'b0, speed} + {1'b0, accel} - 1'b1;
                count = numerator / accel;
                speed_wide = {{(STOP_WIDTH-LIMIT_WIDTH){1'b0}}, speed};
                accel_wide = {{(STOP_WIDTH-LIMIT_WIDTH){1'b0}}, accel};
                stopping_distance = count * speed_wide
                    - ((accel_wide * count * (count - 1'b1)) >> 1);
            end
        end
    endfunction

    // SERIAL_SETUP prepares for 3*LIMIT_WIDTH+1 clocks after acceptance;
    // the first usable tick is on the following edge (3*LIMIT_WIDTH+2).
    // for bounded shift/add arithmetic. No divider or multiplier is elaborated
    // in this configuration. Ticks during preparation leave the reference still.
    localparam integer SUM_WIDTH = 2*LIMIT_WIDTH+2;
    reg [SUM_WIDTH-1:0] stopping_current;
    wire [SUM_WIDTH-1:0] stopping_limit;
    wire preparing;
    generate if (SERIAL_SETUP != 0) begin : serial
        localparam [2:0] IDLE=0, DIVIDE=1, MUL1=2, SETUP2=3, MUL2=4;
        reg [2:0] state;
        localparam integer COUNT_WIDTH = LIMIT_WIDTH < 2 ? 1 : $clog2(LIMIT_WIDTH+1);
        reg [COUNT_WIDTH-1:0] bit_count;
        reg [LIMIT_WIDTH-1:0] numerator;
        reg [LIMIT_WIDTH-1:0] quotient;
        reg [LIMIT_WIDTH:0] remainder;
        reg [LIMIT_WIDTH-1:0] count;
        reg [SUM_WIDTH-1:0] multiplicand;
        reg [LIMIT_WIDTH-1:0] multiplier;
        reg [SUM_WIDTH-1:0] product;
        reg [SUM_WIDTH-1:0] result;
        wire [LIMIT_WIDTH:0] shifted_remainder =
            {remainder[LIMIT_WIDTH-1:0], numerator[LIMIT_WIDTH-1]};
        wire quotient_bit = shifted_remainder >= {1'b0, acceleration_limit};
        wire [LIMIT_WIDTH-1:0] next_quotient =
            (quotient << 1) | quotient_bit;
        wire [SUM_WIDTH-1:0] next_product = product +
            (multiplier[0] ? multiplicand : {SUM_WIDTH{1'b0}});
        assign preparing = state != IDLE;
        assign stopping_limit = result;
        always @(posedge clk) begin
            if (!rst_n || !enable) begin
                state <= IDLE;
                bit_count <= 0;
                numerator <= 0;
                quotient <= 0;
                remainder <= 0;
                count <= 0;
                multiplicand <= 0;
                multiplier <= 0;
                product <= 0;
                result <= 0;
            end else if (!busy && start && start_position != target_position &&
                         max_velocity != 0 && acceleration != 0) begin
                // ceil(v/a) = floor((v-1)/a)+1, with W-bit numerator.
                numerator <= max_velocity - 1'b1;
                quotient <= 0;
                remainder <= 0;
                bit_count <= LIMIT_WIDTH;
                state <= DIVIDE;
            end else case (state)
                DIVIDE: begin
                    numerator <= numerator << 1;
                    quotient <= next_quotient;
                    remainder <= quotient_bit ?
                        shifted_remainder - {1'b0, acceleration_limit} : shifted_remainder;
                    bit_count <= bit_count - 1;
                    if (bit_count == 1) begin
                        count <= next_quotient + 1'b1;
                        multiplicand <= acceleration_limit;
                        multiplier <= next_quotient;
                        product <= 0;
                        bit_count <= LIMIT_WIDTH;
                        state <= MUL1;
                    end
                end
                MUL1: begin
                    product <= next_product;
                    multiplicand <= multiplicand << 1;
                    multiplier <= multiplier >> 1;
                    bit_count <= bit_count - 1;
                    if (bit_count == 1) state <= SETUP2;
                end
                SETUP2: begin
                    // S(v) = n * (2*v - (n-1)*a) / 2. The product
                    // is even, so division by two is an exact right shift.
                    multiplicand <= ({{(SUM_WIDTH-LIMIT_WIDTH){1'b0}}, velocity_limit} << 1) - product;
                    multiplier <= count;
                    product <= 0;
                    bit_count <= LIMIT_WIDTH;
                    state <= MUL2;
                end
                MUL2: begin
                    product <= next_product;
                    multiplicand <= multiplicand << 1;
                    multiplier <= multiplier >> 1;
                    bit_count <= bit_count - 1;
                    if (bit_count == 1) begin
                        result <= next_product >> 1;
                        state <= IDLE;
                    end
                end
                default: state <= IDLE;
            endcase
        end
    end else begin : combinational_setup
        assign preparing = 1'b0;
        assign stopping_limit = {SUM_WIDTH{1'b0}};
    end endgenerate

    reg [LIMIT_WIDTH:0] accelerated;
    reg [LIMIT_WIDTH-1:0] candidate;
    reg [LIMIT_WIDTH-1:0] next_speed;
    reg [SUM_WIDTH-1:0] next_stopping;
    reg [SUM_WIDTH-1:0] candidate_stopping;
    always @* begin
        accelerated = {1'b0, velocity_magnitude} + {1'b0, acceleration_limit};
        candidate = accelerated > {1'b0, velocity_limit}
            ? velocity_limit : accelerated[LIMIT_WIDTH-1:0];
        if (SERIAL_SETUP != 0) begin
            // Adding a full acceleration interval prepends candidate to the
            // existing stopping sequence. A capped step uses cached S(vmax).
            candidate_stopping = accelerated > {1'b0, velocity_limit}
                ? stopping_limit : stopping_current + candidate;
            if (candidate_stopping <= remaining_after_step) begin
                next_speed = candidate;
                next_stopping = candidate_stopping;
            end else if (stopping_current <= remaining_after_step) begin
                next_speed = velocity_magnitude;
                next_stopping = stopping_current;
            end else if (velocity_magnitude > acceleration_limit) begin
                next_speed = velocity_magnitude - acceleration_limit;
                next_stopping = stopping_current - velocity_magnitude;
            end else begin
                next_speed = 0;
                next_stopping = 0;
            end
        end else begin
            candidate_stopping = 0;
            next_stopping = 0;
            // Exact legacy combinational profile, retained for standalone use.
            if (stopping_distance(candidate, acceleration_limit) <= remaining_after_step)
                next_speed = candidate;
            else if (stopping_distance(velocity_magnitude, acceleration_limit)
                     <= remaining_after_step)
                next_speed = velocity_magnitude;
            else if (velocity_magnitude > acceleration_limit)
                next_speed = velocity_magnitude - acceleration_limit;
            else
                next_speed = 0;
        end

        // At low speed, a fractional final step fits within the acceleration
        // bound and lands exactly; this also handles moves shorter than a tick.
        if (velocity_magnitude <= acceleration_limit &&
            remaining_after_step <= acceleration_wide &&
            remaining_after_step <= limit_wide) begin
            next_speed = remaining_after_step[LIMIT_WIDTH-1:0];
            next_stopping = remaining_after_step;
        end
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            reference_position <= 0;
            target_q <= 0;
            stopping_current <= 0;
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
                stopping_current <= 0;
            end else if (!busy) begin
                if (start) begin
                    reference_position <= {start_position, {FRACTION_BITS{1'b0}}};
                    target_q <= {target_position, {FRACTION_BITS{1'b0}}};
                    direction_negative <= target_position < start_position;
                    velocity_magnitude <= 0;
                    stopping_current <= 0;
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
            end else if (tick && !preparing) begin
                if (direction_negative)
                    reference_position <= position_wide - $signed(speed_wide);
                else
                    reference_position <= position_wide + $signed(speed_wide);
                if (speed_wide == remaining) begin
                    velocity_magnitude <= 0;
                    stopping_current <= 0;
                    busy <= 0;
                    done <= 1;
                end else begin
                    velocity_magnitude <= next_speed;
                    stopping_current <= next_stopping;
                end
            end
        end
    end
endmodule
`default_nettype wire
