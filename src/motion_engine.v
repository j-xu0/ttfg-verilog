// SPDX-License-Identifier: Apache-2.0
`default_nettype none
// Bit-serial trapezoidal motion planner. P-only feedback lives in the separate
// pi_p_controller so the planner keeps the whole serial ALU to itself.
// reference_position is Q11 (24 bits: 13 integer + 11 fraction, so pos += vel
// is exact) and reference_velocity is Q11 signed [13:0].
// STABLE_CONFIG=1 requires target/profile stable throughout a move.
// STABLE_CONFIG=0 captures goal/profile at start. start_position is the launch
// origin. Ticks during work are ignored.
module motion_engine #(parameter integer STABLE_CONFIG=0) (
    input wire clk, rst_n, enable, start, tick,
    input wire signed [12:0] start_position, target_position,
    input wire [5:0] plan_code,
    output reg signed [23:0] reference_position,
    output wire signed [13:0] reference_velocity,
    output reg planner_busy, planner_done,
    output wire planner_calculating, calculating,
    output wire [12:0] vmax
);
`include "plan_lut.vh"
    localparam [5:0] IDLE=0, ADVANCE=33, BEGIN_DIST=34, DIST=35,
        BEGIN_ACCEL=40, ACCEL=41, CAP=42, LOAD_LIMIT=43, TRIAL=44,
        TEST_NEW=45, ACCEPT_SPEED=46, CAP_SPEED=47, ACCEPT_ADD=48,
        HOLD_TEST=49, DEC_STOP=50, DEC_SPEED=51, BEGIN_POS=52, POSITION=53;
    localparam [3:0] ZERO=0, SPEED=1, ACCELERATION=2, POSITION_SIGNED=3,
        DISTANCE=4, GOAL=5, LIMIT=6, STOPPING=7, ACCUMULATOR=8;
    (* fsm_encoding = "none" *) reg [5:0] state;
    reg working;
    reg [4:0] bit_index;
    reg [24:0] accumulator;
    reg [3:0] operand_a, operand;
    reg [1:0] carry;
    reg subtract;
    reg [5:0] latched_profile;
    reg signed [12:0] latched_goal;
    wire [5:0] profile = STABLE_CONFIG != 0 ? plan_code : latched_profile;
    wire signed [12:0] goal = STABLE_CONFIG != 0 ? target_position : latched_goal;
    reg negative, source_position_sign;
    reg [12:0] speed;
    reg [23:0] stopping;
    reg capped, grow, finishing;
    always @* begin
        operand_a = ZERO; operand = ZERO; subtract = 0;
        case (state)
            ADVANCE: begin
                operand_a=POSITION_SIGNED; operand=SPEED; subtract=negative;
            end
            DIST: begin
                operand_a=negative ? POSITION_SIGNED : GOAL;
                operand=negative ? GOAL : POSITION_SIGNED; subtract=1;
            end
            ACCEL, ACCEPT_SPEED: begin operand_a=SPEED; operand=ACCELERATION; end
            // acc already holds min(s+a, vmax); adding S(s) yields S(candidate).
            // Holding at the limit keeps S unchanged, so add nothing.
            TRIAL, ACCEPT_ADD: begin
                operand_a=capped && at_limit ? ZERO : ACCUMULATOR;
                operand=STOPPING;
            end
            TEST_NEW: begin operand_a=ACCUMULATOR; operand=DISTANCE; subtract=1; end
            HOLD_TEST: begin operand_a=STOPPING; operand=DISTANCE; subtract=1; end
            DEC_STOP: begin operand_a=STOPPING; operand=SPEED; subtract=1; end
            DEC_SPEED: begin operand_a=SPEED; operand=ACCELERATION; subtract=1; end
            CAP_SPEED, LOAD_LIMIT: operand_a=LIMIT;
            // Within one step of the target the reference snaps exactly onto
            // it (goal + 0) instead of being reconstructed from the residue.
            POSITION: begin operand_a=GOAL; operand=finishing ? ZERO : DISTANCE;
                              subtract=!negative && !finishing; end
        endcase
    end
    // Both velocity and acceleration limits follow the selected motion profile.
    wire [12:0] profile_limit = profile_velocity(profile[2:0]);
    assign vmax = profile_limit;
    wire [10:0] accel = profile_acceleration(profile[5:3]);
    wire signed [12:0] pos_int = reference_position[23:11];
    wire signed [23:0] goal_q11 = {goal, 11'b0};
    // The full position register rotates during every position pass; DIST and
    // POSITION replace it with the remaining distance and restore it again.
    wire position_bit = reference_position[0];
    wire position_signed_bit = bit_index < 24 ? reference_position[0] :
                               source_position_sign;
    wire goal_bit = bit_index < 24 ? goal_q11[bit_index] : goal_q11[23];
    wire [8:0] source_bits = {
        accumulator[0],
        bit_index < 24 ? stopping[0] : 1'b0,
        bit_index < 13 ? profile_limit[bit_index] : 1'b0,
        goal_bit,
        bit_index < 24 ? position_bit : 1'b0,
        position_signed_bit,
        bit_index < 11 ? accel[bit_index] : 1'b0,
        bit_index < 13 ? speed[0] : 1'b0,
        1'b0};

    wire signed [13:0] speed_signed = negative ?
        -$signed({1'b0, speed}) : $signed({1'b0, speed});
    assign reference_velocity = speed_signed;
    assign calculating = state != IDLE || working;
    assign planner_calculating = calculating;

    // Holding at the profile limit must not re-add the cruise term: only a
    // genuine speed increase grows the brake sum.
    wire at_limit = !grow;
    wire write_position = state == ADVANCE || state == DIST || state == POSITION;
    wire write_speed = state == ACCEPT_SPEED || state == DEC_SPEED ||
        state == CAP_SPEED;
    wire write_stopping = state == ACCEPT_ADD || state == DEC_STOP;
    wire position_operand = operand_a == POSITION_SIGNED ||
        operand == POSITION_SIGNED || operand_a == DISTANCE || operand == DISTANCE;
    wire stopping_operand = operand_a == STOPPING || operand == STOPPING;

    task operation;
        input sub;
        input [5:0] next_state;
        begin
            source_position_sign <= reference_position[23]; carry <= {1'b0,sub};
            bit_index <= 0; state <= next_state; working <= 1;
        end
    endtask

    wire alu_a = source_bits[operand_a];
    wire alu_b = source_bits[operand] ^ subtract;
    wire [2:0] alu_sum = {2'b0,alu_a}+{2'b0,alu_b}+{2'b0,carry[0]};

    task begin_planner_advance;
        begin
            finishing <= 0;
            carry <= {1'b0, negative}; bit_index <= 0; state <= ADVANCE;
            working <= 1;
        end
    endtask

    always @(posedge clk) begin
        if (!rst_n) begin
            reference_position <= 0; speed <= 0; planner_busy <= 0; planner_done <= 0;
            latched_profile <= 0; latched_goal <= 0; negative <= 0;
            source_position_sign <= 0; stopping <= 0;
            capped <= 0; grow <= 0; finishing <= 0;
            state <= IDLE; working <= 0; accumulator <= 0;
        end else begin
            planner_done <= 0;
            if (!enable) begin
                planner_busy <= 0; speed <= 0; stopping <= 0;
                state <= IDLE; working <= 0; accumulator <= 0;
            end else if (start && !planner_busy && state == IDLE) begin
                reference_position <= {start_position,11'b0};
                latched_goal <= target_position; negative <= target_position < start_position;
                latched_profile <= plan_code; speed <= 0; stopping <= 0;
                grow <= 0; accumulator <= 0; working <= 0;
                planner_busy <= start_position != target_position;
                planner_done <= start_position == target_position;
            end else if (working) begin
                if ((operand_a == SPEED || operand == SPEED || write_speed) && bit_index < 13)
                    speed <= {write_speed ? alu_sum[0] : speed[0],speed[12:1]};
                if ((write_position || position_operand) && bit_index < 24)
                    reference_position <= {write_position ? alu_sum[0] : position_bit,
                                           reference_position[23:1]};
                if ((write_stopping || stopping_operand) && bit_index < 24)
                    stopping <= {write_stopping ? alu_sum[0] : stopping[0],
                                 stopping[23:1]};
                accumulator <= {alu_sum[0],accumulator[24:1]};
                carry <= {1'b0,alu_sum[1]}; bit_index <= bit_index + 1'b1;
                if (bit_index == 24) working <= 0;
            end else case (state)
                IDLE: if (tick && planner_busy) begin_planner_advance();
                ADVANCE: state <= BEGIN_DIST;
                BEGIN_DIST: operation(1'b1,DIST);
                DIST: begin
                    if (reference_position[23:11] == 0) begin
                        speed <= 0; stopping <= 0; finishing <= 1; state <= BEGIN_POS;
                    end else state <= BEGIN_ACCEL;
                end
                BEGIN_ACCEL: operation(1'b0,ACCEL);
                ACCEL: state <= CAP;
                // Decide against min(s+a, vmax): the accelerate candidate's
                // speed is left in the accumulator for the brake sum.
                CAP: begin
                    capped <= accumulator[13:0] >= {1'b0, profile_limit};
                    grow <= accumulator[13:0] < {1'b0, profile_limit} || speed < profile_limit;
                    if (accumulator[13:0] >= {1'b0, profile_limit}) operation(1'b0,LOAD_LIMIT);
                    else operation(1'b0,TRIAL);
                end
                LOAD_LIMIT: operation(1'b0,TRIAL);
                TRIAL: operation(1'b1,TEST_NEW);
                // Accept when S(candidate) <= remaining distance.
                TEST_NEW: if (!carry[0] || accumulator[23:0] == 0) begin
                    if (capped) operation(1'b0,CAP_SPEED);
                    else operation(1'b0,ACCEPT_SPEED);
                end else operation(1'b1,HOLD_TEST);
                // Hold when S(s) already fits in the remaining distance.
                HOLD_TEST: if (!carry[0] || accumulator[23:0] == 0) state <= BEGIN_POS;
                           else operation(1'b1,DEC_STOP);
                CAP_SPEED: operation(1'b0,ACCEPT_ADD);
                ACCEPT_SPEED: operation(1'b0,ACCEPT_ADD);
                ACCEPT_ADD: state <= BEGIN_POS;
                DEC_STOP: operation(1'b1,DEC_SPEED);
                DEC_SPEED: begin
                    if (!carry[0]) begin speed <= 0; stopping <= 0; end
                    state <= BEGIN_POS;
                end
                BEGIN_POS: operation(!negative && !finishing,POSITION);
                POSITION: begin
                    if (finishing) begin planner_busy <= 0; planner_done <= 1; end
                    state <= IDLE;
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule
`default_nettype wire
