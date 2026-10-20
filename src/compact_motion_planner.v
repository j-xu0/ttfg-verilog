// SPDX-License-Identifier: Apache-2.0
`default_nettype none
// Q16 implementation for the 64 documented motion profiles. While calculating,
// reference_position is arithmetic workspace and MUST NOT be sampled. A tick
// starts calculation; outputs are valid again when calculating falls. Ticks
// during calculation are ignored. Allow 700 clocks between ticks (top: 10000).
// Abort cancels work and clears velocity; next command reinitializes position.
// Every chosen S(v,a)<=remaining<=8191*65536<2**29. ROM values above
// every reachable distance are replaced with 2**29-1 (always rejected).
// Candidate S(v+a)=S(v)+v+a<2**29+661914<2**30, so a 30-bit ALU
// preserves every intermediate. Signed position arithmetic also fits 30 bits.
// STABLE_CONFIG=1 uses live target/profile to save latches; the owner MUST hold
// those inputs while busy. Default configuration latches arbitrary host changes.
module compact_motion_planner #(parameter integer STABLE_CONFIG = 0) (
    input wire clk, rst_n, enable, start, tick,
    input wire signed [12:0] start_position, target_position,
    input wire [5:0] plan_code,
    output reg signed [28:0] reference_position,
    output wire signed [20:0] reference_velocity,
    output reg busy, done,
    output wire calculating
);
`include "plan_lut.vh"
    localparam [4:0] IDLE=0, WORK=1, ADVANCE=2, BEGIN_DIST=3, DIST=4,
        TAIL_SPEED=5, TAIL_ACCEL=6, TAIL_LIMIT=7, BEGIN_ACCEL=8, ACCEL=9,
        CAP=10, STOP_ADD_A=11, STOP=12, TEST_NEW=13, TEST_OLD=14,
        ACCEPT_ADD=15, ACCEPT_STOP=16, ACCEPT_SPEED=17, DEC_STOP=18,
        DEC_SPEED=19, BEGIN_POS=20, POSITION=21, VELOCITY=22;
    localparam [3:0] ZERO=0, SPEED=1, ACCELERATION=2, POSITION_SIGNED=3,
        DISTANCE=4, GOAL=5, LIMIT=6;
    (* fsm_encoding = "none" *) reg [4:0] state, resume;
    reg [4:0] bit_index;
    reg [29:0] accumulator;
    reg [3:0] operand;
    reg carry, subtract;
    reg [5:0] latched_profile;
    reg signed [12:0] latched_goal;
    wire [5:0] profile = STABLE_CONFIG != 0 ? plan_code : latched_profile;
    wire signed [12:0] goal = STABLE_CONFIG != 0 ? target_position : latched_goal;
    reg negative;
    reg [19:0] speed;
    reg [28:0] stopping;
    reg capped, finishing;
    wire [19:0] vmax = profile_velocity(profile[2:0]);
    wire [12:0] accel = profile_acceleration(profile[5:3]);
    wire [28:0] max_stop = profile_stopping(profile);
    wire [29:0] position_extended = {reference_position[28],reference_position};
    wire [29:0] goal_extended = {goal[12],goal,16'b0};
    reg [29:0] operand_value;
    always @* begin
        case (operand)
            SPEED: operand_value = {10'b0,speed};
            ACCELERATION: operand_value = {17'b0,accel};
            POSITION_SIGNED: operand_value = position_extended;
            DISTANCE: operand_value = {1'b0,reference_position};
            GOAL: operand_value = goal_extended;
            LIMIT: operand_value = {10'b0,vmax};
            default: operand_value = 0;
        endcase
    end
    wire [1:0] sum_bit = {1'b0,accumulator[0]} +
        {1'b0,(operand_value[bit_index] ^ subtract)} + carry;
    assign reference_velocity = $signed(accumulator[20:0]);
    assign calculating = state != IDLE;

    // One bit adder and one accumulator serve all arithmetic and comparisons.
    // B operands remain in their original registers, avoiding a second 30-bit
    // scratch register. A full 30-cycle operation restores normal bit ordering.
    task operation;
        input [29:0] a;
        input [3:0] b;
        input sub;
        input [4:0] next_state;
        begin
            accumulator <= a; operand <= b; carry <= sub; subtract <= sub;
            bit_index <= 0; resume <= next_state; state <= WORK;
        end
    endtask

    always @(posedge clk) begin
        if (!rst_n) begin
            reference_position <= 0; speed <= 0; busy <= 0; done <= 0;
            latched_profile <= 0; latched_goal <= 0; negative <= 0; stopping <= 0;
            capped <= 0; finishing <= 0;
            state <= IDLE; resume <= IDLE; bit_index <= 0;
            accumulator <= 0; operand <= ZERO; carry <= 0; subtract <= 0;
        end else begin
            done <= 0;
            if (!enable) begin
                busy <= 0; speed <= 0; stopping <= 0; state <= IDLE; accumulator <= 0;
            end else if (!busy) begin
                if (start) begin
                    reference_position <= {start_position,16'b0};
                    latched_goal <= target_position; negative <= target_position < start_position;
                    latched_profile <= plan_code; speed <= 0; stopping <= 0; state <= IDLE; accumulator <= 0;
                    busy <= start_position != target_position;
                    done <= start_position == target_position;
                end
            end else case (state)
                IDLE: if (tick) begin
                    finishing <= 0;
                    operation(position_extended,SPEED,negative,ADVANCE);
                end
                WORK: begin
                    accumulator <= {sum_bit[0],accumulator[29:1]};
                    carry <= sum_bit[1];
                    bit_index <= bit_index + 1'b1;
                    if (bit_index == 29) state <= resume;
                end
                ADVANCE: begin reference_position <= accumulator[28:0]; state <= BEGIN_DIST; end
                BEGIN_DIST: if (negative) operation(position_extended,GOAL,1'b1,DIST);
                            else operation(goal_extended,POSITION_SIGNED,1'b1,DIST);
                DIST: begin
                    // Reuse the output register for remaining distance. Final
                    // position is reconstructed from goal +/- this distance.
                    reference_position <= accumulator[28:0];
                    if (accumulator == 0) begin
                        speed <= 0; stopping <= 0; finishing <= 1; state <= BEGIN_POS;
                    end else operation({17'b0,accel},SPEED,1'b1,TAIL_SPEED);
                end
                TAIL_SPEED: if (carry) operation({17'b0,accel},DISTANCE,1'b1,TAIL_ACCEL);
                            else state <= BEGIN_ACCEL;
                TAIL_ACCEL: if (carry) operation({10'b0,vmax},DISTANCE,1'b1,TAIL_LIMIT);
                            else state <= BEGIN_ACCEL;
                TAIL_LIMIT: if (carry) begin
                    speed <= reference_position[19:0];
                    stopping <= {1'b0,reference_position}; state <= BEGIN_POS;
                end else state <= BEGIN_ACCEL;
                BEGIN_ACCEL: operation({10'b0,speed},ACCELERATION,1'b0,ACCEL);
                ACCEL: operation(accumulator,LIMIT,1'b1,CAP);
                CAP: begin
                    capped <= carry;
                    if (carry) operation(max_stop,ZERO,1'b0,STOP);
                    else operation(stopping,SPEED,1'b0,STOP_ADD_A);
                end
                STOP_ADD_A: operation(accumulator,ACCELERATION,1'b0,STOP);
                STOP: operation(accumulator,DISTANCE,1'b1,TEST_NEW);
                TEST_NEW: if (!carry || accumulator == 0) begin
                    if (capped) begin stopping <= max_stop; speed <= vmax; state <= BEGIN_POS; end
                    else operation(stopping,SPEED,1'b0,ACCEPT_ADD);
                end else operation(stopping,DISTANCE,1'b1,TEST_OLD);
                TEST_OLD: if (!carry || accumulator == 0) state <= BEGIN_POS;
                          else operation(stopping,SPEED,1'b1,DEC_STOP);
                ACCEPT_ADD: operation(accumulator,ACCELERATION,1'b0,ACCEPT_STOP);
                ACCEPT_STOP: begin
                    stopping <= accumulator;
                    operation({10'b0,speed},ACCELERATION,1'b0,ACCEPT_SPEED);
                end
                ACCEPT_SPEED: begin speed <= accumulator[19:0]; state <= BEGIN_POS; end
                DEC_STOP: begin
                    stopping <= accumulator;
                    operation({10'b0,speed},ACCELERATION,1'b1,DEC_SPEED);
                end
                DEC_SPEED: begin
                    if (carry) speed <= accumulator[19:0];
                    else begin speed <= 0; stopping <= 0; end
                    state <= BEGIN_POS;
                end
                BEGIN_POS: operation(goal_extended,DISTANCE,!negative,POSITION);
                POSITION: begin
                    reference_position <= accumulator[28:0];
                    // Reuse the idle accumulator as signed velocity output,
                    // replacing a separate full-width combinational negation.
                    operation(30'b0,SPEED,negative,VELOCITY);
                end
                VELOCITY: begin
                    if (finishing) begin busy <= 0; done <= 1; end
                    state <= IDLE;
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule
`default_nettype wire
