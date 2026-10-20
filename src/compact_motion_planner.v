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
    localparam [4:0] IDLE=0, ADVANCE=2, BEGIN_DIST=3, DIST=4,
        TAIL_SPEED=5, TAIL_ACCEL=6, TAIL_LIMIT=7, BEGIN_ACCEL=8, ACCEL=9,
        CAP=10, STOP_ADD_A=11, STOP=12, TEST_NEW=13, TEST_OLD=14,
        ACCEPT_ADD=15, ACCEPT_STOP=16, ACCEPT_SPEED=17, DEC_STOP=18,
        DEC_SPEED=19, BEGIN_POS=20, POSITION=21, VELOCITY=22,
        TAIL_COPY=23, CAP_SPEED=24, CAP_STOP=25;
    localparam [3:0] ZERO=0, SPEED=1, ACCELERATION=2, POSITION_SIGNED=3,
        DISTANCE=4, GOAL=5, LIMIT=6, STOPPING=7, MAX_STOP=8, ACCUMULATOR=9;
    (* fsm_encoding = "none" *) reg [4:0] state;
    reg working;
    reg [4:0] bit_index;
    reg [29:0] accumulator;
    reg [3:0] operand_a, operand;
    reg carry, subtract;
    reg [5:0] latched_profile;
    reg signed [12:0] latched_goal;
    wire [5:0] profile = STABLE_CONFIG != 0 ? plan_code : latched_profile;
    wire signed [12:0] goal = STABLE_CONFIG != 0 ? target_position : latched_goal;
    reg negative, source_position_sign;
    reg [19:0] speed;
    reg [28:0] stopping;
    reg capped, finishing;
    always @* begin
        operand_a = ZERO; operand = ZERO; subtract = 0;
        case (state)
            ADVANCE: begin operand_a=POSITION_SIGNED; operand=SPEED; subtract=negative; end
            DIST: begin
                operand_a=negative ? POSITION_SIGNED : GOAL;
                operand=negative ? GOAL : POSITION_SIGNED; subtract=1;
            end
            TAIL_SPEED: begin operand_a=ACCELERATION; operand=SPEED; subtract=1; end
            TAIL_ACCEL: begin operand_a=ACCELERATION; operand=DISTANCE; subtract=1; end
            TAIL_LIMIT: begin operand_a=LIMIT; operand=DISTANCE; subtract=1; end
            ACCEL, ACCEPT_SPEED: begin operand_a=SPEED; operand=ACCELERATION; end
            CAP: begin operand_a=ACCUMULATOR; operand=LIMIT; subtract=1; end
            STOP_ADD_A, ACCEPT_ADD: begin operand_a=STOPPING; operand=SPEED; end
            STOP: begin
                operand_a=capped ? MAX_STOP : ACCUMULATOR;
                operand=capped ? ZERO : ACCELERATION;
            end
            TEST_NEW: begin operand_a=ACCUMULATOR; operand=DISTANCE; subtract=1; end
            TEST_OLD: begin operand_a=STOPPING; operand=DISTANCE; subtract=1; end
            ACCEPT_STOP: begin operand_a=ACCUMULATOR; operand=ACCELERATION; end
            DEC_STOP: begin operand_a=STOPPING; operand=SPEED; subtract=1; end
            DEC_SPEED: begin operand_a=SPEED; operand=ACCELERATION; subtract=1; end
            TAIL_COPY: operand_a=DISTANCE;
            CAP_SPEED: operand_a=LIMIT;
            CAP_STOP: operand_a=MAX_STOP;
            POSITION: begin operand_a=GOAL; operand=DISTANCE; subtract=!negative; end
            VELOCITY: begin operand=SPEED; subtract=negative; end
        endcase
    end
    wire [19:0] vmax = profile_velocity(profile[2:0]);
    wire [12:0] accel = profile_acceleration(profile[5:3]);
    wire [28:0] max_stop = profile_stopping(profile);
    wire [29:0] goal_extended = {goal[12],goal,16'b0};
    wire [9:0] source_bits = {
        accumulator[0],
        bit_index < 29 ? max_stop[bit_index] : 1'b0,
        bit_index < 29 ? stopping[0] : 1'b0,
        bit_index < 20 ? vmax[bit_index] : 1'b0,
        goal_extended[bit_index],
        bit_index < 29 ? reference_position[0] : 1'b0,
        bit_index < 29 ? reference_position[0] : source_position_sign,
        bit_index < 13 ? accel[bit_index] : 1'b0,
        bit_index < 20 ? speed[0] : 1'b0,
        1'b0};
    wire [1:0] sum_bit = {1'b0,source_bits[operand_a]} +
        {1'b0,(source_bits[operand] ^ subtract)} + carry;
    assign reference_velocity = $signed(accumulator[20:0]);
    assign calculating = state != IDLE || working;

    wire write_position = state == ADVANCE || state == DIST || state == POSITION;
    wire write_speed = state == ACCEPT_SPEED || state == DEC_SPEED ||
        state == TAIL_COPY || state == CAP_SPEED;
    wire write_stopping = state == ACCEPT_STOP || state == DEC_STOP ||
        state == TAIL_COPY || state == CAP_STOP;

    // Stream both ALU inputs directly from rotating registers. Selected
    // destinations capture each result bit in place, avoiding parallel word
    // load/copy muxes. Registers rotate through their own width, restoring
    // undisturbed operands after every 30-cycle operation. The next state also
    // selects the operation while working, so there is no saved instruction.
    // Preserve the old position sign before its in-place replacement.
    task operation;
        input sub;
        input [4:0] next_state;
        begin
            source_position_sign <= reference_position[28]; carry <= sub;
            bit_index <= 0; state <= next_state; working <= 1;
        end
    endtask

    always @(posedge clk) begin
        if (!rst_n) begin
            reference_position <= 0; speed <= 0; busy <= 0; done <= 0;
            latched_profile <= 0; latched_goal <= 0; negative <= 0;
            source_position_sign <= 0; stopping <= 0;
            capped <= 0; finishing <= 0;
            state <= IDLE; working <= 0; bit_index <= 0;
            accumulator <= 0; carry <= 0;
        end else begin
            done <= 0;
            if (!enable) begin
                busy <= 0; speed <= 0; stopping <= 0;
                state <= IDLE; working <= 0; accumulator <= 0;
            end else if (!busy) begin
                if (start) begin
                    reference_position <= {start_position,16'b0};
                    latched_goal <= target_position; negative <= target_position < start_position;
                    latched_profile <= plan_code; speed <= 0; stopping <= 0;
                    state <= IDLE; working <= 0; accumulator <= 0;
                    busy <= start_position != target_position;
                    done <= start_position == target_position;
                end
            end else if (working) begin
                if ((operand_a == SPEED || operand == SPEED || write_speed) && bit_index < 20)
                    speed <= {write_speed ? sum_bit[0] : speed[0],speed[19:1]};
                if ((operand_a == POSITION_SIGNED || operand_a == DISTANCE ||
                     operand == POSITION_SIGNED || operand == DISTANCE || write_position) && bit_index < 29)
                    reference_position <= {
                        write_position ? sum_bit[0] : reference_position[0],
                        reference_position[28:1]};
                if ((operand_a == STOPPING || operand == STOPPING || write_stopping) && bit_index < 29)
                    stopping <= {write_stopping ? sum_bit[0] : stopping[0],stopping[28:1]};
                accumulator <= {sum_bit[0],accumulator[29:1]};
                carry <= sum_bit[1];
                bit_index <= bit_index + 1'b1;
                if (bit_index == 29) working <= 0;
            end else case (state)
                IDLE: if (tick) begin
                    finishing <= 0;
                    operation(negative,ADVANCE);
                end
                ADVANCE: state <= BEGIN_DIST;
                BEGIN_DIST: operation(1'b1,DIST);
                DIST: begin
                    // Reuse the output register for remaining distance. Final
                    // position is reconstructed from goal +/- this distance.
                    if (accumulator == 0) begin
                        speed <= 0; stopping <= 0; finishing <= 1; state <= BEGIN_POS;
                    end else operation(1'b1,TAIL_SPEED);
                end
                TAIL_SPEED: if (carry) operation(1'b1,TAIL_ACCEL);
                            else state <= BEGIN_ACCEL;
                TAIL_ACCEL: if (carry) operation(1'b1,TAIL_LIMIT);
                            else state <= BEGIN_ACCEL;
                TAIL_LIMIT: if (carry) begin
                    operation(1'b0,TAIL_COPY);
                end else state <= BEGIN_ACCEL;
                BEGIN_ACCEL: operation(1'b0,ACCEL);
                ACCEL: operation(1'b1,CAP);
                CAP: begin
                    capped <= carry;
                    if (carry) operation(1'b0,STOP);
                    else operation(1'b0,STOP_ADD_A);
                end
                STOP_ADD_A: operation(1'b0,STOP);
                STOP: operation(1'b1,TEST_NEW);
                TEST_NEW: if (!carry || accumulator == 0) begin
                    if (capped) operation(1'b0,CAP_SPEED);
                    else operation(1'b0,ACCEPT_ADD);
                end else operation(1'b1,TEST_OLD);
                TEST_OLD: if (!carry || accumulator == 0) state <= BEGIN_POS;
                          else operation(1'b1,DEC_STOP);
                ACCEPT_ADD: operation(1'b0,ACCEPT_STOP);
                ACCEPT_STOP: begin
                    operation(1'b0,ACCEPT_SPEED);
                end
                ACCEPT_SPEED: state <= BEGIN_POS;
                DEC_STOP: begin
                    operation(1'b1,DEC_SPEED);
                end
                DEC_SPEED: begin
                    if (!carry) begin speed <= 0; stopping <= 0; end
                    state <= BEGIN_POS;
                end
                TAIL_COPY, CAP_STOP: state <= BEGIN_POS;
                CAP_SPEED: operation(1'b0,CAP_STOP);
                BEGIN_POS: operation(!negative,POSITION);
                POSITION: begin
                    // Reuse the idle accumulator as signed velocity output,
                    // replacing a separate full-width combinational negation.
                    operation(negative,VELOCITY);
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
