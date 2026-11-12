// SPDX-License-Identifier: Apache-2.0
`default_nettype none
// One scheduled PI/planner bit-serial engine. PI consumes the completed
// reference before the planner advances it. Position advancement, remaining
// distance, the brake sum, PI sums and every planner operation share the same
// 1-bit adder, so there is no wide parallel arithmetic anywhere in the engine.
// reference_position is Q11 (25 bits: 14 integer + 11 fraction, so pos += vel
// is exact) and reference_velocity is Q11 signed [13:0]; both are meaningful
// only when calculating=0 (every register rotates in place during a pass).
// STABLE_CONFIG=1 requires target/profile/gains stable throughout a move.
// STABLE_CONFIG=0 captures goal/profile at start and gains on each accepted
// tick. start_position is the launch origin on start and live encoder sample
// on tick. Busy starts and ticks during calculating are ignored.
// PI publishes atomically before planner advances. clear_control or a low
// control_enable clears PI immediately without canceling planner work; global
// enable=0 aborts both. The caller must allow 2000 clocks between ticks (the
// integrated 10 MHz / 1 kHz controller provides 10000). Reset spans a clock.
//
// Numeric spec: Q11 rates (ref_vel [13:0], speed_limit [12:0], accel [10:0]),
// Q11 position and brake distance (25 bits), no stopping table. The brake sum
// S is maintained exactly in Q11: S(s) = s + S(s-a), so accelerating from s to
// min(s+a, vmax) adds the new speed, holding keeps S and decelerating
// subtracts the current speed. A candidate is accepted only when its exact
// brake sum fits in the exact remaining distance, so the reference can never
// overshoot the target. Finishing is integer: the reference stops once its
// integer position equals the target (sub-step residue < 1 step).
module shared_motion_controller #(parameter integer STABLE_CONFIG=0) (
    input wire clk, rst_n, enable, control_enable, clear_control, start, tick,
    input wire signed [12:0] start_position, target_position,
    input wire [5:0] plan_code,
    input wire [1:0] kp_select, ki_select,
    output reg signed [24:0] reference_position,
    output wire signed [13:0] reference_velocity,
    output reg planner_busy, planner_done,
    output wire planner_calculating, pi_calculating, calculating,
    output reg signed [13:0] velocity_command,
    output reg saturated
);
`include "plan_lut.vh"
    localparam [5:0] IDLE=0, PI_READY=1, PI_SUM=2, PI_SUM_DONE=3,
        PI_CLAMP=4, PI_COMPARE_START=5, PI_COMPARE=6, PI_FILL_START=7,
        PI_FILL=8, PI_CLAMP_DONE=9, PI_PUBLISH=10, PI_ERROR=11;
    localparam [5:0] P_IDLE=32, ADVANCE=33, BEGIN_DIST=34, DIST=35,
        BEGIN_ACCEL=40, ACCEL=41, CAP=42, LOAD_LIMIT=43, TRIAL=44,
        TEST_NEW=45, ACCEPT_SPEED=46, CAP_SPEED=47, ACCEPT_ADD=48,
        HOLD_TEST=49, DEC_STOP=50, DEC_SPEED=51, BEGIN_POS=52, POSITION=53;
    localparam [3:0] ZERO=0, SPEED=1, ACCELERATION=2, POSITION_SIGNED=3,
        DISTANCE=4, GOAL=5, LIMIT=6, STOPPING=7, ACCUMULATOR=8;
    (* fsm_encoding = "none" *) reg [5:0] state;
    reg working;
    reg [4:0] bit_index;
    reg [25:0] accumulator;
    reg [3:0] operand_a, operand;
    reg [1:0] carry;
    reg subtract;
    reg [5:0] latched_profile;
    reg signed [12:0] latched_goal;
    wire [5:0] profile = STABLE_CONFIG != 0 ? plan_code : latched_profile;
    wire signed [12:0] goal = STABLE_CONFIG != 0 ? target_position : latched_goal;
    reg negative, source_position_sign;
    reg [12:0] speed;
    reg [24:0] stopping;
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
    wire [12:0] vmax = profile_velocity(profile[2:0]);
    wire [10:0] accel = profile_acceleration(profile[5:3]);
    wire signed [13:0] pos_int = reference_position[24:11];
    wire signed [24:0] goal_q11 = {{1{goal[12]}}, goal, 11'b0};
    // The full position register rotates during every position pass; DIST and
    // POSITION replace it with the remaining distance and restore it again.
    wire position_bit = reference_position[0];
    wire position_signed_bit = bit_index < 25 ? reference_position[0] :
                               source_position_sign;
    wire goal_bit = bit_index < 25 ? goal_q11[bit_index] : goal_q11[24];
    wire [8:0] source_bits = {
        accumulator[0],
        bit_index < 25 ? stopping[0] : 1'b0,
        bit_index < 13 ? vmax[bit_index] : 1'b0,
        goal_bit,
        bit_index < 25 ? position_bit : 1'b0,
        position_signed_bit,
        bit_index < 11 ? accel[bit_index] : 1'b0,
        bit_index < 13 ? speed[0] : 1'b0,
        1'b0};

    wire signed [13:0] speed_signed = negative ?
        -$signed({1'b0, speed}) : $signed({1'b0, speed});
    assign reference_velocity = speed_signed;
    assign calculating = state != IDLE || working;
    assign planner_calculating = state >= P_IDLE || working;
    assign pi_calculating = state > IDLE && state < P_IDLE;

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
            source_position_sign <= reference_position[24]; carry <= {1'b0,sub};
            bit_index <= 0; state <= next_state; working <= 1;
        end
    endtask

    reg signed [13:0] integral;
    reg signed [13:0] work;
    reg [1:0] saved_kp, saved_ki;
    reg error_negative, integral_negative, pi_cancelled;
    reg integral_phase, overflow, low_sign, pi_negative, clipped;
    reg pending_saturated, comparison_nonzero;
    wire [1:0] kp = STABLE_CONFIG ? kp_select : saved_kp;
    wire [1:0] ki = STABLE_CONFIG ? ki_select : saved_ki;
    wire [1:0] gain = integral_phase ? ki : kp;
    wire error_tap = gain == 0 ? accumulator[3] :
                     gain == 1 ? accumulator[2] :
                     gain == 2 ? accumulator[1] : accumulator[0];
    wire [4:0] last_error_bit = gain == 0 ? 22 : gain == 1 ? 23 : gain == 2 ? 24 : 25;
    wire error_bit = bit_index <= last_error_bit ? error_tap : error_negative;
    wire integral_bit = bit_index < 14 ? integral[0] : integral_negative;
    wire ff_bit = !integral_phase &&
        ((bit_index < 13 ? speed[0] : 1'b0) ^ negative);
    // PI error is (pos_int - measured) << 11 (Q11). Low 11 bits are zero;
    // bits 11..24 carry the 14-bit integer difference; bit 25 is the sign.
    // Index muxes preserve pos_int/work (no rotation needed here).
    wire [4:0] err_idx = bit_index - 5'd11;
    wire pos_err_bit = bit_index < 11 ? 1'b0 :
        bit_index < 25 ? pos_int[err_idx[3:0]] : pos_int[13];
    wire meas_err_bit = bit_index < 11 ? 1'b0 :
        bit_index < 25 ? work[err_idx[3:0]] : work[13];
    wire measured_bit = meas_err_bit;
    wire [13:0] value = integral_phase ? integral : work;
    wire limit_bit = bit_index == 13 ? 1'b0 : vmax[bit_index];
    wire magnitude_bit = pi_negative ? (~value[0] ^ carry[1]) : value[0];
    wire limit_signed_bit = pi_negative ? (~limit_bit ^ carry[0]) : limit_bit;
    wire windup = clipped && pi_negative == error_negative;
    wire alu_a = state == PI_ERROR ? pos_err_bit :
                 state == PI_SUM ? error_bit :
                 state == PI_COMPARE ? magnitude_bit : source_bits[operand_a];
    wire alu_b = state == PI_ERROR ? ~measured_bit :
                 state == PI_SUM ? integral_bit :
                 state == PI_COMPARE ? ~limit_bit : (source_bits[operand] ^ subtract);
    wire alu_c = state == PI_SUM ? ff_bit : 1'b0;
    wire [2:0] alu_sum = {2'b0,alu_a}+{2'b0,alu_b}+{2'b0,alu_c}+
                        (state == PI_SUM ? {1'b0,carry} : {2'b0,carry[0]});
    wire compare_clipped = alu_sum[1] && (comparison_nonzero || alu_sum[0]);
    task begin_planner_advance;
        begin
            finishing <= 0;
            carry <= {1'b0, negative}; bit_index <= 0; state <= ADVANCE;
            working <= 1;
        end
    endtask
    task begin_planner;
        begin
            if (planner_busy) begin
                begin_planner_advance();
            end else begin
                accumulator <= 0;
                state <= IDLE;
            end
        end
    endtask

    always @(posedge clk) begin
        if (state == PI_READY) begin
            error_negative <= accumulator[25];
            carry <= {1'b0,negative}; bit_index <= 0;
        end
        if (state == PI_ERROR) begin
            accumulator <= {alu_sum[0],accumulator[25:1]};
            carry <= {1'b0,alu_sum[1]}; bit_index <= bit_index + 1'b1;
        end
        if (state == PI_SUM) begin
            if (bit_index < 26) accumulator <= {accumulator[0],accumulator[25:1]};
            if (!integral_phase && bit_index < 13) speed <= {speed[0],speed[12:1]};
            if (bit_index < 14 && !integral_phase) work <= {alu_sum[0],work[13:1]};
            if (bit_index == 13) low_sign <= alu_sum[0];
            if (bit_index > 13 && alu_sum[0] != low_sign) overflow <= 1;
            carry <= alu_sum[2:1]; bit_index <= bit_index + 1'b1;
            if (bit_index == 26) pi_negative <= alu_sum[0];
        end
        if (state == PI_CLAMP) clipped <= overflow;
        if (state == PI_COMPARE_START) begin
            comparison_nonzero <= 0; bit_index <= 0; carry <= 3;
        end
        if (state == PI_COMPARE) begin
            if (!integral_phase) work <= {work[0],work[13:1]};
            carry <= {pi_negative && (~value[0] & carry[1]),alu_sum[1]};
            comparison_nonzero <= comparison_nonzero | alu_sum[0];
            bit_index <= bit_index + 1'b1;
            if (bit_index == 13) clipped <= compare_clipped;
        end
        if (state == PI_FILL_START) begin carry <= 1; bit_index <= 0; end
        if (state == PI_FILL) begin
            if (!integral_phase) work <= {limit_signed_bit,work[13:1]};
            carry <= {1'b0,(~limit_bit & carry[0])};
            bit_index <= bit_index + 1'b1;
        end
        if (state == PI_CLAMP_DONE && !integral_phase) begin
            pending_saturated <= clipped; integral_phase <= 1;
            integral_negative <= integral[13]; overflow <= 0;
            if (windup) pi_negative <= integral[13];
            else begin carry <= 0; bit_index <= 0; end
        end
        if (!rst_n) begin
            reference_position <= 0; speed <= 0; planner_busy <= 0; planner_done <= 0;
            latched_profile <= 0; latched_goal <= 0; negative <= 0;
            source_position_sign <= 0; stopping <= 0;
            capped <= 0; grow <= 0; finishing <= 0;
            state <= IDLE; working <= 0; accumulator <= 0;
            integral <= 0; velocity_command <= 0; saturated <= 0; pi_cancelled <= 0;
        end else begin
            planner_done <= 0;
            if (clear_control || !control_enable) pi_cancelled <= 1;
            if (!enable) begin
                planner_busy <= 0; speed <= 0; stopping <= 0;
                state <= IDLE; working <= 0; accumulator <= 0;
                integral <= 0; velocity_command <= 0; saturated <= 0;
            end else if (start && !planner_busy && state == IDLE) begin
                reference_position <= {{1{start_position[12]}}, start_position,11'b0};
                latched_goal <= target_position; negative <= target_position < start_position;
                latched_profile <= plan_code; speed <= 0; stopping <= 0;
                grow <= 0; accumulator <= 0; working <= 0; pi_cancelled <= 0;
                planner_busy <= start_position != target_position;
                planner_done <= start_position == target_position;
                integral <= 0; velocity_command <= 0; saturated <= 0;
            end else begin
                if (clear_control || !control_enable || pi_cancelled) begin
                    integral <= 0; velocity_command <= 0; saturated <= 0;
                end else begin
                    if (state == PI_SUM && bit_index < 14)
                        integral <= {integral_phase ? alu_sum[0] : integral[0],integral[13:1]};
                    else if (state == PI_COMPARE && integral_phase)
                        integral <= {integral[0],integral[13:1]};
                    else if (state == PI_FILL && integral_phase)
                        integral <= {limit_signed_bit,integral[13:1]};
                    if (state == PI_PUBLISH) begin
                        velocity_command <= work; saturated <= pending_saturated;
                    end
                end
                if ((clear_control || !control_enable || pi_cancelled) && pi_calculating &&
                    state != PI_ERROR && state != PI_SUM) begin
                    begin_planner();
                end else if (working) begin
                    if ((operand_a == SPEED || operand == SPEED || write_speed) && bit_index < 13)
                        speed <= {write_speed ? alu_sum[0] : speed[0],speed[12:1]};
                    if ((write_position || position_operand) && bit_index < 25)
                        reference_position <= {write_position ? alu_sum[0] : position_bit,
                                               reference_position[24:1]};
                    if ((write_stopping || stopping_operand) && bit_index < 25)
                        stopping <= {write_stopping ? alu_sum[0] : stopping[0],
                                     stopping[24:1]};
                    accumulator <= {alu_sum[0],accumulator[25:1]};
                    carry <= {1'b0,alu_sum[1]}; bit_index <= bit_index + 1'b1;
                    if (bit_index == 25) working <= 0;
                end else case (state)
                    IDLE: if (tick) begin
                        if (control_enable && !clear_control) begin
                            work <= {{1{start_position[12]}}, start_position};
                            source_position_sign <= reference_position[24];
                            saved_kp <= kp_select; saved_ki <= ki_select;
                            integral_negative <= integral[13]; pi_cancelled <= 0;
                            integral_phase <= 0; carry <= 1; bit_index <= 0; overflow <= 0;
                            state <= PI_ERROR;
                        end else begin_planner();
                    end
                    PI_ERROR: if (bit_index == 25) begin
                        if (clear_control || !control_enable || pi_cancelled) begin_planner();
                        else state <= PI_READY;
                    end
                    PI_READY: state <= PI_SUM;
                    PI_SUM: if (bit_index == 26) begin
                        if (clear_control || !control_enable || pi_cancelled) begin_planner();
                        else state <= PI_SUM_DONE;
                    end
                    PI_SUM_DONE: state <= PI_CLAMP;
                    PI_CLAMP: state <= overflow ? PI_FILL_START : PI_COMPARE_START;
                    PI_COMPARE_START: state <= PI_COMPARE;
                    PI_COMPARE: if (bit_index == 13) state <= compare_clipped ? PI_FILL_START : PI_CLAMP_DONE;
                    PI_FILL_START: state <= PI_FILL;
                    PI_FILL: if (bit_index == 13) state <= PI_CLAMP_DONE;
                    PI_CLAMP_DONE: if (!integral_phase) state <= windup ? PI_CLAMP : PI_SUM;
                                   else state <= PI_PUBLISH;
                    PI_PUBLISH: begin_planner();
                    ADVANCE: state <= BEGIN_DIST;
                    BEGIN_DIST: operation(1'b1,DIST);
                    DIST: begin
                        if (reference_position[24:11] == 0) begin
                            speed <= 0; stopping <= 0; finishing <= 1; state <= BEGIN_POS;
                        end else state <= BEGIN_ACCEL;
                    end
                    BEGIN_ACCEL: operation(1'b0,ACCEL);
                    ACCEL: state <= CAP;
                    // Decide against min(s+a, vmax): the accelerate candidate's
                    // speed is left in the accumulator for the brake sum.
                    CAP: begin
                        capped <= accumulator[13:0] >= {1'b0, vmax};
                        grow <= accumulator[13:0] < {1'b0, vmax} || speed < vmax;
                        if (accumulator[13:0] >= {1'b0, vmax}) operation(1'b0,LOAD_LIMIT);
                        else operation(1'b0,TRIAL);
                    end
                    LOAD_LIMIT: operation(1'b0,TRIAL);
                    TRIAL: operation(1'b1,TEST_NEW);
                    // Accept when S(candidate) <= remaining distance.
                    TEST_NEW: if (!carry[0] || accumulator[24:0] == 0) begin
                        if (capped) operation(1'b0,CAP_SPEED);
                        else operation(1'b0,ACCEPT_SPEED);
                    end else operation(1'b1,HOLD_TEST);
                    // Hold when S(s) already fits in the remaining distance.
                    HOLD_TEST: if (!carry[0] || accumulator[24:0] == 0) state <= BEGIN_POS;
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
    end
endmodule
`default_nettype wire
