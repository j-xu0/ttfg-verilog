// SPDX-License-Identifier: Apache-2.0
`default_nettype none
// One scheduled PI/planner bit-serial engine. PI consumes the completed
// reference before the planner advances it. Position subtraction, PI sums and
// planner operations all use the same adder. PI temporarily rotates the
// reference to form error, then streams feedforward from the retained speed.
// reference_position/reference_velocity are valid only when calculating=0.
// STABLE_CONFIG=1 requires target/profile/gains stable throughout a move.
// STABLE_CONFIG=0 captures goal/profile at start and gains on each accepted
// tick. start_position is the launch origin on start and live encoder sample
// on tick. Busy starts and ticks during calculating are ignored.
// PI publishes atomically before planner advances. clear_control or a low
// control_enable clears PI immediately without canceling planner work; global
// enable=0 aborts both. The caller must allow 2000 clocks between ticks (the
// integrated 10 MHz / 1 kHz controller provides 10000). Reset spans a clock.
module shared_motion_controller #(parameter integer STABLE_CONFIG=0) (
    input wire clk, rst_n, enable, control_enable, clear_control, start, tick,
    input wire signed [12:0] start_position, target_position,
    input wire [5:0] plan_code,
    input wire [1:0] kp_select, ki_select,
    output reg signed [28:0] reference_position,
    output wire signed [20:0] reference_velocity,
    output reg planner_busy, planner_done,
    output wire planner_calculating, pi_calculating, calculating,
    output reg signed [20:0] velocity_command,
    output reg saturated
);
`include "plan_lut.vh"
    localparam [5:0] IDLE=0, PI_READY=1, PI_SUM=2, PI_SUM_DONE=3,
        PI_CLAMP=4, PI_COMPARE_START=5, PI_COMPARE=6, PI_FILL_START=7,
        PI_FILL=8, PI_CLAMP_DONE=9, PI_PUBLISH=10, PI_ERROR=11;
    localparam [5:0] P_IDLE=32, ADVANCE=34, BEGIN_DIST=35, DIST=36,
        TAIL_SPEED=37, TAIL_ACCEL=38, TAIL_LIMIT=39, BEGIN_ACCEL=40, ACCEL=41,
        CAP=42, STOP_ADD_A=43, STOP=44, TEST_NEW=45, TEST_OLD=46,
        ACCEPT_ADD=47, ACCEPT_STOP=48, ACCEPT_SPEED=49, DEC_STOP=50,
        DEC_SPEED=51, BEGIN_POS=52, POSITION=53, VELOCITY=54,
        TAIL_COPY=55, CAP_SPEED=56, CAP_STOP=57;
    localparam [3:0] ZERO=0, SPEED=1, ACCELERATION=2, POSITION_SIGNED=3,
        DISTANCE=4, GOAL=5, LIMIT=6, STOPPING=7, MAX_STOP=8, ACCUMULATOR=9;
    (* fsm_encoding = "none" *) reg [5:0] state;
    reg working;
    reg [4:0] bit_index;
    reg [29:0] accumulator;
    reg [3:0] operand_a, operand;
    reg [1:0] carry;
    reg subtract;
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

    assign reference_velocity = $signed(accumulator[20:0]);
    assign calculating = state != IDLE || working;
    assign planner_calculating = state >= P_IDLE || working;
    assign pi_calculating = state > IDLE && state < P_IDLE;

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
        input [5:0] next_state;
        begin
            source_position_sign <= reference_position[28]; carry <= {1'b0,sub};
            bit_index <= 0; state <= next_state; working <= 1;
        end
    endtask


    reg signed [20:0] integral;
    reg [20:0] work;
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
    wire [4:0] last_error_bit = gain == 0 ? 26 : gain == 1 ? 27 : gain == 2 ? 28 : 29;
    wire error_bit = bit_index <= last_error_bit ? error_tap : error_negative;
    wire integral_bit = bit_index < 21 ? integral[0] : integral_negative;
    wire ff_bit = !integral_phase &&
        ((bit_index < 20 ? speed[0] : 1'b0) ^ negative);
    wire measured_bit = bit_index < 16 ? 1'b0 :
        bit_index < 29 ? work[0] : work[12];
    wire position_bit = bit_index < 29 ? reference_position[0] : source_position_sign;
    wire [20:0] value = integral_phase ? integral : work;
    wire limit_bit = bit_index == 20 ? 1'b0 : vmax[bit_index];
    wire magnitude_bit = pi_negative ? (~value[0] ^ carry[1]) : value[0];
    wire limit_signed_bit = pi_negative ? (~limit_bit ^ carry[0]) : limit_bit;
    wire windup = clipped && pi_negative == error_negative;
    // The same full adder handles planner arithmetic, PI three-input sums,
    // and PI magnitude comparisons. Only the PI sum needs a two-bit carry.
    wire alu_a = state == PI_ERROR ? position_bit :
                 state == PI_SUM ? error_bit :
                 state == PI_COMPARE ? magnitude_bit : source_bits[operand_a];
    wire alu_b = state == PI_ERROR ? ~measured_bit :
                 state == PI_SUM ? integral_bit :
                 state == PI_COMPARE ? ~limit_bit : (source_bits[operand] ^ subtract);
    wire alu_c = state == PI_SUM ? ff_bit : 1'b0;
    wire [2:0] alu_sum = {2'b0,alu_a}+{2'b0,alu_b}+{2'b0,alu_c}+
                        (state == PI_SUM ? {1'b0,carry} : {2'b0,carry[0]});
    wire compare_clipped = alu_sum[1] && (comparison_nonzero || alu_sum[0]);
    task begin_planner;
        begin
            if (planner_busy) begin
                finishing <= 0;
                operation(negative,ADVANCE);
            end else begin
                accumulator <= 0;
                state <= IDLE;
            end
        end
    endtask

    always @(posedge clk) begin
        // PI scratch values need no reset: each accepted operation initializes
        // them before use. Shared accumulator reset also defines idle velocity.
        // On PI-only cancellation, finish rotating persistent operands before
        // returning control to the planner. Global abort can stop immediately
        // because the next start reinitializes those operands.
        if (state == PI_READY) begin
            error_negative <= accumulator[29];
            carry <= {1'b0,negative}; bit_index <= 0;
        end
        if (state == PI_ERROR) begin
            accumulator <= {alu_sum[0],accumulator[29:1]};
            if (bit_index < 29)
                reference_position <= {reference_position[0],reference_position[28:1]};
            if (bit_index >= 16 && bit_index < 29)
                work[12:0] <= {work[0],work[12:1]};
            carry <= {1'b0,alu_sum[1]}; bit_index <= bit_index + 1'b1;
        end
        if (state == PI_SUM) begin
            if (bit_index < 30) accumulator <= {accumulator[0],accumulator[29:1]};
            if (!integral_phase && bit_index < 20) speed <= {speed[0],speed[19:1]};
            if (bit_index < 21 && !integral_phase) work <= {alu_sum[0],work[20:1]};
            if (bit_index == 20) low_sign <= alu_sum[0];
            if (bit_index > 20 && alu_sum[0] != low_sign) overflow <= 1;
            carry <= alu_sum[2:1]; bit_index <= bit_index + 1'b1;
            if (bit_index == 30) pi_negative <= alu_sum[0];
        end
        if (state == PI_CLAMP) clipped <= overflow;
        if (state == PI_COMPARE_START) begin
            comparison_nonzero <= 0; bit_index <= 0; carry <= 3;
        end
        if (state == PI_COMPARE) begin
            if (!integral_phase) work <= {work[0],work[20:1]};
            carry <= {pi_negative && (~value[0] & carry[1]),alu_sum[1]};
            comparison_nonzero <= comparison_nonzero | alu_sum[0];
            bit_index <= bit_index + 1'b1;
            if (bit_index == 20) clipped <= compare_clipped;
        end
        if (state == PI_FILL_START) begin carry <= 1; bit_index <= 0; end
        if (state == PI_FILL) begin
            if (!integral_phase) work <= {limit_signed_bit,work[20:1]};
            carry <= {1'b0,(~limit_bit & carry[0])};
            bit_index <= bit_index + 1'b1;
        end
        if (state == PI_CLAMP_DONE && !integral_phase) begin
            pending_saturated <= clipped; integral_phase <= 1;
            integral_negative <= integral[20]; overflow <= 0;
            if (windup) pi_negative <= integral[20];
            else begin carry <= 0; bit_index <= 0; end
        end
        if (!rst_n) begin
            reference_position <= 0; speed <= 0; planner_busy <= 0; planner_done <= 0;
            latched_profile <= 0; latched_goal <= 0; negative <= 0;
            source_position_sign <= 0; stopping <= 0; capped <= 0; finishing <= 0;
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
                reference_position <= {start_position,16'b0};
                latched_goal <= target_position; negative <= target_position < start_position;
                latched_profile <= plan_code; speed <= 0; stopping <= 0;
                accumulator <= 0; working <= 0; pi_cancelled <= 0;
                planner_busy <= start_position != target_position;
                planner_done <= start_position == target_position;
                integral <= 0; velocity_command <= 0; saturated <= 0;
            end else begin
                if (clear_control || !control_enable || pi_cancelled) begin
                    integral <= 0; velocity_command <= 0; saturated <= 0;
                end else begin
                    if (state == PI_SUM && bit_index < 21)
                        integral <= {integral_phase ? alu_sum[0] : integral[0],integral[20:1]};
                    else if (state == PI_COMPARE && integral_phase)
                        integral <= {integral[0],integral[20:1]};
                    else if (state == PI_FILL && integral_phase)
                        integral <= {limit_signed_bit,integral[20:1]};
                    if (state == PI_PUBLISH) begin
                        velocity_command <= work; saturated <= pending_saturated;
                    end
                end
                if ((clear_control || !control_enable || pi_cancelled) && pi_calculating &&
                    state != PI_ERROR && state != PI_SUM) begin
                    begin_planner();
                end else if (working) begin
                    if ((operand_a == SPEED || operand == SPEED || write_speed) && bit_index < 20)
                        speed <= {write_speed ? alu_sum[0] : speed[0],speed[19:1]};
                    if ((operand_a == POSITION_SIGNED || operand_a == DISTANCE ||
                         operand == POSITION_SIGNED || operand == DISTANCE || write_position) && bit_index < 29)
                        reference_position <= {write_position ? alu_sum[0] : reference_position[0],reference_position[28:1]};
                    if ((operand_a == STOPPING || operand == STOPPING || write_stopping) && bit_index < 29)
                        stopping <= {write_stopping ? alu_sum[0] : stopping[0],stopping[28:1]};
                    accumulator <= {alu_sum[0],accumulator[29:1]};
                    carry <= {1'b0,alu_sum[1]}; bit_index <= bit_index + 1'b1;
                    if (bit_index == 29) working <= 0;
                end else case (state)
                    IDLE: if (tick) begin
                        if (control_enable && !clear_control) begin
                            work[12:0] <= start_position;
                            source_position_sign <= reference_position[28];
                            saved_kp <= kp_select; saved_ki <= ki_select;
                            integral_negative <= integral[20]; pi_cancelled <= 0;
                            integral_phase <= 0; carry <= 1; bit_index <= 0; overflow <= 0;
                            state <= PI_ERROR;
                        end else begin_planner();
                    end
                    PI_ERROR: if (bit_index == 29) begin
                        if (clear_control || !control_enable || pi_cancelled) begin_planner();
                        else state <= PI_READY;
                    end
                    PI_READY: state <= PI_SUM;
                    PI_SUM: if (bit_index == 30) begin
                        if (clear_control || !control_enable || pi_cancelled) begin_planner();
                        else state <= PI_SUM_DONE;
                    end
                    PI_SUM_DONE: state <= PI_CLAMP;
                    PI_CLAMP: state <= overflow ? PI_FILL_START : PI_COMPARE_START;
                    PI_COMPARE_START: state <= PI_COMPARE;
                    PI_COMPARE: if (bit_index == 20) state <= compare_clipped ? PI_FILL_START : PI_CLAMP_DONE;
                    PI_FILL_START: state <= PI_FILL;
                    PI_FILL: if (bit_index == 20) state <= PI_CLAMP_DONE;
                    PI_CLAMP_DONE: if (!integral_phase) state <= windup ? PI_CLAMP : PI_SUM;
                                   else state <= PI_PUBLISH;
                    PI_PUBLISH: begin_planner();
                    ADVANCE: state <= BEGIN_DIST;
                    BEGIN_DIST: operation(1'b1,DIST);
                    DIST: begin
                        // Reuse the output register for remaining distance. Final
                        // position is reconstructed from goal +/- this distance.
                        if (accumulator == 0) begin
                            speed <= 0; stopping <= 0; finishing <= 1; state <= BEGIN_POS;
                        end else operation(1'b1,TAIL_SPEED);
                    end
                    TAIL_SPEED: if (carry[0]) operation(1'b1,TAIL_ACCEL);
                                else state <= BEGIN_ACCEL;
                    TAIL_ACCEL: if (carry[0]) operation(1'b1,TAIL_LIMIT);
                                else state <= BEGIN_ACCEL;
                    TAIL_LIMIT: if (carry[0]) begin
                        operation(1'b0,TAIL_COPY);
                    end else state <= BEGIN_ACCEL;
                    BEGIN_ACCEL: operation(1'b0,ACCEL);
                    ACCEL: operation(1'b1,CAP);
                    CAP: begin
                        capped <= carry[0];
                        if (carry[0]) operation(1'b0,STOP);
                        else operation(1'b0,STOP_ADD_A);
                    end
                    STOP_ADD_A: operation(1'b0,STOP);
                    STOP: operation(1'b1,TEST_NEW);
                    TEST_NEW: if (!carry[0] || accumulator == 0) begin
                        if (capped) operation(1'b0,CAP_SPEED);
                        else operation(1'b0,ACCEPT_ADD);
                    end else operation(1'b1,TEST_OLD);
                    TEST_OLD: if (!carry[0] || accumulator == 0) state <= BEGIN_POS;
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
                        if (!carry[0]) begin speed <= 0; stopping <= 0; end
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
