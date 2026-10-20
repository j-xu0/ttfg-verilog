/*
 * Copyright (c) 2026 Jason Xu
 * SPDX-License-Identifier: Apache-2.0
 */
`default_nettype none
// Single-clock DDS pulse generator. velocity_command is signed Q16 steps per
// control tick. driver_enable is logical active high (invert externally if needed).
// Timing parameters are a conservative configurable interface contract, not a
// claim of compatibility with an unspecified external motor driver.
// Parameters must be positive; MAX_STEP_HZ must not exceed CLOCK_HZ.
// Requested rates beyond pulse timing capacity discard events and assert
// rate_limited instead of queuing a later burst.
// Reset, disable and fault suppress outputs immediately and may truncate STEP.
// A zero command completes an active pulse, clears phase and prevents catch-up.
module step_dir_generator #(
    parameter integer CLOCK_HZ = 10000000,
    parameter integer CONTROL_HZ = 1000,
    parameter integer MAX_STEP_HZ = 10000,
    parameter integer STEP_HIGH_CYCLES = 50,
    parameter integer STEP_LOW_CYCLES = 50,
    parameter integer DIR_SETUP_CYCLES = 50,
    parameter integer DIR_HOLD_CYCLES = 50,
    parameter integer ENABLE_SETUP_CYCLES = 100
)(
    input wire clk,
    input wire rst_n,
    input wire enable,
    input wire fault,
    input wire signed [20:0] velocity_command,
    output wire step,
    output reg dir,
    output wire driver_enable,
    output wire rate_limited
);
`ifndef SYNTHESIS
    initial begin
        if (CLOCK_HZ <= 0 || CONTROL_HZ <= 0 || MAX_STEP_HZ <= 0 ||
            MAX_STEP_HZ > CLOCK_HZ || STEP_HIGH_CYCLES <= 0 ||
            STEP_LOW_CYCLES <= 0 || DIR_SETUP_CYCLES <= 0 ||
            DIR_HOLD_CYCLES <= 0 || ENABLE_SETUP_CYCLES <= 0)
            $fatal(1, "STEP generator requires positive parameters and MAX_STEP_HZ <= CLOCK_HZ");
    end
`endif
    // Every old phase state is a multiple of gcd(65536*CLOCK_HZ,
    // CONTROL_HZ). Divide the threshold and increment by that exact factor;
    // this preserves every carry/event cycle while shrinking the accumulator.
    // The Euclidean loop runs only during elaboration, never in hardware.
    function [63:0] constant_gcd;
        input [63:0] first;
        input [63:0] second;
        reg [63:0] a, b, remainder;
        begin
            a = first;
            b = second;
            while (b != 0) begin
                remainder = a % b;
                a = b;
                b = remainder;
            end
            constant_gcd = a;
        end
    endfunction
    function integer trailing_zero_bits;
        input [63:0] value;
        reg [63:0] remaining;
        begin
            remaining = value;
            trailing_zero_bits = 0;
            while (remaining != 0 && !remaining[0]) begin
                remaining = remaining >> 1;
                trailing_zero_bits = trailing_zero_bits + 1;
            end
        end
    endfunction
    localparam [63:0] RAW_PHASE_LIMIT = 64'd65536 * CLOCK_HZ;
    localparam [63:0] PHASE_GCD = constant_gcd(RAW_PHASE_LIMIT, CONTROL_HZ);
    localparam [63:0] PHASE_LIMIT = RAW_PHASE_LIMIT / PHASE_GCD;
    localparam [63:0] INCREMENT_SCALE = CONTROL_HZ / PHASE_GCD;
    localparam [63:0] MAX_COMMAND = (64'd65536 * MAX_STEP_HZ) / CONTROL_HZ;
    localparam integer PHASE_BITS = PHASE_LIMIT > 1 ? $clog2(PHASE_LIMIT) : 1;
    localparam integer LOW_MAX = (STEP_LOW_CYCLES > DIR_HOLD_CYCLES ?
                                  STEP_LOW_CYCLES : DIR_HOLD_CYCLES) - 1;
    localparam integer PULSE_MAX = STEP_HIGH_CYCLES-1 > LOW_MAX ?
                                      STEP_HIGH_CYCLES-1 : LOW_MAX;
    localparam integer PULSE_BITS = PULSE_MAX > 0 ? $clog2(PULSE_MAX+1) : 1;
    localparam integer DIR_MAX = DIR_SETUP_CYCLES-1;
    localparam integer ENABLE_MAX = ENABLE_SETUP_CYCLES-1;
    localparam integer SETUP_MAX = DIR_MAX > ENABLE_MAX ? DIR_MAX : ENABLE_MAX;
    localparam integer SETUP_BITS = SETUP_MAX > 0 ? $clog2(SETUP_MAX+1) : 1;
    wire command_limited = velocity_command[20] ?
        $signed(velocity_command) < -$signed(MAX_COMMAND) :
        $signed(velocity_command) > $signed(MAX_COMMAND);
    wire [20:0] magnitude = velocity_command[20] ?
        (~velocity_command + 21'd1) : velocity_command;
    wire [20:0] bounded_magnitude = magnitude > MAX_COMMAND ? MAX_COMMAND : magnitude;
    wire [PHASE_BITS:0] increment = bounded_magnitude * INCREMENT_SCALE;
    wire active = rst_n && enable && !fault;
    reg pulse;
    reg [PHASE_BITS-1:0] phase;
    // High time and fall-to-rise/hold time are mutually exclusive. Keep one
    // countdown: high time while pulse is set, low/hold time otherwise.
    reg [PULSE_BITS-1:0] pulse_left;
    // The next event waits for both enable and direction setup. Their maximum
    // remaining delay is sufficient; a direction change extends that maximum.
    reg [SETUP_BITS-1:0] setup_left;
    reg timing_limited;
    localparam integer FRACTION_BITS = trailing_zero_bits(PHASE_LIMIT);
    localparam [63:0] HIGH_LIMIT = PHASE_LIMIT >> FRACTION_BITS;
    localparam [63:0] MAX_MAGNITUDE = MAX_COMMAND < 64'd1048576 ?
                                       MAX_COMMAND : 64'd1048576;
    localparam [63:0] MAX_INCREMENT = MAX_MAGNITUDE * INCREMENT_SCALE;
    wire [PHASE_BITS-1:0] phase_next;
    wire phase_event;
    generate if (FRACTION_BITS > 0 && FRACTION_BITS < PHASE_BITS &&
                 MAX_INCREMENT <= (64'd1 << FRACTION_BITS)) begin : split_phase
        // limit = HIGH_LIMIT * 2^FRACTION_BITS. A bounded increment can
        // overflow the fractional word at most once per clock, so the upper
        // word only needs an increment and equality test instead of a wide
        // threshold subtraction. The concatenation preserves the same phase.
        localparam integer HIGH_BITS = PHASE_BITS - FRACTION_BITS;
        // Fold the absolute-value +1 into the accumulator carry input. The
        // direct signed range check avoids a separate magnitude comparator.
        // Other scale/modulus cases retain the general exact increment.
        wire [FRACTION_BITS-1:0] signed_command = $signed(velocity_command);
        wire [FRACTION_BITS-1:0] addend = command_limited ?
            MAX_COMMAND : signed_command ^
                                  {FRACTION_BITS{velocity_command[20]}};
        wire [FRACTION_BITS:0] fractional_sum = INCREMENT_SCALE == 1 &&
            MAX_INCREMENT < (64'd1 << FRACTION_BITS) ?
            {1'b0, phase[FRACTION_BITS-1:0]} + {1'b0, addend} +
                (velocity_command[20] && !command_limited) :
            {1'b0, phase[FRACTION_BITS-1:0]} + increment;
        wire [HIGH_BITS-1:0] high = phase[PHASE_BITS-1:FRACTION_BITS];
        wire [HIGH_BITS-1:0] advanced_high = high + fractional_sum[FRACTION_BITS];
        wire [HIGH_BITS-1:0] high_next;
        genvar bit_index;
        // On a wrap, advanced_high equals HIGH_LIMIT. Only its set bits
        // need clearing; all other bits already have their required zero.
        for (bit_index=0; bit_index<HIGH_BITS; bit_index=bit_index+1) begin : high_reset
            if ((HIGH_LIMIT >> bit_index) & 1) begin
                assign high_next[bit_index] = advanced_high[bit_index] && !phase_event;
            end else begin
                assign high_next[bit_index] = advanced_high[bit_index];
            end
        end
        assign phase_event = fractional_sum[FRACTION_BITS] && high == HIGH_LIMIT-1;
        assign phase_next = {high_next, fractional_sum[FRACTION_BITS-1:0]};
    end else begin : general_phase
        wire [PHASE_BITS:0] next_phase = phase + increment;
        // phase < limit and increment <= limit bound this signed difference
        // to [-limit, limit-1]; its extra bit is exactly the underflow flag.
        wire [PHASE_BITS:0] difference = next_phase - PHASE_LIMIT;
        assign phase_event = !difference[PHASE_BITS];
        assign phase_next = phase_event ? difference[PHASE_BITS-1:0] :
                                          next_phase[PHASE_BITS-1:0];
    end endgenerate
    assign step = active && pulse;
    assign driver_enable = active;
    assign rate_limited = active && (command_limited || timing_limited);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pulse <= 1'b0;
            dir <= 1'b0;
            phase <= 0;
            pulse_left <= 0;
            setup_left <= SETUP_MAX;
            timing_limited <= 0;
        end else if (!enable || fault) begin
            pulse <= 1'b0;
            dir <= 1'b0;
            phase <= 0;
            pulse_left <= 0;
            setup_left <= SETUP_MAX;
            timing_limited <= 0;
        end else begin
            if (setup_left != 0) setup_left <= setup_left - 1;
            if (pulse_left != 0) pulse_left <= pulse_left - 1;
            timing_limited <= 0;
            if (pulse) begin
                if (pulse_left == 0) begin
                    pulse <= 0;
                    pulse_left <= LOW_MAX;
                end
            end

            if (velocity_command == 0) begin
                phase <= 0;
            end else if (dir != !velocity_command[20]) begin
                // Finish the current pulse and hold direction beyond its fall.
                phase <= 0;
                if (!pulse && (DIR_HOLD_CYCLES <= 1 || pulse_left <= LOW_MAX-(DIR_HOLD_CYCLES-1))) begin
                    dir <= !velocity_command[20];
                    if (setup_left <= DIR_MAX+1) setup_left <= DIR_MAX;
                end
            end else if (setup_left != 0) begin
                phase <= 0;
            end else begin
                // Preserve fractional carry; discard physically unserviceable
                // events instead of queuing pulses for a later burst.
                if (phase_event) begin
                    phase <= phase_next;
                    if (!pulse && (STEP_LOW_CYCLES <= 1 || pulse_left <= LOW_MAX-(STEP_LOW_CYCLES-1))) begin
                        pulse <= 1;
                        pulse_left <= STEP_HIGH_CYCLES-1;
                    end else timing_limited <= 1;
                end else phase <= phase_next;
            end
        end
    end
endmodule

`default_nettype wire
