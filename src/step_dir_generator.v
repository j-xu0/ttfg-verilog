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
    localparam integer LOW_BITS = LOW_MAX > 0 ? $clog2(LOW_MAX+1) : 1;
    localparam integer HIGH_BITS = STEP_HIGH_CYCLES > 1 ? $clog2(STEP_HIGH_CYCLES) : 1;
    localparam integer DIR_MAX = DIR_SETUP_CYCLES-1;
    localparam integer DIR_BITS = DIR_MAX > 0 ? $clog2(DIR_MAX+1) : 1;
    localparam integer ENABLE_MAX = ENABLE_SETUP_CYCLES-1;
    localparam integer ENABLE_BITS = ENABLE_MAX > 0 ? $clog2(ENABLE_MAX+1) : 1;
    wire [20:0] magnitude = velocity_command[20] ?
        (~velocity_command + 21'd1) : velocity_command;
    wire [20:0] bounded_magnitude = magnitude > MAX_COMMAND ? MAX_COMMAND : magnitude;
    wire [PHASE_BITS:0] increment = bounded_magnitude * INCREMENT_SCALE;
    wire active = rst_n && enable && !fault;
    reg pulse;
    reg [PHASE_BITS-1:0] phase;
    reg [HIGH_BITS-1:0] high_left;
    reg [LOW_BITS-1:0] low_age;
    reg [DIR_BITS-1:0] dir_age;
    reg [ENABLE_BITS-1:0] enable_age;
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
        wire [FRACTION_BITS:0] fractional_sum =
            {1'b0, phase[FRACTION_BITS-1:0]} + increment;
        wire [HIGH_BITS-1:0] high = phase[PHASE_BITS-1:FRACTION_BITS];
        wire [HIGH_BITS-1:0] high_next = phase_event ? {HIGH_BITS{1'b0}} :
            high + fractional_sum[FRACTION_BITS];
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
    assign rate_limited = active && ((magnitude > MAX_COMMAND) || timing_limited);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pulse <= 1'b0;
            dir <= 1'b0;
            phase <= 0;
            high_left <= 0;
            low_age <= LOW_MAX;
            dir_age <= 0;
            enable_age <= 0;
            timing_limited <= 0;
        end else if (!enable || fault) begin
            pulse <= 1'b0;
            dir <= 1'b0;
            phase <= 0;
            high_left <= 0;
            low_age <= LOW_MAX;
            dir_age <= 0;
            enable_age <= 0;
            timing_limited <= 0;
        end else begin
            if (enable_age < ENABLE_MAX) enable_age <= enable_age + 1;
            if (dir_age < DIR_MAX) dir_age <= dir_age + 1;
            if (!pulse && low_age < LOW_MAX) low_age <= low_age + 1;
            timing_limited <= 0;
            if (pulse) begin
                if (high_left == 0) begin
                    pulse <= 0;
                    low_age <= 0;
                end else high_left <= high_left - 1;
            end

            if (magnitude == 0) begin
                phase <= 0;
            end else if (dir != !velocity_command[20]) begin
                // Finish the current pulse and hold direction beyond its fall.
                phase <= 0;
                if (!pulse && (DIR_HOLD_CYCLES <= 1 || low_age >= DIR_HOLD_CYCLES-1)) begin
                    dir <= !velocity_command[20];
                    dir_age <= 0;
                end
            end else if ((DIR_SETUP_CYCLES > 1 && dir_age < DIR_SETUP_CYCLES-1) ||
                         (ENABLE_SETUP_CYCLES > 1 && enable_age < ENABLE_SETUP_CYCLES-1)) begin
                phase <= 0;
            end else begin
                // Preserve fractional carry; discard physically unserviceable
                // events instead of queuing pulses for a later burst.
                if (phase_event) begin
                    phase <= phase_next;
                    if (!pulse && (STEP_LOW_CYCLES <= 1 || low_age >= STEP_LOW_CYCLES-1)) begin
                        pulse <= 1;
                        high_left <= STEP_HIGH_CYCLES-1;
                    end else timing_limited <= 1;
                end else phase <= phase_next;
            end
        end
    end
endmodule

`default_nettype wire
