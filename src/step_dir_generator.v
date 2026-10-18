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
    localparam [63:0] PHASE_LIMIT = 64'd65536 * CLOCK_HZ;
    localparam [63:0] MAX_COMMAND = (64'd65536 * MAX_STEP_HZ) / CONTROL_HZ;
    localparam integer PHASE_BITS = $clog2(PHASE_LIMIT);
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
    wire [PHASE_BITS:0] increment = bounded_magnitude * CONTROL_HZ;
    wire active = rst_n && enable && !fault;
    reg pulse;
    reg [PHASE_BITS-1:0] phase;
    reg [HIGH_BITS-1:0] high_left;
    reg [LOW_BITS-1:0] low_age;
    reg [DIR_BITS-1:0] dir_age;
    reg [ENABLE_BITS-1:0] enable_age;
    reg timing_limited;
    wire [PHASE_BITS:0] next_phase = phase + increment;
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
                if (next_phase >= PHASE_LIMIT) begin
                    phase <= next_phase - PHASE_LIMIT;
                    if (!pulse && (STEP_LOW_CYCLES <= 1 || low_age >= STEP_LOW_CYCLES-1)) begin
                        pulse <= 1;
                        high_left <= STEP_HIGH_CYCLES-1;
                    end else timing_limited <= 1;
                end else phase <= next_phase;
            end
        end
    end
endmodule

`default_nettype wire
