/* SPDX-License-Identifier: Apache-2.0 */
`default_nettype none

// Oversampled SPI mode 0 (CPOL=0, CPHA=0), MSB first. Keep each SCLK
// half-period and CS setup/hold interval at least four system-clock cycles.
// MOSI must be stable at least two system-clock cycles before rising SCLK
// and remain stable until falling SCLK. Hold CS high for four clocks after
// reset before starting a frame; a frame already active at reset is ignored.
// Transactions are exactly 16 bits: {write, address[1:0], payload[12:0]}.
// Address 0 = signed target, 1 = reserved, 2 = plan[2:0] (acceleration
// code only; velocity is fixed). Address 1 is reserved.
// Write 0xE000 zeros position/target only while motion is disabled.
// Writes commit only after CS rises. No partial frame can write.
// Every frame returns a status snapshot captured when synchronized CS falls:
// {1'b0, busy, at_target, fault, 10'b0, range_fault, encoder_fault}.
// Complete reads (write=0, address/payload ignored) acknowledge faults unless
// a new event arrived during the frame. Writes never acknowledge faults. MISO is zero while idle.
module stepper_spi (
    input wire clk, rst_n,
    input wire sclk, cs_n, mosi,
    input wire busy, fault, at_target,
    input wire [1:0] fault_code,
    input wire fault_event, motion_disabled,
    output wire miso,
    output reg signed [12:0] target,
    output reg [2:0] plan,
    output reg write_pulse, reject_pulse,
    output reg fault_clear_pulse, zero_position_pulse,
    output reg target_write_pulse
);
    (* async_reg = "true" *) reg sclk_meta, sclk_sync;
    (* async_reg = "true" *) reg cs_meta, cs_sync;
    (* async_reg = "true" *) reg mosi_meta, mosi_sync;
    reg sclk_last, cs_last;
    reg [1:0] input_valid;
    reg cs_armed;
    reg active, busy_seen;
    reg fault_during_frame;
    reg [4:0] bit_count;
    reg [15:0] rx_shift, tx_shift;

    assign miso = active && !cs_sync ? tx_shift[15] : 1'b0;

    // All 16 receive bits are overwritten before a frame may commit. Short,
    // empty, overlong and reset-aborted frames never expose this scratch data,
    // so it needs neither a reset tree nor a frame-start clear mux.
    always @(posedge clk) begin
        if (active && !cs_sync && !sclk_last && sclk_sync)
            rx_shift <= {rx_shift[14:0], mosi_sync};
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sclk_meta <= 1'b0;
            sclk_sync <= 1'b0;
            sclk_last <= 1'b0;
            cs_meta <= 1'b1;
            cs_sync <= 1'b1;
            cs_last <= 1'b1;
            mosi_meta <= 1'b0;
            mosi_sync <= 1'b0;
            input_valid <= 2'b00;
            cs_armed <= 1'b0;
            active <= 1'b0;
            fault_during_frame <= 1'b0;
            busy_seen <= 1'b0;
            bit_count <= 5'd0;
            tx_shift <= 16'd0;
            target <= 13'sd0;
            plan <= 3'd0;
            write_pulse <= 1'b0;
            target_write_pulse <= 1'b0;
            fault_clear_pulse <= 1'b0;
            zero_position_pulse <= 1'b0;
            reject_pulse <= 1'b0;
        end else begin
            input_valid <= {input_valid[0], 1'b1};
            // Ignore reset values in the CS synchronizer. Only an actual
            // post-reset high sample permits a subsequent falling edge.
            if (input_valid[1] && cs_sync)
                cs_armed <= 1'b1;
            sclk_meta <= sclk;
            sclk_sync <= sclk_meta;
            sclk_last <= sclk_sync;
            cs_meta <= cs_n;
            cs_sync <= cs_meta;
            cs_last <= cs_sync;
            mosi_meta <= mosi;
            mosi_sync <= mosi_meta;
            write_pulse <= 1'b0;
            target_write_pulse <= 1'b0;
            fault_clear_pulse <= 1'b0;
            zero_position_pulse <= 1'b0;
            reject_pulse <= 1'b0;

            if (active && fault_event) fault_during_frame <= 1'b1;
            if (cs_armed && cs_last && !cs_sync) begin
                active <= 1'b1;
                busy_seen <= busy;
                bit_count <= 5'd0;
                fault_during_frame <= fault_event;
                tx_shift <= {1'b0, busy, at_target, fault, 10'd0, fault_code};
            end else if (!cs_last && cs_sync) begin
                active <= 1'b0;
                if (active && bit_count != 0) begin
                    if (bit_count != 16) begin
                        reject_pulse <= 1'b1;
                    end else if (!rx_shift[15]) begin
                        if (!fault_during_frame && !fault_event)
                            fault_clear_pulse <= 1'b1;
                    end else if (busy_seen || busy) begin
                        reject_pulse <= 1'b1;
                    end else if (rx_shift[15]) begin
                        case (rx_shift[14:13])
                            2'd0: begin
                                target <= rx_shift[12:0];
                                write_pulse <= 1'b1;
                                target_write_pulse <= 1'b1;
                            end
                            2'd1: reject_pulse <= 1'b1;
                            2'd2: begin
                                plan <= rx_shift[2:0];
                                write_pulse <= 1'b1;
                            end
                            2'd3: begin
                                if (rx_shift[12:0] == 0 && motion_disabled) begin
                                    zero_position_pulse <= 1'b1;
                                    target <= 0;
                                    write_pulse <= 1'b1;
                                end else reject_pulse <= 1'b1;
                            end
                        endcase
                    end
                end
            end else if (active && !cs_sync) begin
                busy_seen <= busy_seen || busy;
                if (!sclk_last && sclk_sync) begin
                    // Saturation ensures even a very long frame cannot wrap
                    // the counter back to the valid length.
                    if (bit_count < 17)
                        bit_count <= bit_count + 1'b1;
                end
                if (sclk_last && !sclk_sync)
                    tx_shift <= {tx_shift[14:0], 1'b0};
            end
        end
    end
endmodule
`default_nettype wire
