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
// Write 0xE000 zeros position/target only while motion_disabled stays high
// from the sampled CS assertion through commit. The input is synchronous.
// The project must stay selected through frame completion; deselection aborts
// the frame, so no commit and no acknowledgement can follow it. Writes commit
// only after CS rises. No partial frame can write.
// Each selected frame returns a status snapshot captured when voted CS
// falls: {1'b0, busy, at_target, 11'b0, range_fault, encoder_fault}. The
// snapshot BUSY bit is the write verdict: a write is accepted only when it
// reads zero, because frame_holds_motion freezes motion starts for the whole
// frame. Complete reads (write=0, address/payload ignored) acknowledge faults
// and the interrupt latch unless a new event arrived during the frame. Writes
// never acknowledge. MISO is zero while idle and while deselected.
module stepper_spi (
    input wire clk, rst_n,
    input wire selected,
    input wire sclk, cs_n, mosi,
    input wire busy, at_target,
    input wire [1:0] fault_code,
    input wire fault_event, motion_event, motion_disabled,
    output wire miso,
    output wire frame_holds_motion,
    output reg int_pending,
    output reg signed [12:0] target,
    output reg [2:0] plan,
    output reg write_pulse,
    output reg fault_clear_pulse, zero_position_pulse,
    output reg target_write_pulse
);
    (* async_reg = "true" *) reg [2:0] spi_meta, spi_sync;
    reg [2:0] spi_delay_one, spi_delay_two;
    wire [2:0] spi_vote = (spi_sync & spi_delay_one) |
                          ((spi_sync | spi_delay_one) & spi_delay_two);
    wire sclk_voted = spi_vote[2];
    wire cs_voted = spi_vote[1];
    wire mosi_voted = spi_vote[0];
    (* async_reg = "true" *) reg selected_meta, selected_sync;
    reg sclk_last, cs_last;
    // Align selection release with the extra cycle of CS qualification.
    reg selected_at_cs;
    reg cs_armed;
    reg active, frame_busy, frame_write;
    reg frame_at_target;
    reg [1:0] frame_fault;
    reg event_during_frame, disabled_through_frame;
    reg [4:0] bit_count;
    reg [15:0] rx_shift;

    // Capture deselection even when the low interval falls between clocks, so a
    // frame cannot outlive a deselect that happened between two edges. Release
    // only after the selection synchronizer has settled.
    wire selection_reset_n = rst_n && selected;
    always @(posedge clk or negedge selection_reset_n) begin
        if (!selection_reset_n) begin
            selected_meta <= 1'b0;
            selected_sync <= 1'b0;
            selected_at_cs <= 1'b0;
        end else begin
            selected_meta <= 1'b1;
            selected_sync <= selected_meta;
            selected_at_cs <= selected_sync;
        end
    end

    wire event_now = fault_event || motion_event;
    // The snapshot is four bits, not a 16-bit shift image: the transmitted bit
    // is selected by frame position instead, which replaces the whole transmit
    // shift register with a decoder. bit_count is the index of the bit the host
    // samples next, so index i carries status bit 15-i.
    wire status_bit = bit_count == 5'd1 ? frame_busy :
                      bit_count == 5'd2 ? frame_at_target :
                      bit_count == 5'd14 ? frame_fault[1] :
                      bit_count == 5'd15 ? frame_fault[0] : 1'b0;
    assign miso = selected && selected_sync && active && !cs_voted ? status_bit : 1'b0;

    // Hold motion starts from voted CS assertion until CS releases.
    // A frame is treated as a write until its first bit proves otherwise, so
    // the snapshot BUSY bit cannot disagree with the write decision. A queued
    // command therefore cannot begin moving in the middle of a write frame.
    assign frame_holds_motion = selected_at_cs && cs_armed && !cs_voted && frame_write;

    // All 16 receive bits are overwritten before a frame may commit. Short,
    // empty, overlong and reset-aborted frames never expose this scratch data,
    // so it needs neither a reset tree nor a frame-start clear mux.
    always @(posedge clk) begin
        if (active && !cs_voted && !sclk_last && sclk_voted)
            rx_shift <= {rx_shift[14:0], mosi_voted};
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spi_meta <= 3'b000;
            spi_sync <= 3'b000;
            spi_delay_one <= 3'b000;
            spi_delay_two <= 3'b000;
            sclk_last <= 1'b0;
            cs_last <= 1'b0;
            cs_armed <= 1'b0;
            active <= 1'b0;
            frame_busy <= 1'b0;
            frame_at_target <= 1'b0;
            frame_fault <= 2'b00;
            frame_write <= 1'b1;
            event_during_frame <= 1'b0;
            disabled_through_frame <= 1'b0;
            int_pending <= 1'b0;
            bit_count <= 5'd0;
            target <= 13'sd0;
            plan <= 3'd0;
            write_pulse <= 1'b0;
            target_write_pulse <= 1'b0;
            fault_clear_pulse <= 1'b0;
            zero_position_pulse <= 1'b0;
        end else begin
            // CS history resets low so only a post-reset majority of high
            // samples permits a subsequent falling edge.
            if (cs_voted)
                cs_armed <= 1'b1;
            spi_meta <= {sclk, cs_n, mosi};
            spi_sync <= spi_meta;
            spi_delay_one <= spi_sync;
            spi_delay_two <= spi_delay_one;
            sclk_last <= sclk_voted;
            cs_last <= cs_voted;
            write_pulse <= 1'b0;
            target_write_pulse <= 1'b0;
            fault_clear_pulse <= 1'b0;
            zero_position_pulse <= 1'b0;

            // A fault or a completed move inside a read frame makes that
            // frame stale, so its read must acknowledge nothing and the host
            // must poll again.
            if (active && event_now)
                event_during_frame <= 1'b1;
            // Align selection with sampled CS. Raw deselection aborts on
            // the next clock, even while CS completion is in flight, so a
            // frame the host never finished cannot acknowledge a fault.
            if (!selected_sync) begin
                active <= 1'b0;
            end else if (selected_at_cs && cs_armed && cs_last && !cs_voted) begin
                active <= 1'b1;
                frame_busy <= busy;
                frame_at_target <= at_target;
                frame_fault <= fault_code;
                frame_write <= 1'b1;
                disabled_through_frame <= motion_disabled;
                bit_count <= 5'd0;
                event_during_frame <= event_now;
            end else if (!cs_last && cs_voted) begin
                active <= 1'b0;
                if (active && bit_count == 5'd16) begin
                    if (!rx_shift[15]) begin
                        if (!event_during_frame && !event_now) begin
                            fault_clear_pulse <= 1'b1;
                            int_pending <= 1'b0;
                        end
                    end else if (!frame_busy) begin
                        case (rx_shift[14:13])
                            2'd0: begin
                                target <= rx_shift[12:0];
                                write_pulse <= 1'b1;
                                target_write_pulse <= 1'b1;
                            end
                            // Reserved address: silently ignored.
                            2'd1: ;
                            2'd2: begin
                                plan <= rx_shift[2:0];
                                write_pulse <= 1'b1;
                            end
                            2'd3: begin
                                if (rx_shift[12:0] == 0 && disabled_through_frame &&
                                    motion_disabled) begin
                                    zero_position_pulse <= 1'b1;
                                    target <= 0;
                                    write_pulse <= 1'b1;
                                end
                                // Illegal zero command: silently ignored.
                            end
                        endcase
                    end
                    // A frame that read BUSY is dropped without side effects.
                    // The host already has that same BUSY bit and must retry.
                end
            end else if (active && !cs_voted) begin
                disabled_through_frame <= disabled_through_frame && motion_disabled;
                if (!sclk_last && sclk_voted) begin
                    // First sampled bit is the write/read flag for the gate.
                    if (bit_count == 5'd0)
                        frame_write <= mosi_voted;
                    // Saturation ensures even a very long frame cannot wrap
                    // the counter back to the valid length.
                    if (bit_count < 17)
                        bit_count <= bit_count + 1'b1;
                end
            end

            // Events always raise the interrupt, even on the frame that is
            // acknowledging an older one; the host polls again when INT stays
            // high.
            if (event_now)
                int_pending <= 1'b1;
        end
    end
endmodule
`default_nettype wire
