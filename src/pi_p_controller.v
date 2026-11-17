`default_nettype none
// Fixed-gain proportional position stage. The velocity feedforward term and
// runtime Kp selection were removed for 1x1 area; SPI addr 1 (ex-CONTROL) is reserved.
// Kp is fixed at the former code 1: clamp the integer error to +/-32 so the
// 16-bit P term cannot overflow, then scale by 2^9 (Q11). The clamp to vmax
// keeps the published command inside the profile's velocity limit.
module pi_p_controller (
    input wire clk, rst_n, enable, clear, accept,
    input wire signed [23:0] reference_position,
    input wire signed [12:0] measured,
    input wire [12:0] vmax,
    output reg signed [13:0] velocity_command,
    output reg saturated
);
    wire signed [12:0] pos_int = reference_position[23:11];
    wire signed [13:0] error = pos_int - measured;
    wire signed [8:0] bound = 9'sd32;
    wire signed [13:0] bound_ext = {{5{bound[8]}}, bound};
    wire signed [8:0] error_clamped =
        error > bound_ext ? bound : error < -bound_ext ? -bound : error[8:0];
    wire signed [15:0] total =
        {{7{error_clamped[8]}}, error_clamped} <<< 9;
    wire signed [15:0] limit = {3'b0, vmax};
    wire over = total > limit;
    wire under = total < -limit;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            velocity_command <= 0; saturated <= 0;
        end else if (!enable || clear) begin
            velocity_command <= 0; saturated <= 0;
        end else if (accept) begin
            velocity_command <= over ? limit[13:0] :
                                under ? -limit[13:0] : total[13:0];
            saturated <= over || under;
        end
    end
endmodule
`default_nettype wire
