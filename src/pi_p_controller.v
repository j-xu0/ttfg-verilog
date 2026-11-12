`default_nettype none
module pi_p_controller (
    input wire clk, rst_n, enable, clear, accept,
    input wire signed [23:0] reference_position,
    input wire signed [12:0] measured,
    input wire signed [13:0] feedforward,
    input wire [12:0] vmax,
    input wire [1:0] kp_select,
    output reg signed [13:0] velocity_command,
    output reg saturated
);
    wire signed [12:0] pos_int = reference_position[23:11];
    wire signed [13:0] error = pos_int - measured;
    wire signed [8:0] bound = kp_select == 0 ? 9'sd64 :
                              kp_select == 1 ? 9'sd32 :
                              kp_select == 2 ? 9'sd16 : 9'sd8;
    wire signed [13:0] bound_ext = {{5{bound[8]}}, bound};
    wire signed [8:0] error_clamped =
        error > bound_ext ? bound : error < -bound_ext ? -bound : error[8:0];
    wire signed [15:0] proportional =
        {{7{error_clamped[8]}}, error_clamped} <<< ({2'b0, kp_select} + 4'd8);
    wire signed [15:0] total =
        {{2{feedforward[13]}}, feedforward} + proportional;
    wire signed [15:0] limit = {3'b0, vmax};
    wire over = total > limit;
    wire under = total < -limit;
    always @(posedge clk) begin
        if (!rst_n || !enable || clear) begin
            velocity_command <= 0; saturated <= 0;
        end else if (accept) begin
            velocity_command <= over ? limit[13:0] :
                                under ? -limit[13:0] : total[13:0];
            saturated <= over || under;
        end
    end
endmodule
`default_nettype wire
