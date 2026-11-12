// Measurement prototype: dedicated parallel Q11 PI with every current feature
// (integer error, 4 kp/ki codes, feedforward, command clamp+saturated,
// integral clamp and anti-windup freeze). Not integrated.
`default_nettype none
module pi_parallel (
    input wire clk, rst_n, enable, clear, accept,
    input wire signed [24:0] reference_position,
    input wire signed [12:0] measured,
    input wire signed [13:0] feedforward,
    input wire [12:0] vmax,
    input wire [1:0] kp_select, ki_select,
    output reg signed [13:0] velocity_command,
    output reg saturated
);
    wire signed [13:0] pos_int = reference_position[24:11];
    wire signed [13:0] error = pos_int - measured;
    // Gain codes select an arithmetic shift of the integer error.
    wire signed [13:0] err_kp = kp_select == 3 ? error : error >>> (3 - kp_select);
    wire signed [13:0] err_ki = ki_select == 3 ? error : error >>> (3 - ki_select);
    wire signed [15:0] vlimit = {3'b0, vmax};
    wire signed [15:0] ff_ext = {{2{feedforward[13]}}, feedforward};
    wire signed [15:0] kp_ext = {{2{err_kp[13]}}, err_kp};
    reg  signed [15:0] integral;
    wire signed [15:0] total = ff_ext + kp_ext + integral;
    wire over = total > vlimit;
    wire under = total < -vlimit;
    wire magnitude_over = over || under;
    wire windup = (over && error > 0) || (under && error < 0);
    wire signed [15:0] sum_clamped = over ? vlimit : under ? -vlimit : total;
    wire signed [15:0] ki_ext = {{2{err_ki[13]}}, err_ki};
    wire signed [15:0] grown = integral + ki_ext;
    wire signed [15:0] int_clamped = grown > vlimit ? vlimit :
                                     grown < -vlimit ? -vlimit : grown;
    always @(posedge clk) begin
        if (!rst_n || !enable || clear) begin
            integral <= 0; velocity_command <= 0; saturated <= 0;
        end else if (accept) begin
            integral <= windup ? integral : int_clamped;
            velocity_command <= sum_clamped[13:0];
            saturated <= magnitude_over;
        end
    end
endmodule
`default_nettype wire
