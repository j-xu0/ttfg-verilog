// Q8 experiment (temporary) - derived from scripts/generate_plan_lut.py; do not use for tapeout.
function [11:0] profile_velocity;
    input [2:0] code;
    begin case (code)
        3'd0: profile_velocity = 12'd26;
        3'd1: profile_velocity = 12'd51;
        3'd2: profile_velocity = 12'd128;
        3'd3: profile_velocity = 12'd256;
        3'd4: profile_velocity = 12'd512;
        3'd5: profile_velocity = 12'd1024;
        3'd6: profile_velocity = 12'd1792;
        3'd7: profile_velocity = 12'd2560;
    endcase end
endfunction
function [4:0] profile_acceleration;
    input [2:0] code;
    begin case (code)
        3'd0: profile_acceleration = 5'd0;
        3'd1: profile_acceleration = 5'd0;
        3'd2: profile_acceleration = 5'd1;
        3'd3: profile_acceleration = 5'd1;
        3'd4: profile_acceleration = 5'd3;
        3'd5: profile_acceleration = 5'd5;
        3'd6: profile_acceleration = 5'd13;
        3'd7: profile_acceleration = 5'd26;
    endcase end
endfunction
// Stop sums above every legal move are rejected with a 21-bit sentinel.
function [20:0] profile_stopping;
    input [5:0] code;
    begin case (code)
        6'd0: profile_stopping = 21'd2097151;
        6'd1: profile_stopping = 21'd2097151;
        6'd2: profile_stopping = 21'd2097151;
        6'd3: profile_stopping = 21'd2097151;
        6'd4: profile_stopping = 21'd2097151;
        6'd5: profile_stopping = 21'd2097151;
        6'd6: profile_stopping = 21'd2097151;
        6'd7: profile_stopping = 21'd2097151;
        6'd8: profile_stopping = 21'd2097151;
        6'd9: profile_stopping = 21'd2097151;
        6'd10: profile_stopping = 21'd2097151;
        6'd11: profile_stopping = 21'd2097151;
        6'd12: profile_stopping = 21'd2097151;
        6'd13: profile_stopping = 21'd2097151;
        6'd14: profile_stopping = 21'd2097151;
        6'd15: profile_stopping = 21'd2097151;
        6'd16: profile_stopping = 21'd351;
        6'd17: profile_stopping = 21'd1326;
        6'd18: profile_stopping = 21'd8256;
        6'd19: profile_stopping = 21'd32896;
        6'd20: profile_stopping = 21'd131328;
        6'd21: profile_stopping = 21'd524800;
        6'd22: profile_stopping = 21'd1606528;
        6'd23: profile_stopping = 21'd2097151;
        6'd24: profile_stopping = 21'd351;
        6'd25: profile_stopping = 21'd1326;
        6'd26: profile_stopping = 21'd8256;
        6'd27: profile_stopping = 21'd32896;
        6'd28: profile_stopping = 21'd131328;
        6'd29: profile_stopping = 21'd524800;
        6'd30: profile_stopping = 21'd1606528;
        6'd31: profile_stopping = 21'd2097151;
        6'd32: profile_stopping = 21'd126;
        6'd33: profile_stopping = 21'd459;
        6'd34: profile_stopping = 21'd2795;
        6'd35: profile_stopping = 21'd11051;
        6'd36: profile_stopping = 21'd43947;
        6'd37: profile_stopping = 21'd175275;
        6'd38: profile_stopping = 21'd536107;
        6'd39: profile_stopping = 21'd1093547;
        6'd40: profile_stopping = 21'd81;
        6'd41: profile_stopping = 21'd286;
        6'd42: profile_stopping = 21'd1703;
        6'd43: profile_stopping = 21'd6682;
        6'd44: profile_stopping = 21'd26471;
        6'd45: profile_stopping = 21'd105370;
        6'd46: profile_stopping = 21'd322023;
        6'd47: profile_stopping = 21'd656640;
        6'd48: profile_stopping = 21'd39;
        6'd49: profile_stopping = 21'd126;
        6'd50: profile_stopping = 21'd695;
        6'd51: profile_stopping = 21'd2650;
        6'd52: profile_stopping = 21'd10340;
        6'd53: profile_stopping = 21'd40843;
        6'd54: profile_stopping = 21'd124407;
        6'd55: profile_stopping = 21'd253342;
        6'd56: profile_stopping = 21'd26;
        6'd57: profile_stopping = 21'd76;
        6'd58: profile_stopping = 21'd380;
        6'd59: profile_stopping = 21'd1390;
        6'd60: profile_stopping = 21'd5300;
        6'd61: profile_stopping = 21'd20680;
        6'd62: profile_stopping = 21'd62652;
        6'd63: profile_stopping = 21'd127314;
    endcase end
endfunction

