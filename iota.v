`timescale 1ns / 1ps
module iota #(
    // Keccak-f[1600] 核心参数 (固定)
    localparam LANE_WIDTH = 64,
    localparam DIM_SIZE   = 5,
    localparam STATE_WIDTH = LANE_WIDTH * DIM_SIZE * DIM_SIZE // 1600
)(
    input  [STATE_WIDTH-1:0] A_in_flat,
    input  [4:0]    i_round_index, 
    output [STATE_WIDTH-1:0] Ap_out_flat
);
    // !! 警告: 轮常量 RC 是为 LANE_WIDTH=64 硬编码的 !!
    reg [LANE_WIDTH-1:0] w_round_constant;
    always @(*) begin
        case (i_round_index)
            5'd0:  w_round_constant = 64'h0000000000000001;
            5'd1:  w_round_constant = 64'h0000000000008082;
            5'd2:  w_round_constant = 64'h800000000000808a;
            5'd3:  w_round_constant = 64'h8000000080008000;
            5'd4:  w_round_constant = 64'h000000000000808b;
            5'd5:  w_round_constant = 64'h0000000080000001;
            5'd6:  w_round_constant = 64'h8000000080008081;
            5'd7:  w_round_constant = 64'h8000000000008009;
            5'd8:  w_round_constant = 64'h000000000000008a;
            5'd9:  w_round_constant = 64'h0000000000000088;
            5'd10: w_round_constant = 64'h0000000080008009;
            5'd11: w_round_constant = 64'h000000008000000a;
            5'd12: w_round_constant = 64'h000000008000808b;
            5'd13: w_round_constant = 64'h800000000000008b;
            5'd14: w_round_constant = 64'h8000000000008089;
            5'd15: w_round_constant = 64'h8000000000008003;
            5'd16: w_round_constant = 64'h8000000000008002;
            5'd17: w_round_constant = 64'h8000000000000080;
            5'd18: w_round_constant = 64'h000000000000800a;
            5'd19: w_round_constant = 64'h800000008000000a;
            5'd20: w_round_constant = 64'h8000000080008081;
            5'd21: w_round_constant = 64'h8000000000008080;
            5'd22: w_round_constant = 64'h0000000080000001;
            5'd23: w_round_constant = 64'h8000000080008008;
            default: w_round_constant = 64'h0000000000000000;
        endcase
    end

    // --- 核心 IOTA 逻辑 ---
    // (A[0][0] = A[0][0] ^ RC)
    // A[0][0] 对应扁平化后的 [LANE_WIDTH-1:0]
    assign Ap_out_flat[LANE_WIDTH-1:0] = A_in_flat[LANE_WIDTH-1:0] ^ w_round_constant;
    assign Ap_out_flat[STATE_WIDTH-1:LANE_WIDTH] = A_in_flat[STATE_WIDTH-1:LANE_WIDTH];
    
endmodule