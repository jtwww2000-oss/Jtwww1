`timescale 1ns / 1ps
module rho #(
    // Keccak-f[1600] 核心参数 (固定)
    localparam LANE_WIDTH = 64,
    localparam DIM_SIZE   = 5,
    localparam STATE_WIDTH = LANE_WIDTH * DIM_SIZE * DIM_SIZE // 1600
)(
    input  [STATE_WIDTH-1:0] A_in_flat,
    output [STATE_WIDTH-1:0] Ap_out_flat
);
    wire [LANE_WIDTH-1:0] A_in [0:DIM_SIZE-1][0:DIM_SIZE-1];
    wire [LANE_WIDTH-1:0] Ap_out [0:DIM_SIZE-1][0:DIM_SIZE-1];

    genvar x_map, y_map;
    generate
        for (x_map = 0; x_map < DIM_SIZE; x_map = x_map + 1) begin : port_map_x
            for (y_map = 0; y_map < DIM_SIZE; y_map = y_map + 1) begin : port_map_y
                localparam offset = (y_map * DIM_SIZE + x_map) * LANE_WIDTH;
                assign A_in[x_map][y_map] = A_in_flat[offset + LANE_WIDTH-1 : offset];
                assign Ap_out_flat[offset + LANE_WIDTH-1 : offset] = Ap_out[x_map][y_map];
            end
        end
    endgenerate

    // --- 核心 RHO 逻辑 (固定) ---
    // !! 警告: 以下旋转量是为 LANE_WIDTH=64 硬编码的 !!
    // (x=0)
    assign Ap_out[0][0] = A_in[0][0];
    assign Ap_out[0][1] = {A_in[0][1][27:0], A_in[0][1][63:28]}; // ROL 36
    assign Ap_out[0][2] = {A_in[0][2][60:0], A_in[0][2][63:61]}; // ROL 3
    assign Ap_out[0][3] = {A_in[0][3][22:0], A_in[0][3][63:23]}; // ROL 41
    assign Ap_out[0][4] = {A_in[0][4][45:0], A_in[0][4][63:46]}; // ROL 18
    // (x=1)
    assign Ap_out[1][0] = {A_in[1][0][62:0], A_in[1][0][63:63]}; // ROL 1
    assign Ap_out[1][1] = {A_in[1][1][19:0], A_in[1][1][63:20]}; // ROL 44
    assign Ap_out[1][2] = {A_in[1][2][53:0], A_in[1][2][63:54]}; // ROL 10
    assign Ap_out[1][3] = {A_in[1][3][18:0], A_in[1][3][63:19]}; // ROL 45
    assign Ap_out[1][4] = {A_in[1][4][61:0], A_in[1][4][63:62]}; // ROL 2
    // (x=2)
    assign Ap_out[2][0] = {A_in[2][0][ 1:0], A_in[2][0][63: 2]}; // ROL 62
    assign Ap_out[2][1] = {A_in[2][1][57:0], A_in[2][1][63:58]}; // ROL 6
    assign Ap_out[2][2] = {A_in[2][2][20:0], A_in[2][2][63:21]}; // ROL 43
    assign Ap_out[2][3] = {A_in[2][3][48:0], A_in[2][3][63:49]}; // ROL 15
    assign Ap_out[2][4] = {A_in[2][4][ 2:0], A_in[2][4][63: 3]}; // ROL 61
    // (x=3)
    assign Ap_out[3][0] = {A_in[3][0][35:0], A_in[3][0][63:36]}; // ROL 28
    assign Ap_out[3][1] = {A_in[3][1][ 8:0], A_in[3][1][63: 9]}; // ROL 55
    assign Ap_out[3][2] = {A_in[3][2][38:0], A_in[3][2][63:39]}; // ROL 25
    assign Ap_out[3][3] = {A_in[3][3][42:0], A_in[3][3][63:43]}; // ROL 21
    assign Ap_out[3][4] = {A_in[3][4][ 7:0], A_in[3][4][63: 8]}; // ROL 56
    // (x=4)
    assign Ap_out[4][0] = {A_in[4][0][36:0], A_in[4][0][63:37]}; // ROL 27
    assign Ap_out[4][1] = {A_in[4][1][43:0], A_in[4][1][63:44]}; // ROL 20
    assign Ap_out[4][2] = {A_in[4][2][24:0], A_in[4][2][63:25]}; // ROL 39
    assign Ap_out[4][3] = {A_in[4][3][55:0], A_in[4][3][63:56]}; // ROL 8
    assign Ap_out[4][4] = {A_in[4][4][49:0], A_in[4][4][63:50]}; // ROL 14
endmodule