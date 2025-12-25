`timescale 1ns / 1ps
module theta #(
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

    // --- 核心 THETA 逻辑 ---
    wire [LANE_WIDTH-1:0] C [0:DIM_SIZE-1];
    genvar x_c;
    generate
        for (x_c = 0; x_c < DIM_SIZE; x_c = x_c + 1) begin : C_gen
            assign C[x_c] = A_in[x_c][0] ^ A_in[x_c][1] ^ A_in[x_c][2] ^ A_in[x_c][3] ^ A_in[x_c][4];
        end
    endgenerate

    wire [LANE_WIDTH-1:0] C_rot [0:DIM_SIZE-1]; 
    assign C_rot[0] = {C[0][LANE_WIDTH-2:0], C[0][LANE_WIDTH-1]}; // ROL(C[0], 1)
    assign C_rot[1] = {C[1][LANE_WIDTH-2:0], C[1][LANE_WIDTH-1]}; // ROL(C[1], 1)
    assign C_rot[2] = {C[2][LANE_WIDTH-2:0], C[2][LANE_WIDTH-1]}; // ROL(C[2], 1)
    assign C_rot[3] = {C[3][LANE_WIDTH-2:0], C[3][LANE_WIDTH-1]}; // ROL(C[3], 1)
    assign C_rot[4] = {C[4][LANE_WIDTH-2:0], C[4][LANE_WIDTH-1]}; // ROL(C[4], 1)

    wire [LANE_WIDTH-1:0] D [0:DIM_SIZE-1];
    assign D[0] = C[4] ^ C_rot[1];
    assign D[1] = C[0] ^ C_rot[2];
    assign D[2] = C[1] ^ C_rot[3];
    assign D[3] = C[2] ^ C_rot[4];
    assign D[4] = C[3] ^ C_rot[0];

    genvar x_ap, y_ap;
    generate
        for (x_ap = 0; x_ap < DIM_SIZE; x_ap = x_ap + 1) begin : Ap_x_gen
            for (y_ap = 0; y_ap < DIM_SIZE; y_ap = y_ap + 1) begin : Ap_y_gen
                assign Ap_out[x_ap][y_ap] = A_in[x_ap][y_ap] ^ D[x_ap];
            end
        end
    endgenerate
endmodule