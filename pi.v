`timescale 1ns / 1ps
module pi #(
    // Keccak-f[1600] 核心参数 (固定)
    localparam LANE_WIDTH = 64,
    localparam DIM_SIZE   = 5,
    localparam STATE_WIDTH = LANE_WIDTH * DIM_SIZE * DIM_SIZE // 1600
)(
    input  [STATE_WIDTH-1:0] A_in_flat,
    output [STATE_WIDTH-1:0] Ap_out_flat
);
    wire [LANE_WIDTH-1:0] A_in [0:DIM_SIZE-1][0:DIM_SIZE-1];

    genvar x_map, y_map;
    generate
        for (x_map = 0; x_map < DIM_SIZE; x_map = x_map + 1) begin : port_map_x_in
            for (y_map = 0; y_map < DIM_SIZE; y_map = y_map + 1) begin : port_map_y_in
                localparam offset = (y_map * DIM_SIZE + x_map) * LANE_WIDTH;
                assign A_in[x_map][y_map] = A_in_flat[offset + LANE_WIDTH-1 : offset];
            end
        end
    endgenerate

    // --- 核心 PI 逻辑 (固定) ---
    // !! 警告: 以下置换是为 DIM_SIZE=5 硬编码的 !!
    // Ap_out[x_pi][y_pi] = A_in[(x_pi + 3*y_pi)%5][x_pi]
    genvar x_pi, y_pi;
    generate
        for (x_pi = 0; x_pi < DIM_SIZE; x_pi = x_pi + 1) begin : pi_gen_x
            for (y_pi = 0; y_pi < DIM_SIZE; y_pi = y_pi + 1) begin : pi_gen_y
                localparam in_x = (x_pi + 3*y_pi) % 5;
                localparam in_y = x_pi;
                
                localparam out_offset = (y_pi * DIM_SIZE + x_pi) * LANE_WIDTH;
                assign Ap_out_flat[out_offset + LANE_WIDTH-1 : out_offset] = A_in[in_x][in_y];
            end
        end
    endgenerate
endmodule