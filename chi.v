`timescale 1ns / 1ps
module chi #(
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

    genvar x_map_in, y_map_in;
    generate
        for (x_map_in = 0; x_map_in < DIM_SIZE; x_map_in = x_map_in + 1) begin : map_in_x
            for (y_map_in = 0; y_map_in < DIM_SIZE; y_map_in = y_map_in + 1) begin : map_in_y
                localparam offset = (y_map_in * DIM_SIZE + x_map_in) * LANE_WIDTH;
                assign A_in[x_map_in][y_map_in] = A_in_flat[offset + LANE_WIDTH-1 : offset];
            end
        end
    endgenerate

    // --- 核心 CHI 逻辑 ---
    genvar x_chi, y_chi;
    generate
        for (y_chi = 0; y_chi < DIM_SIZE; y_chi = y_chi + 1) begin : chi_row_gen
            for (x_chi = 0; x_chi < DIM_SIZE; x_chi = x_chi + 1) begin : chi_col_gen
                assign Ap_out[x_chi][y_chi] = A_in[x_chi][y_chi] ^ 
                    ( (~A_in[(x_chi+1)%DIM_SIZE][y_chi]) & A_in[(x_chi+2)%DIM_SIZE][y_chi] );
            end
        end
    endgenerate

    genvar x_map_out, y_map_out;
    generate
        for (x_map_out = 0; x_map_out < DIM_SIZE; x_map_out = x_map_out + 1) begin : map_out_x
            for (y_map_out = 0; y_map_out < DIM_SIZE; y_map_out = y_map_out + 1) begin : map_out_y
                localparam offset = (y_map_out * DIM_SIZE + x_map_out) * LANE_WIDTH;
                assign Ap_out_flat[offset + LANE_WIDTH-1 : offset] = Ap_out[x_map_out][y_map_out];
            end
        end
    endgenerate
endmodule