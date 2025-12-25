`timescale 1ns / 1ps

module SHAKE256 #(
    parameter RATE = 1088,             
    parameter CAPACITY = 512,          
    parameter STATE_WIDTH = 1600,
    parameter OUTPUT_LEN_BYTES = 128,  
    parameter ABSORB_LEN = 256         
)(
    input  wire                 clk,
    input  wire                 rst_n,

    input  wire                 i_start,  
    input  wire [ABSORB_LEN-1:0] i_seed,   
    output reg                  o_busy,   

    input  wire                 i_squeeze_req,
    output reg                  o_squeeze_valid,
    output wire [OUTPUT_LEN_BYTES*8-1:0] o_squeeze_data
);

    localparam S_IDLE           = 3'd0; 
    localparam S_WAIT_KECCAK    = 3'd1; 
    localparam S_PAD_EXTRA      = 3'd2; 
    localparam S_SQUEEZE        = 3'd3; 

    reg [2:0] current_state;
    reg [STATE_WIDTH-1:0] r_state;          
    reg [2:0] r_next_action;    

    reg                     keccak_start;
    reg  [STATE_WIDTH-1:0]  keccak_in_data;
    wire                    keccak_done;
    wire [STATE_WIDTH-1:0]  keccak_out_data;
    wire                    keccak_busy;

    wire internal_absorb_last;
    wire [RATE-1:0] internal_absorb_data;
    
    assign internal_absorb_last = 1'b1;
    assign internal_absorb_data = { {(RATE-ABSORB_LEN){1'b0}}, i_seed };

    wire [RATE-1:0] pad_in_N;
    wire [10:0]     pad_in_m;
    wire [RATE-1:0] pad_out_P;
    wire [RATE-1:0] w_absorb_chunk;   
    wire            w_need_extra_pad;

    Keccak_f #( .STATE_WIDTH(STATE_WIDTH) ) u_keccak (
        .clk(clk), .rst_n(rst_n), .i_start(keccak_start), .i_data(keccak_in_data),
        .o_valid(keccak_done), .o_data(keccak_out_data), .o_busy(keccak_busy)
    );

    pad #( .X(RATE), .MAX_LEN(RATE) ) u_pad (
        .N(pad_in_N), .m(pad_in_m), .P(pad_out_P)
    );

    assign w_need_extra_pad = (ABSORB_LEN > (RATE - 5)); 
    assign pad_in_N = (current_state == S_PAD_EXTRA) ? {RATE{1'b0}} : internal_absorb_data;
    assign pad_in_m = (current_state == S_PAD_EXTRA) ? 11'd0       : ABSORB_LEN[10:0];
    assign w_absorb_chunk = (current_state == S_PAD_EXTRA) ? pad_out_P : pad_out_P; 

    // --- 主状态机 (已修复重启动逻辑) ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state   <= S_IDLE;
            r_state         <= {STATE_WIDTH{1'b0}};
            r_next_action   <= 3'd0;
            o_busy          <= 1'b0;
            o_squeeze_valid <= 1'b0;
            keccak_start    <= 1'b0;
            keccak_in_data  <= {STATE_WIDTH{1'b0}};
        end else begin
            keccak_start <= 1'b0; 

            // ★★★ 修复点：优先处理 i_start，实现强制复位与启动 ★★★
            if (i_start) begin
                o_busy <= 1'b1;
                o_squeeze_valid <= 1'b0;

                // 强制新一轮 Absorb (忽略旧 r_state，相当于 0 ^ chunk)
                keccak_in_data[RATE-1:0] <= w_absorb_chunk;
                keccak_in_data[STATE_WIDTH-1:RATE] <= { (STATE_WIDTH-RATE){1'b0} }; // Capacity 清零

                keccak_start <= 1'b1;

                if (w_need_extra_pad) begin
                    r_next_action <= S_PAD_EXTRA;
                end else begin
                    r_next_action <= S_SQUEEZE;
                end
                
                current_state <= S_WAIT_KECCAK;

            end else begin
                // 正常的 FSM 逻辑
                case (current_state)
                    S_IDLE: begin
                        o_busy <= 1'b0;
                        o_squeeze_valid <= 1'b0;
                    end

                    S_WAIT_KECCAK: begin
                        if (keccak_done) begin
                            r_state <= keccak_out_data; 
                            if (r_next_action == S_IDLE) begin
                                current_state <= S_IDLE;
                                o_busy <= 1'b0; 
                            end else begin
                                current_state <= r_next_action;
                            end
                        end
                    end

                    S_PAD_EXTRA: begin
                        keccak_in_data[RATE-1:0] <= r_state[RATE-1:0] ^ w_absorb_chunk;
                        keccak_in_data[STATE_WIDTH-1:RATE] <= r_state[STATE_WIDTH-1:RATE];
                        keccak_start   <= 1'b1;
                        r_next_action  <= S_SQUEEZE;
                        current_state  <= S_WAIT_KECCAK;
                    end

                    S_SQUEEZE: begin
                        o_busy          <= 1'b1;
                        o_squeeze_valid <= 1'b1;

                        if (i_squeeze_req) begin
                            o_squeeze_valid <= 1'b0;
                            keccak_in_data <= r_state; 
                            keccak_start   <= 1'b1;
                            r_next_action <= S_SQUEEZE;
                            current_state <= S_WAIT_KECCAK;
                        end
                    end

                    default: current_state <= S_IDLE;
                endcase
            end
        end
    end

    assign o_squeeze_data = r_state[OUTPUT_LEN_BYTES*8-1:0];

endmodule