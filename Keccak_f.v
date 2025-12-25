`timescale 1ns / 1ps
module Keccak_f #(
    // Keccak-f[1600] 核心参数 (固定)
    parameter STATE_WIDTH = 1600,
    parameter NUM_ROUNDS  = 24,
    // 轮计数器位宽
    parameter ROUND_CNT_WIDTH = $clog2(NUM_ROUNDS) // $clog2(24) = 5
)(
    input           clk,
    input           rst_n,
    
    input           i_start,
    input  [STATE_WIDTH-1:0] i_data,
    
    output          o_valid,
    output [STATE_WIDTH-1:0] o_data,
    output          o_busy
);
    // --- FSM 状态定义 ---
    localparam S_IDLE = 2'd0;
    localparam S_RUN  = 2'd1;
    localparam S_DONE = 2'd2;

    // --- 内部寄存器 ---
    reg [1:0]    fsm_state_reg, fsm_state_next;
    reg [STATE_WIDTH-1:0] state_reg;
    reg [ROUND_CNT_WIDTH-1:0] round_index_reg; // 0-23

    // --- 连线 ---
    wire [STATE_WIDTH-1:0] w_next_state; 

    // --- 1. 例化 "Rnd" 组合逻辑模块 ---
    Rnd u_Rnd (
        .A_in_flat      (state_reg),
        .i_round_index  (round_index_reg),
        .Ap_out_flat    (w_next_state)
    );

    // --- 2. 时序逻辑 (FSM 和数据寄存器) ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fsm_state_reg   <= S_IDLE;
            state_reg       <= {STATE_WIDTH{1'b0}};
            round_index_reg <= {ROUND_CNT_WIDTH{1'b0}};
        end else begin
            fsm_state_reg <= fsm_state_next;
            case (fsm_state_reg)
                S_IDLE: begin
                    if (i_start) begin
                        state_reg       <= i_data;
                        round_index_reg <= {ROUND_CNT_WIDTH{1'b0}};
                    end
                end
                S_RUN: begin
                    state_reg       <= w_next_state;
                    round_index_reg <= round_index_reg + 1;
                end
                S_DONE: begin
                    // 保持不变
                end
            endcase
        end
    end
       
    // --- 3. 组合逻辑 (FSM 状态跳转) ---
    always @(*) begin
        fsm_state_next = fsm_state_reg;
        case (fsm_state_reg)
            S_IDLE: begin
                if (i_start) begin
                    fsm_state_next = S_RUN;
                end
            end
            S_RUN: begin
                if (round_index_reg == NUM_ROUNDS - 1) begin
                    fsm_state_next = S_DONE;
                end
            end
            S_DONE: begin
                fsm_state_next = S_IDLE;
            end
        endcase
    end
    
    // --- 4. 输出逻辑 ---
    assign o_data  = state_reg;
    assign o_valid = (fsm_state_reg == S_DONE);
    assign o_busy  = (fsm_state_reg == S_RUN);

endmodule