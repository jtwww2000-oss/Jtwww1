`timescale 1ns / 1ps

module gpow #(
    parameter WIDTH = 24
)(
    input  wire             clk,
    input  wire             rst_n,
    
    // 控制接口
    input  wire             start,
    output reg              done,
    output reg              busy,
    
    // 数据接口
    input  wire [WIDTH-1:0] a,      // 底数 Base
    input  wire [WIDTH-1:0] b,      // 指数 Exponent
    output reg  [WIDTH-1:0] res     // 结果 Result
);

    // --- 参数定义 ---
    localparam [23:0] Q  = 24'd8380417; // Modulus
    // Barrett Constant mu = floor(2^48 / 8380417)
//    localparam [25:0] MU = 26'd33591261; 
    localparam [25:0] MU = 26'd33587228;  
    
    // Barrett 模块的延迟 (prod -> res)
    // 假设使用我们之前修正后的 3 级流水线 Barrett_reduce
    // 加上输入寄存一拍，通常等待 4-5 拍足够。这里设置保守一点。
    localparam [3:0] PIPE_LATENCY = 4'd5;

    // --- 状态机定义 ---
    localparam S_IDLE       = 4'd0;
    localparam S_CHECK_B    = 4'd1; // 检查 b > 0 和 b[0]
    localparam S_CALC_RES   = 4'd2; // 发起 res = res * a
    localparam S_WAIT_RES   = 4'd3; // 等待结果
    localparam S_CALC_SQ    = 4'd4; // 发起 a = a * a
    localparam S_WAIT_SQ    = 4'd5; // 等待结果
    localparam S_UPDATE     = 4'd6; // 更新 b = b >> 1
    localparam S_DONE       = 4'd7;

    reg [3:0] state;
    reg [3:0] wait_cnt;

    // --- 内部寄存器 ---
    reg [WIDTH-1:0] cur_a;   // 当前的 a
    reg [WIDTH-1:0] cur_b;   // 当前的 b
    reg [WIDTH-1:0] cur_res; // 当前的 res

    // --- 乘法与约减接口 ---
    reg  [WIDTH-1:0] mul_op1;
    reg  [WIDTH-1:0] mul_op2;
    
    // DSP 乘法
    (* use_dsp = "yes" *) reg [2*WIDTH-1:0] prod_reg;
    always @(posedge clk) begin
        prod_reg <= mul_op1 * mul_op2;
    end

    // Barrett 约减实例
    wire [WIDTH-1:0] barrett_out;
    Barrett_reduce #( .WIDTH(WIDTH) ) u_barrett (
        .clk (clk),
        .prod(prod_reg),
        .q   (Q),
        .mu  (MU),
        .res (barrett_out)
    );

    // --- 主逻辑 ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= S_IDLE;
            done    <= 0;
            busy    <= 0;
            res     <= 0;
            cur_a   <= 0;
            cur_b   <= 0;
            cur_res <= 1;
            mul_op1 <= 0;
            mul_op2 <= 0;
            wait_cnt<= 0;
        end else begin
            case (state)
                S_IDLE: begin
                    done <= 0;
                    if (start) begin
                        busy    <= 1;
                        cur_a   <= a;
                        cur_b   <= b;
                        cur_res <= 24'd1; // res = 1
                        state   <= S_CHECK_B;
                    end else begin
                        busy <= 0;
                    end
                end

                S_CHECK_B: begin
                    // while b > 0
                    if (cur_b != 0) begin
                        // if bitand(b, 1) -> res = res * a
                        if (cur_b[0] == 1'b1) begin
                            state <= S_CALC_RES;
                        end else begin
                            // Skip res update, go to a = a * a
                            state <= S_CALC_SQ;
                        end
                    end else begin
                        // Loop finished
                        state <= S_DONE;
                    end
                end

                // --- 步骤 1: res = (res * a) % q ---
                S_CALC_RES: begin
                    mul_op1  <= cur_res;
                    mul_op2  <= cur_a;
                    wait_cnt <= PIPE_LATENCY;
                    state    <= S_WAIT_RES;
                end

                S_WAIT_RES: begin
                    if (wait_cnt == 0) begin
                        cur_res <= barrett_out; // 更新 res
                        state   <= S_CALC_SQ;   // 下一步算 a^2
                    end else begin
                        wait_cnt <= wait_cnt - 1;
                    end
                end

                // --- 步骤 2: a = (a * a) % q ---
                S_CALC_SQ: begin
                    mul_op1  <= cur_a;
                    mul_op2  <= cur_a;
                    wait_cnt <= PIPE_LATENCY;
                    state    <= S_WAIT_SQ;
                end

                S_WAIT_SQ: begin
                    if (wait_cnt == 0) begin
                        cur_a <= barrett_out; // 更新 a
                        state <= S_UPDATE;
                    end else begin
                        wait_cnt <= wait_cnt - 1;
                    end
                end

                // --- 步骤 3: b = b >> 1 ---
                S_UPDATE: begin
                    cur_b <= cur_b >> 1;
                    state <= S_CHECK_B; // 回到循环开头
                end

                S_DONE: begin
                    done <= 1;
                    busy <= 0;
                    res  <= cur_res; // 输出最终结果
                    if (start == 0) begin
                        state <= S_IDLE;
                    end
                end
                
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule