`timescale 1ns / 1ps

module Barrett_reduce #(
    parameter WIDTH = 24
)(
    input  wire               clk,
    input  wire [2*WIDTH-1:0] prod, // 输入乘积 (48 bits)
    input  wire [WIDTH-1:0]   q,    // Modulus Q
    input  wire [WIDTH+1:0]   mu,   // Constant mu (2^48 / Q)
    output reg  [WIDTH-1:0]   res
);

    // --------------------------------------------------------
    // Stage 1: 估算商的第一步 big_prod = prod * mu
    // --------------------------------------------------------
    (* use_dsp = "yes" *) reg [3*WIDTH+1:0] r1_big_prod;
    reg [2*WIDTH-1:0] r1_prod; // 传递原始输入
    reg [WIDTH-1:0]   r1_q;

    always @(posedge clk) begin
        r1_big_prod <= prod * mu;
        r1_prod     <= prod;
        r1_q        <= q;
    end

    // --------------------------------------------------------
    // Stage 2: 计算减数 sub_term = floor(...) * q
    // --------------------------------------------------------
    (* use_dsp = "yes" *) reg [WIDTH+2:0] r2_q_times_qhat;
    reg [WIDTH+2:0]   r2_prod_low; // 只需要保留低位用于减法
    reg [WIDTH-1:0]   r2_q;

    wire [WIDTH+2:0] q_hat;
    
    // ★★★ 修正点：必须右移 48 位 (匹配 mu = 2^48 / Q) ★★★
    assign q_hat = r1_big_prod >> 48; 

    always @(posedge clk) begin
        r2_q_times_qhat <= q_hat * r1_q;
        // 我们只需要 prod 的低位来进行最后的减法比较
        // 取 27 位足够容纳差异 (WIDTH + 3)
        r2_prod_low     <= r1_prod[WIDTH+2:0]; 
        r2_q            <= r1_q;
    end

    // --------------------------------------------------------
    // Stage 3: 最终减法与修正
    // --------------------------------------------------------
    wire [WIDTH+2:0] r_raw;
    wire [WIDTH+2:0] r_corr;
    wire [WIDTH-1:0] r_final;

    assign r_raw = r2_prod_low - r2_q_times_qhat;
    
    // 修正逻辑
    assign r_corr  = (r_raw >= r2_q) ? (r_raw - r2_q) : r_raw;
    assign r_final = (r_corr >= r2_q) ? (r_corr - r2_q) : r_corr[WIDTH-1:0];

    always @(posedge clk) begin
        res <= r_final;
    end

endmodule