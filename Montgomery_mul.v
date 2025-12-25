`timescale 1ns / 1ps

module Montgomery_mul #(
    parameter WIDTH = 24
)(
    input  wire             clk,
    // input  wire             rst_n, // 数据通路通常不需要复位，节省布线资源
    input  wire [WIDTH-1:0] a,
    input  wire [WIDTH-1:0] b,
    input  wire [WIDTH-1:0] q,       // Modulus Q
    input  wire [WIDTH-1:0] q_prime, // Q'
    output reg  [WIDTH-1:0] res
);

    // --------------------------------------------------------
    // Stage 1: 计算 prod = A * B
    // --------------------------------------------------------
    (* use_dsp = "yes" *) reg [2*WIDTH-1:0] r1_prod;
    
    // 同时也需要把 q 和 q_prime 传递下去，或者假设它们是常数端口
    // 这里为了通用性，通过寄存器传递（如果 q 是常数，综合器会优化掉）
    reg [WIDTH-1:0] r1_q, r1_q_prime;

    always @(posedge clk) begin
        r1_prod    <= a * b;
        r1_q       <= q;
        r1_q_prime <= q_prime;
    end

    // --------------------------------------------------------
    // Stage 2: 计算 m = (prod * Q') mod R
    // --------------------------------------------------------
    (* use_dsp = "yes" *) reg [WIDTH-1:0] r2_m;
    reg [2*WIDTH-1:0] r2_prod; // 将 prod 传递到下一级
    reg [WIDTH-1:0]   r2_q;

    always @(posedge clk) begin
        // 注意：这里只取乘积的低 WIDTH 位
        r2_m    <= r1_prod[WIDTH-1:0] * r1_q_prime; 
        r2_prod <= r1_prod;
        r2_q    <= r1_q;
    end

    // --------------------------------------------------------
    // Stage 3: 计算 mq = m * Q
    // --------------------------------------------------------
    (* use_dsp = "yes" *) reg [2*WIDTH-1:0] r3_mq;
    reg [2*WIDTH-1:0] r3_prod;
    reg [WIDTH-1:0]   r3_q;

    always @(posedge clk) begin
        r3_mq   <= r2_m * r2_q;
        r3_prod <= r2_prod;
        r3_q    <= r2_q;
    end

    // --------------------------------------------------------
    // Stage 4: 最终加法与归约
    // --------------------------------------------------------
    // 组合逻辑部分
    wire [2*WIDTH:0] sum_comb;
    wire [WIDTH:0]   t_comb;
    wire [WIDTH-1:0] res_comb;

    assign sum_comb = r3_prod + r3_mq;
    assign t_comb   = sum_comb[2*WIDTH : WIDTH]; // 除以 R (右移)
    
    // 最后的条件减法
    assign res_comb = (t_comb >= r3_q) ? (t_comb - r3_q) : t_comb[WIDTH-1:0];

    always @(posedge clk) begin
        res <= res_comb;
    end

endmodule