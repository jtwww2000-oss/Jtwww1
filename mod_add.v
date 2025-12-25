`timescale 1ns / 1ps

module mod_add #(
    parameter WIDTH = 24
)(
    input  wire [WIDTH-1:0] a,
    input  wire [WIDTH-1:0] b,
    input  wire [WIDTH-1:0] q, // Modulus
    output wire [WIDTH-1:0] res
);

    wire [WIDTH:0] sum;
    
    // 计算 a + b
    assign sum = a + b;
    
    // 如果和 >= q，则减去 q；否则直接输出
    // 这是一个纯组合逻辑选择器
    assign res = (sum >= q) ? (sum - q) : sum[WIDTH-1:0];

endmodule