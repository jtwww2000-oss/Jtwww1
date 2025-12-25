`timescale 1ns / 1ps

module mod_sub #(
    parameter WIDTH = 24
)(
    input  wire [WIDTH-1:0] a,
    input  wire [WIDTH-1:0] b,
    input  wire [WIDTH-1:0] q, // Modulus
    output wire [WIDTH-1:0] res
);

    wire [WIDTH:0] sub_res;

    // 如果 a >= b，直接减
    // 如果 a < b，结果是负数，在模域下等同于 a + q - b
    // 我们可以统一写成：如果 a >= b, res = a - b; else res = a + q - b;
    
    assign sub_res = (a >= b) ? (a - b) : (a + q - b);
    
    assign res = sub_res[WIDTH-1:0];

endmodule