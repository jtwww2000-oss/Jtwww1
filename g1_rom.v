`timescale 1ns / 1ps

(* use_dsp48="no" *) // LUT 查找表不需要消耗 DSP
module g1_rom(
    g1,
    mid,
    clk
);

    output reg [23:0] g1;
    input      [8:0]  mid;
    input             clk;

    reg [23:0] lut_val;

    // 组合逻辑查找表
    always @(*) begin
        case(mid)
            // Mid = 1 (Step 9) -> g^256 mod Q = -1
            9'd1:   lut_val = 24'd8380416; 
            
            // Mid = 2 (Step 8) -> g^128 mod Q
            9'd2:   lut_val = 24'd4808194;   
            
            // Mid = 4 (Step 7) -> g^64 mod Q
            9'd4:   lut_val = 24'd3765607; 
            
            // Mid = 8 (Step 6) -> g^32 mod Q
            9'd8:   lut_val = 24'd5178923;  
            
            // Mid = 16 (Step 5) -> g^16 mod Q
            9'd16:  lut_val = 24'd7778734; 
            
            // Mid = 32 (Step 4) -> g^8 mod Q
            9'd32:  lut_val = 24'd5010068; 
            
            // Mid = 64 (Step 3) -> g^4 mod Q
            9'd64:  lut_val = 24'd3602218; 
            
            // Mid = 128 (Step 2) -> g^2 mod Q
            9'd128: lut_val = 24'd3073009; 
            
            // 异常处理：如果 mid=100 或其他非2幂次值，默认返回 1 或报错
            // 在实际 NTT 中，这代表状态机跑飞了
            default: lut_val = 24'd1;
        endcase
    end

    // 时序逻辑输出 (打一拍)
    always @(posedge clk) begin
        g1 <= lut_val;
    end

endmodule