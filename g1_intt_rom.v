`timescale 1ns / 1ps

(* use_dsp48="no" *)
module g1_intt_rom(
    input  wire       clk,
    input  wire [8:0] mid,
    output reg [23:0] g1
);
    reg [23:0] lut_val;

    always @(*) begin
        case(mid)
            // ¼ÆËã¹«Ê½: pow(1753, -256/mid, 8380417)
           9'd1  :   lut_val = 24'd8380416;
9'd2  :   lut_val = 24'd3572223;
9'd4  :   lut_val = 24'd4618904;
9'd8  :   lut_val = 24'd3201430;
9'd16 :   lut_val = 24'd1221177;
9'd32 :   lut_val = 24'd6096684;
9'd64 :   lut_val = 24'd6522001;
9'd128:   lut_val = 24'd6635910; 
            default: lut_val = 24'd1;
        endcase
    end

    always @(posedge clk) begin
        g1 <= lut_val;
    end
endmodule