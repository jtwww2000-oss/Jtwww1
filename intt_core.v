`timescale 1ns / 1ps

module intt_core #(
    parameter WIDTH = 24
)(
    input  wire          clk,
    input  wire          rst_n,
    input  wire          start,
    output reg           done,
    output reg  [7:0]    mem_addr,
    output reg           mem_we,
    output reg  [WIDTH-1:0]   mem_wdata,
    input  wire [WIDTH-1:0]   mem_rdata,
    
    // --- DEBUG PORTS (新增) ---
    output wire [WIDTH-1:0] dbg_gk,      // 当前旋转因子
    output wire [WIDTH-1:0] dbg_g1,      // 当前阶段基数
    output wire [WIDTH-1:0] dbg_val_u,   // 蝶形输入 u
    output wire [WIDTH-1:0] dbg_val_v,   // 蝶形输入 v
    output wire [WIDTH-1:0] dbg_prod_y,  // 乘法结果 v*gk
    output wire             dbg_butterfly_done // 蝶形运算完成信号
);

    // --- 参数与常量 ---
    localparam [WIDTH-1:0] Q = 24'd8380417;
    localparam [25:0]      MU = 26'd33587228;
    localparam [2:0]       MUL_LATENCY = 3'd5;
    localparam [WIDTH-1:0] N_INV = 24'd8347681;
    localparam [WIDTH-1:0] G_INV = 24'd731434;

    // --- 状态机 ---
    localparam S_IDLE           = 6'd0;
    // Bit Reversal
    localparam S_BR_CHECK       = 6'd1;
    localparam S_BR_READ_A      = 6'd2;
    localparam S_BR_WAIT_A      = 6'd3;
    localparam S_BR_READ_B      = 6'd4;
    localparam S_BR_WAIT_B      = 6'd5;
    localparam S_BR_WRITE_A     = 6'd6;
    localparam S_BR_WRITE_B     = 6'd7;
    // Loops
    localparam S_STAGE_INIT     = 6'd8;
    localparam S_STAGE_INIT_WAIT= 6'd9;
    localparam S_LOOP_I         = 6'd10;
    localparam S_LOOP_J         = 6'd11;
    // Butterfly
    localparam S_READ_U         = 6'd12;
    localparam S_READ_U_WAIT    = 6'd13;
    localparam S_READ_V         = 6'd14;
    localparam S_READ_V_WAIT    = 6'd15;
    localparam S_CALC_PARALLEL  = 6'd16;
    localparam S_CALC_WAIT      = 6'd17;
    localparam S_CALC_ADD       = 6'd18;
    localparam S_WRITE_U        = 6'd19;
    localparam S_WRITE_V        = 6'd20;
    // Post-Process
    localparam S_POST_READ      = 6'd21;
    localparam S_POST_READ_WAIT = 6'd22;
    localparam S_POST_MUL_DAT   = 6'd23;
    localparam S_POST_WAIT_DAT  = 6'd24;
    localparam S_POST_MUL_G     = 6'd25;
    localparam S_POST_WAIT_G    = 6'd26;
    
    localparam S_DONE           = 6'd27;

    reg [5:0] state;
    reg [2:0] wait_cnt;

    // 内部变量
    reg [8:0] cnt;
    reg [7:0] idx_u, idx_rev;
    reg [WIDTH-1:0] temp_val_u;
    reg [8:0] mid, i, j;
    reg [WIDTH-1:0] g1, gk, gk_next;
    reg [WIDTH-1:0] val_u, val_v, prod_y, res_u, res_v;
    reg [WIDTH-1:0] post_factor, post_cal_data;

    // 模块实例化
    wire [WIDTH-1:0] mod_add_out, mod_sub_out;
    mod_add #( .WIDTH(WIDTH) ) u_mod_add ( .a(val_u), .b(prod_y), .q(Q), .res(mod_add_out) );
    mod_sub #( .WIDTH(WIDTH) ) u_mod_sub ( .a(val_u), .b(prod_y), .q(Q), .res(mod_sub_out) );

    wire [WIDTH-1:0] g1_rom_out;
    g1_intt_rom u_g1_rom ( .clk(clk), .mid(mid), .g1(g1_rom_out) );

    reg  [WIDTH-1:0] mul_op1, mul_op2;
    wire [WIDTH-1:0] mul_res;
    (* use_dsp = "yes" *) reg [47:0] prod_reg;
    always @(posedge clk) prod_reg <= mul_op1 * mul_op2;
    Barrett_reduce #( .WIDTH(WIDTH) ) u_barrett_main ( .clk(clk), .prod(prod_reg), .q(Q), .mu(MU), .res(mul_res) );

    reg  [WIDTH-1:0] mul_gk_op1, mul_gk_op2;
    wire [WIDTH-1:0] mul_gk_res;
    (* use_dsp = "yes" *) reg [47:0] prod_reg_gk;
    always @(posedge clk) prod_reg_gk <= mul_gk_op1 * mul_gk_op2;
    Barrett_reduce #( .WIDTH(WIDTH) ) u_barrett_gk ( .clk(clk), .prod(prod_reg_gk), .q(Q), .mu(MU), .res(mul_gk_res) );

    function [7:0] bit_reverse(input [7:0] in);
        integer k;
        begin for (k = 0; k < 8; k = k + 1) bit_reverse[k] = in[7-k]; end
    endfunction

    // --- 连接调试信号 ---
    assign dbg_gk = gk;
    assign dbg_g1 = g1;
    assign dbg_val_u = val_u;
    assign dbg_val_v = val_v;
    assign dbg_prod_y = prod_y;
    // 当写回 V 时，说明本次蝶形运算的所有数据都已稳定有效
    assign dbg_butterfly_done = (state == S_WRITE_V);

    // --- 主逻辑 ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; done <= 0; mem_we <= 0; cnt <= 0; mid <= 1; post_factor <= 0;
        end else begin
            case (state)
                S_IDLE: begin
                    done <= 0; mem_we <= 0;
                    if (start) begin cnt <= 0; state <= S_BR_CHECK; end
                end
                // ... (Bit Reverse 逻辑保持不变) ...
                S_BR_CHECK: begin
                    mem_we <= 0;
                    if (cnt > 255) begin mid <= 1; state <= S_STAGE_INIT; end
                    else begin
                        idx_u = cnt[7:0]; idx_rev = bit_reverse(idx_u);
                        if (idx_u < idx_rev) state <= S_BR_READ_A;
                        else begin cnt <= cnt + 1; state <= S_BR_CHECK; end
                    end
                end
                S_BR_READ_A:  begin mem_addr <= idx_u; state <= S_BR_WAIT_A; end 
                S_BR_WAIT_A:  begin state <= S_BR_READ_B; end
                S_BR_READ_B:  begin temp_val_u <= mem_rdata; mem_addr <= idx_rev; state <= S_BR_WAIT_B; end
                S_BR_WAIT_B:  begin state <= S_BR_WRITE_A; end
                S_BR_WRITE_A: begin mem_we <= 1; mem_addr <= idx_u; mem_wdata <= mem_rdata; state <= S_BR_WRITE_B; end
                S_BR_WRITE_B: begin mem_we <= 1; mem_addr <= idx_rev; mem_wdata <= temp_val_u; cnt <= cnt + 1; state <= S_BR_CHECK; end

                // ... (Loops 逻辑保持不变) ...
                S_STAGE_INIT: begin
                    mem_we <= 0;
                    if (mid < 256) state <= S_STAGE_INIT_WAIT;
                    else begin cnt <= 0; post_factor <= N_INV; state <= S_POST_READ; end
                end
                S_STAGE_INIT_WAIT: begin g1 <= g1_rom_out; i <= 0; state <= S_LOOP_I; end
                S_LOOP_I: begin
                    if (i < 256) begin gk <= 24'd1; j <= 0; state <= S_LOOP_J; end
                    else begin mid <= mid << 1; state <= S_STAGE_INIT; end
                end
                S_LOOP_J: begin
                    if (j < mid) state <= S_READ_U;
                    else begin i <= i + (mid << 1); state <= S_LOOP_I; end
                end

                // ... (Butterfly 逻辑保持不变) ...
                S_READ_U: begin mem_addr <= i + j; state <= S_READ_U_WAIT; end
                S_READ_U_WAIT: begin state <= S_READ_V; end
                S_READ_V: begin val_u <= mem_rdata; mem_addr <= i + j + mid; state <= S_READ_V_WAIT; end
                S_READ_V_WAIT: begin state <= S_CALC_PARALLEL; end
                
                S_CALC_PARALLEL: begin
                    val_v <= mem_rdata;
                    mul_op1 <= mem_rdata; mul_op2 <= gk; 
                    mul_gk_op1 <= gk; mul_gk_op2 <= g1;
                    wait_cnt <= MUL_LATENCY; state <= S_CALC_WAIT;
                end
                S_CALC_WAIT: begin
                    if (wait_cnt == 0) begin prod_y <= mul_res; gk_next <= mul_gk_res; state <= S_CALC_ADD; end
                    else wait_cnt <= wait_cnt - 1;
                end
                S_CALC_ADD: begin res_u <= mod_add_out; res_v <= mod_sub_out; state <= S_WRITE_U; end
                S_WRITE_U: begin mem_we <= 1; mem_addr <= i + j; mem_wdata <= res_u; state <= S_WRITE_V; end
                S_WRITE_V: begin 
                    mem_we <= 1; mem_addr <= i + j + mid; mem_wdata <= res_v; 
                    gk <= gk_next; j <= j + 1; state <= S_LOOP_J; 
                end

                // ... (Post-Process 逻辑保持不变) ...
                S_POST_READ: begin mem_we <= 0; mem_addr <= cnt[7:0]; state <= S_POST_READ_WAIT; end
                S_POST_READ_WAIT: state <= S_POST_MUL_DAT;
                S_POST_MUL_DAT: begin
                    mul_op1 <= mem_rdata; mul_op2 <= post_factor;
                    mul_gk_op1 <= post_factor; mul_gk_op2 <= G_INV;
                    wait_cnt <= MUL_LATENCY; state <= S_POST_WAIT_DAT;
                end
                S_POST_WAIT_DAT: begin
                    if (wait_cnt == 0) begin
                        post_cal_data <= mul_res; post_factor <= mul_gk_res;
                        state <= S_POST_MUL_G;
                    end else wait_cnt <= wait_cnt - 1;
                end
                S_POST_MUL_G: begin mem_we <= 1; mem_addr <= cnt[7:0]; mem_wdata <= post_cal_data; state <= S_POST_WAIT_G; end
                S_POST_WAIT_G: begin
                    mem_we <= 0; cnt <= cnt + 1;
                    if (cnt == 255) begin cnt <= 0; state <= S_DONE; end
                    else state <= S_POST_READ;
                end

                S_DONE: begin done <= 1; if (start == 0) state <= S_IDLE; end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule