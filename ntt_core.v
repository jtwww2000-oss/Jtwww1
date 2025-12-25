`timescale 1ns / 1ps

module ntt_core #(
    parameter WIDTH = 24
)(
    input  wire          clk,
    input  wire          rst_n,
    input  wire          start,
    output reg           done,
    
    // --- 适配 ram_24x256 IP 核的双端口接口 ---
    // Port A
    output reg  [7:0]    ram_addr_a,
    output reg           ram_we_a,       // 对应 IP 核的 wea[0:0]
    output reg  [WIDTH-1:0] ram_wdata_a,
    input  wire [WIDTH-1:0] ram_rdata_a,
    
    // Port B
    output reg  [7:0]    ram_addr_b,
    output reg           ram_we_b,       // 对应 IP 核的 web[0:0]
    output reg  [WIDTH-1:0] ram_wdata_b,
    input  wire [WIDTH-1:0] ram_rdata_b,
    
    // --- 调试端口 (保持不变) ---
    output wire [WIDTH-1:0] dbg_gk,
    output wire [WIDTH-1:0] dbg_g1,
    output wire [WIDTH-1:0] dbg_val_u,
    output wire [WIDTH-1:0] dbg_val_v,
    output wire [WIDTH-1:0] dbg_prod_y,
    output wire             dbg_butterfly_done
);

    // --- 1. 参数与常量 ---
    localparam [WIDTH-1:0] Q = 24'd8380417;
    localparam [WIDTH-1:0] G = 24'd1753;
    localparam [25:0]      MU = 26'd33587228;
    // 乘法器流水线延迟 (Barrett reduce 内部有寄存)
    localparam [2:0]       MUL_LATENCY = 3'd5;

    // --- 2. 状态机定义 ---
    localparam S_IDLE           = 6'd0;
    
    // 预处理 (Pre-treatment) - 依然使用单端口模式(Port A)处理
    localparam S_PRE_READ       = 6'd1;
    localparam S_PRE_READ_WAIT  = 6'd2; // 等待 RAM 读延迟 (1 clk)
    localparam S_PRE_MUL_DAT    = 6'd3;
    localparam S_PRE_WAIT_DAT   = 6'd4;
    localparam S_PRE_MUL_G      = 6'd5;
    localparam S_PRE_WAIT_G     = 6'd6;
    
    // Bit Reversal (位反转) - ★双端口并行优化★
    localparam S_BR_CHECK       = 6'd7;
    localparam S_BR_READ_AB     = 6'd8; // 并行读 A, B
    localparam S_BR_WAIT_AB     = 6'd9; // 等待读数据
    localparam S_BR_WRITE_AB    = 6'd10;// 并行写 A, B (交换)
    
    // NTT Loops (层级循环)
    localparam S_STAGE_INIT     = 6'd11;
    localparam S_STAGE_INIT_WAIT= 6'd12;
    localparam S_LOOP_I         = 6'd13;
    localparam S_LOOP_J         = 6'd14;
    
    // 蝶形运算 (Butterfly) - ★双端口并行优化★
    localparam S_READ_UV        = 6'd15; // 并行读 U, V
    localparam S_READ_UV_WAIT   = 6'd16; // 等待读数据
    localparam S_CALC_MUL_REQ   = 6'd17; // 锁存数据并启动乘法
    localparam S_CALC_MUL_WAIT  = 6'd18; // 等待乘法结果
    localparam S_CALC_ADD       = 6'd19; // 加减运算
    localparam S_WRITE_UV       = 6'd20; // 并行写回结果 U, V
    
    // 更新旋转因子
    localparam S_UPD_GK_REQ     = 6'd21;
    localparam S_UPD_GK_WAIT    = 6'd22;
    
    localparam S_DONE           = 6'd23;

    reg [5:0] state;
    reg [2:0] wait_cnt;

    // --- 3. 内部变量 ---
    reg [8:0] cnt;
    reg [WIDTH-1:0] g_pow;
    reg [7:0] idx_u, idx_rev;
    
    reg [8:0] mid;
    reg [8:0] i, j;
    reg [WIDTH-1:0] g1, gk;
    reg [WIDTH-1:0] val_u, val_v, prod_y, res_u, res_v;
    reg [WIDTH-1:0] pre_cal_data;

    // --- 4. 运算单元实例化 ---
    wire [WIDTH-1:0] mod_add_out, mod_sub_out;
    // 实例化模加减模块 (注意文件名需对应你的工程)
    mod_add #( .WIDTH(WIDTH) ) u_mod_add ( .a(val_u), .b(prod_y), .q(Q), .res(mod_add_out) );
    mod_sub #( .WIDTH(WIDTH) ) u_mod_sub ( .a(val_u), .b(prod_y), .q(Q), .res(mod_sub_out) );

    wire [WIDTH-1:0] g1_rom_out;
    g1_rom u_g1_rom ( .clk(clk), .mid(mid), .g1(g1_rom_out) );

    // 乘法与 Barrett 约减
    reg  [WIDTH-1:0] mul_op1, mul_op2;
    wire [WIDTH-1:0] mul_res;
    (* use_dsp = "yes" *) reg [47:0] prod_reg;
    
    always @(posedge clk) prod_reg <= mul_op1 * mul_op2;
    
    Barrett_reduce #( .WIDTH(WIDTH) ) u_barrett (
        .clk(clk), .prod(prod_reg), .q(Q), .mu(MU), .res(mul_res)
    );

    // --- 5. 辅助函数 ---
    function [7:0] bit_reverse(input [7:0] in);
        integer k;
        begin
            for (k = 0; k < 8; k = k + 1) bit_reverse[k] = in[7-k];
        end
    endfunction

    // 调试信号连接
    assign dbg_gk = gk;
    assign dbg_g1 = g1;
    assign dbg_val_u = val_u;
    assign dbg_val_v = val_v;
    assign dbg_prod_y = prod_y;
    assign dbg_butterfly_done = (state == S_WRITE_UV);

    // --- 6. 主状态机逻辑 ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            done <= 0;
            // RAM 信号复位
            ram_we_a <= 0; ram_addr_a <= 0; ram_wdata_a <= 0;
            ram_we_b <= 0; ram_addr_b <= 0; ram_wdata_b <= 0;
            // 内部变量复位
            cnt <= 0; g_pow <= 1;
            mid <= 1; mul_op1 <= 0; mul_op2 <= 0;
            val_u <= 0; val_v <= 0;
        end else begin
            case (state)
                S_IDLE: begin
                    done <= 0;
                    ram_we_a <= 0; ram_we_b <= 0;
                    if (start) begin
                        cnt <= 0;
                        g_pow <= 24'd1; 
                        state <= S_PRE_READ;
                    end
                end

                // ==================================================
                // 1. 预处理 (Pre-treatment)
                // 使用单端口模式 (Port A) 顺序处理
                // ==================================================
                S_PRE_READ: begin
                    ram_we_a <= 0;
                    ram_addr_a <= cnt[7:0];
                    state <= S_PRE_READ_WAIT;
                end
                S_PRE_READ_WAIT: begin
                    // 等待 IP 核读延迟 (Latency=1)
                    state <= S_PRE_MUL_DAT;
                end
                S_PRE_MUL_DAT: begin
                    // 此时 ram_rdata_a 数据有效
                    mul_op1 <= ram_rdata_a;
                    mul_op2 <= g_pow; 
                    wait_cnt <= MUL_LATENCY;
                    state <= S_PRE_WAIT_DAT;
                end
                S_PRE_WAIT_DAT: begin
                    if (wait_cnt == 0) begin
                        pre_cal_data <= mul_res; // 结果暂存
                        // 准备计算下一个 g_pow (g_pow * G)
                        mul_op1 <= g_pow; mul_op2 <= G;
                        wait_cnt <= MUL_LATENCY;
                        state <= S_PRE_MUL_G;
                    end else wait_cnt <= wait_cnt - 1;
                end
                S_PRE_MUL_G: begin
                    // 将预处理后的系数写回 RAM
                    ram_we_a <= 1;
                    ram_addr_a <= cnt[7:0];
                    ram_wdata_a <= pre_cal_data;
                    state <= S_PRE_WAIT_G;
                end
                S_PRE_WAIT_G: begin
                    ram_we_a <= 0; // 停止写
                    if (wait_cnt == 0) begin
                        g_pow <= mul_res; // 更新 g_pow
                        cnt <= cnt + 1;
                        if (cnt == 255) begin
                            cnt <= 0;
                            state <= S_BR_CHECK; 
                        end else state <= S_PRE_READ;
                    end else wait_cnt <= wait_cnt - 1;
                end

                // ==================================================
                // 2. 位反转 (Bit Reversal) - 双端口并行优化
                // ==================================================
                S_BR_CHECK: begin
                    ram_we_a <= 0; ram_we_b <= 0;
                    if (cnt > 255) begin
                        mid <= 1;
                        state <= S_STAGE_INIT;
                    end else begin
                        idx_u = cnt[7:0];
                        idx_rev = bit_reverse(idx_u);
                        
                        // 只有当 idx_u < idx_rev 时才交换，避免重复交换或自身交换
                        if (idx_u < idx_rev) begin
                            state <= S_BR_READ_AB;
                        end else begin
                            cnt <= cnt + 1;
                            state <= S_BR_CHECK;
                        end
                    end
                end
                
                S_BR_READ_AB: begin
                    // 利用 Port A 和 Port B 同时读取两个位置
                    ram_addr_a <= idx_u;
                    ram_addr_b <= idx_rev;
                    state <= S_BR_WAIT_AB;
                end
                
                S_BR_WAIT_AB: begin
                    // 等待 IP 核读延迟
                    state <= S_BR_WRITE_AB;
                end
                
                S_BR_WRITE_AB: begin
                    // 交叉写回：数据互换
                    // A端口写入原来B位置的数据，B端口写入原来A位置的数据
                    ram_we_a <= 1;
                    ram_addr_a <= idx_u;
                    ram_wdata_a <= ram_rdata_b; // 读出的 data_rev
                    
                    ram_we_b <= 1;
                    ram_addr_b <= idx_rev;
                    ram_wdata_b <= ram_rdata_a; // 读出的 data_u
                    
                    cnt <= cnt + 1;
                    state <= S_BR_CHECK;
                end

                // ==================================================
                // 3. NTT 核心循环
                // ==================================================
                S_STAGE_INIT: begin
                    ram_we_a <= 0; ram_we_b <= 0;
                    if (mid < 256) state <= S_STAGE_INIT_WAIT;
                    else state <= S_DONE;
                end
                S_STAGE_INIT_WAIT: begin
                    g1 <= g1_rom_out; // 从 ROM 获取当前 stage 的 g1
                    i <= 0;
                    state <= S_LOOP_I;
                end
                S_LOOP_I: begin
                    if (i < 256) begin
                        gk <= 24'd1;
                        j <= 0; state <= S_LOOP_J;
                    end else begin
                        mid <= mid << 1; // mid 翻倍
                        state <= S_STAGE_INIT;
                    end
                end
                S_LOOP_J: begin
                    if (j < mid) state <= S_READ_UV;
                    else begin 
                        i <= i + (mid << 1); 
                        state <= S_LOOP_I;
                    end
                end

                // ==================================================
                // 4. 蝶形运算 (Butterfly) - 双端口并行优化
                // ==================================================
                S_READ_UV: begin
                    // 并行发起读请求
                    // U: 地址 i + j
                    // V: 地址 i + j + mid
                    ram_addr_a <= i + j;
                    ram_addr_b <= i + j + mid;
                    state <= S_READ_UV_WAIT;
                end
                
                S_READ_UV_WAIT: begin
                    // 等待读延迟
                    state <= S_CALC_MUL_REQ;
                end
                
                S_CALC_MUL_REQ: begin
                    // 数据已就绪，锁存输入
                    val_u <= ram_rdata_a; // U
                    val_v <= ram_rdata_b; // V
                    
                    // 启动乘法: V * gk
                    mul_op1 <= ram_rdata_b; 
                    mul_op2 <= gk; 
                    wait_cnt <= MUL_LATENCY;
                    state <= S_CALC_MUL_WAIT;
                end
                
                S_CALC_MUL_WAIT: begin
                    if (wait_cnt == 0) begin
                        prod_y <= mul_res; // 得到乘积结果
                        state <= S_CALC_ADD;
                    end else wait_cnt <= wait_cnt - 1;
                end
                
                S_CALC_ADD: begin
                    // 组合逻辑计算 res_u = u + y, res_v = u - y
                    // 此时 mod_add/mod_sub 模块已经有了结果
                    res_u <= mod_add_out;
                    res_v <= mod_sub_out;
                    state <= S_WRITE_UV;
                end
                
                S_WRITE_UV: begin
                    // 并行写回结果
                    // Port A 写 U
                    ram_we_a <= 1;
                    ram_addr_a <= i + j;
                    ram_wdata_a <= res_u;
                    
                    // Port B 写 V
                    ram_we_b <= 1;
                    ram_addr_b <= i + j + mid;
                    ram_wdata_b <= res_v;
                    
                    state <= S_UPD_GK_REQ;
                end
                
                // --- 更新 gk = gk * g1 ---
                S_UPD_GK_REQ: begin
                    ram_we_a <= 0; ram_we_b <= 0;
                    mul_op1 <= gk; mul_op2 <= g1; 
                    wait_cnt <= MUL_LATENCY;
                    state <= S_UPD_GK_WAIT;
                end
                S_UPD_GK_WAIT: begin
                    if (wait_cnt == 0) begin
                        gk <= mul_res;
                        j <= j + 1;
                        state <= S_LOOP_J;
                    end else wait_cnt <= wait_cnt - 1;
                end

                S_DONE: begin
                    done <= 1;
                    if (start == 0) state <= S_IDLE;
                end
                
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule