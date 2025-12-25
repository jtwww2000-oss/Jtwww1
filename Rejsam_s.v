`timescale 1ns / 1ps

module Rejsam_s (
    input  wire                 clk,
    input  wire                 rst_n,
    
    // --- 控制接口 ---
    input  wire                 i_start,          // 启动信号
    input  wire [2:0]           i_security_level, // 安全等级 (2, 3, 5)
    input  wire [511:0]         i_rho_prime,      // rho' (512 bits)
    input  wire [15:0]          i_row,            // 行索引 (16 bits)
    
    // --- 输出接口 (流式输出 256 个系数) ---
    output reg                  o_coeff_valid,    // 系数有效标志
    output reg  [3:0]           o_coeff_data,     // 系数 (最大为 8, 所以 4 bits 足够)
    output reg                  o_done            // 完成信号
);

    // --- 参数定义 ---
    localparam CNT_TARGET = 9'd256;
    
    // SHAKE256 参数
    localparam RATE_BYTES = 136;             // SHAKE256 Rate = 1088 bits = 136 bytes
    localparam ABSORB_LEN = 512 + 16;        // rho'(512) + row(16) = 528 bits

    // --- 状态机 ---
    localparam S_IDLE        = 3'd0;
    localparam S_START_SHAKE = 3'd1;
    localparam S_REQ_SQUEEZE = 3'd2;
    localparam S_WAIT_ACK    = 3'd3;         // 关键：等待握手应答
    localparam S_WAIT_DATA   = 3'd4;
    localparam S_PROCESS     = 3'd5;
    localparam S_DONE        = 3'd6;

    reg [2:0] state;
    
    // --- 内部信号 ---
    reg [8:0] coeff_cnt;                     // 计数器 (0-256)
    reg [10:0] bit_ptr;                      // 指针 (0-1088)
    
    // SHAKE256 接口
    reg  shake_start;
    wire [ABSORB_LEN-1:0] shake_seed;
    wire shake_busy;
    reg  shake_squeeze_req;
    wire shake_squeeze_valid;
    wire [RATE_BYTES*8-1:0] shake_out_data;
    
    // 缓存
    reg [RATE_BYTES*8-1:0] data_buffer;

    // --- 1. 构造输入 Seed ---
    // MATLAB: [rho', row_matrix] (rho' 在低位)
    // Verilog: {row, rho_prime}
    assign shake_seed = {i_row, i_rho_prime};

    // --- 2. 实例化 SHAKE256 ---
    SHAKE256 #(
        .RATE(1088),                    // SHAKE256 Rate
        .OUTPUT_LEN_BYTES(RATE_BYTES),  // 每次输出 136 字节
        .ABSORB_LEN(ABSORB_LEN)         // 输入 528 bits
    ) u_shake256 (
        .clk             (clk),
        .rst_n           (rst_n),
        .i_start         (shake_start),
        .i_seed          (shake_seed),
        .o_busy          (shake_busy),
        .i_squeeze_req   (shake_squeeze_req),
        .o_squeeze_valid (shake_squeeze_valid),
        .o_squeeze_data  (shake_out_data)
    );

    // --- 3. 采样判定逻辑 (组合逻辑) ---
    wire [3:0] raw_nibble;
    reg        is_valid_candidate;
    reg [3:0]  final_coeff;

    // 动态提取 4 bits
    assign raw_nibble = data_buffer[bit_ptr +: 4];

    // 根据安全等级判定
    always @(*) begin
        is_valid_candidate = 1'b0;
        final_coeff = 4'd0;

        if (i_security_level == 3'd3) begin
            // Level 3 规则: < 9 直接输出
            if (raw_nibble < 9) begin
                is_valid_candidate = 1'b1;
                final_coeff = raw_nibble;
            end
        end else begin
            // Level 2/5 规则: < 15 输出 mod 5
            if (raw_nibble < 15) begin
                is_valid_candidate = 1'b1;
                // 计算 mod 5 (因为最大是 14，可以用简单的减法代替除法)
                if (raw_nibble >= 10)
                    final_coeff = raw_nibble - 10;
                else if (raw_nibble >= 5)
                    final_coeff = raw_nibble - 5;
                else
                    final_coeff = raw_nibble;
            end
        end
    end

    // --- 4. 主状态机 ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            o_coeff_valid <= 1'b0;
            o_coeff_data  <= 4'd0;
            o_done <= 1'b0;
            shake_start <= 1'b0;
            shake_squeeze_req <= 1'b0;
            coeff_cnt <= 9'd0;
            bit_ptr <= 11'd0;
            data_buffer <= 0;
        end else begin
            shake_start <= 1'b0;
            shake_squeeze_req <= 1'b0; // 脉冲
            o_coeff_valid <= 1'b0;

            case (state)
                S_IDLE: begin
                    o_done <= 1'b0;
                    coeff_cnt <= 9'd0;
                    if (i_start) begin
                        state <= S_START_SHAKE;
                    end
                end

                S_START_SHAKE: begin
                    shake_start <= 1'b1; 
                    // 启动后直接请求数据，SHAKE256 模块会自动处理 Absorb -> Squeeze 的转换
                    state <= S_REQ_SQUEEZE; 
                end

                S_REQ_SQUEEZE: begin
                    shake_squeeze_req <= 1'b1;
                    state <= S_WAIT_ACK; // 跳到 ACK 状态
                end

                // --- 关键的等待应答状态 ---
                S_WAIT_ACK: begin
                    shake_squeeze_req <= 1'b0;
                    // 等待 SHAKE 模块把 valid 拉低 (响应了请求)
                    if (shake_squeeze_valid == 1'b0) begin
                        state <= S_WAIT_DATA;
                    end
                end

                S_WAIT_DATA: begin
                    if (shake_squeeze_valid) begin
                        data_buffer <= shake_out_data;
                        bit_ptr <= 11'd0; // 重置指针
                        state <= S_PROCESS;
                    end
                end

                S_PROCESS: begin
                    // 检查 Buffer 是否耗尽
                    // Rate 1088 bits. 每次取 4 bits. 
                    // 当 bit_ptr > (1088 - 4) = 1084 时，不够取了
                    if (bit_ptr > (RATE_BYTES*8 - 4)) begin
                        state <= S_REQ_SQUEEZE;
                    end else begin
                        // 处理当前 Nibble
                        if (is_valid_candidate) begin
                            o_coeff_valid <= 1'b1;
                            o_coeff_data  <= final_coeff;
                            coeff_cnt <= coeff_cnt + 1;

                            if (coeff_cnt == (CNT_TARGET - 1)) begin
                                state <= S_DONE;
                            end
                        end
                        
                        // 移动指针 (每次 4 bits)
                        bit_ptr <= bit_ptr + 4;
                    end
                end

                S_DONE: begin
                    o_done <= 1'b1;
                    if (i_start) begin
                        state <= S_START_SHAKE;
                        o_done <= 1'b0;
                        coeff_cnt <= 9'd0;
                    end
                end
            endcase
        end
    end

endmodule