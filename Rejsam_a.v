`timescale 1ns / 1ps

module Rejsam_a (
    input  wire                 clk,
    input  wire                 rst_n,
    
    // --- 控制接口 ---
    input  wire                 i_start,        // 启动信号
    input  wire [255:0]         i_rho,          // rho (256 bits)
    input  wire [7:0]           i_row,          // 行索引 row
    input  wire [7:0]           i_column,       // 列索引 column
    
    // --- 输出接口 (流式输出) ---
    output reg                  o_coeff_valid,  // 系数有效标志
    output reg  [22:0]          o_coeff_data,   // 生成的系数 (< 8380417)
    output reg                  o_done          // 完成信号 (生成了 256 个系数)
);

    // --- 参数定义 ---
    localparam Q_VAL = 23'd8380417;
    localparam CNT_TARGET = 9'd256;
    
    // SHAKE128 参数
    localparam RATE_BYTES = 168;            // SHAKE128 Rate = 1344 bits = 168 bytes
    localparam ABSORB_LEN = 256 + 8 + 8;    // rho(256) + col(8) + row(8) = 272 bits

    // --- 状态机 ---
    localparam S_IDLE        = 3'd0;
    localparam S_START_SHAKE = 3'd1;
    localparam S_REQ_SQUEEZE = 3'd2;
    localparam S_WAIT_DATA   = 3'd3;
    localparam S_PROCESS     = 3'd4;
    localparam S_DONE        = 3'd5;
    // 新增状态：等待握手应答
    localparam S_WAIT_ACK    = 3'd6;

    reg [2:0] state;
    
    // --- 内部信号 ---
    reg [8:0] coeff_cnt;                    // 已生成的系数计数 (0-256)
    reg [10:0] bit_ptr;                     // 当前处理到的 bit 指针 (0-1344)
    
    // SHAKE128 接口信号
    reg  shake_start;
    wire [ABSORB_LEN-1:0] shake_seed;
    wire shake_busy;
    reg  shake_squeeze_req;
    wire shake_squeeze_valid;
    wire [RATE_BYTES*8-1:0] shake_out_data;
    
    // 缓存 SHAKE 输出的数据块
    reg [RATE_BYTES*8-1:0] data_buffer;

    // --- 1. 构造输入 Seed ---
    // MATLAB: [rho, column, row] (索引 1 是 LSB)
    // Verilog: {row, column, rho}
    assign shake_seed = {i_row, i_column, i_rho};

    // --- 2. 实例化 SHAKE128 ---
    // 注意：需确保你的 SHAKE128 模块参数名与此处一致
    SHAKE128 #(
        .OUTPUT_LEN_BYTES(RATE_BYTES), // 每次 Squeeze 输出 168 字节
        .ABSORB_LEN(ABSORB_LEN)        // 输入 272 bits
    ) u_shake128 (
        .clk             (clk),
        .rst_n           (rst_n),
        .i_start         (shake_start),
        .i_seed          (shake_seed),
        .o_busy          (shake_busy),
        .i_squeeze_req   (shake_squeeze_req),
        .o_squeeze_valid (shake_squeeze_valid),
        .o_squeeze_data  (shake_out_data)
    );

    // --- 3. 主逻辑 ---
    
    // 候选值提取 (从 buffer 中取 24 bits)
    wire [23:0] raw_chunk;
    wire [22:0] candidate;
    
    // 动态切片: 取 buffer[bit_ptr +: 24]
    // 注意: Verilog 不支持变量作为 +: 的索引基址，通常需要用 generate 或 case，
    // 但在这里我们可以通过移位 buffer 来实现流式处理。
    // 为了节省面积，我们在 S_PROCESS 状态下移动 bit_ptr， combinational logic 提取
    assign raw_chunk = data_buffer[bit_ptr +: 24];
    
    // 对应 MATLAB: bin2dec(flip(...)) 取 23 bits
    // 这里的 raw_chunk[22:0] 即为低 23 位
    assign candidate = raw_chunk[22:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            o_coeff_valid <= 1'b0;
            o_coeff_data  <= 23'd0;
            o_done <= 1'b0;
            shake_start <= 1'b0;
            shake_squeeze_req <= 1'b0;
            coeff_cnt <= 9'd0;
            bit_ptr <= 11'd0;
            data_buffer <= 0;
        end else begin
            // 默认脉冲复位
            shake_start <= 1'b0;
            shake_squeeze_req <= 1'b0;
            o_coeff_valid <= 1'b0; // valid 信号只维持一拍

            case (state)
                S_IDLE: begin
                    o_done <= 1'b0;
                    coeff_cnt <= 9'd0;
                    if (i_start) begin
                        state <= S_START_SHAKE;
                    end
                end

                S_START_SHAKE: begin
                    shake_start <= 1'b1; // 启动 SHAKE 吸收
                    state <= S_REQ_SQUEEZE; // 直接去请求输出 (One-Shot 模式会自动处理 absorb)
                end

                S_REQ_SQUEEZE: begin
                    // 等待 SHAKE 忙完吸收，或者忙完上一次 squeeze
                    // 注意：根据之前的设计，busy 为 1 时不能操作。需等待 busy 变高再变低？
                    // 之前的 SHAKE Controller 设计是：Start -> Busy=1 -> Valid -> SqueezeReq -> Valid...
                    // 这里我们等待 shake_busy 变低(如果它已经握手完成) 或者直接看 squeeze_valid
                    // 为了稳健，我们发送请求
                    // 假设 SHAKE 模块在 absorb 完成后会自动进入 SQUEEZE 状态并等待 req
                    shake_squeeze_req <= 1'b1;
                    state <= S_WAIT_ACK;
                end
                
                
                S_WAIT_ACK: begin
                    shake_squeeze_req <= 1'b0; // 撤销请求脉冲
                    
                    // 在这里停一拍，给 SHAKE 模块时间去响应 req 并把 valid 拉低
                    // 这样进入下一个状态时，valid 就会稳稳地是 0 了
                    if (shake_squeeze_valid == 1'b0) begin
                         state <= S_WAIT_DATA;
                    end
                    // 如果 SHAKE 反应慢，valid 还没变低，就留在这里等它变低
                    // (通常一拍就够，直接 else state <= S_WAIT_DATA 也可以，但加判断更稳健)
                end
                
                
                S_WAIT_DATA: begin
                    if (shake_squeeze_valid) begin
                        data_buffer <= shake_out_data;
                        bit_ptr <= 11'd0; // 重置指针到 buffer 头部 (LSB)
                        state <= S_PROCESS;
                    end
                end

                S_PROCESS: begin
                    // 检查是否 buffer 剩余不足 24 bits
                    // 1344 - 24 = 1320. 如果 ptr > 1320，说明剩下的不够了
                    if (bit_ptr > (RATE_BYTES*8 - 24)) begin
                        // Buffer 用完了，需要更多数据
                        state <= S_REQ_SQUEEZE;
                    end else begin
                        // 检查当前候选值
                        if (candidate < Q_VAL) begin
                            o_coeff_valid <= 1'b1;
                            o_coeff_data <= candidate;
                            coeff_cnt <= coeff_cnt + 1;
                            
                            // 检查是否完成
                            if (coeff_cnt == (CNT_TARGET - 1)) begin
                                state <= S_DONE;
                            end
                        end
                        
                        // 移动指针 (处理下一个 24 bits)
                        // 对应 MATLAB: 循环 i 加 1，每次消耗 24 bits
                        bit_ptr <= bit_ptr + 24; 
                    end
                end

                S_DONE: begin
                    o_done <= 1'b1;
                    // 等待外部复位或新的 start
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