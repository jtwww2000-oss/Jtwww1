//`timescale 1ns / 1ps

//module pad #(
//    parameter X = 1088,          // Rate (SHAKE256)
//    parameter MAX_LEN = 1088     // 缓冲区大小
//)(
//    input  wire [MAX_LEN-1:0]        N,     // 输入消息数据 N (低位对齐)
//    input  wire [$clog2(MAX_LEN)-1:0] m,     // 输入消息长度 (bits)
//    output reg  [MAX_LEN-1:0]        P      // 输出 P
//);

//    // 定义 SHAKE256 域分隔符 (4 bits)
//    // 注意：这里定义为 4'b1111。
//    // 如果你的 MATLAB 是高位在后的逻辑，这 4 位会被放到 N 的高位方向。
//    localparam [3:0] DOMAIN_SEP = 4'b1111;

//    integer i;

//    always @(*) begin
//        // 1. 初始化 P 为全 0
//        // 这一步非常重要，它自动填充了中间的 0 (j个0)
//        P = {MAX_LEN{1'b0}};

//        // 2. 放置数据 N
//        // N 放在最低位 [0 到 m-1]
//        // 为了综合器友好，这里使用简单的掩码逻辑或直接赋值
//        // (注意：这里假设输入的 N 在 m 位以上本身就是 0，如果不是，需要截断)
//        // 在仿真逻辑中，这等同于: P[m-1:0] = N[m-1:0];
//        // 但 Verilog 不支持变量范围切片，所以我们直接全赋值，依靠后续覆盖或假设 N 高位纯净
//        P = N; 

//        // 3. 放置 SHAKE256 域分隔符 '1111'
//        // 位置：紧接着数据 N 的高位方向
//        // 索引范围：[m+3 : m]
//        // 这完全符合 "1111 是 N 的高位" 这一描述
//        for (i = 0; i < 4; i = i + 1) begin
//            P[m + i] = DOMAIN_SEP[i];
//        end
        
//        // 4. 放置 Padding Start '1'
//        // 位置：在域分隔符的上一位
//        // 索引：m + 4
//        P[m + 4] = 1'b1;

//        // 5. 放置 Padding End '1'
//        // 位置：Rate 的最高位 (X-1)
//        // 对应 MATLAB P 的最后一个元素
//        P[X - 1] = 1'b1;
        
//        // 补充：关于重叠 (Overlap)
//        // 如果 m + 4 刚好等于 X - 1 (即数据填得非常满)，
//        // 上面的赋值会先后对同一个 bit 进行操作。
//        // 因为都是置 1，所以逻辑上是兼容的 (1 | 1 = 1)。
//        // 但为了严谨，Verilog 的阻塞赋值 '=' 后一条会覆盖前一条。
//        // 因为 P[X-1] = 1 是最后执行的，所以无论如何最高位都是 1，符合 Sponge 规范。
//    end

//endmodule


`timescale 1ns / 1ps
module pad #(
    parameter X = 1088,          // Rate
    parameter MAX_LEN = 1088     // Buffer Size
)(
    input  wire [MAX_LEN-1:0]        N,
    input  wire [$clog2(MAX_LEN)-1:0] m,
    output reg  [MAX_LEN-1:0]        P
);

    localparam [3:0] DOMAIN_SEP = 4'b1111;

    always @(*) begin
        // 1. 先复制数据 (假设 N 高位已补0)
        P = N; 
        
        // 2. 放置 SHAKE256 域分隔符 '1111'
        // 使用 Verilog-2001 "Indexed Part Select" 语法: [base +: width]
        // 从 m 开始，向上取 4 位
        P[m +: 4] = DOMAIN_SEP;
        
        // 3. 放置 Padding Start '1'
        // 注意：m+4 是紧接着域分隔符的下一位
        P[m + 4] = 1'b1;
        
        // 4. 放置 Padding End '1' at Rate block boundary
        P[X - 1] = 1'b1;
    end

endmodule