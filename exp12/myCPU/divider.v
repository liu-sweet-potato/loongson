/*
* 32bit恢复余数绝对值迭代除法器
*/
module div (
    input   wire        div_clk,        // 时钟
    input   wire        resetn,         // 低电平复位
    input   wire        div,            // 除法指令
    input   wire        div_signed,     // 高电平有符号数，低电平无符号数
    input   wire [31:0] x,              // 被除数        
    input   wire [31:0] y,              // 除数
    output  wire [31:0] q,              // 商
    output  wire [31:0] r,              // 余数
    output  wire        complete        // 高电平完成，内部计数器33
);

//*********************************管脚定义****************************************
// caculate abs
wire        x_neg;                      // 除数为负数
wire        y_neg;                      // 被除数为负数
wire [31:0] abs_dividend;               // 被除数绝对值
wire [31:0] abs_divisor;                // 除数绝对值

// trial subtraction
wire [32:0] minus_result;               // 试减结果
wire [32:0] iteration_result;           // 迭代结果

// divisor iteration
reg [5:0] count;                        // 状态机计数器
reg [63:0] dividend;                    // 被除数
reg [31:0] quotient;                    // 商

// caculate quotient and remainder
wire        q_neg;                      // 商为负数
wire        r_neg;                      // 余数为负数



//********************************* 连线赋值 ****************************************
// caculate abs
assign x_neg = div_signed && x[31];                                         // 拆分信号，先生成y_neg，再生成abs_divisor，更清晰
assign y_neg = div_signed && y[31];
assign abs_dividend = x_neg ? (~x + 32'h1) : x;
assign abs_divisor = y_neg ? (~y + 32'h1) : y;

// trial subtraction
assign minus_result = dividend[63:31] + ~{1'b0, abs_divisor} + 33'h1;
assign iteration_result  = minus_result[32] ? dividend[63:31] : minus_result;

// divisor iteration
always @ (posedge div_clk) begin
    if(!resetn) begin
        count <= 6'd0;
        dividend <= 64'b0;
        quotient <= 32'b0;
    end
    else begin
        // caculate abs(initial the register at begining)
        if(count==0 && div) begin
            dividend <= {32'b0, abs_dividend};
            quotient <= 32'b0;
            count <= count + 1;
        end
        // divider iteration
        else if(count>=1 && count<=32) begin
            dividend <= {iteration_result[31:0], dividend[30:0], 1'b0};
            quotient <= {quotient[30:0], ~minus_result[32]};                // 这里一开始写错了：先左移，再赋值，最终结果用合线器，不要写错多驱动了
            count <= count + 1;
        end
        // reset count
        else if(count==33) begin
            count <= 0;
        end
    end
end

// caculate quotient and remainder
assign q_neg = div_signed && (x[31] ^ y[31]);
assign r_neg = x_neg;
assign q = q_neg ? (~quotient + 32'h1) : quotient;
assign r = r_neg ? (~dividend[63:32]+32'h1) : dividend[63:32];

// complete
assign complete = (count==33);                                              // 书中模块端口定义有暗示

endmodule