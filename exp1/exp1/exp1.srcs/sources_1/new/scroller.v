`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2017/12/11 00:10:26
// Design Name: 
// Module Name: scroller
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module scroller #(
    // 参数定义部分
    parameter CNT_1S = 27'd100_000_000   // 主频100MHZ
)(
    // 引脚定义部分
    input         clk,
    input         resetn,
    output reg [15:0] led
);

reg [26:0] cnt;                          // 计数器
wire cnt_eq_1s;                          // 判断条件
assign cnt_eq_1s = cnt==CNT_1S;

// 计数器
always @(posedge clk, negedge resetn)    // 异步重置清零 
begin
    if (!resetn)                         // 低电平触发
    begin
        cnt <= 27'd0;
    end
    else if (cnt_eq_1s)
    begin
        cnt <= 27'd0;
    end
    else
    begin
        cnt <= cnt + 1'b1;
    end
end

// 跑马灯
always @(posedge clk, negedge resetn)
begin
    if (!resetn)
    begin
        led <= 16'hfffe;
    end
    else if (cnt_eq_1s)
    begin
        led <= {led[14:0],led[15]};
    end
end
endmodule
