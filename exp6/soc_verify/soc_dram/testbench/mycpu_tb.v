/*------------------------------------------------------------------------------
--------------------------------------------------------------------------------
Copyright (c) 2016, Loongson Technology Corporation Limited.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this 
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, 
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

3. Neither the name of Loongson Technology Corporation Limited nor the names of 
its contributors may be used to endorse or promote products derived from this 
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL LOONGSON TECHNOLOGY CORPORATION LIMITED BE LIABLE
TO ANY PARTY FOR DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE 
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--------------------------------------------------------------------------------
------------------------------------------------------------------------------*/
// 定义仿真时间单位为1ns，时间精度为1ps（用于控制仿真中时间延迟的精度）
`timescale 1ns / 1ps

// 宏定义：参考跟踪文件路径（存储预期的指令执行结果，用于对比验证）
`define TRACE_REF_FILE "../../../../../../../../gettrace/golden_trace.txt"
// 宏定义：通过层次化路径引用SOC内部配置寄存器的数码管数据信号
`define CONFREG_NUM_REG      soc_lite.u_confreg.num_data
// 宏定义：引用配置寄存器的"开启跟踪功能"信号
`define CONFREG_OPEN_TRACE   soc_lite.u_confreg.open_trace
// 宏定义：引用配置寄存器的"数码管监控使能"信号
`define CONFREG_NUM_MONITOR  soc_lite.u_confreg.num_monitor
// 宏定义：引用配置寄存器的"串口输出有效"信号
`define CONFREG_UART_DISPLAY soc_lite.u_confreg.write_uart_valid
// 宏定义：引用配置寄存器的"串口输出数据"信号
`define CONFREG_UART_DATA    soc_lite.u_confreg.write_uart_data
// 宏定义：程序结束的PC（程序计数器）值，用于判断测试是否完成
`define END_PC 32'h1c000100

// 测试平台顶层模块（无输入输出端口，因为是仿真顶层）
module tb_top( );
// 定义复位信号（低电平有效）
reg resetn;
// 定义时钟信号
reg clk;

// 定义GPIO相关信号（外设接口）
// 16位LED灯信号
wire [15:0] led;
// 2位RGB LED0信号
wire [1 :0] led_rg0;
// 2位RGB LED1信号
wire [1 :0] led_rg1;
// 8位数码管片选信号（低电平有效，控制哪个数码管点亮）
wire [7 :0] num_csn;
// 7位数码管段选信号（控制数码管显示的字符）
wire [6 :0] num_a_g;
// 8位拨码开关信号
wire [7 :0] switch;
// 4位按键列信号
wire [3 :0] btn_key_col;
// 4位按键行信号
wire [3 :0] btn_key_row;
// 2位步进按键信号
wire [1 :0] btn_step;
// 给拨码开关赋值：全为高电平（模拟开关全部打开）
assign switch      = 8'hff;
// 给按键行信号赋值：全为0（模拟无按键按下）
assign btn_key_row = 4'd0;
// 给步进按键赋值：2'd3（模拟步进功能使能）
assign btn_step    = 2'd3;

// 初始化块：生成复位信号和初始时钟状态
initial
begin
    // 初始时钟为低电平
    clk = 1'b0;
    // 初始复位有效（低电平）
    resetn = 1'b0;
    // 延迟2000ns（等待系统稳定）
    #2000;
    // 释放复位（高电平）
    resetn = 1'b1;
end
// 时钟生成：每5ns翻转一次，生成周期10ns的时钟
always #5 clk=~clk;

// 实例化被测试的SOC模块（soc_lite_top），实例名为soc_lite
// 参数SIMULATION=1'b1表示当前为仿真模式（可能关闭硬件专用逻辑）
soc_lite_top #(.SIMULATION(1'b1)) soc_lite
(
       .resetn      (resetn     ),  // 连接复位信号
       .clk         (clk        ),  // 连接时钟信号
    
        //------gpio接口连接-------
        .num_csn    (num_csn    ),  // 数码管片选信号
        .num_a_g    (num_a_g    ),  // 数码管段选信号
        .led        (led        ),  // LED灯信号
        .led_rg0    (led_rg0    ),  // RGB LED0信号
        .led_rg1    (led_rg1    ),  // RGB LED1信号
        .switch     (switch     ),  // 拨码开关信号
        .btn_key_col(btn_key_col),  // 按键列信号
        .btn_key_row(btn_key_row),  // 按键行信号
        .btn_step   (btn_step   )   // 步进按键信号
    );   

// SOC内部调试信号引出（用于监控CPU执行状态）
// "soc_clk"表示CPU核心时钟
// "wb"表示流水线的写回阶段
// "rf"表示CPU的寄存器堆
// "w"在"wen/wnum/wdata"中表示"写操作"（write）
// 定义CPU核心时钟信号
wire soc_clk;
// 定义写回阶段的PC（程序计数器）值信号（当前执行的指令地址）
wire [31:0] debug_wb_pc;
// 定义寄存器堆写使能信号（4位，每bit对应1个字节的写使能）
wire [3 :0] debug_wb_rf_we;
// 定义写寄存器号信号（0-31，对应32个通用寄存器）
wire [4 :0] debug_wb_rf_wnum;
// 定义写入寄存器的数据信号
wire [31:0] debug_wb_rf_wdata;
// 从SOC实例中引出CPU时钟
assign soc_clk           = soc_lite.cpu_clk;
// 从SOC实例中引出写回阶段的PC值
assign debug_wb_pc       = soc_lite.debug_wb_pc;
// 从SOC实例中引出寄存器堆写使能
assign debug_wb_rf_we    = soc_lite.debug_wb_rf_we;
// 从SOC实例中引出写寄存器号
assign debug_wb_rf_wnum  = soc_lite.debug_wb_rf_wnum;
// 从SOC实例中引出写入寄存器的数据
assign debug_wb_rf_wdata = soc_lite.debug_wb_rf_wdata;

// 打开参考跟踪文件（用于存储预期结果）
integer trace_ref;  // 定义文件句柄，用于操作参考跟踪文件
initial begin
	// 生成VCD波形文件（dump.vcd），用于仿真后查看波形
	$dumpfile("dump.vcd");
	// 记录soc_lite模块的所有信号到波形文件
	$dumpvars(0, soc_lite); 
    // 以只读模式打开参考跟踪文件，获取文件句柄
    trace_ref = $fopen(`TRACE_REF_FILE, "r");
end

// 在时钟上升沿读取参考跟踪文件中的预期结果
reg        trace_cmp_flag;  // 标记是否读取到有效参考数据
reg        debug_end;       // 标记测试是否结束

// 定义参考结果的寄存器（存储从文件中读取的预期值）
reg [31:0] ref_wb_pc;       // 参考的写回阶段PC值
reg [4 :0] ref_wb_rf_wnum;  // 参考的写寄存器号
reg [31:0] ref_wb_rf_wdata; // 参考的写入寄存器数据
integer a;  // 用于接收$fscanf的返回值（读取的字段数）
always @(posedge soc_clk)  // 在CPU时钟上升沿触发
begin 
    #1;  // 延迟1ns（避开时钟沿竞争，确保信号稳定）
    // 当满足以下条件时，读取参考文件：
    // 1. 有寄存器写操作（|debug_wb_rf_we表示至少1个字节使能）
    // 2. 不是写0号寄存器（0号寄存器通常恒为0，不写入）
    // 3. 测试未结束（!debug_end）
    // 4. 开启跟踪功能（`CONFREG_OPEN_TRACE）
    // 5. 复位已释放（resetn）
    if(|debug_wb_rf_we && debug_wb_rf_wnum!=5'd0 && !debug_end && `CONFREG_OPEN_TRACE && resetn)
    begin
        trace_cmp_flag=1'b0;  // 初始化有效标志为0
        // 循环读取文件，直到获取有效数据或文件结束
        while (!trace_cmp_flag && !($feof(trace_ref)))
        begin
           // 按格式读取文件内容：标志、PC、寄存器号、数据
           a =  $fscanf(trace_ref, "%h %h %h %h", trace_cmp_flag,
                    ref_wb_pc, ref_wb_rf_wnum, ref_wb_rf_wdata);
        end
    end
end

// 处理有效数据（仅保留写使能对应的字节，屏蔽未使能的字节）
// 因为寄存器写操作可能是字节粒度（如只写低8位），需屏蔽未使能的字节再比较
wire [31:0] debug_wb_rf_wdata_v;  // 实际写入的有效数据（已屏蔽无效字节）
wire [31:0] ref_wb_rf_wdata_v;    // 参考的有效数据（已屏蔽无效字节）
// 高8位有效数据：仅保留写使能位3对应的字节
assign debug_wb_rf_wdata_v[31:24] = debug_wb_rf_wdata[31:24] & {8{debug_wb_rf_we[3]}};
// 次高8位有效数据：仅保留写使能位2对应的字节
assign debug_wb_rf_wdata_v[23:16] = debug_wb_rf_wdata[23:16] & {8{debug_wb_rf_we[2]}};
// 次低8位有效数据：仅保留写使能位1对应的字节
assign debug_wb_rf_wdata_v[15: 8] = debug_wb_rf_wdata[15: 8] & {8{debug_wb_rf_we[1]}};
// 低8位有效数据：仅保留写使能位0对应的字节
assign debug_wb_rf_wdata_v[7 : 0] = debug_wb_rf_wdata[7 : 0] & {8{debug_wb_rf_we[0]}};
// 参考数据的高8位有效数据（同实际数据的屏蔽逻辑）
assign   ref_wb_rf_wdata_v[31:24] =   ref_wb_rf_wdata[31:24] & {8{debug_wb_rf_we[3]}};
// 参考数据的次高8位有效数据
assign   ref_wb_rf_wdata_v[23:16] =   ref_wb_rf_wdata[23:16] & {8{debug_wb_rf_we[2]}};
// 参考数据的次低8位有效数据
assign   ref_wb_rf_wdata_v[15: 8] =   ref_wb_rf_wdata[15: 8] & {8{debug_wb_rf_we[1]}};
// 参考数据的低8位有效数据
assign   ref_wb_rf_wdata_v[7 : 0] =   ref_wb_rf_wdata[7 : 0] & {8{debug_wb_rf_we[0]}};


// 在时钟上升沿对比实际结果与参考结果
reg debug_wb_err;  // 标记写回阶段的错误
always @(posedge soc_clk)  // 在CPU时钟上升沿触发
begin
    #2;  // 延迟2ns（避开时钟沿竞争，确保信号稳定）
    if(!resetn)  // 复位期间
    begin
        debug_wb_err <= 1'b0;  // 清除错误标志
    end
    // 当满足写操作条件且测试未结束时，进行结果对比
    else if(|debug_wb_rf_we && debug_wb_rf_wnum!=5'd0 && !debug_end && `CONFREG_OPEN_TRACE)
    begin
        // 若PC、寄存器号或有效数据不匹配，则判定为错误
        if (  (debug_wb_pc!==ref_wb_pc) || (debug_wb_rf_wnum!==ref_wb_rf_wnum)
            ||(debug_wb_rf_wdata_v!==ref_wb_rf_wdata_v) )
        begin
            // 打印错误信息（时间、预期值、实际值）
            $display("--------------------------------------------------------------");
            $display("[%t] Error!!!",$time);
            $display("    reference: PC = 0x%8h, wb_rf_wnum = 0x%2h, wb_rf_wdata = 0x%8h",
                      ref_wb_pc, ref_wb_rf_wnum, ref_wb_rf_wdata_v);
            $display("    mycpu    : PC = 0x%8h, wb_rf_wnum = 0x%2h, wb_rf_wdata = 0x%8h",
                      debug_wb_pc, debug_wb_rf_wnum, debug_wb_rf_wdata_v);
            $display("--------------------------------------------------------------");
            debug_wb_err <= 1'b1;  // 置位错误标志
            #40;  // 延迟40ns（确保信息打印完成）
            $finish;  // 结束仿真
        end
    end
end

// 监控数码管显示状态（验证测试点是否按预期执行）
reg [7:0] err_count;  // 错误计数器
// 从配置寄存器中获取数码管显示数据
wire [31:0] confreg_num_reg = `CONFREG_NUM_REG;
// 寄存数码管数据（用于检测数据变化）
reg  [31:0] confreg_num_reg_r;
always @(posedge soc_clk)  // 在CPU时钟上升沿触发
begin
    confreg_num_reg_r <= confreg_num_reg;  // 寄存当前数码管数据（用于下一拍对比）
    if (!resetn)  // 复位期间
    begin
        err_count <= 8'd0;  // 清除错误计数器
    end
    // 当数码管数据变化且开启监控功能时
    else if (confreg_num_reg_r != confreg_num_reg && `CONFREG_NUM_MONITOR)
    begin
        // 检查低8位是否按预期递增（测试点内部计数）
        if(confreg_num_reg[7:0]!=confreg_num_reg_r[7:0]+1'b1)
        begin
            // 打印错误信息（时间、错误次数、测试点编号）
            $display("--------------------------------------------------------------");
            $display("[%t] Error(%d)!!! Occurred in number 8'd%02d Functional Test Point!",$time, err_count, confreg_num_reg[31:24]);
            $display("--------------------------------------------------------------");
            err_count <= err_count + 1'b1;  // 错误计数+1
        end
        // 检查高8位（测试点编号）是否按预期递增
        else if(confreg_num_reg[31:24]!=confreg_num_reg_r[31:24]+1'b1)
        begin
            // 打印测试点编号不匹配的错误信息
            $display("--------------------------------------------------------------");
            $display("[%t] Error(%d)!!! Unknown, Functional Test Point numbers are unequal!",$time,err_count);
            $display("--------------------------------------------------------------");
            $display("==============================================================");
            err_count <= err_count + 1'b1;  // 错误计数+1
        end
        else
        begin
            // 打印测试点通过信息
            $display("----[%t] Number 8'd%02d Functional Test Point PASS!!!", $time, confreg_num_reg[31:24]);
        end
    end
end

// 监控测试进度（打印开始信息和运行状态）
initial
begin
    // 设置时间格式：单位为ns，0位小数，后缀" ns"，字段宽度10
    $timeformat(-9,0," ns",10);
    // 等待复位释放（复位期间循环等待）
    while(!resetn) #5;
    // 打印测试开始信息
    $display("==============================================================");
    $display("Test begin!");

    #10000;  // 延迟10000ns（等待测试启动）
    // 当开启数码管监控时，每隔10000ns打印当前PC值（显示测试进度）
    while(`CONFREG_NUM_MONITOR)
    begin
        #10000;
        $display ("        [%t] Test is running, debug_wb_pc = 0x%8h",$time, debug_wb_pc);
    end
end

// 模拟串口打印输出（将SOC的串口数据打印到仿真控制台）
wire uart_display;  // 串口输出使能信号
wire [7:0] uart_data;  // 串口输出数据
// 从配置寄存器中获取串口输出使能信号
assign uart_display = `CONFREG_UART_DISPLAY;
// 从配置寄存器中获取串口输出数据
assign uart_data    = `CONFREG_UART_DATA;

always @(posedge soc_clk)  // 在CPU时钟上升沿触发
begin
    if(uart_display)  // 当串口输出有效时
    begin
        if(uart_data==8'hff)  // 若数据为0xff（结束符）
        begin
            ;//$finish;  // 预留结束逻辑（当前不操作）
        end
        else
        begin
            $write("%c",uart_data);  // 打印字符（模拟串口输出）
        end
    end
end

// 测试结束判断与结果输出
// 总错误标志：写回阶段错误或数码管错误计数不为0
wire global_err = debug_wb_err || (err_count!=8'd0);
// 测试结束条件：PC达到结束值，或串口收到结束符
wire test_end = (debug_wb_pc==`END_PC) || (uart_display && uart_data==8'hff);
always @(posedge soc_clk)  // 在CPU时钟上升沿触发
begin
    if (!resetn)  // 复位期间
    begin
        debug_end <= 1'b0;  // 清除结束标志
    end
    // 当测试结束且未标记结束时
    else if(test_end && !debug_end)
    begin
        debug_end <= 1'b1;  // 标记测试结束
        // 打印测试结束信息
        $display("==============================================================");
        $display("Test end!");
        #40;  // 延迟40ns（确保信息打印完成）
        $fclose(trace_ref);  // 关闭参考跟踪文件
        // 根据总错误标志输出结果
        if (global_err)
        begin
            $display("Fail!!!Total %d errors!",err_count);  // 输出失败信息
        end
        else
        begin
            $display("----PASS!!!");  // 输出通过信息
        end
	    $finish;  // 结束仿真
	end
end
endmodule