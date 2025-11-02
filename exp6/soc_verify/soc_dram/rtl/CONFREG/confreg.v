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

//*************************************************************************
//   > File Name   : confreg.v
//   > Description : Control module of
//                   16 red leds, 2 green/red leds,
//                   7-segment display,
//                   switchs,
//                   key board,
//                   bottom STEP,
//                   timer.
//
//   > Author      : LOONGSON
//   > Date        : 2017-08-04
//*************************************************************************
`define RANDOM_SEED {7'b1010101,16'h01FF}  // 定义随机种子常量，由7位和16位值拼接而成

// 配置寄存器地址定义（低16位，完整地址为32'hbfaf_xxxx）
`define CR0_ADDR       16'h8000   // 配置寄存器0地址（地址步长为16）
`define CR1_ADDR       16'h8010   // 配置寄存器1地址
`define CR2_ADDR       16'h8020   // 配置寄存器2地址
`define CR3_ADDR       16'h8030   // 配置寄存器3地址
`define CR4_ADDR       16'h8040   // 配置寄存器4地址
`define CR5_ADDR       16'h8050   // 配置寄存器5地址
`define CR6_ADDR       16'h8060   // 配置寄存器6地址
`define CR7_ADDR       16'h8070   // 配置寄存器7地址

// 外设地址定义（低16位）
`define LED_ADDR       16'hf020   // 16个红色LED的控制地址
`define LED_RG0_ADDR   16'hf030   // 第一个绿/红双色LED的控制地址
`define LED_RG1_ADDR   16'hf040   // 第二个绿/红双色LED的控制地址
`define NUM_ADDR       16'hf050   // 七段数码管的控制地址
`define SWITCH_ADDR    16'hf060   // 开关状态的读取地址
`define BTN_KEY_ADDR   16'hf070   // 矩阵键盘的读取地址
`define BTN_STEP_ADDR  16'hf080   // 底部STEP按键的读取地址
`define SW_INTER_ADDR  16'hf090   // 开关交错数据的读取地址
`define TIMER_ADDR     16'he000   // 定时器的控制/读取地址

// 模拟相关地址定义
`define IO_SIMU_ADDR      16'hff00  // IO模拟功能的控制地址
`define VIRTUAL_UART_ADDR 16'hff10  // 虚拟UART的控制地址
`define SIMU_FLAG_ADDR    16'hff20  // 模拟环境标志的读取地址
`define OPEN_TRACE_ADDR   16'hff30  // 跟踪功能使能的控制地址
`define NUM_MONITOR_ADDR  16'hff40  // 数码管监控功能的控制地址

// 模块定义：配置寄存器模块，用于连接CPU与外设
// 参数SIMULATION：1表示模拟环境，0表示真实硬件
module confreg
#(parameter SIMULATION=1'b0)
(
    input  wire        clk,         // 主时钟信号（CPU时钟域）
    input  wire        timer_clk,   // 定时器专用时钟信号（独立时钟域）
    input  wire        resetn,      // 异步复位信号（低电平有效）
    // CPU读写接口
    input  wire        conf_en,     // 访问使能信号（高电平有效）
    input  wire        conf_we,     // 写使能信号（高电平为写，低电平为读）
    input  wire [31:0] conf_addr,   // 访问地址（32位）
    input  wire [31:0] conf_wdata,  // 写入数据（32位）
    output wire [31:0] conf_rdata,  // 读出数据（32位）
    // 外设接口
    output wire [15:0] led,         // 16个红色LED输出（高电平亮）
    output wire [1 :0] led_rg0,     // 第一个绿/红双色LED（bit0红，bit1绿）
    output wire [1 :0] led_rg1,     // 第二个绿/红双色LED（bit0红，bit1绿）
    output reg  [7 :0] num_csn,     // 8位七段数码管位选信号（低电平有效）
    output reg  [6 :0] num_a_g,     // 七段数码管段选信号（a~g，低电平有效）
    output reg  [31:0] num_data,    // 七段数码管显示数据缓存（供扫描用）
    input  wire [7 :0] switch,      // 8个开关输入（高电平表示开关闭合）
    output wire [3 :0] btn_key_col, // 4x4矩阵键盘列选信号（低电平有效）
    input  wire [3 :0] btn_key_row, // 4x4矩阵键盘行输入信号（高电平表示按键未按）
    input  wire [1 :0] btn_step     // 底部STEP按键输入（高电平表示未按）
);
    // 配置寄存器（CR0~CR7）：用于存储用户配置信息
    reg  [31:0] cr0;  // 配置寄存器0
    reg  [31:0] cr1;  // 配置寄存器1
    reg  [31:0] cr2;  // 配置寄存器2
    reg  [31:0] cr3;  // 配置寄存器3
    reg  [31:0] cr4;  // 配置寄存器4
    reg  [31:0] cr5;  // 配置寄存器5
    reg  [31:0] cr6;  // 配置寄存器6
    reg  [31:0] cr7;  // 配置寄存器7

    // LED相关数据缓存
    reg  [31:0] led_data;         // 16个红色LED的状态缓存（低16位有效）
    reg  [31:0] led_rg0_data;     // 第一个绿/红LED的状态缓存（低2位有效）
    reg  [31:0] led_rg1_data;     // 第二个绿/红LED的状态缓存（低2位有效）

    // 开关相关数据信号
    wire [31:0] switch_data;      // 开关状态（8位开关扩展为32位，高24位补0）
    wire [31:0] sw_inter_data;    // 开关交错数据（每bit后补0，用于特定场景）

    // 按键相关数据信号
    wire [31:0] btn_key_data;     // 矩阵键盘状态（16位按键扩展为32位）
    wire [31:0] btn_step_data;    // STEP按键状态（2位按键扩展为32位）

    // UART相关寄存器
    reg  [7 :0] confreg_uart_data;// UART发送数据缓存
    reg         confreg_uart_valid;// UART数据有效标志（高电平表示数据有效）

    // 定时器相关寄存器
    reg  [31:0] timer_r2;         // 定时器值（同步到主时钟域的二级缓存）
    reg  [31:0] simu_flag;        // 模拟环境标志（全位为SIMULATION参数值）
    reg  [31:0] io_simu;          // IO模拟数据缓存
    reg  [7 :0] virtual_uart_data;// 虚拟UART数据缓存
    reg         open_trace;       // 跟踪功能使能标志（高电平使能）
    reg         num_monitor;      // 数码管监控使能标志（高电平使能）


// 根据CPU访问地址，选择对应的寄存器或外设数据作为读出值
assign conf_rdata = (conf_addr[15:0]==`CR0_ADDR         )? cr0 :                  // 读配置寄存器0
                    (conf_addr[15:0]==`CR1_ADDR         )? cr1 :                  // 读配置寄存器1
                    (conf_addr[15:0]==`CR2_ADDR         )? cr2 :                  // 读配置寄存器2
                    (conf_addr[15:0]==`CR3_ADDR         )? cr3 :                  // 读配置寄存器3
                    (conf_addr[15:0]==`CR4_ADDR         )? cr4 :                  // 读配置寄存器4
                    (conf_addr[15:0]==`CR5_ADDR         )? cr5 :                  // 读配置寄存器5
                    (conf_addr[15:0]==`CR6_ADDR         )? cr6 :                  // 读配置寄存器6
                    (conf_addr[15:0]==`CR7_ADDR         )? cr7 :                  // 读配置寄存器7
                    (conf_addr[15:0]==`LED_ADDR         )? led_data:               // 读红色LED状态
                    (conf_addr[15:0]==`LED_RG0_ADDR     )? led_rg0_data :          // 读第一个绿/红LED状态
                    (conf_addr[15:0]==`LED_RG1_ADDR     )? led_rg1_data :          // 读第二个绿/红LED状态
                    (conf_addr[15:0]==`NUM_ADDR         )? num_data     :          // 读七段数码管显示数据
                    (conf_addr[15:0]==`SWITCH_ADDR      )? switch_data  :          // 读开关状态
                    (conf_addr[15:0]==`BTN_KEY_ADDR     )? btn_key_data :          // 读矩阵键盘状态
                    (conf_addr[15:0]==`BTN_STEP_ADDR    )? btn_step_data :         // 读STEP按键状态
                    (conf_addr[15:0]==`SW_INTER_ADDR    )? sw_inter_data :         // 读开关交错数据
                    (conf_addr[15:0]==`TIMER_ADDR       )? timer_r2     :          // 读定时器值
                    (conf_addr[15:0]==`SIMU_FLAG_ADDR   )? simu_flag    :          // 读模拟环境标志
                    (conf_addr[15:0]==`IO_SIMU_ADDR     )? io_simu      :          // 读IO模拟数据
                    (conf_addr[15:0]==`VIRTUAL_UART_ADDR)? {24'd0,virtual_uart_data} :  // 读虚拟UART数据
                    (conf_addr[15:0]==`OPEN_TRACE_ADDR  )? {31'd0,open_trace}  :  // 读跟踪使能状态
                    (conf_addr[15:0]==`NUM_MONITOR_ADDR )? {31'd0,num_monitor} :  // 读数码管监控使能状态
                                                           32'd0;                 // 无效地址返回0


// 写使能信号：仅当访问使能且写标志有效时，允许写入操作
wire conf_write = conf_en & conf_we;

//-------------------------{confreg register}begin-----------------------//
// 配置寄存器写使能信号：根据地址判断是否对对应寄存器执行写操作
wire write_cr0 = conf_write & (conf_addr[15:0]==`CR0_ADDR);  // CR0写使能
wire write_cr1 = conf_write & (conf_addr[15:0]==`CR1_ADDR);  // CR1写使能
wire write_cr2 = conf_write & (conf_addr[15:0]==`CR2_ADDR);  // CR2写使能
wire write_cr3 = conf_write & (conf_addr[15:0]==`CR3_ADDR);  // CR3写使能
wire write_cr4 = conf_write & (conf_addr[15:0]==`CR4_ADDR);  // CR4写使能
wire write_cr5 = conf_write & (conf_addr[15:0]==`CR5_ADDR);  // CR5写使能
wire write_cr6 = conf_write & (conf_addr[15:0]==`CR6_ADDR);  // CR6写使能
wire write_cr7 = conf_write & (conf_addr[15:0]==`CR7_ADDR);  // CR7写使能

// 配置寄存器赋值逻辑：复位时清零，写使能有效时更新为CPU写入数据
always @(posedge clk)
begin
    cr0 <= !resetn    ? 32'd0      :  // 复位时CR0清零
           write_cr0 ? conf_wdata : cr0;  // 写使能有效时更新CR0
    cr1 <= !resetn    ? 32'd0      :  // 复位时CR1清零
           write_cr1 ? conf_wdata : cr1;  // 写使能有效时更新CR1
    cr2 <= !resetn    ? 32'd0      :  // 复位时CR2清零
           write_cr2 ? conf_wdata : cr2;  // 写使能有效时更新CR2
    cr3 <= !resetn    ? 32'd0      :  // 复位时CR3清零
           write_cr3 ? conf_wdata : cr3;  // 写使能有效时更新CR3
    cr4 <= !resetn    ? 32'd0      :  // 复位时CR4清零
           write_cr4 ? conf_wdata : cr4;  // 写使能有效时更新CR4
    cr5 <= !resetn    ? 32'd0      :  // 复位时CR5清零
           write_cr5 ? conf_wdata : cr5;  // 写使能有效时更新CR5
    cr6 <= !resetn    ? 32'd0      :  // 复位时CR6清零
           write_cr6 ? conf_wdata : cr6;  // 写使能有效时更新CR6
    cr7 <= !resetn    ? 32'd0      :  // 复位时CR7清零
           write_cr7 ? conf_wdata : cr7;  // 写使能有效时更新CR7
end
//--------------------------{confreg register}end------------------------//

//-------------------------------{timer}begin----------------------------//
// 定时器跨时钟域同步信号：用于将主时钟域的写请求同步到定时器时钟域
reg         write_timer_begin;        // 定时器写启动标志（主时钟域）
reg         write_timer_begin_r1;     // 定时器写启动标志（定时器时钟域一级同步）
reg         write_timer_begin_r2;     // 定时器写启动标志（定时器时钟域二级同步）
reg         write_timer_begin_r3;     // 定时器写启动标志（定时器时钟域三级同步，用于边沿检测）
reg         write_timer_end_r1;       // 定时器写结束标志（主时钟域一级同步）
reg         write_timer_end_r2;       // 定时器写结束标志（主时钟域二级同步）
reg  [31:0] conf_wdata_r;             // 定时器写入数据（主时钟域缓存）
reg  [31:0] conf_wdata_r1;            // 定时器写入数据（定时器时钟域一级同步）
reg  [31:0] conf_wdata_r2;            // 定时器写入数据（定时器时钟域二级同步）

// 定时器值缓存：用于将定时器时钟域的值同步到主时钟域
reg  [31:0] timer_r1;  // 定时器值（主时钟域一级同步）
reg  [31:0] timer;     // 定时器计数器（定时器时钟域核心计数）

// 定时器写使能：CPU写TIMER_ADDR时有效
wire write_timer = conf_write & (conf_addr[15:0]==`TIMER_ADDR);

// 主时钟域：处理定时器写请求的启动与结束
always @(posedge clk)
begin
    if (!resetn)  // 复位时关闭写启动标志
    begin
        write_timer_begin <= 1'b0;
    end
    else if (write_timer)  // 收到写请求时，启动写标志并锁存写入数据
    begin
        write_timer_begin <= 1'b1;
        conf_wdata_r      <= conf_wdata;
    end
    else if (write_timer_end_r2)  // 写操作完成后，关闭写启动标志
    begin
        write_timer_begin <= 1'b0;
    end

    // 写结束标志同步（打两拍，避免亚稳态）
    write_timer_end_r1 <= write_timer_begin_r2;
    write_timer_end_r2 <= write_timer_end_r1;
end

// 定时器时钟域：更新定时器值（加载初始值或自增）
always @(posedge timer_clk)
begin
    // 跨时钟域同步写请求和写入数据（打拍避免亚稳态）
    write_timer_begin_r1 <= write_timer_begin;
    write_timer_begin_r2 <= write_timer_begin_r1;
    write_timer_begin_r3 <= write_timer_begin_r2;
    conf_wdata_r1        <= conf_wdata_r;
    conf_wdata_r2        <= conf_wdata_r1;

    if(!resetn)  // 复位时定时器清零
    begin
        timer <= 32'd0;
    end
    else if (write_timer_begin_r2 && !write_timer_begin_r3)  // 检测到写请求上升沿，加载初始值
    begin
        timer <= conf_wdata_r2[31:0];
    end
    else  // 无写请求时，定时器自增
    begin
        timer <= timer + 1'b1;
    end
end

// 将定时器值同步到主时钟域（打两拍，避免亚稳态）
always @(posedge clk)
begin
    timer_r1 <= timer;       // 一级同步
    timer_r2 <= timer_r1;    // 二级同步（供CPU读取）
end
//--------------------------------{timer}end-----------------------------//

//--------------------------{simulation flag}begin-----------------------//
// 模拟环境标志：复位时初始化为32个SIMULATION参数值（固定不变）
always @(posedge clk)
begin
    if(!resetn)
    begin
        simu_flag <= {32{SIMULATION}};  // 若SIMULATION=1，则全为1；否则全为0
    end
end
//---------------------------{simulation flag}end------------------------//

//---------------------------{io simulation}begin------------------------//
// IO模拟写使能：CPU写IO_SIMU_ADDR时有效
wire write_io_simu = conf_write & (conf_addr[15:0]==`IO_SIMU_ADDR);

// IO模拟数据缓存：写入时将高低16位交换存储
always @(posedge clk)
begin
    if(!resetn)  // 复位时清零
    begin
        io_simu <= 32'd0;
    end
    else if(write_io_simu)  // 写使能有效时，高低16位交换
    begin
        io_simu <= {conf_wdata[15:0],conf_wdata[31:16]};
    end
end
//----------------------------{io simulation}end-------------------------//

//-----------------------------{open trace}begin-------------------------//
// 跟踪功能写使能：CPU写OPEN_TRACE_ADDR时有效
wire write_open_trace = conf_write & (conf_addr[15:0]==`OPEN_TRACE_ADDR);

// 跟踪功能使能标志：复位时默认开启，写入非零值开启，零值关闭
always @(posedge clk)
begin
    if(!resetn)  // 复位时开启跟踪
    begin
        open_trace <= 1'b1;
    end
    else if(write_open_trace)  // 写使能有效时，根据输入值更新（或操作判断非零）
    begin
        open_trace <= |conf_wdata;
    end
end
//-----------------------------{open trace}end---------------------------//

//----------------------------{num monitor}begin-------------------------//
// 数码管监控写使能：CPU写NUM_MONITOR_ADDR时有效
wire write_num_monitor = conf_write & (conf_addr[15:0]==`NUM_MONITOR_ADDR);

// 数码管监控使能标志：复位时默认开启，由写入数据最低位控制
always @(posedge clk)
begin
    if(!resetn)  // 复位时开启监控
    begin
        num_monitor <= 1'b1;
    end
    else if(write_num_monitor)  // 写使能有效时，更新为输入最低位
    begin
        num_monitor <= conf_wdata[0];
    end
end
//----------------------------{num monitor}end---------------------------//

//---------------------------{virtual uart}begin-------------------------//
// 虚拟UART写入数据（取输入低8位）
wire [7:0] write_uart_data;
// 虚拟UART写使能：CPU写VIRTUAL_UART_ADDR时有效
wire write_uart_valid  = conf_write & (conf_addr[15:0]==`VIRTUAL_UART_ADDR);
assign write_uart_data = conf_wdata[7:0];  // 截取输入低8位作为UART数据

// 虚拟UART数据缓存：写入时更新数据并置位有效标志
always @(posedge clk)
begin
    if(!resetn)  // 复位时清零
    begin
        virtual_uart_data <= 8'd0;
        confreg_uart_data <= 8'd0;
        confreg_uart_valid <= 1'd0;
    end
    else if(write_uart_valid)  // 写使能有效时，更新数据和有效标志
    begin
        virtual_uart_data <= write_uart_data;
        confreg_uart_data <= write_uart_data;
        confreg_uart_valid <= write_uart_valid;
    end
end
//----------------------------{virtual uart}end--------------------------//

//--------------------------------{led}begin-----------------------------//
//led display
//led_data[31:0]
// 红色LED写使能：CPU写LED_ADDR时有效
wire write_led = conf_write & (conf_addr[15:0]==`LED_ADDR);
// 红色LED输出：取缓存的低16位（对应16个LED）
assign led = led_data[15:0];

// 红色LED数据缓存：写入时更新状态
always @(posedge clk)
begin
    if(!resetn)  // 复位时LED全灭
    begin
        led_data <= 32'h0;
    end
    else if(write_led)  // 写使能有效时，更新为输入数据
    begin
        led_data <= conf_wdata[31:0];
    end
end
//---------------------------------{led}end------------------------------//

//-------------------------------{switch}begin---------------------------//
//switch data
//switch_data[7:0]
// 开关数据：8位开关输入扩展为32位（高24位补0）
assign switch_data   = {24'd0,switch};
// 开关交错数据：每个开关位后补0（如switch[7]→bit15，switch[6]→bit13...）
assign sw_inter_data = {16'd0,
                        switch[7],1'b0,switch[6],1'b0,
                        switch[5],1'b0,switch[4],1'b0,
                        switch[3],1'b0,switch[2],1'b0,
                        switch[1],1'b0,switch[0],1'b0};
//--------------------------------{switch}end----------------------------//

//------------------------------{btn key}begin---------------------------//
//btn key data
// 矩阵键盘状态缓存（16位对应16个按键，高电平表示按下）
reg [15:0] btn_key_r;
// 矩阵键盘数据：16位状态扩展为32位（高16位补0）
assign btn_key_data = {16'd0,btn_key_r};

//state machine：矩阵键盘扫描状态机
reg  [2:0] state;          // 当前状态（3位表示8种状态）
wire [2:0] next_state;     // 下一状态

//eliminate jitter：按键消抖逻辑
reg        key_flag;       // 消抖标志（高电平表示需要消抖）
reg [19:0] key_count;      // 消抖计数器（计数到2^20周期，约10ms@100MHz）
reg [ 3:0] state_count;    // 状态切换计数器（控制状态切换频率）
// 按键按下检测：初始态且行不全为1（有按键按下）
wire key_start = (state==3'b000) && !(&btn_key_row);
// 按键释放检测：等待态且行全为1（按键释放）
wire key_end   = (state==3'b111) &&  (&btn_key_row);
// 消抖完成标志：计数器最高位为1（计数满）
wire key_sample= key_count[19];

// 消抖标志和计数器控制
always @(posedge clk)
begin
    if(!resetn)  // 复位时消抖标志清零
    begin
        key_flag <= 1'd0;
    end
    else if (key_sample && state_count[3])  // 消抖完成且状态计数满，清除标志
    begin
        key_flag <= 1'b0;
    end
    else if( key_start || key_end )  // 检测到按下或释放，启动消抖
    begin
        key_flag <= 1'b1;
    end

    if(!resetn || !key_flag)  // 复位或未消抖时，计数器清零
    begin
        key_count <= 20'd0;
    end
    else  // 消抖中，计数器自增
    begin
        key_count <= key_count + 1'b1;
    end
end

// 状态切换计数器：控制状态机切换速度（计数到15时切换）
always @(posedge clk)
begin
    if(!resetn || state_count[3])  // 复位或计数满（15）时清零
    begin
        state_count <= 4'd0;
    end
    else  // 否则自增
    begin
        state_count <= state_count + 1'b1;
    end
end

// 状态寄存器更新：状态计数满时切换到下一状态
always @(posedge clk)
begin
    if(!resetn)  // 复位时初始化为000态
    begin
        state <= 3'b000;
    end
    else if (state_count[3])  // 计数满（15）时，更新为下一状态
    begin
        state <= next_state;
    end
end

// 下一状态逻辑：轮询4列扫描，或等待按键释放
assign next_state = (state == 3'b000) ? ( (key_sample && !(&btn_key_row)) ? 3'b001 : 3'b000 ) :  // 初始态→列0（有按键）
                    (state == 3'b001) ? (                !(&btn_key_row)  ? 3'b111 : 3'b010 ) :  // 列0→等待态（未释放）或列1
                    (state == 3'b010) ? (                !(&btn_key_row)  ? 3'b111 : 3'b011 ) :  // 列1→等待态（未释放）或列2
                    (state == 3'b011) ? (                !(&btn_key_row)  ? 3'b111 : 3'b100 ) :  // 列2→等待态（未释放）或列3
                    (state == 3'b100) ? (                !(&btn_key_row)  ? 3'b111 : 3'b000 ) :  // 列3→等待态（未释放）或初始态
                    (state == 3'b111) ? ( (key_sample &&  (&btn_key_row)) ? 3'b000 : 3'b111 ) :  // 等待态→初始态（已释放）
                                                                                        3'b000;  // 默认初始态

// 列选信号：低电平有效，每次选通一列（4列轮询）
assign btn_key_col = (state == 3'b000) ? 4'b0000:  // 初始态：不选通任何列
                     (state == 3'b001) ? 4'b1110:  // 列0：bit0=0（选通列0）
                     (state == 3'b010) ? 4'b1101:  // 列1：bit1=0（选通列1）
                     (state == 3'b011) ? 4'b1011:  // 列2：bit2=0（选通列2）
                     (state == 3'b100) ? 4'b0111:  // 列3：bit3=0（选通列3）
                                         4'b0000;  // 其他态：不选通

// 按键值临时信号：根据当前列和行值确定按下的按键
wire [15:0] btn_key_tmp;
// 按键状态更新：按键释放后锁存按键值
always @(posedge clk) begin
    if(!resetn) begin  // 复位时按键状态清零
        btn_key_r   <= 16'd0;
    end
    else if(next_state==3'b000)  // 回到初始态时清零
    begin
        btn_key_r   <=16'd0;
    end
    // 从非等待态进入等待态且状态计数满时，锁存按键值
    else if(next_state == 3'b111 && state != 3'b111 && state_count[3]) begin
        btn_key_r   <= btn_key_tmp;
    end
end

// 按键值映射：列选+行输入→16位按键编码（每bit对应一个按键）
assign btn_key_tmp = (state == 3'b001)&(btn_key_row == 4'b1110) ? 16'h0001:  // 列0行0→按键0
                     (state == 3'b001)&(btn_key_row == 4'b1101) ? 16'h0010:  // 列0行1→按键1
                     (state == 3'b001)&(btn_key_row == 4'b1011) ? 16'h0100:  // 列0行2→按键2
                     (state == 3'b001)&(btn_key_row == 4'b0111) ? 16'h1000:  // 列0行3→按键3
                     (state == 3'b010)&(btn_key_row == 4'b1110) ? 16'h0002:  // 列1行0→按键4
                     (state == 3'b010)&(btn_key_row == 4'b1101) ? 16'h0020:  // 列1行1→按键5
                     (state == 3'b010)&(btn_key_row == 4'b1011) ? 16'h0200:  // 列1行2→按键6
                     (state == 3'b010)&(btn_key_row == 4'b0111) ? 16'h2000:  // 列1行3→按键7
                     (state == 3'b011)&(btn_key_row == 4'b1110) ? 16'h0004:  // 列2行0→按键8
                     (state == 3'b011)&(btn_key_row == 4'b1101) ? 16'h0040:  // 列2行1→按键9
                     (state == 3'b011)&(btn_key_row == 4'b1011) ? 16'h0400:  // 列2行2→按键10
                     (state == 3'b011)&(btn_key_row == 4'b0111) ? 16'h4000:  // 列2行3→按键11
                     (state == 3'b100)&(btn_key_row == 4'b1110) ? 16'h0008:  // 列3行0→按键12
                     (state == 3'b100)&(btn_key_row == 4'b1101) ? 16'h0080:  // 列3行1→按键13
                     (state == 3'b100)&(btn_key_row == 4'b1011) ? 16'h0800:  // 列3行2→按键14
                     (state == 3'b100)&(btn_key_row == 4'b0111) ? 16'h8000:  // 列3行3→按键15
                                                                 16'h0000;    // 无按键按下
//-------------------------------{btn key}end----------------------------//

//-----------------------------{btn step}begin---------------------------//
//btn step data
// STEP按键状态缓存（1表示释放，0表示按下）
reg btn_step0_r; //0:press
reg btn_step1_r; //0:press
// STEP按键数据：取反后扩展为32位（1表示按下）
assign btn_step_data = {30'd0,~btn_step0_r,~btn_step1_r}; //1:press

//-----step0：第一个STEP按键消抖逻辑
reg        step0_flag;       // 消抖标志（高电平表示需要消抖）
reg [19:0] step0_count;      // 消抖计数器（约10ms）
// 按下检测：当前释放且输入为0（按下）
wire step0_start = btn_step0_r && !btn_step[0];
// 释放检测：当前按下且输入为1（释放）
wire step0_end   = !btn_step0_r && btn_step[0];
// 消抖完成标志：计数器最高位为1
wire step0_sample= step0_count[19];

always @(posedge clk)
begin
    if(!resetn)  // 复位时消抖标志清零
    begin
        step0_flag <= 1'd0;
    end
    else if (step0_sample)  // 消抖完成，清除标志
    begin
        step0_flag <= 1'b0;
    end
    else if( step0_start || step0_end )  // 检测到按下或释放，启动消抖
    begin
        step0_flag <= 1'b1;
    end

    if(!resetn || !step0_flag)  // 复位或未消抖时，计数器清零
    begin
        step0_count <= 20'd0;
    end
    else  // 消抖中，计数器自增
    begin
        step0_count <= step0_count + 1'b1;
    end

    if(!resetn)  // 复位时默认释放（1）
    begin
        btn_step0_r <= 1'b1;
    end
    else if(step0_sample)  // 消抖完成，更新状态为输入值
    begin
        btn_step0_r <= btn_step[0];
    end
end

//-----step1：第二个STEP按键消抖逻辑（同step0）
reg        step1_flag;       // 消抖标志
reg [19:0] step1_count;      // 消抖计数器
wire step1_start = btn_step1_r && !btn_step[1];  // 按下检测
wire step1_end   = !btn_step1_r && btn_step[1];  // 释放检测
wire step1_sample= step1_count[19];  // 消抖完成标志

always @(posedge clk)
begin
    if(!resetn)  // 复位时消抖标志清零
    begin
        step1_flag <= 1'd0;
    end
    else if (step1_sample)  // 消抖完成，清除标志
    begin
        step1_flag <= 1'b0;
    end
    else if( step1_start || step1_end )  // 检测到按下或释放，启动消抖
    begin
        step1_flag <= 1'b1;
    end

    if(!resetn || !step1_flag)  // 复位或未消抖时，计数器清零
    begin
        step1_count <= 20'd0;
    end
    else  // 消抖中，计数器自增
    begin
        step1_count <= step1_count + 1'b1;
    end

    if(!resetn)  // 复位时默认释放（1）
    begin
        btn_step1_r <= 1'b1;
    end
    else if(step1_sample)  // 消抖完成，更新状态为输入值
    begin
        btn_step1_r <= btn_step[1];
    end
end
//------------------------------{btn step}end----------------------------//

//-------------------------------{led rg}begin---------------------------//
//led_rg0_data[31:0]  led_rg0_data[31:0]
//bfd0_f010           bfd0_f014
// 绿/红LED0写使能：CPU写LED_RG0_ADDR时有效
wire write_led_rg0 = conf_write & (conf_addr[15:0]==`LED_RG0_ADDR);
// 绿/红LED1写使能：CPU写LED_RG1_ADDR时有效
wire write_led_rg1 = conf_write & (conf_addr[15:0]==`LED_RG1_ADDR);
// 绿/红LED0输出：取缓存的低2位（bit0红，bit1绿）
assign led_rg0 = led_rg0_data[1:0];
// 绿/红LED1输出：取缓存的低2位
assign led_rg1 = led_rg1_data[1:0];

// 绿/红LED数据缓存：写入时更新状态
always @(posedge clk)
begin
    if(!resetn)  // 复位时LED全灭
    begin
        led_rg0_data <= 32'h0;
    end
    else if(write_led_rg0)  // 写使能有效时，更新为输入数据
    begin
        led_rg0_data <= conf_wdata[31:0];
    end

    if(!resetn)  // 复位时LED全灭
    begin
        led_rg1_data <= 32'h0;
    end
    else if(write_led_rg1)  // 写使能有效时，更新为输入数据
    begin
        led_rg1_data <= conf_wdata[31:0];
    end
end
//--------------------------------{led rg}end----------------------------//

//---------------------------{digital number}begin-----------------------//
//digital number display
//num_data[31:0]
// 七段数码管写使能：CPU写NUM_ADDR时有效
wire write_num = conf_write & (conf_addr[15:0]==`NUM_ADDR);

// 七段数码管数据缓存：写入时更新显示数据
always @(posedge clk)
begin
    if(!resetn)  // 复位时显示0
    begin
        num_data <= 32'h0;
    end
    else if(write_num)  // 写使能有效时，更新为输入数据
    begin
        num_data <= conf_wdata[31:0];
    end
end

// 扫描计数器：用于控制数码管刷新频率（约100Hz@100MHz）
reg [19:0] count;
always @(posedge clk)
begin
    if(!resetn)  // 复位时计数器清零
    begin
        count <= 20'd0;
    end
    else  // 计数器自增
    begin
        count <= count + 1'b1;
    end
end

//scan data：扫描当前数码管的数据和位选信号
reg [3:0] scan_data;  // 当前扫描的4位数码（对应一个数码管）
always @ ( posedge clk )
begin
    if ( !resetn )  // 复位时关闭所有数码管
    begin
        scan_data <= 32'd0;
        num_csn   <= 8'b1111_1111;  // 位选全为1（不选通）
    end
    else
    begin
        // 根据计数器高3位选择当前扫描的数码（8个数码管轮询）
        case(count[19:17])
            3'b000 : scan_data <= num_data[31:28];  // 第1个数码管（最高4位）
            3'b001 : scan_data <= num_data[27:24];  // 第2个数码管
            3'b010 : scan_data <= num_data[23:20];  // 第3个数码管
            3'b011 : scan_data <= num_data[19:16];  // 第4个数码管
            3'b100 : scan_data <= num_data[15:12];  // 第5个数码管
            3'b101 : scan_data <= num_data[11: 8];  // 第6个数码管
            3'b110 : scan_data <= num_data[7 : 4];  // 第7个数码管
            3'b111 : scan_data <= num_data[3 : 0];  // 第8个数码管（最低4位）
        endcase

        // 位选信号：低电平有效，选中当前扫描的数码管
        case(count[19:17])
            3'b000 : num_csn <= 8'b0111_1111;  // 选通第1个数码管
            3'b001 : num_csn <= 8'b1011_1111;  // 选通第2个数码管
            3'b010 : num_csn <= 8'b1101_1111;  // 选通第3个数码管
            3'b011 : num_csn <= 8'b1110_1111;  // 选通第4个数码管
            3'b100 : num_csn <= 8'b1111_0111;  // 选通第5个数码管
            3'b101 : num_csn <= 8'b1111_1011;  // 选通第6个数码管
            3'b110 : num_csn <= 8'b1111_1101;  // 选通第7个数码管
            3'b111 : num_csn <= 8'b1111_1110;  // 选通第8个数码管
        endcase
    end
end

// 段选信号：根据当前扫描的4位数据，输出七段编码（a~g）
always @(posedge clk)
begin
    if ( !resetn )  // 复位时所有段灭
    begin
        num_a_g <= 7'b0000000;
    end
    else
    begin
        // 七段编码（低电平有效）：0~f对应不同显示
        case ( scan_data )
            4'd0 : num_a_g <= 7'b1111110;   // 显示0
            4'd1 : num_a_g <= 7'b0110000;   // 显示1
            4'd2 : num_a_g <= 7'b1101101;   // 显示2
            4'd3 : num_a_g <= 7'b1111001;   // 显示3
            4'd4 : num_a_g <= 7'b0110011;   // 显示4
            4'd5 : num_a_g <= 7'b1011011;   // 显示5
            4'd6 : num_a_g <= 7'b1011111;   // 显示6
            4'd7 : num_a_g <= 7'b1110000;   // 显示7
            4'd8 : num_a_g <= 7'b1111111;   // 显示8
            4'd9 : num_a_g <= 7'b1111011;   // 显示9
            4'd10: num_a_g <= 7'b1110111;   // 显示a
            4'd11: num_a_g <= 7'b0011111;   // 显示b
            4'd12: num_a_g <= 7'b1001110;   // 显示c
            4'd13: num_a_g <= 7'b0111101;   // 显示d
            4'd14: num_a_g <= 7'b1001111;   // 显示e
            4'd15: num_a_g <= 7'b1000111;   // 显示f
        endcase
    end
end
//----------------------------{digital number}end------------------------//
endmodule