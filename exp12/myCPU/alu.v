module alu(
  input  wire        clk,
  input  wire        reset,
  input  wire [18:0] alu_op,
  input  wire [31:0] alu_src1,
  input  wire [31:0] alu_src2,
  output wire        alu_ready_go,
  output wire [31:0] alu_result,
  output wire [31:0] mem_addr,  // 优化时序
  output wire [63:0] mul_result
);

//*********************************管脚定义****************************************
wire op_add;      //add operation
wire op_sub;      //sub operation
wire op_slt;      //signed compared and set less than
wire op_sltu;     //unsigned compared and set less than
wire op_and;      //bitwise and
wire op_nor;      //bitwise nor
wire op_or;       //bitwise or
wire op_xor;      //bitwise xor
wire op_sll;      //logic left shift
wire op_srl;      //logic right shift
wire op_sra;      //arithmetic right shift
wire op_lui;      //Load Upper Immediate
wire op_mul_w;
wire op_mulh_w;
wire op_mulh_wu;
wire op_div_w;
wire op_mod_w;
wire op_div_wu;
wire op_mod_wu;

wire [31:0] add_sub_result;
wire [31:0] slt_result;
wire [31:0] sltu_result;
wire [31:0] and_result;
wire [31:0] nor_result;
wire [31:0] or_result;
wire [31:0] xor_result;
wire [31:0] lui_result;
wire [31:0] sll_result;
wire [63:0] sr64_result;
wire [31:0] sr_result;
wire [31:0] quotient_result;
wire [31:0] remainder_result;


// 32-bit adder
wire [31:0] adder_a;
wire [31:0] adder_b;
wire        adder_cin;
wire [31:0] adder_result;
wire        adder_cout;

// 32-bit divider
wire        inst_div;
wire        div_signed;
wire        div_complete;

//********************************* 连线赋值 ****************************************

// control code decomposition
assign {
    op_mod_wu,
    op_div_wu,
    op_mod_w,
    op_div_w,
    op_mulh_wu,
    op_mulh_w,
    op_mul_w,
    op_lui,
    op_sra,
    op_srl,
    op_sll,
    op_xor,
    op_or,
    op_nor,
    op_and,
    op_sltu,
    op_slt,
    op_sub,
    op_add
} = alu_op;

// 32-bit adder
assign adder_a   = alu_src1;
assign adder_b   = (op_sub | op_slt | op_sltu) ? ~alu_src2 : alu_src2;  //src1 - src2 rj-rk
assign adder_cin = (op_sub | op_slt | op_sltu) ? 33'h1      : 33'h0;
assign {adder_cout, adder_result} = {1'b0, adder_a} + {1'b0, adder_b} + adder_cin;      // 全加器用的很巧妙 

// ADD, SUB result
assign add_sub_result = adder_result;

// SLT result
assign slt_result[31:1] = 31'b0;   //rj < rk 1
assign slt_result[0]    = (alu_src1[31] & ~alu_src2[31])  // rj_value < 0 && rk_value > 0
                        | ((alu_src1[31] ~^ alu_src2[31]) & adder_result[31]);  // 同号

// SLTU result
assign sltu_result[31:1] = 31'b0;
assign sltu_result[0]    = ~adder_cout;  // 减数做1位0扩展取反使然

// bitwise operation
assign and_result = alu_src1 & alu_src2;
assign or_result  = alu_src1 | alu_src2;
assign nor_result = ~or_result;
assign xor_result = alu_src1 ^ alu_src2;
assign lui_result = alu_src2;

// 直接使用桶型移位电路
// SLL result
assign sll_result = alu_src1 << alu_src2[4:0];   //rj << i5

// SRL, SRA result
assign sr64_result = {{32{op_sra & alu_src1[31]}}, alu_src1[31:0]} >> alu_src2[4:0];  //rj >> i5，巧妙复用

assign sr_result   = sr64_result[31:0];

// MUL_W, MULH_W, MULH_WU result
mul mul (
    .mul_clk(clk),
    .resetn(~reset),
    .mul_signed(op_mulh_w),
    .x(alu_src1),
    .y(alu_src2),
    .result(mul_result)
);

// DIV_W, MOD_W, DIV_WU, MOD_WU result
assign inst_div = op_div_w || op_div_wu || op_mod_w || op_mod_wu;
assign div_signed = op_div_w || op_mod_w;
div div (
    .div_clk(clk),                // 时钟
    .resetn(~reset),              // 低电平复位
    .div(inst_div),               // 除法指令
    .div_signed(div_signed),      // 高电平有符号数，低电平无符号数
    .x(alu_src1),                 // 被除数        
    .y(alu_src2),                 // 除数
    .q(quotient_result),          // 商
    .r(remainder_result),         // 余数
    .complete(div_complete)       // 高电平完成，内部计数器33
);

// ready go 
assign alu_ready_go = ~inst_div | div_complete;

// final result mux
assign alu_result = ({32{op_add|op_sub     }} & add_sub_result)
                  | ({32{op_slt            }} & slt_result)
                  | ({32{op_sltu           }} & sltu_result)
                  | ({32{op_and            }} & and_result)
                  | ({32{op_nor            }} & nor_result)
                  | ({32{op_or             }} & or_result)
                  | ({32{op_xor            }} & xor_result)
                  | ({32{op_lui            }} & lui_result)
                  | ({32{op_sll            }} & sll_result)
                  | ({32{op_srl|op_sra     }} & sr_result)
                  | ({32{op_div_w|op_div_wu}} & quotient_result)
                  | ({32{op_mod_w|op_mod_wu}} & remainder_result);

assign mem_addr = adder_result;  // 优化时序

endmodule
