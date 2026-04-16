module alu(
  input  wire        clk,
  input  wire        reset,
  input  wire [18:0] alu_op,
  input  wire [31:0] alu_src1,
  input  wire [31:0] alu_src2,
  output wire        alu_ready_go,
  output wire [31:0] alu_result,
  output wire [31:0] mem_addr  // 优化时序
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
wire [66:0] mul_result;
wire [63:0] div_w_result;
wire [63:0] div_wu_result;


// 32-bit adder
wire [31:0] adder_a;
wire [31:0] adder_b;
wire        adder_cin;
wire [31:0] adder_result;
wire        adder_cout;

// 32-bit multiplier
wire [32:0] multiplier_a;
wire [32:0] multiplier_b;


// 32-bit divider
wire        inst_div;

// 32-bit divider_w
wire        inst_div_w;               // 执行除法指令
reg         doing_div_w;            // 除法指令执行中
reg         s_axis_divisor_tvalid_w;
wire        s_axis_divisor_tready_w;

reg         s_axis_dividend_tvalid_w;
wire        s_axis_dividend_tready_w;

wire        m_axis_dout_tvalid_w;

// 32-bit divider_wu
wire        inst_div_wu;               // 执行除法指令
reg         doing_div_wu;            // 除法指令执行中
reg         s_axis_divisor_tvalid_wu;
wire        s_axis_divisor_tready_wu;

reg         s_axis_dividend_tvalid_wu;
wire        s_axis_dividend_tready_wu;

wire        m_axis_dout_tvalid_wu;

//********************************* 连线赋值 ****************************************

// control code decomposition
assign op_add     = alu_op[ 0];
assign op_sub     = alu_op[ 1];
assign op_slt     = alu_op[ 2];
assign op_sltu    = alu_op[ 3];
assign op_and     = alu_op[ 4];
assign op_nor     = alu_op[ 5];
assign op_or      = alu_op[ 6];
assign op_xor     = alu_op[ 7];
assign op_sll     = alu_op[ 8];
assign op_srl     = alu_op[ 9];
assign op_sra     = alu_op[10];
assign op_lui     = alu_op[11];
assign op_mul_w   = alu_op[12];
assign op_mulh_w  = alu_op[13];
assign op_mulh_wu = alu_op[14];
assign op_div_w   = alu_op[15];
assign op_mod_w   = alu_op[16];
assign op_div_wu  = alu_op[17];
assign op_mod_wu  = alu_op[18];

// 32-bit adder
assign adder_a   = alu_src1;
assign adder_b   = (op_sub | op_slt | op_sltu) ? ~alu_src2 : alu_src2;  //src1 - src2 rj-rk
assign adder_cin = (op_sub | op_slt | op_sltu) ? 1'b1      : 1'b0;
assign {adder_cout, adder_result} = adder_a + adder_b + adder_cin;      // 全加器用的很巧妙 

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
assign multiplier_a = op_mulh_wu ? {{1{1'b0}}, alu_src1[31:0]} : {{1{alu_src1[31]}}, alu_src1[31:0]};
assign multiplier_b = op_mulh_wu ? {{1{1'b0}}, alu_src2[31:0]} : {{1{alu_src2[31]}}, alu_src2[31:0]};
assign mul_result = $signed(multiplier_a) * $signed(multiplier_b);

// DIV_W, MOD_W result
assign inst_div_w = op_div_w || op_mod_w;

always @(posedge clk) begin
  if(reset) begin
      s_axis_divisor_tvalid_w <= 1'b0;
      s_axis_dividend_tvalid_w <= 1'b0;
      doing_div_w <= 1'b0;
  end

  else begin
      // 除法指令，未执行
      if (inst_div_w && !doing_div_w) begin
          s_axis_divisor_tvalid_w <= 1'b1;
          s_axis_dividend_tvalid_w <= 1'b1;
          doing_div_w <= 1'b1;
      end
      // 握手成功
      else if (s_axis_divisor_tready_w && s_axis_dividend_tready_w && s_axis_divisor_tvalid_w && s_axis_dividend_tvalid_w) begin
          s_axis_divisor_tvalid_w <= 1'b0;
          s_axis_dividend_tvalid_w <= 1'b0;
      end
      // 执行结束
      else if(m_axis_dout_tvalid_w) begin
        doing_div_w <= 1'b0;
      end
      // 迭代运算
  end
end

div_w_gen div_w_gen (
  .aclk(clk),                                      // input wire aclk
  .s_axis_divisor_tvalid(s_axis_divisor_tvalid_w),    // input wire s_axis_divisor_tvalid
  .s_axis_divisor_tready(s_axis_divisor_tready_w),    // output wire s_axis_divisor_tready
  .s_axis_divisor_tdata(alu_src2[31:0]),      // input wire [31 : 0] s_axis_divisor_tdata
  .s_axis_dividend_tvalid(s_axis_dividend_tvalid_w),  // input wire s_axis_dividend_tvalid
  .s_axis_dividend_tready(s_axis_dividend_tready_w),  // output wire s_axis_dividend_tready
  .s_axis_dividend_tdata(alu_src1[31:0]),    // input wire [31 : 0] s_axis_dividend_tdata
  .m_axis_dout_tvalid(m_axis_dout_tvalid_w),          // output wire m_axis_dout_tvalid
  .m_axis_dout_tdata(div_w_result[63:0])            // output wire [63 : 0] m_axis_dout_tdata
);



// DIV_WU, MOD_WU result
assign inst_div_wu = op_div_wu || op_mod_wu;

always @(posedge clk) begin
  if(reset) begin
      s_axis_divisor_tvalid_wu <= 1'b0;
      s_axis_dividend_tvalid_wu <= 1'b0;
      doing_div_wu <= 1'b0;
  end

  else begin
      // 除法指令，未执行
      if (inst_div_wu && !doing_div_wu) begin
          s_axis_divisor_tvalid_wu <= 1'b1;
          s_axis_dividend_tvalid_wu <= 1'b1;
          doing_div_wu <= 1'b1;
      end
      // 握手成功
      else if (s_axis_divisor_tready_wu && s_axis_dividend_tready_wu && s_axis_divisor_tvalid_wu && s_axis_dividend_tvalid_wu) begin
          s_axis_divisor_tvalid_wu <= 1'b0;
          s_axis_dividend_tvalid_wu <= 1'b0;
      end
      // 执行结束
      else if(m_axis_dout_tvalid_wu) begin
        doing_div_wu <= 1'b0;
      end
      // 迭代运算
  end
end

div_wu_gen div_wu_gen (
  .aclk(clk),                                      // input wire aclk
  .s_axis_divisor_tvalid(s_axis_divisor_tvalid_wu),    // input wire s_axis_divisor_tvalid
  .s_axis_divisor_tready(s_axis_divisor_tready_wu),    // output wire s_axis_divisor_tready
  .s_axis_divisor_tdata(alu_src2[31:0]),      // input wire [31 : 0] s_axis_divisor_tdata
  .s_axis_dividend_tvalid(s_axis_dividend_tvalid_wu),  // input wire s_axis_dividend_tvalid
  .s_axis_dividend_tready(s_axis_dividend_tready_wu),  // output wire s_axis_dividend_tready
  .s_axis_dividend_tdata(alu_src1[31:0]),    // input wire [31 : 0] s_axis_dividend_tdata
  .m_axis_dout_tvalid(m_axis_dout_tvalid_wu),          // output wire m_axis_dout_tvalid
  .m_axis_dout_tdata(div_wu_result[63:0])            // output wire [63 : 0] m_axis_dout_tdata
);

assign inst_div = inst_div_w || inst_div_wu;

// ready_go
assign alu_ready_go = m_axis_dout_tvalid_w || m_axis_dout_tvalid_wu || !inst_div;

// final result mux
assign alu_result = ({32{op_add|op_sub}} & add_sub_result)
                  | ({32{op_slt       }} & slt_result)
                  | ({32{op_sltu      }} & sltu_result)
                  | ({32{op_and       }} & and_result)
                  | ({32{op_nor       }} & nor_result)
                  | ({32{op_or        }} & or_result)
                  | ({32{op_xor       }} & xor_result)
                  | ({32{op_lui       }} & lui_result)
                  | ({32{op_sll       }} & sll_result)
                  | ({32{op_srl|op_sra}} & sr_result)
                  | ({32{op_mulh_wu|op_mulh_w}} & mul_result[63:32])
                  | ({32{op_mul_w}} & mul_result[31:0])
                  | ({32{op_div_w}} & div_w_result[63:32])
                  | ({32{op_mod_w}} & div_w_result[31:0])
                  | ({32{op_div_wu}} & div_wu_result[63:32])
                  | ({32{op_mod_wu}} & div_wu_result[31:0]);

assign mem_addr = adder_result;  // 优化时序

endmodule
