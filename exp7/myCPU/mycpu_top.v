module mycpu_top(
    input  wire        clk,
    input  wire        resetn,
    // inst sram interface
    output wire        inst_sram_we,
    output wire [31:0] inst_sram_addr,
    output wire [31:0] inst_sram_wdata,
    input  wire [31:0] inst_sram_rdata,
    // data sram interface
    output wire        data_sram_we,
    output wire [31:0] data_sram_addr,
    output wire [31:0] data_sram_wdata,
    input  wire [31:0] data_sram_rdata,
    // trace debug interface
    output wire [31:0] debug_wb_pc,
    output wire [ 3:0] debug_wb_rf_we,
    output wire [ 4:0] debug_wb_rf_wnum,
    output wire [31:0] debug_wb_rf_wdata
);
//*********************************顶层信号****************************************
reg         reset;  // 复位信号，高电平复位
always @(posedge clk) reset <= ~resetn;

reg         valid;  // 有效位
always @(posedge clk) begin
    if (reset) begin
        valid <= 1'b0;
    end
    else begin
        valid <= 1'b1;
    end
end

//*********************************管脚定义****************************************
// ID->BR中间变量
wire [31:0] br_offs;
wire [31:0] jirl_offs;

// BR I/O
wire [31:0] seq_pc;  // default
wire [31:0] nextpc;
wire        rj_eq_rd;
wire        br_taken;
wire [31:0] br_target;
wire [31:0] inst;

// IF I/O
reg  [31:0] pc;

// ID I/O
wire [2:0]  sel_nextpc      ;  // one-hot略显鸡肋
wire [11:0] alu_op          ;  // one-hot
wire        load_op         ;  // unused?
wire        src1_is_pc      ;  // sel_alu_src1
wire        src2_is_imm     ;  // sel_alu_src2
wire        src2_is_4       ;  // sel_alu_src2
wire [3:0]  sel_alu_src2    ;  // sel_alu_src2
wire        res_from_mem    ;  // sel_rf_res
wire        dst_is_r1       ;  // sel_rf_dst
wire        gr_we           ;  // rf_we
wire        mem_we          ;  // data_ram_we
wire        src_reg_is_rd   ;  // sel_rf_ra2

// 指令分段
wire [ 5:0] op_31_26;
wire [ 3:0] op_25_22;
wire [ 1:0] op_21_20;
wire [ 4:0] op_19_15;
wire [ 4:0] rd;
wire [ 4:0] rj;
wire [ 4:0] rk;
wire [11:0] i12;
wire [19:0] i20;
wire [15:0] i16;
wire [25:0] i26;

// 指令解码
wire [63:0] op_31_26_d;
wire [15:0] op_25_22_d;
wire [ 3:0] op_21_20_d;
wire [31:0] op_19_15_d;

// 20条指令译码
wire        inst_add_w;
wire        inst_sub_w;
wire        inst_slt;
wire        inst_sltu;
wire        inst_nor;
wire        inst_and;
wire        inst_or;
wire        inst_xor;
wire        inst_slli_w;
wire        inst_srli_w;
wire        inst_srai_w;
wire        inst_addi_w;
wire        inst_ld_w;
wire        inst_st_w;
wire        inst_jirl;
wire        inst_b;
wire        inst_bl;
wire        inst_beq;
wire        inst_bne;
wire        inst_lu12i_w;

// 20条指令附加信号
wire        need_ui5;
wire        need_si12;
wire        need_si16;
wire        need_si20;
wire        need_si26;

// GR I/O
wire [ 4:0] rf_raddr1;
wire [31:0] rf_rdata1;
wire [ 4:0] rf_raddr2;
wire [31:0] rf_rdata2;
wire        rf_we   ;
wire [ 4:0] rf_waddr;
wire [31:0] rf_wdata;


// GR->ALU中间变量
wire [31:0] rj_value;
wire [31:0] rkd_value;
wire [31:0] imm;

//ALU I/O
wire [31:0] alu_src1   ;
wire [31:0] alu_src2   ;
wire [31:0] alu_result ;

// ALU->GR中间变量
wire [4: 0] dest;

// MEM I/O
wire [31:0] mem_addr   ;
wire [31:0] mem_result ;

// MEM->GR中间变量
wire [31:0] final_result;  // res

//*********************************连线赋值****************************************
// ID->BR中间变量
assign br_offs = need_si26 ? {{ 4{i26[25]}}, i26[25:0], 2'b0} :
                             {{14{i16[15]}}, i16[15:0], 2'b0} ;
assign jirl_offs = {{14{i16[15]}}, i16[15:0], 2'b0};

// BR unit
assign sel_nextpc[0] = !sel_nextpc[1] && !sel_nextpc[2];
assign sel_nextpc[1] = inst_beq || inst_bne || inst_b || inst_bl;
assign sel_nextpc[2] = inst_jirl;

assign rj_eq_rd = (rj_value == rkd_value);
assign br_taken = (   inst_beq  &&  rj_eq_rd
                   || inst_bne  && !rj_eq_rd
                   || inst_jirl
                   || inst_bl
                   || inst_b
                  ) && valid;
assign br_target = sel_nextpc[1] ? (pc + br_offs) : (rj_value + jirl_offs);


// IF unit
assign seq_pc       = pc + 32'h4;
assign nextpc       = br_taken ? br_target : seq_pc;

always @(posedge clk) begin
    if (reset) begin
        pc <= 32'h1bfffffc;     //trick: to make nextpc be 0x1c000000 during reset 
    end
    else begin
        pc <= nextpc;
    end
end

// 指令存储器
assign inst_sram_we    = 1'b0           ;  // 默认不可写
assign inst_sram_addr  = pc             ;  // 可读
assign inst_sram_wdata = 32'b0          ;  // 默认不可写
assign inst            = inst_sram_rdata;  // 可读

// ID unit
assign op_31_26  = inst[31:26];
assign op_25_22  = inst[25:22];
assign op_21_20  = inst[21:20];
assign op_19_15  = inst[19:15];

assign rd   = inst[ 4: 0];
assign rj   = inst[ 9: 5];
assign rk   = inst[14:10];

assign i12  = inst[21:10];
assign i20  = inst[24: 5];
assign i16  = inst[25:10];
assign i26  = {inst[ 9: 0], inst[25:10]};

// 指令译码
decoder_6_64 u_dec0(.in(op_31_26 ), .out(op_31_26_d ));
decoder_4_16 u_dec1(.in(op_25_22 ), .out(op_25_22_d ));
decoder_2_4  u_dec2(.in(op_21_20 ), .out(op_21_20_d ));
decoder_5_32 u_dec3(.in(op_19_15 ), .out(op_19_15_d ));

assign inst_add_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h00];
assign inst_sub_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h02];
assign inst_slt    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h04];
assign inst_sltu   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h05];
assign inst_nor    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h08];
assign inst_and    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h09];
assign inst_or     = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0a];
assign inst_xor    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0b];
assign inst_slli_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h01];
assign inst_srli_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h09];
assign inst_srai_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h11];
assign inst_addi_w = op_31_26_d[6'h00] & op_25_22_d[4'ha];
assign inst_ld_w   = op_31_26_d[6'h0a] & op_25_22_d[4'h2];
assign inst_st_w   = op_31_26_d[6'h0a] & op_25_22_d[4'h6];
assign inst_jirl   = op_31_26_d[6'h13];
assign inst_b      = op_31_26_d[6'h14];
assign inst_bl     = op_31_26_d[6'h15];
assign inst_beq    = op_31_26_d[6'h16];
assign inst_bne    = op_31_26_d[6'h17];
assign inst_lu12i_w= op_31_26_d[6'h05] & ~inst[25];


// 20条指令附加控制信号
assign need_ui5   =  inst_slli_w | inst_srli_w | inst_srai_w;
assign need_si12  =  inst_addi_w | inst_ld_w | inst_st_w;
assign need_si16  =  inst_jirl | inst_beq | inst_bne;
assign need_si20  =  inst_lu12i_w;
assign need_si26  =  inst_b | inst_bl;

// 控制信号
assign alu_op[ 0] = inst_add_w | inst_addi_w | inst_ld_w | inst_st_w | inst_jirl | inst_bl;  // op_add
assign alu_op[ 1] = inst_sub_w;
assign alu_op[ 2] = inst_slt;
assign alu_op[ 3] = inst_sltu;
assign alu_op[ 4] = inst_and;
assign alu_op[ 5] = inst_nor;
assign alu_op[ 6] = inst_or;
assign alu_op[ 7] = inst_xor;
assign alu_op[ 8] = inst_slli_w;
assign alu_op[ 9] = inst_srli_w;
assign alu_op[10] = inst_srai_w;
assign alu_op[11] = inst_lu12i_w;

assign src1_is_pc    = inst_jirl | inst_bl;

assign src2_is_imm   = inst_slli_w |
                       inst_srli_w |
                       inst_srai_w |
                       inst_addi_w |
                       inst_ld_w   |
                       inst_st_w   |
                       inst_lu12i_w|
                       inst_jirl   |
                       inst_bl     ;

assign src2_is_4  =  inst_jirl | inst_bl;

assign sel_alu_src2[3] = need_si20              ;  // lu12i
assign sel_alu_src2[2] = src2_is_4              ;  // bl || jirl
assign sel_alu_src2[1] = need_ui5 || need_si12  ;
assign sel_alu_src2[0] = !src2_is_imm           ;

assign res_from_mem  = inst_ld_w;
assign dst_is_r1     = inst_bl;
assign gr_we         = ~inst_st_w & ~inst_beq & ~inst_bne & ~inst_b;
assign mem_we        = inst_st_w;
assign src_reg_is_rd = inst_beq | inst_bne | inst_st_w;

// GR
assign rf_raddr1 = rj;
assign rf_raddr2 = src_reg_is_rd ? rd :rk;

regfile u_regfile(
    .clk    (clk      ),
    .raddr1 (rf_raddr1),
    .rdata1 (rf_rdata1),
    .raddr2 (rf_raddr2),
    .rdata2 (rf_rdata2),
    .we     (rf_we    ),
    .waddr  (rf_waddr ),
    .wdata  (rf_wdata )
    );

assign rj_value  = rf_rdata1;
assign rkd_value = rf_rdata2;

// 多路选择器语法糖
assign imm = {32{sel_alu_src2[3]}} & {i20[19:0], 12'b0}         |
             {32{sel_alu_src2[2]}} & 32'h0000_0004              |
             {32{sel_alu_src2[1]}} & {{20{i12[11]}}, i12[11:0]} ;
             
assign alu_src1 = src1_is_pc  ? pc[31:0] : rj_value;
assign alu_src2 = src2_is_imm ? imm : rkd_value;

// EX unit
alu u_alu(
    .alu_op     (alu_op    ),
    .alu_src1   (alu_src1  ),
    .alu_src2   (alu_src2  ),
    .alu_result (alu_result),
    .mem_addr   (mem_addr  )
    );

// MEM unit
assign data_sram_we    = mem_we && valid;
assign data_sram_addr  = mem_addr;  // 优化时序
assign data_sram_wdata = rkd_value;

// WB unit
assign rf_waddr = dest;
assign mem_result   = data_sram_rdata;
assign final_result = res_from_mem ? mem_result : alu_result;

assign rf_we    = gr_we && valid;
assign dest          = dst_is_r1 ? 5'd1 : rd;
assign rf_wdata = final_result;

// debug info generate
assign debug_wb_pc       = pc;
assign debug_wb_rf_we   = {4{rf_we}};
assign debug_wb_rf_wnum  = dest;
assign debug_wb_rf_wdata = final_result;

endmodule
