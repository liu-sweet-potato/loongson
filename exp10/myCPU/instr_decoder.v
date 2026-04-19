module instr_decoder #
(
    parameter WIDTH=32
)(
    // instr_decoder
    input  wire [31     :0] inst,   
    output wire [WIDTH-1:0] control_signal,   
    // br_unit
    input  wire          valid,
    input  wire  [31:0]  pc,
    input  wire  [31:0]  rj_value,
    input  wire  [31:0]  rkd_value,
    output wire         br_taken,
    output wire [31:0]  br_target,
    // imm_extension
    output wire [31:0]  imm,
    // data_hazard_detect
    input  wire [25:0]  data_hazard_signals,
    output wire         not_ready_go,
    output wire [3 :0]  sel_rj_value,
    output wire [3 :0]  sel_rkd_value,
    // control_hazard_detect
    input  wire         allowin,
    output  wire         cancel
);


//*********************************管脚定义****************************************

// ID unit
wire [ 5:0] op_31_26;
wire [ 3:0] op_25_22;
wire [ 1:0] op_21_20;
wire [ 4:0] op_19_15;
wire [11:0] i12;
wire [19:0] i20;
wire [15:0] i16;
wire [25:0] i26;

wire [63:0] op_31_26_d;
wire [15:0] op_25_22_d;
wire [ 3:0] op_21_20_d;
wire [31:0] op_19_15_d;

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
wire        inst_slti;
wire        inst_sltui;
wire        inst_andi;
wire        inst_ori;
wire        inst_xori;
wire        inst_sll_w;
wire        inst_srl_w;
wire        inst_sra_w;
wire        inst_pcaddu12i;
wire        inst_mul_w;
wire        inst_mulh_w;
wire        inst_mulh_wu;
wire        inst_div_w;
wire        inst_mod_w;
wire        inst_div_wu;
wire        inst_mod_wu;

wire        inst_mul;
wire        inst_mulh;

wire        need_ui5;
wire        need_ui12;
wire        need_si12;
wire        need_si16;
wire        need_si20;
wire        need_si26;

// control signal
wire [2 :0] sel_nextpc      ;  // one-hot
wire [18:0] alu_op          ;  // one-hot
wire        src1_is_pc      ;  // sel_alu_src1
wire        src2_is_imm     ;  // sel_alu_src2
wire        src2_is_4       ;  // sel_alu_src2
wire [4 :0] sel_alu_src2    ;  // sel_alu_src2
wire        res_from_mem    ;  // sel_rf_res
wire        dst_is_r1       ;  // sel_rf_dst
wire        gr_we           ;  // rf_we
wire        mem_we          ;  // data_ram_we
wire        src_reg_is_rd   ;  // sel_rf_ra2
wire [2 :0] sel_mul_result  ;  // sel_rf_ra2

// br unit
wire [31:0] br_offs;
wire [31:0] jirl_offs;
wire        rj_eq_rd;

// data_hazard_detect
wire [4 :0] rd;
wire [4 :0] rj;
wire [4 :0] rk;

wire        inst_src_include_rd;
wire        inst_src_include_rj;
wire        inst_src_include_rk;

wire        EX_valid;
wire        EX_gr_we;
wire        EX_dst_is_r1;
wire [4 :0] EX_rd;

wire        MEM_valid;
wire        MEM_gr_we;
wire        MEM_dst_is_r1;
wire [4 :0] MEM_rd;

wire        WB_valid;
wire        WB_gr_we;
wire        WB_dst_is_r1;
wire [4 :0] WB_rd;

wire        EX_load;
wire        EX_mul;

wire        ID_EX_rd_value_data_hazard;
wire        ID_EX_rj_value_data_hazard;
wire        ID_EX_rk_value_data_hazard;
wire        ID_EX_rkd_value_data_hazard;

wire        ID_MEM_rd_value_data_hazard;
wire        ID_MEM_rj_value_data_hazard;
wire        ID_MEM_rk_value_data_hazard;
wire        ID_MEM_rkd_value_data_hazard;

wire        ID_WB_rd_value_data_hazard;
wire        ID_WB_rj_value_data_hazard;
wire        ID_WB_rk_value_data_hazard;
wire        ID_WB_rkd_value_data_hazard;

wire        select_ID_rj_value;
wire        select_EX_rj_value;
wire        select_MEM_rj_value;
wire        select_WB_rj_value;

wire        select_ID_rkd_value;
wire        select_EX_rkd_value;
wire        select_MEM_rkd_value;
wire        select_WB_rkd_value;

//*********************************连线赋值****************************************

// ID unit
assign op_31_26  = inst[31:26];
assign op_25_22  = inst[25:22];
assign op_21_20  = inst[21:20];
assign op_19_15  = inst[19:15];
assign i12  = inst[21:10];
assign i20  = inst[24: 5];
assign i16  = inst[25:10];
assign i26  = {inst[ 9: 0], inst[25:10]};

decoder_6_64 u_dec0(.in(op_31_26 ), .out(op_31_26_d ));
decoder_4_16 u_dec1(.in(op_25_22 ), .out(op_25_22_d ));
decoder_2_4  u_dec2(.in(op_21_20 ), .out(op_21_20_d ));
decoder_5_32 u_dec3(.in(op_19_15 ), .out(op_19_15_d ));


assign inst_add_w     = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h00];
assign inst_sub_w     = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h02];
assign inst_slt       = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h04];
assign inst_sltu      = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h05];
assign inst_nor       = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h08];
assign inst_and       = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h09];
assign inst_or        = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0a];
assign inst_xor       = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0b];
assign inst_slli_w    = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h01];
assign inst_srli_w    = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h09];
assign inst_srai_w    = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h11];
assign inst_addi_w    = op_31_26_d[6'h00] & op_25_22_d[4'ha];
assign inst_ld_w      = op_31_26_d[6'h0a] & op_25_22_d[4'h2];
assign inst_st_w      = op_31_26_d[6'h0a] & op_25_22_d[4'h6];
assign inst_jirl      = op_31_26_d[6'h13];
assign inst_b         = op_31_26_d[6'h14];
assign inst_bl        = op_31_26_d[6'h15];
assign inst_beq       = op_31_26_d[6'h16];
assign inst_bne       = op_31_26_d[6'h17];
assign inst_lu12i_w   = op_31_26_d[6'h05] & ~inst[25];
assign inst_slti      = op_31_26_d[6'h00] & op_25_22_d[4'h8];
assign inst_sltui     = op_31_26_d[6'h00] & op_25_22_d[4'h9];
assign inst_andi      = op_31_26_d[6'h00] & op_25_22_d[4'hd];
assign inst_ori       = op_31_26_d[6'h00] & op_25_22_d[4'he];
assign inst_xori      = op_31_26_d[6'h00] & op_25_22_d[4'hf];
assign inst_sll_w     = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0e];
assign inst_srl_w     = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0f];
assign inst_sra_w     = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h10];
assign inst_pcaddu12i = op_31_26_d[6'h07] & ~inst[25];
assign inst_mul_w     = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h18];
assign inst_mulh_w    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h19];
assign inst_mulh_wu   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h1a];
assign inst_div_w     = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h00];
assign inst_mod_w     = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h01];
assign inst_div_wu    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h02];
assign inst_mod_wu    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h03];


assign need_ui5   =  inst_slli_w | inst_srli_w | inst_srai_w;
assign need_ui12  =  inst_andi | inst_ori | inst_xori;
assign need_si12  =  inst_addi_w | inst_ld_w | inst_st_w | inst_slti | inst_sltui;
assign need_si16  =  inst_jirl | inst_beq | inst_bne;
assign need_si20  =  inst_lu12i_w | inst_pcaddu12i;
assign need_si26  =  inst_b | inst_bl;

// WB
assign gr_we         = ~inst_st_w & ~inst_beq & ~inst_bne & ~inst_b;
assign dst_is_r1     = inst_bl;


// MEM
assign res_from_mem  = inst_ld_w;
assign mem_we        = inst_st_w;

assign inst_mulh = inst_mulh_w | inst_mulh_wu;
assign inst_mul = inst_mulh | inst_mul_w;
assign sel_mul_result={inst_mulh, inst_mul_w, ~inst_mul};  // control signals of two-cycle multiplier

// EXE
assign alu_op = {
    inst_mod_wu,
    inst_div_wu,
    inst_mod_w,
    inst_div_w,
    inst_mulh_wu,
    inst_mulh_w,
    inst_mul_w,
    inst_lu12i_w,
    inst_sra_w | inst_srai_w,
    inst_srl_w | inst_srli_w,
    inst_sll_w | inst_slli_w,
    inst_xor | inst_xori,
    inst_or | inst_ori,
    inst_nor,
    inst_and | inst_andi,
    inst_sltu | inst_sltui,
    inst_slt | inst_slti,
    inst_sub_w,
    inst_add_w | inst_addi_w | inst_ld_w | inst_st_w | inst_jirl | inst_bl | inst_pcaddu12i    // op_add
};

assign src1_is_pc    = inst_jirl | inst_bl | inst_pcaddu12i;
assign src2_is_imm   = inst_slli_w      |
                       inst_srli_w      |
                       inst_srai_w      |
                       inst_addi_w      |
                       inst_ld_w        |
                       inst_st_w        |
                       inst_lu12i_w     |
                       inst_jirl        |
                       inst_bl          |
                       inst_slti        |
                       inst_sltui       |
                       inst_andi        |
                       inst_ori         |
                       inst_xori        |
                       inst_pcaddu12i;


// ID
assign src_reg_is_rd = inst_beq | inst_bne | inst_st_w;

// imm_extension
assign src2_is_4  =  inst_jirl | inst_bl;

assign sel_alu_src2[4] = need_ui12              ;
assign sel_alu_src2[3] = need_si20              ;  // lu12i
assign sel_alu_src2[2] = src2_is_4              ;  // bl || jirl
assign sel_alu_src2[1] = need_ui5 || need_si12  ;  // 复用
assign sel_alu_src2[0] = !src2_is_imm           ;

assign imm = {32{sel_alu_src2[4]}} & {20'b0, i12[11:0]}         |
             {32{sel_alu_src2[3]}} & {i20[19:0], 12'b0}         |
             {32{sel_alu_src2[2]}} & 32'h0000_0004              |
             {32{sel_alu_src2[1]}} & {{20{i12[11]}}, i12[11:0]} ;


// control_signal
assign control_signal = {
    src_reg_is_rd,
    src2_is_imm,
    src1_is_pc,
    alu_op,
    {4{mem_we}},
    sel_mul_result,
    res_from_mem,
    dst_is_r1,
    gr_we
};

// br unit
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


assign br_offs = need_si26 ? {{ 4{i26[25]}}, i26[25:0], 2'b0} :
                             {{14{i16[15]}}, i16[15:0], 2'b0} ;
assign jirl_offs = {{14{i16[15]}}, i16[15:0], 2'b0};

assign br_target = sel_nextpc[1] ? (pc + br_offs) : (rj_value + jirl_offs);

// data_hazard_detect
assign rd   = inst[ 4: 0];
assign rj   = inst[ 9: 5];
assign rk   = inst[14:10];

assign inst_src_include_rd = inst_bne | inst_beq | inst_st_w;
assign inst_src_include_rj = ~inst_lu12i_w & ~inst_b & ~inst_bl & ~inst_pcaddu12i;
assign inst_src_include_rk = inst_add_w | inst_sub_w | inst_slt | inst_sltu | inst_and | inst_or | inst_nor | inst_xor | inst_sll_w | inst_srl_w | inst_sra_w | inst_mul_w | inst_mulh_w | inst_mulh_wu | inst_div_w | inst_mod_w | inst_div_wu | inst_mod_wu;

assign {
    EX_mul, EX_load,  // load-use
    WB_rd, WB_dst_is_r1, WB_gr_we, WB_valid,
    MEM_rd, MEM_dst_is_r1, MEM_gr_we, MEM_valid,
    EX_rd, EX_dst_is_r1, EX_gr_we, EX_valid
} = data_hazard_signals;


assign ID_EX_rd_value_data_hazard = valid & EX_valid & EX_gr_we & (inst_src_include_rd & rd==EX_rd | EX_dst_is_r1 & rd==1'd1) & rd!=5'd0;
assign ID_EX_rk_value_data_hazard = valid & EX_valid & EX_gr_we & (inst_src_include_rk & rk==EX_rd | EX_dst_is_r1 & rk==1'd1) & rk!=5'd0;
assign ID_EX_rkd_value_data_hazard  = ID_EX_rd_value_data_hazard | ID_EX_rk_value_data_hazard;
assign ID_EX_rj_value_data_hazard = valid & EX_valid & EX_gr_we & (inst_src_include_rj & rj==EX_rd | EX_dst_is_r1 & rj==1'd1) & rj!=5'd0;


assign ID_MEM_rd_value_data_hazard = valid & MEM_valid & MEM_gr_we & (inst_src_include_rd & rd==MEM_rd | MEM_dst_is_r1 & rd==1'd1) & rd!=5'd0;
assign ID_MEM_rk_value_data_hazard = valid & MEM_valid & MEM_gr_we & (inst_src_include_rk & rk==MEM_rd | MEM_dst_is_r1 & rk==1'd1) & rk!=5'd0;
assign ID_MEM_rkd_value_data_hazard  = ID_MEM_rd_value_data_hazard | ID_MEM_rk_value_data_hazard;
assign ID_MEM_rj_value_data_hazard = valid & MEM_valid & MEM_gr_we & (inst_src_include_rj & rj==MEM_rd | MEM_dst_is_r1 & rj==1'd1) & rj!=5'd0;


assign ID_WB_rd_value_data_hazard = valid & WB_valid & WB_gr_we & (inst_src_include_rd & rd==WB_rd | WB_dst_is_r1 & rd==1'd1) & rd!=5'd0;
assign ID_WB_rk_value_data_hazard = valid & WB_valid & WB_gr_we & (inst_src_include_rk & rk==WB_rd | WB_dst_is_r1 & rk==1'd1) & rk!=5'd0;
assign ID_WB_rkd_value_data_hazard  = ID_WB_rd_value_data_hazard | ID_WB_rk_value_data_hazard;
assign ID_WB_rj_value_data_hazard = valid & WB_valid & WB_gr_we & (inst_src_include_rj & rj==WB_rd | WB_dst_is_r1 & rj==1'd1) & rj!=5'd0;

assign {select_EX_rj_value, select_MEM_rj_value, select_WB_rj_value, select_ID_rj_value} = ID_EX_rj_value_data_hazard   ? 4'b1000 :
                                                                                           ID_MEM_rj_value_data_hazard  ? 4'b0100 :
                                                                                           ID_WB_rj_value_data_hazard   ? 4'b0010 : 4'b0001;

assign {select_EX_rkd_value, select_MEM_rkd_value, select_WB_rkd_value, select_ID_rkd_value} = ID_EX_rkd_value_data_hazard  ? 4'b1000 :
                                                                                               ID_MEM_rkd_value_data_hazard ? 4'b0100 :
                                                                                               ID_WB_rkd_value_data_hazard  ? 4'b0010 : 4'b0001;

assign sel_rj_value = {select_EX_rj_value, select_MEM_rj_value, select_WB_rj_value, select_ID_rj_value};
assign sel_rkd_value = {select_EX_rkd_value, select_MEM_rkd_value, select_WB_rkd_value, select_ID_rkd_value};

// load-use
assign not_ready_go = valid & EX_valid & EX_gr_we & inst_src_include_rd & rd==EX_rd & (EX_load | EX_mul) & rd!=5'd0 |
                      valid & EX_valid & EX_gr_we & inst_src_include_rj & rj==EX_rd & (EX_load | EX_mul) & rj!=5'd0 |
                      valid & EX_valid & EX_gr_we & inst_src_include_rk & rk==EX_rd & (EX_load | EX_mul) & rk!=5'd0;

// control_hazard_detect
assign cancel = valid && allowin && br_taken;

endmodule