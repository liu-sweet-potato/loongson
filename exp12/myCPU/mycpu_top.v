module mycpu_top(
    input  wire        clk,
    input  wire        resetn,
    // inst sram interface
    output wire        inst_sram_en,
    output wire [3 :0] inst_sram_we,
    output wire [31:0] inst_sram_addr,
    output wire [31:0] inst_sram_wdata,
    input  wire [31:0] inst_sram_rdata,
    // data sram interface
    output wire        data_sram_en,
    output wire [3 :0] data_sram_we,
    output wire [31:0] data_sram_addr,
    output wire [31:0] data_sram_wdata,
    input  wire [31:0] data_sram_rdata,
    // trace debug interface
    output wire [31:0] debug_wb_pc,
    output wire [3 :0] debug_wb_rf_we,
    output wire [4 :0] debug_wb_rf_wnum,
    output wire [31:0] debug_wb_rf_wdata
);
//*********************************参数定义****************************************
// GENERAL
localparam BYTE_W           =   8;
localparam HALFWORD_W       =  16;
localparam DATA_W           =  32;
localparam DDATA_W          =  64;

// IF
localparam IF_REG_W         =   1;

// ID
localparam ID_CTRL_W        =  36;
localparam ID_REG_W         =  DATA_W + DATA_W;

localparam RF_ADDR_W        =   5;

localparam HAZARD_SIG_W     =  26;

localparam SEL_RJ_VAL_W     =   4;
localparam SEL_RKD_VAL_W    =   4;

// EX
localparam EX_CTRL_W        =  35;
localparam EX_REG_W         = DATA_W + DATA_W + DATA_W + EX_CTRL_W + RF_ADDR_W + DATA_W;

localparam ALU_OP_W         =  19;

localparam ST_OP_W          =   3;

localparam MEM_WE_W         =   4;

// MEM
localparam MEM_CTRL_W       =  11;
localparam MEM_REG_W        =  DATA_W + MEM_CTRL_W + RF_ADDR_W + DATA_W;

localparam SEL_MUL_W        =   3;

localparam LD_OP_W          =   5;

// WB
localparam WB_CTRL_W        =   2;
localparam WB_REG_W         =  DATA_W + WB_CTRL_W + RF_ADDR_W + DATA_W;



//*********************************管脚定义****************************************
// rst
reg         reset;  // 复位信号，高电平复位

// IFReg I/O
wire IF_allowin;
wire IF_validout;
wire IF_datain;
wire IF_dataout;
wire IF_ready_go;
wire IF_cancel;
wire IF_valid;

// IF I/O
wire [DATA_W-1:0] seq_pc;  // default
wire [DATA_W-1:0] nextpc;
reg  [DATA_W-1:0] pc;
wire [DATA_W-1:0] IF_inst;

// IDReg I/O
wire                ID_allowin;
wire                ID_validout;
wire [ID_REG_W-1:0] ID_datain;
wire [ID_REG_W-1:0] ID_dataout;
wire                ID_ready_go;
wire                ID_cancel;
wire                ID_valid;

wire [DATA_W-1:0]   ID_pc;
wire [DATA_W-1:0]   ID_inst;

// ID I/O
wire [ID_CTRL_W-1       :0] ID_control_signal  ;
wire [DATA_W-1          :0] ID_rj_value        ;
wire [DATA_W-1          :0] ID_rkd_value       ;
wire                        br_taken           ;
wire [DATA_W-1          :0] br_target          ;
wire [DATA_W-1          :0] ID_imm             ;
wire [HAZARD_SIG_W-1    :0] data_hazard_signals;
wire [SEL_RJ_VAL_W-1    :0] sel_rj_value       ;
wire [SEL_RKD_VAL_W-1   :0] sel_rkd_value      ;
wire                        ID_not_ready_go    ;
wire                        cancel             ;

wire                        ID_src_reg_is_rd   ;  // sel_rf_ra2
wire                        ID_src2_is_imm     ;  // sel_alu_src2
wire                        ID_src1_is_pc      ;  // sel_alu_src1
wire [ALU_OP_W-1        :0] ID_alu_op          ;  // one-ho
wire [ST_OP_W-1         :0] ID_st_op           ;
wire [SEL_MUL_W-1       :0] ID_sel_mul_result;
wire                        ID_res_from_mem    ;  // sel_rf_res
wire [LD_OP_W-1         :0] ID_ld_op           ;
wire                        ID_dst_is_r1       ;  // sel_rf_dst
wire                        ID_gr_we           ;  // rf_we

wire                        select_ID_rj_value;
wire                        select_EX_rj_value;
wire                        select_MEM_rj_value;
wire                        select_WB_rj_value;

wire                        select_ID_rkd_value;
wire                        select_EX_rkd_value;
wire                        select_MEM_rkd_value;
wire                        select_WB_rkd_value;

// GR I/O
wire [RF_ADDR_W-1       :0] rf_raddr1;
wire [DATA_W-1          :0] rf_rdata1;
wire [RF_ADDR_W-1       :0] rf_raddr2;
wire [DATA_W-1          :0] rf_rdata2;
wire                        rf_we    ;
wire [RF_ADDR_W-1       :0] rf_waddr ;
wire [DATA_W-1          :0] rf_wdata ;

wire [RF_ADDR_W-1       :0] ID_rj    ;
wire [RF_ADDR_W-1       :0] ID_rd    ;
wire [RF_ADDR_W-1       :0] ID_rk    ;

// EXReg I/O
wire                        EX_allowin;
wire                        EX_validout;
wire [EX_REG_W-1        :0] EX_datain;
wire [EX_REG_W-1        :0] EX_dataout;
wire                        EX_ready_go;
wire                        EX_cancel;
wire                        EX_valid;

wire [EX_CTRL_W-1       :0] EX_control_signal  ;

wire                        EX_src2_is_imm     ;  // sel_alu_src2
wire                        EX_src1_is_pc      ;  // sel_alu_src1
wire [ALU_OP_W-1        :0] EX_alu_op          ;  // one-hot
wire [ST_OP_W-1         :0] EX_st_op           ;
wire [SEL_MUL_W-1       :0] EX_sel_mul_result  ;
wire                        EX_res_from_mem    ;  // sel_rf_res
wire [LD_OP_W-1         :0] EX_ld_op           ;
wire                        EX_dst_is_r1       ;  // sel_rf_dst
wire                        EX_gr_we           ;  // rf_we

wire                        EX_inst_st_h       ;
wire                        EX_inst_st_b       ;
wire                        EX_inst_st_w       ;
wire [MEM_WE_W-1        :0] EX_mem_we_h        ;
wire [MEM_WE_W-1        :0] EX_mem_we_b        ;
wire [MEM_WE_W-1        :0] EX_mem_we_w        ;


wire [DATA_W-1          :0] EX_pc              ;
wire [DATA_W-1          :0] EX_rj_value        ;
wire [DATA_W-1          :0] EX_rkd_value       ;
wire [DATA_W-1          :0] EX_imm             ;
wire [RF_ADDR_W-1       :0] EX_rd              ;

//ALU I/O
wire [DATA_W-1          :0] alu_src1           ;
wire [DATA_W-1          :0] alu_src2           ;
wire                        alu_ready_go       ;
wire [DATA_W-1          :0] EX_alu_result      ;
wire [DATA_W-1          :0] EX_mem_addr        ;
wire [DDATA_W-1         :0] mul_result         ;

// MEMReg I/O
wire                        MEM_allowin        ;
wire                        MEM_validout       ;
wire [MEM_REG_W-1       :0] MEM_datain         ;
wire [MEM_REG_W-1       :0] MEM_dataout        ;
wire                        MEM_ready_go       ;
wire                        MEM_cancel         ;
wire                        MEM_valid          ;

wire [MEM_CTRL_W-1      :0] MEM_control_signal ;

wire [SEL_MUL_W-1       :0] MEM_sel_mul_result ;
wire                        MEM_res_from_mem   ;  // sel_rf_res
wire [LD_OP_W-1         :0] MEM_ld_op          ;
wire                        MEM_dst_is_r1      ;  // sel_rf_dst
wire                        MEM_gr_we          ;  // rf_we

wire                        MEM_inst_ld_hu       ;
wire                        MEM_inst_ld_bu       ;
wire                        MEM_inst_ld_h       ;
wire                        MEM_inst_ld_b       ;
wire                        MEM_inst_ld_w       ;
wire [HALFWORD_W-1      :0] MEM_mem_rdata_h    ;
wire [BYTE_W-1          :0] MEM_mem_rdata_b    ;
wire [DATA_W-1          :0] MEM_mem_rdata_w     ;

wire [DATA_W-1          :0] MEM_pc             ;
wire [RF_ADDR_W-1       :0] MEM_rd             ;
wire [DATA_W-1          :0] MEM_alu_result     ;

// MEM I/O
wire [DATA_W-1          :0] MEM_mem_result     ;
wire [DATA_W-1          :0] MEM_final_result   ;
wire [DATA_W-1          :0] final_alu_result   ;

// WB I/O
wire                        WB_allowin         ;
wire                        WB_validout        ;
wire [WB_REG_W-1        :0] WB_datain          ;
wire [WB_REG_W-1        :0] WB_dataout         ;
wire                        WB_ready_go        ;
wire                        WB_cancel          ;
wire                        WB_valid           ;

wire [WB_CTRL_W-1      :0] WB_control_signal   ;

wire                       WB_gr_we            ;  // rf_we
wire                       WB_dst_is_r1        ;  // sel_rf_dst

wire [DATA_W-1         :0] WB_pc               ;
wire [RF_ADDR_W-1      :0] WB_rd               ;
wire [DATA_W-1         :0] WB_final_result     ;
wire [RF_ADDR_W-1      :0] dest                ;

//********************************* 连线赋值 ****************************************
//********************************* IF unit ****************************************
// pre-IF
always @(posedge clk) reset <= ~resetn;  // 恰可以提前一周期恢复指令存储器使能以取指

assign seq_pc       = pc + 32'h4;
assign nextpc       = br_taken ? br_target : seq_pc;

// IFReg
assign IF_ready_go = 1'b1;
assign IF_cancel = 1'b0;
assign IF_datain = 1'b0;

pipeline_buffer #
(
    .WIDTH(IF_REG_W)
) IFReg (
    .clk(clk),
    .reset(reset),
    // allowin
    .post_allowin(ID_allowin),
    .allowin(IF_allowin),
    // validout
    .pre_validout(resetn),
    .validout(IF_validout),
    // data 
    .datain(IF_datain),
    .dataout(IF_dataout),
    // control field
    .ready_go(IF_ready_go),
    .cancel(IF_cancel),
    .valid(IF_valid)
);

// IF
always @(posedge clk) begin
    if (reset) begin
        pc <= 32'h1bfffffc;     //trick: to make nextpc be 0x1c000000 during reset 
    end
    else if (IF_allowin) begin
        pc <= nextpc;
    end
end

// inst_sram
assign inst_sram_we    = 4'b0                ;  // 默认不可写
assign inst_sram_addr  = nextpc              ;  // 可读
assign inst_sram_wdata = 32'b0               ;  // 默认不可写
assign IF_inst         = inst_sram_rdata     ;  // 可读
assign inst_sram_en    = resetn && IF_allowin;  // 提前一周期恢复使能预读指令,可被阻塞

//********************************* IDReg ****************************************
assign ID_ready_go = ~ID_not_ready_go;
assign ID_cancel = cancel;
assign ID_datain = {IF_inst, pc};

pipeline_buffer #
(
    .WIDTH(ID_REG_W)
) IDReg (
    .clk(clk),
    .reset(reset),
    // allowin
    .post_allowin(EX_allowin),
    .allowin(ID_allowin),
    // validout
    .pre_validout(IF_validout),
    .validout(ID_validout),
    // data 
    .datain(ID_datain),
    .dataout(ID_dataout),
    // control field
    .ready_go(ID_ready_go),
    .cancel(ID_cancel),
    .valid(ID_valid)
);

assign {
    ID_inst, 
    ID_pc
} = ID_dataout;

//********************************* ID unit ****************************************
assign data_hazard_signals = {
    ~EX_sel_mul_result[0], EX_res_from_mem,  // load-use
    WB_rd, WB_dst_is_r1, WB_gr_we, WB_valid,
    MEM_rd, MEM_dst_is_r1, MEM_gr_we, MEM_valid,
    EX_rd, EX_dst_is_r1, EX_gr_we, EX_valid
};

instr_decoder # 
(
    .WIDTH(ID_CTRL_W)
) u_id (
    // instr_decoder
    .inst(ID_inst),
    .control_signal(ID_control_signal),
    // br_unit
    .valid(ID_valid), 
    .pc(ID_pc),
    .rj_value(ID_rj_value),
    .rkd_value(ID_rkd_value),
    .br_taken(br_taken),
    .br_target(br_target),
    // imm_extension
    .imm(ID_imm),
    // data_hazard_detect
    .data_hazard_signals(data_hazard_signals),
    .not_ready_go(ID_not_ready_go),
    .sel_rj_value(sel_rj_value),
    .sel_rkd_value(sel_rkd_value),
    // control_hazard_detect
    .allowin(ID_allowin),
    .cancel(cancel)
);

assign {ID_src_reg_is_rd,
        ID_src2_is_imm,
        ID_src1_is_pc,
        ID_alu_op,
        ID_st_op,
        ID_sel_mul_result,
        ID_res_from_mem,
        ID_ld_op,
        ID_dst_is_r1,
        ID_gr_we
} = ID_control_signal;

// GR
assign {
    ID_rk,
    ID_rj,
    ID_rd
} = ID_inst;

assign rf_raddr1 = ID_rj;
assign rf_raddr2 = ID_src_reg_is_rd ? ID_rd :ID_rk;

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

assign {
    select_EX_rj_value,
    select_MEM_rj_value,
    select_WB_rj_value,
    select_ID_rj_value
} = sel_rj_value;

assign {
    select_EX_rkd_value,
    select_MEM_rkd_value,
    select_WB_rkd_value,
    select_ID_rkd_value
} = sel_rkd_value;

assign ID_rj_value  = {32{select_ID_rj_value}} & rf_rdata1 |
                      {32{select_EX_rj_value}} & EX_alu_result |
                      {32{select_MEM_rj_value}} & MEM_final_result |
                      {32{select_WB_rj_value}} & WB_final_result;

assign ID_rkd_value = {32{select_ID_rkd_value}} & rf_rdata2 |
                      {32{select_EX_rkd_value}} & EX_alu_result |
                      {32{select_MEM_rkd_value}} & MEM_final_result |
                      {32{select_WB_rkd_value}} & WB_final_result;

//********************************* EXReg ****************************************
assign EX_ready_go = alu_ready_go;
assign EX_cancel = 1'b0;

assign EX_datain = {
    ID_imm,
    ID_rkd_value,
    ID_rj_value,
    ID_control_signal[EX_CTRL_W-1:0],    // ID_src_reg_is_rd has been used
    ID_rd,
    ID_pc
};

pipeline_buffer #
(
    .WIDTH(EX_REG_W)
) EXReg (
    .clk(clk),
    .reset(reset),
    // allowin
    .post_allowin(MEM_allowin),
    .allowin(EX_allowin),
    // validout
    .pre_validout(ID_validout),
    .validout(EX_validout),
    // data 
    .datain(EX_datain),
    .dataout(EX_dataout),
    // control field
    .ready_go(EX_ready_go),
    .cancel(EX_cancel),
    .valid(EX_valid)
);

assign {
    EX_imm,
    EX_rkd_value,
    EX_rj_value,
    EX_control_signal,
    EX_rd,
    EX_pc
} = EX_dataout;

assign {
    EX_src2_is_imm,
    EX_src1_is_pc,
    EX_alu_op,
    EX_st_op,
    EX_sel_mul_result,
    EX_res_from_mem,
    EX_ld_op,
    EX_dst_is_r1,
    EX_gr_we
} = EX_control_signal;

//********************************* EX unit ****************************************
assign alu_src1 = EX_src1_is_pc  ? EX_pc : EX_rj_value;
assign alu_src2 = EX_src2_is_imm ? EX_imm : EX_rkd_value;

alu u_alu(
    .clk            (clk       ),
    .reset          (reset     ),
    .alu_op         (EX_alu_op ),
    .alu_src1       (alu_src1  ),
    .alu_src2       (alu_src2  ),
    .alu_ready_go   (alu_ready_go),
    .alu_result     (EX_alu_result),
    .mem_addr       (EX_mem_addr ),
    .mul_result     (mul_result)   // two-cycle multiplier
);

assign EX_mem_we_b = {MEM_WE_W{EX_mem_addr[1:0]==2'b00}} & 4'b0001
                   | {MEM_WE_W{EX_mem_addr[1:0]==2'b01}} & 4'b0010
                   | {MEM_WE_W{EX_mem_addr[1:0]==2'b10}} & 4'b0100
                   | {MEM_WE_W{EX_mem_addr[1:0]==2'b11}} & 4'b1000;

assign EX_mem_we_h = {MEM_WE_W{EX_mem_addr[1:0]==2'b00}} & 4'b0011      // 未如指令手册中触发非对齐例外
                   | {MEM_WE_W{EX_mem_addr[1:0]==2'b10}} & 4'b1100;

assign EX_mem_we_w = 4'b1111;

assign {
    EX_inst_st_h,
    EX_inst_st_b,
    EX_inst_st_w
} = EX_st_op;

assign data_sram_en    = EX_valid & EX_validout & (EX_st_op || EX_ld_op);
assign data_sram_we    = {MEM_WE_W{EX_inst_st_b}} & EX_mem_we_b
                       | {MEM_WE_W{EX_inst_st_h}} & EX_mem_we_h
                       | {MEM_WE_W{EX_inst_st_w}} & EX_mem_we_w;
assign data_sram_addr  = EX_mem_addr;  // 优化时序
assign data_sram_wdata = {DATA_W{EX_inst_st_b}} & {4{EX_rkd_value[BYTE_W-1     :0]}}
                       | {DATA_W{EX_inst_st_h}} & {2{EX_rkd_value[HALFWORD_W-1 :0]}}
                       | {DATA_W{EX_inst_st_w}} &    EX_rkd_value[DATA_W-1     :0];

//********************************* MEMReg ****************************************
assign MEM_ready_go = 1'b1;
assign MEM_cancel = 1'b0;
assign MEM_datain = {
    EX_alu_result,
    EX_control_signal[MEM_CTRL_W-1:0],  // EX_src2_is_imm, EX_src1_is_pc, EX_alu_op, EX_st_op have been used
    EX_rd,
    EX_pc
};

pipeline_buffer #
(
    .WIDTH(MEM_REG_W)
) MEMReg (
    .clk(clk),
    .reset(reset),
    // allowin
    .post_allowin(WB_allowin),
    .allowin(MEM_allowin),
    // validout
    .pre_validout(EX_validout),
    .validout(MEM_validout),
    // data 
    .datain(MEM_datain),
    .dataout(MEM_dataout),
    // control field
    .ready_go(MEM_ready_go),
    .cancel(MEM_cancel),
    .valid(MEM_valid)
);

assign {
    MEM_alu_result,
    MEM_control_signal,
    MEM_rd,
    MEM_pc
} = MEM_dataout;

assign {
    MEM_sel_mul_result,
    MEM_res_from_mem,
    MEM_ld_op,
    MEM_dst_is_r1,
    MEM_gr_we
} = MEM_control_signal;

//********************************* MEM unit ****************************************
assign {
    MEM_inst_ld_hu,
    MEM_inst_ld_bu,
    MEM_inst_ld_h,
    MEM_inst_ld_b,
    MEM_inst_ld_w
} = MEM_ld_op;

assign MEM_mem_rdata_b = {BYTE_W{MEM_alu_result[1:0]==2'b00}} & data_sram_rdata[7:0]
                       | {BYTE_W{MEM_alu_result[1:0]==2'b01}} & data_sram_rdata[15:8]
                       | {BYTE_W{MEM_alu_result[1:0]==2'b10}} & data_sram_rdata[23:16]
                       | {BYTE_W{MEM_alu_result[1:0]==2'b11}} & data_sram_rdata[31:24];

assign MEM_mem_rdata_h = {HALFWORD_W{MEM_alu_result[1:0]==2'b00}} & data_sram_rdata[15:0]
                       | {HALFWORD_W{MEM_alu_result[1:0]==2'b10}} & data_sram_rdata[31:16];

assign MEM_mem_rdata_w  = data_sram_rdata;

assign MEM_mem_result   = {32{MEM_inst_ld_hu}} & {16'b0                    , MEM_mem_rdata_h}
                        | {32{MEM_inst_ld_bu}} & {24'b0                    , MEM_mem_rdata_b}
                        | {32{MEM_inst_ld_h }} & {{16{MEM_mem_rdata_h[15]}}, MEM_mem_rdata_h}
                        | {32{MEM_inst_ld_b }} & {{24{MEM_mem_rdata_b[ 7]}}, MEM_mem_rdata_b}
                        | {32{MEM_inst_ld_w }} &                             MEM_mem_rdata_w;

assign final_alu_result = {32{MEM_sel_mul_result[0]}} & MEM_alu_result
                        | {32{MEM_sel_mul_result[1]}} & mul_result[DATA_W-1:0]
                        | {32{MEM_sel_mul_result[2]}} & mul_result[DDATA_W-1:DATA_W];

assign MEM_final_result = MEM_res_from_mem ? MEM_mem_result : final_alu_result;

//********************************* WBReg ****************************************
assign WB_ready_go = 1'b1;
assign WB_cancel = 1'b0;
assign WB_datain = {
    MEM_final_result,
    MEM_control_signal[WB_CTRL_W-1:0],        // MEM_dst_is_r1, MEM_res_from_mem, MEM_sel_mul_result have been used
    MEM_rd,
    MEM_pc
};

pipeline_buffer #
(
    .WIDTH(WB_REG_W)
) WBReg (
    .clk(clk),
    .reset(reset),
    // allowin
    .post_allowin(1'b1),
    .allowin(WB_allowin),
    // validout
    .pre_validout(MEM_validout),
    .validout(WB_validout),
    // data 
    .datain(WB_datain),
    .dataout(WB_dataout),
    // control field
    .ready_go(WB_ready_go),
    .cancel(WB_cancel),
    .valid(WB_valid)
);

assign {
    WB_final_result,
    WB_control_signal,
    WB_rd,
    WB_pc
} = WB_dataout;

assign {
    WB_dst_is_r1, 
    WB_gr_we
} = WB_control_signal;

//********************************* WB unit ****************************************
assign rf_we    = WB_gr_we && WB_valid;
assign dest     = WB_dst_is_r1 ? 5'd1 : WB_rd;
assign rf_waddr = dest;
assign rf_wdata = WB_final_result;

//***************************** debug info generate ********************************
assign debug_wb_pc       = WB_pc;
assign debug_wb_rf_we    = {4{rf_we}};
assign debug_wb_rf_wnum  = dest;
assign debug_wb_rf_wdata = WB_final_result;

endmodule