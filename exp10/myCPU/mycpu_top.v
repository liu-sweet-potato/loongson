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
wire [31:0] seq_pc;  // default
wire [31:0] nextpc;
reg  [31:0] pc;
wire [31:0] IF_inst;

// IDReg I/O
wire        ID_allowin;
wire        ID_validout;
wire [63:0] ID_datain;
wire [63:0] ID_dataout;
wire        ID_ready_go;
wire        ID_cancel;
wire        ID_valid;

wire [31:0] ID_pc;
wire [31:0] ID_inst;

// ID I/O
wire [28:0] ID_control_signal  ;
wire [31:0] ID_rj_value        ;
wire [31:0] ID_rkd_value       ;
wire        br_taken           ;
wire [31:0] br_target          ;
wire [31:0] ID_imm             ;
wire [24:0] data_hazard_signals;
wire [3 :0] sel_rj_value       ;
wire [3 :0] sel_rkd_value      ;
wire        ID_not_ready_go    ;
wire        cancel             ;

wire        ID_gr_we           ;  // rf_we
wire        ID_dst_is_r1       ;  // sel_rf_dst
wire        ID_res_from_mem    ;  // sel_rf_res
wire [3 :0] ID_mem_we          ;  // data_ram_we
wire [18:0] ID_alu_op          ;  // one-hot
wire        ID_src1_is_pc      ;  // sel_alu_src1
wire        ID_src2_is_imm     ;  // sel_alu_src2
wire        ID_src_reg_is_rd   ;  // sel_rf_ra2

wire        select_ID_rj_value;
wire        select_EX_rj_value;
wire        select_MEM_rj_value;
wire        select_WB_rj_value;

wire        select_ID_rkd_value;
wire        select_EX_rkd_value;
wire        select_MEM_rkd_value;
wire        select_WB_rkd_value;

// GR I/O
wire [ 4:0] rf_raddr1;
wire [31:0] rf_rdata1;
wire [ 4:0] rf_raddr2;
wire [31:0] rf_rdata2;
wire        rf_we   ;
wire [ 4:0] rf_waddr;
wire [31:0] rf_wdata;

wire [4 :0] ID_rj;
wire [4 :0] ID_rd;
wire [4 :0] ID_rk;

// EXReg I/O
wire        EX_allowin;
wire        EX_validout;
wire [160:0] EX_datain;
wire [160:0] EX_dataout;
wire        EX_ready_go;
wire        EX_cancel;
wire        EX_valid;

wire [27:0] EX_control_signal  ;

wire        EX_gr_we           ;  // rf_we
wire        EX_dst_is_r1       ;  // sel_rf_dst
wire        EX_res_from_mem    ;  // sel_rf_res
wire [3 :0] EX_mem_we          ;  // data_ram_we
wire [18:0] EX_alu_op          ;  // one-hot
wire        EX_src1_is_pc      ;  // sel_alu_src1
wire        EX_src2_is_imm     ;  // sel_alu_src2

wire [31:0] EX_pc              ;
wire [31:0] EX_rj_value        ;
wire [31:0] EX_rkd_value       ;
wire [31:0] EX_imm             ;
wire [4 :0] EX_rd;

//ALU I/O
wire [31:0] alu_src1   ;
wire [31:0] alu_src2   ;
wire        alu_ready_go;
wire [31:0] EX_alu_result ;
wire [31:0] EX_mem_addr   ;

// MEMReg I/O
wire        MEM_allowin;
wire        MEM_validout;
wire [71:0] MEM_datain;
wire [71:0] MEM_dataout;
wire        MEM_ready_go;
wire        MEM_cancel;
wire        MEM_valid;

wire [2 :0] MEM_control_signal  ;

wire        MEM_gr_we           ;  // rf_we
wire        MEM_dst_is_r1       ;  // sel_rf_dst
wire        MEM_res_from_mem    ;  // sel_rf_res
wire [3 :0] MEM_mem_we          ;  // data_ram_we

wire [31:0] MEM_pc              ;
wire [31:0] MEM_rkd_value       ;
wire [4 :0] MEM_rd;
wire [31:0] MEM_alu_result ;

// MEM I/O
wire [31:0] MEM_mem_result ;
wire [31:0] MEM_final_result;

// WB I/O
wire        WB_allowin;
wire        WB_validout;
wire [70:0] WB_datain;
wire [70:0] WB_dataout;
wire        WB_ready_go;
wire        WB_cancel;
wire        WB_valid;

wire [1 :0] WB_control_signal  ;

wire        WB_gr_we           ;  // rf_we
wire        WB_dst_is_r1       ;  // sel_rf_dst

wire [31:0] WB_pc              ;
wire [4 :0] WB_rd;
wire [31:0] WB_final_result ;
wire [4: 0] dest;

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
    .WIDTH(1)
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
assign ID_datain[31:0]= pc;
assign ID_datain[63:32] = IF_inst;

pipeline_buffer #
(
    .WIDTH(64)
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

assign ID_pc = ID_dataout[31:0];
assign ID_inst = ID_dataout[63:32];

//********************************* ID unit ****************************************
assign data_hazard_signals[0] = EX_valid;
assign data_hazard_signals[1] = EX_gr_we;
assign data_hazard_signals[2] = EX_dst_is_r1;
assign data_hazard_signals[7:3] = EX_rd;

assign data_hazard_signals[8] = MEM_valid;
assign data_hazard_signals[9] = MEM_gr_we;
assign data_hazard_signals[10] = MEM_dst_is_r1;
assign data_hazard_signals[15:11] = MEM_rd;

assign data_hazard_signals[16] = WB_valid;
assign data_hazard_signals[17] = WB_gr_we;
assign data_hazard_signals[18] = WB_dst_is_r1;
assign data_hazard_signals[23:19] = WB_rd;

assign data_hazard_signals[24] = EX_res_from_mem;

instr_decoder # 
(
    .WIDTH(29)
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

assign ID_gr_we = ID_control_signal[0];
assign ID_dst_is_r1 = ID_control_signal[1];
assign ID_res_from_mem = ID_control_signal[2];
assign ID_mem_we = ID_control_signal[6:3];
assign ID_alu_op = ID_control_signal[25:7];
assign ID_src1_is_pc = ID_control_signal[26];
assign ID_src2_is_imm = ID_control_signal[27];
assign ID_src_reg_is_rd = ID_control_signal[28]; 


// GR
assign ID_rd   = ID_inst[ 4: 0];
assign ID_rj   = ID_inst[ 9: 5];
assign ID_rk   = ID_inst[14:10];

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

assign select_ID_rj_value = sel_rj_value[0];
assign select_EX_rj_value = sel_rj_value[3];
assign select_MEM_rj_value = sel_rj_value[2];
assign select_WB_rj_value = sel_rj_value[1];

assign select_ID_rkd_value = sel_rkd_value[0];
assign select_EX_rkd_value = sel_rkd_value[3];
assign select_MEM_rkd_value = sel_rkd_value[2];
assign select_WB_rkd_value = sel_rkd_value[1];

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
assign EX_datain[31:0]= ID_pc;
assign EX_datain[36:32] = ID_rd;
assign EX_datain[64:37] = ID_control_signal[27:0];
assign EX_datain[96:65] = ID_rj_value;
assign EX_datain[128:97] = ID_rkd_value;
assign EX_datain[160:129] = ID_imm;

pipeline_buffer #
(
    .WIDTH(161)
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

assign EX_pc = EX_dataout[31:0];
assign EX_rd = EX_dataout[36:32];
assign EX_control_signal[27:0] = EX_dataout[64:37];
assign EX_rj_value = EX_dataout[96:65];
assign EX_rkd_value = EX_dataout[128:97];
assign EX_imm = EX_dataout[160:129];

assign EX_gr_we = EX_control_signal[0];
assign EX_dst_is_r1 = EX_control_signal[1];
assign EX_res_from_mem = EX_control_signal[2];
assign EX_mem_we = EX_control_signal[6:3];
assign EX_alu_op = EX_control_signal[25:7];
assign EX_src1_is_pc = EX_control_signal[26];
assign EX_src2_is_imm = EX_control_signal[27];

//********************************* EX unit ****************************************
assign alu_src1 = EX_src1_is_pc  ? EX_pc[31:0] : EX_rj_value;
assign alu_src2 = EX_src2_is_imm ? EX_imm : EX_rkd_value;

alu u_alu(
    .clk            (clk       ),
    .reset          (reset     ),
    .alu_op         (EX_alu_op ),
    .alu_src1       (alu_src1  ),
    .alu_src2       (alu_src2  ),
    .alu_ready_go   (alu_ready_go),
    .alu_result (EX_alu_result),
    .mem_addr   (EX_mem_addr )
);

assign data_sram_en    = EX_valid && EX_validout;
assign data_sram_we    = EX_mem_we;
assign data_sram_addr  = EX_mem_addr;  // 优化时序
assign data_sram_wdata = EX_rkd_value;

//********************************* MEMReg ****************************************
assign MEM_ready_go = 1'b1;
assign MEM_cancel = 1'b0;
assign MEM_datain[31:0]= EX_pc;
assign  MEM_datain[36:32] = EX_rd;
assign MEM_datain[39:37] = EX_control_signal[2:0];
assign MEM_datain[71:40] = EX_alu_result;

pipeline_buffer #
(
    .WIDTH(72)
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

assign MEM_pc = MEM_dataout[31:0];
assign MEM_rd = MEM_dataout[36:32];
assign MEM_control_signal[2:0] = MEM_dataout[39:37];
assign MEM_alu_result = MEM_dataout[71:40];

assign MEM_gr_we = MEM_control_signal[0];
assign MEM_dst_is_r1 = MEM_control_signal[1];
assign MEM_res_from_mem = MEM_control_signal[2];

//********************************* MEM unit ****************************************
assign MEM_mem_result   = data_sram_rdata;

assign MEM_final_result = MEM_res_from_mem ? MEM_mem_result : MEM_alu_result;

//********************************* WBReg ****************************************
assign WB_ready_go = 1'b1;
assign WB_cancel = 1'b0;
assign WB_datain[31:0]= MEM_pc;
assign WB_datain[36:32] = MEM_rd;
assign WB_datain[38:37] = MEM_control_signal[1:0];
assign WB_datain[70:39] = MEM_final_result;

pipeline_buffer #
(
    .WIDTH(71)
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

assign WB_pc = WB_dataout[31:0];
assign  WB_rd = WB_dataout[36:32];
assign WB_control_signal[1:0] = WB_dataout[38:37];
assign WB_final_result = WB_dataout[70:39];

assign WB_gr_we = WB_control_signal[0];
assign WB_dst_is_r1 = WB_control_signal[1];

//********************************* WB unit ****************************************
assign rf_we    = WB_gr_we && WB_valid;
assign dest          = WB_dst_is_r1 ? 5'd1 : WB_rd;
assign rf_waddr = dest;
assign rf_wdata = WB_final_result;

//***************************** debug info generate ********************************
assign debug_wb_pc       = WB_pc;
assign debug_wb_rf_we   = {4{rf_we}};
assign debug_wb_rf_wnum  = dest;
assign debug_wb_rf_wdata = WB_final_result;

endmodule