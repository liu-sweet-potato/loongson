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

// IF I/O
wire [31:0] seq_pc;  // default
wire [31:0] nextpc;
reg  [31:0] pc;
wire [31:0] IF_inst;
reg         IF_valid;

// IDReg I/O
wire        ID_allowin;
wire        ID_validout;
wire [63:0] ID_datain;
wire [63:0] ID_dataout;
wire        ID_ready_go;
wire        ID_valid;

wire [31:0] ID_pc;
wire [31:0] ID_inst;

// ID I/O
wire [21:0] ID_control_signal  ;
wire [31:0] ID_rj_value        ;
wire [31:0] ID_rkd_value       ;
wire        br_taken           ;
wire [31:0] br_target          ;
wire [31:0] ID_imm             ;

wire        ID_gr_we           ;  // rf_we
wire        ID_dst_is_r1       ;  // sel_rf_dst
wire        ID_res_from_mem    ;  // sel_rf_res
wire [3 :0] ID_mem_we          ;  // data_ram_we
wire [11:0] ID_alu_op          ;  // one-hot
wire        ID_src1_is_pc      ;  // sel_alu_src1
wire        ID_src2_is_imm     ;  // sel_alu_src2
wire        ID_src_reg_is_rd   ;  // sel_rf_ra2

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
wire [153:0] EX_datain;
wire [153:0] EX_dataout;
wire        EX_ready_go;
wire        EX_valid;

wire [20:0] EX_control_signal  ;

wire        EX_gr_we           ;  // rf_we
wire        EX_dst_is_r1       ;  // sel_rf_dst
wire        EX_res_from_mem    ;  // sel_rf_res
wire [3 :0] EX_mem_we          ;  // data_ram_we
wire [11:0] EX_alu_op          ;  // one-hot
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
wire [31:0] EX_alu_result ;
wire [31:0] EX_mem_addr   ;

// MEMReg I/O
wire        MEM_allowin;
wire        MEM_validout;
wire [71:0] MEM_datain;
wire [71:0] MEM_dataout;
wire        MEM_ready_go;
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
always @(posedge clk) reset <= ~resetn;  // 恰可以提前一周期恢复指令存储器使能以取指

always @(posedge clk) begin
    if (reset) begin
        IF_valid <= 1'b0;
    end
    else begin
        IF_valid <= 1'b1;
    end
end

assign seq_pc       = pc + 32'h4;
assign nextpc       = (br_taken && ID_valid) ? br_target : seq_pc;
always @(posedge clk) begin
    if (reset) begin
        pc <= 32'h1bfffffc;     //trick: to make nextpc be 0x1c000000 during reset 
    end
    else begin
        pc <= nextpc;
    end
end

// inst_sram
assign inst_sram_we    = 4'b0           ;  // 默认不可写
assign inst_sram_addr  = nextpc         ;  // 可读
assign inst_sram_wdata = 32'b0          ;  // 默认不可写
assign IF_inst         = inst_sram_rdata;  // 可读
assign inst_sram_en    = resetn         ;  // 提前一周期恢复使能预读指令

//********************************* IDReg ****************************************
assign ID_ready_go = 1'b1;
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
    .pre_validout(IF_valid),
    .validout(ID_validout),
    // data 
    .datain(ID_datain),
    .dataout(ID_dataout),
    // control field
    .ready_go(ID_ready_go),
    .valid(ID_valid)
);

assign ID_pc = ID_dataout[31:0];
assign ID_inst = ID_dataout[63:32];

//********************************* ID unit ****************************************
instr_decoder u_id(
    .inst(ID_inst),
    .control_signal(ID_control_signal),   
    .pc(ID_pc),
    .rj_value(ID_rj_value),
    .rkd_value(ID_rkd_value),
    .br_taken(br_taken),
    .br_target(br_target),
    .imm(ID_imm)
);

assign ID_gr_we = ID_control_signal[0];
assign ID_dst_is_r1 = ID_control_signal[1];
assign ID_res_from_mem = ID_control_signal[2];
assign ID_mem_we = ID_control_signal[6:3];
assign ID_alu_op = ID_control_signal[18:7];
assign ID_src1_is_pc = ID_control_signal[19];
assign ID_src2_is_imm = ID_control_signal[20];
assign ID_src_reg_is_rd = ID_control_signal[21]; 

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
assign ID_rj_value  = rf_rdata1;
assign ID_rkd_value = rf_rdata2;

//********************************* EXReg ****************************************
assign EX_ready_go = 1'b1;
assign EX_datain[31:0]= ID_pc;
assign  EX_datain[36:32] = ID_rd;
assign EX_datain[57:37] = ID_control_signal[20:0];
assign EX_datain[89:58] = ID_rj_value;
assign EX_datain[121:90] = ID_rkd_value;
assign EX_datain[153:122] = ID_imm;

pipeline_buffer #
(
    .WIDTH(154)
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
    .valid(EX_valid)
);

assign EX_pc = EX_dataout[31:0];
assign EX_rd = EX_dataout[36:32];
assign EX_control_signal[20:0] = EX_dataout[57:37];
assign EX_rj_value = EX_dataout[89:58];
assign EX_rkd_value = EX_dataout[121:90];
assign EX_imm = EX_dataout[153:122];

assign EX_gr_we = EX_control_signal[0];
assign EX_dst_is_r1 = EX_control_signal[1];
assign EX_res_from_mem = EX_control_signal[2];
assign EX_mem_we = EX_control_signal[6:3];
assign EX_alu_op = EX_control_signal[18:7];
assign EX_src1_is_pc = EX_control_signal[19];
assign EX_src2_is_imm = EX_control_signal[20];

//********************************* EX unit ****************************************
assign alu_src1 = EX_src1_is_pc  ? EX_pc[31:0] : EX_rj_value;
assign alu_src2 = EX_src2_is_imm ? EX_imm : EX_rkd_value;

alu u_alu(
    .alu_op     (EX_alu_op ),
    .alu_src1   (alu_src1  ),
    .alu_src2   (alu_src2  ),
    .alu_result (EX_alu_result),
    .mem_addr   (EX_mem_addr )
);

assign data_sram_en    = EX_valid;
assign data_sram_we    = EX_mem_we;
assign data_sram_addr  = EX_mem_addr;  // 优化时序
assign data_sram_wdata = EX_rkd_value;

//********************************* MEMReg ****************************************
assign MEM_ready_go = 1'b1;
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
    .valid(MEM_valid)
);

assign MEM_pc = MEM_dataout[31:0];
assign  MEM_rd = MEM_dataout[36:32];
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
assign WB_datain[31:0]= MEM_pc;
assign  WB_datain[36:32] = MEM_rd;
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