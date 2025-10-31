// 2-4译码器
module decoder_2_4(
    input  wire [ 1:0] in,
    output wire [ 3:0] co
);

genvar i;  // 声明索引变量
generate for (i=0; i<4; i=i+1) begin : gen_for_dec_2_4  // 生成块标签
    assign co[i] = (in == i);  // 赋值语句
end endgenerate

endmodule


// 4-16译码器
module decoder_4_16(
    input  wire [ 3:0] in,
    output wire [15:0] co
);

genvar i;
generate for (i=0; i<16; i=i+1) begin : gen_for_dec_4_16
    assign co[i] = (in == i);
end endgenerate

endmodule


// 5-32译码器
module decoder_5_32(
    input  wire [ 4:0] in,
    output wire [31:0] co
);

genvar i;
generate for (i=0; i<32; i=i+1) begin : gen_for_dec_5_32
    assign co[i] = (in == i);
end endgenerate

endmodule


// 6-64译码器
module decoder_6_64(
    input  wire [ 5:0] in,
    output wire [63:0] co
);

genvar i;
generate for (i=0; i<64; i=i+1) begin : gen_for_dec_6_64
    assign co[i] = (in == i);
end endgenerate

endmodule



