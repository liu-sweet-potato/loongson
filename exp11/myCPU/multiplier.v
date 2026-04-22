/*
* 33bit booth编码器、华莱士树搭建乘法器
*/


//*****************************************************************************************
//*********************************2-bit booth encoder*************************************
//*****************************************************************************************
module booth_encoder (
    input   wire [65:0] x,
    input   wire [2:0]  y,
    output  wire [65:0] p,
    output  wire        c
);
assign p = ({66{y==3'b000 || y==3'b111}} & 66'h0           )
         | ({66{y==3'b001 || y==3'b010}} & x               )
         | ({66{y==3'b011             }} & {x[64:0], 1'b0} )    
         | ({66{y==3'b100             }} & ~{x[64:0], 1'b0})  
         | ({66{y==3'b101 || y==3'b110}} & ~x              );

assign c = (y==3'b100 || y==3'b101 || y==3'b110) ? 1'b1 : 1'b0;
endmodule


//*****************************************************************************************
//***********************************1-bit full adder**************************************
//*****************************************************************************************
module full_adder (
    input   wire a,
    input   wire b,
    input   wire c,
    output  wire cout,
    output  wire s
);
assign {cout, s} = a + b + c;
endmodule



//*****************************************************************************************
//*********************************1-bit Wallace Tree**************************************
//*****************************************************************************************
module wallace_tree (
    // carry
    input   wire [14:0] cin,
    output  wire [14:0] cout,
    // input data
    input   wire [16:0] din,
    // output bit
    output  wire c,
    output  wire s
);
// inner sum bit
wire [14:0] s_inner;

// the first layer
full_adder adder_00 (
    .a(din[1]),
    .b(din[0]),
    .c(1'b0),
    .cout(cout[0]),
    .s(s_inner[0])
);

full_adder adder_01 (
    .a(din[4]),
    .b(din[3]),
    .c(din[2]),
    .cout(cout[1]),
    .s(s_inner[1])
);

full_adder adder_02 (
    .a(din[7]),
    .b(din[6]),
    .c(din[5]),
    .cout(cout[2]),
    .s(s_inner[2])
);

full_adder adder_03 (
    .a(din[10]),
    .b(din[9]),
    .c(din[8]),
    .cout(cout[3]),
    .s(s_inner[3])
);

full_adder adder_04 (
    .a(din[13]),
    .b(din[12]),
    .c(din[11]),
    .cout(cout[4]),
    .s(s_inner[4])
);

full_adder adder_05 (
    .a(din[16]),
    .b(din[15]),
    .c(din[14]),
    .cout(cout[5]),
    .s(s_inner[5])
);

// the second layer
full_adder adder_06 (
    .a(cin[2]),
    .b(cin[1]),
    .c(cin[0]),
    .cout(cout[6]),
    .s(s_inner[6])
);

full_adder adder_07 (
    .a(cin[5]),
    .b(cin[4]),
    .c(cin[3]),
    .cout(cout[7]),
    .s(s_inner[7])
);

full_adder adder_08 (
    .a(s_inner[2]),
    .b(s_inner[1]),
    .c(s_inner[0]),
    .cout(cout[8]),
    .s(s_inner[8])
);

full_adder adder_09 (
    .a(s_inner[5]),
    .b(s_inner[4]),
    .c(s_inner[3]),
    .cout(cout[9]),
    .s(s_inner[9])
);

// the third layer
full_adder adder_10 (
    .a(s_inner[6]),
    .b(cin[7]),
    .c(cin[6]),
    .cout(cout[10]),
    .s(s_inner[10])
);

full_adder adder_11 (
    .a(s_inner[7]),
    .b(s_inner[8]),
    .c(s_inner[9]),
    .cout(cout[11]),
    .s(s_inner[11])
);

// the fourth layer
full_adder adder_12 (
    .a(cin[10]),
    .b(cin[9]),
    .c(cin[8]),
    .cout(cout[12]),
    .s(s_inner[12])
);

full_adder adder_13 (
    .a(s_inner[11]),
    .b(s_inner[10]),
    .c(cin[11]),
    .cout(cout[13]),
    .s(s_inner[13])
);

// the fifth layer
full_adder adder_14 (
    .a(s_inner[13]),
    .b(s_inner[12]),
    .c(cin[12]),
    .cout(cout[14]),
    .s(s_inner[14])
);

// the sixth layer
full_adder adder_15 (
    .a(s_inner[14]),
    .b(cin[14]),
    .c(cin[13]),
    .cout(c),
    .s(s)
);

endmodule



//*****************************************************************************************
//*********************************33bit multiplier****************************************
//*****************************************************************************************
module mul (
    input   wire        mul_clk,
    input   wire        resetn,
    input   wire        mul_signed,
    input   wire [31:0] x,
    input   wire [31:0] y,
    output  wire [63:0] result
);
//*********************************参数定义****************************************
// 2-bit booth encoder
localparam booth_encoder_amount = 17;
localparam extended_x_width = 66;
localparam extended_y_width = 33;
localparam p_width = 66;
localparam c_width = 1;

// wallace tree
localparam wallace_tree_amount = 66;
localparam din_width = 17;
localparam cin_width = 15;
localparam cout_width = 15;

// 66-bit adder
localparam adder_width = 66;

// temporary variate for block generation
genvar i;
genvar j;

//*********************************管脚定义****************************************
// 2-bit booth_encoder
wire [extended_x_width-1: 0] extended_x;
wire [extended_x_width-1: 0] shift_extended_x [booth_encoder_amount-1: 0];
wire [extended_y_width-1: 0] extended_y;

wire [p_width-1:              0] p            [booth_encoder_amount-1: 0];
wire [booth_encoder_amount-1: 0] c;

// buffer_layer
reg [p_width-1: 0] p_buffer[booth_encoder_amount-1: 0];
reg [booth_encoder_amount-1: 0] c_buffer;

// generate_wallace_tree_layer
wire [din_width-1  : 0] wallace_tree_input[wallace_tree_amount-1: 0];
wire [cout_width-1 : 0] wallace_tree_cout [wallace_tree_amount-1: 0];
wire [adder_width-1: 0] adder_a;    // {carry, c[]}
wire [adder_width-1: 0] adder_b;    // sum
wire useless_c;


//********************************* 连线赋值 ****************************************
assign extended_x = mul_signed ? {{34{x[31]}}, x} : {34'b0, x};
assign extended_y = mul_signed ? {y[31], y} : {1'b0, y};

// left shift x adjust
assign shift_extended_x[0] = $signed(extended_x) >>> 1;    // signed number
generate for(i=1; i<booth_encoder_amount; i=i+1) begin : left_shift_extended_x_adjust
    assign shift_extended_x[i] = $signed(extended_x) <<< (2*i-1);
end endgenerate

// generate_booth_encoder_layer
// booth_encoder_0(怕右移再左移精度有损失)
assign p[0] = y[0] ? ~extended_x : 66'h0;
assign c[0] = y[0] ? 1'b1: 1'b0;

generate for(i=1; i<booth_encoder_amount; i=i+1) begin : generate_booth_encoder_layer
    booth_encoder booth_encoder_i (
    .x(shift_extended_x[i]),
    .y({extended_y[2*i], extended_y[2*i-1], extended_y[2*i-2]}),
    .p(p[i]),
    .c(c[i])
    );
end endgenerate

// buffer_layer
generate for(i=0; i<booth_encoder_amount; i=i+1) begin: init_buffer
    always @ (posedge mul_clk) begin
        if(!resetn) begin                      
                p_buffer[i] <= 0; 
                c_buffer[i] <= 0;
        end
        else begin
            p_buffer[i] <= p[i];
            c_buffer[i] <= c[i];
        end
    end
end endgenerate

// transpose buffer output into wallace tree input
generate for(i=0; i<wallace_tree_amount; i=i+1) begin : transpose_outer
            for(j=0; j<din_width; j=j+1) begin : transpose_inner
                assign wallace_tree_input[i][j] = p_buffer[j][i];
            end 
        end
endgenerate

// generate_wallace_tree_layer 
wallace_tree wallace_tree_0 (
    .cin(c_buffer[cin_width-1: 0]), // 17个c，先送15个
    .din(wallace_tree_input[0]),
    .cout(wallace_tree_cout[0]),
    .c(adder_a[1]),
    .s(adder_b[0])
);

generate for(i=1; i<wallace_tree_amount-1; i=i+1) begin : generate_wallace_tree_layer
    wallace_tree wallace_tree_i (
    .cin(wallace_tree_cout[i-1]),
    .din(wallace_tree_input[i]),
    .cout(wallace_tree_cout[i]),
    .c(adder_a[i+1]),
    .s(adder_b[i])
);
end endgenerate

wallace_tree wallace_tree_65 (
    .cin(wallace_tree_cout[64]),
    .din(wallace_tree_input[65]),
    .cout(wallace_tree_cout[65]),
    .c(useless_c),
    .s(adder_b[65])
);

// 66-bit adder
assign adder_a[0] = c_buffer[booth_encoder_amount-2]; // 再送一个c
assign result = adder_a + adder_b + c_buffer[booth_encoder_amount-1]; // 最后一个c

endmodule