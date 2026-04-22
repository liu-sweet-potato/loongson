module pipeline_buffer #
(
    parameter WIDTH = 100
)(
    input wire clk,
    input wire reset,
    // allowin
    input wire post_allowin,
    output wire allowin,
    // validout
    input wire pre_validout,
    output wire validout,
    // data 
    input wire [WIDTH-1:0] datain,
    output reg [WIDTH-1:0] dataout,
    // control field
    input wire ready_go,
    input wire cancel,
    output reg valid
);

assign allowin = !valid || ready_go && post_allowin;
assign validout = valid && ready_go;
always @ (posedge clk) begin
    if(reset) begin
        valid <= 1'b0;
    end
    else if(cancel) begin
        valid <= 1'b0;
    end
    else if(allowin) begin
        valid <= pre_validout;
    end
    if(pre_validout && allowin) begin
        dataout <= datain;
    end
end
endmodule