module alu(
  input  wire [11:0] alu_op,
  input  wire [31:0] alu_src1,
  input  wire [31:0] alu_src2,
  output wire [31:0] alu_result
);

// control code decomposition: one bit per operation
wire op_add;   //add operation
wire op_sub;   //sub operation
wire op_slt;   //signed compared and set less than
wire op_sltu;  //unsigned compared and set less than
wire op_and;   //bitwise and
wire op_nor;   //bitwise nor
wire op_or;    //bitwise or
wire op_xor;   //bitwise xor
wire op_sll;   //logic left shift
wire op_srl;   //logic right shift
wire op_sra;   //arithmetic right shift
wire op_lui;   //Load Upper Immediate

assign op_add  = alu_op[ 0];
assign op_sub  = alu_op[ 1];
assign op_slt  = alu_op[ 2];
assign op_sltu = alu_op[ 3];
assign op_and  = alu_op[ 4];
assign op_nor  = alu_op[ 5];
assign op_or   = alu_op[ 6];
assign op_xor  = alu_op[ 7];
assign op_sll  = alu_op[ 8];
assign op_srl  = alu_op[ 9];
assign op_sra  = alu_op[10];
assign op_lui  = alu_op[11];

// every candidate is computed in parallel, straight from the raw operands

wire [31:0] add_result;
assign add_result = alu_src1 + alu_src2;

wire [32:0] sub_ext;
wire [31:0] sub_result;
assign sub_ext    = {1'b0, alu_src1} - {1'b0, alu_src2};
assign sub_result = sub_ext[31:0];

wire cmp_bit;
assign cmp_bit = (op_slt  & ((alu_src1[31] & ~alu_src2[31])
                | ((alu_src1[31] ~^ alu_src2[31]) & sub_result[31])))
                | (op_sltu & sub_ext[32]);

// bitwise / lui
wire [31:0] and_result;
wire [31:0] nor_result;
wire [31:0] or_result;
wire [31:0] xor_result;
wire [31:0] lui_result;

assign and_result = alu_src1 & alu_src2;
assign or_result  = alu_src1 | alu_src2;
assign nor_result = ~or_result;
assign xor_result = alu_src1 ^ alu_src2;
assign lui_result = alu_src2;

// shifts
wire [31:0] sll_result;
wire [63:0] sr64_result;   //only [31:0] is used below
wire [31:0] sr_result;

assign sll_result  = alu_src1 << alu_src2[4:0];                                       //rj << i5
assign sr64_result = {{32{op_sra & alu_src1[31]}}, alu_src1[31:0]} >> alu_src2[4:0];  //rj >> i5
assign sr_result   = sr64_result[31:0];

// one-hot OR select
wire sub_form;
wire add_grp_sel;
wire sub_grp_sel;

assign sub_form    = op_sub | op_slt | op_sltu;
assign add_grp_sel = (op_add | op_sub) & ~sub_form;
assign sub_grp_sel = (op_add | op_sub) &  sub_form;

// final result mux: one AND per candidate, one OR stage
assign alu_result = ({32{add_grp_sel}} & add_result)
                  | ({32{sub_grp_sel}} & sub_result)
                  | {31'b0, cmp_bit}
                  | ({32{op_and         }} & and_result)
                  | ({32{op_nor         }} & nor_result)
                  | ({32{op_or          }} & or_result)
                  | ({32{op_xor         }} & xor_result)
                  | ({32{op_lui         }} & lui_result)
                  | ({32{op_sll         }} & sll_result)
                  | ({32{op_srl | op_sra}} & sr_result);

endmodule
