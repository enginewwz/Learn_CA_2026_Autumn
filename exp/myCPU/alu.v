// ALU: one-hot select over all candidates, arithmetic core on bare `+` / `-`.
//
// The comparison tree is NOT here any more: the branch unit (br.v) owns the
// lt/ltu/eq compares, so this module only has to produce alu_result.  The
// subtractor is still needed for sub/slt/sltu; its borrow bit is used
// internally by the slt/sltu result bit and is not exported.
module alu(
  input  wire [11:0] alu_op,
  input  wire [31:0] src1,
  input  wire [31:0] src2,
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

//---------------------------------------------------------------------------
// candidates that are one or two LUTs away from the raw operands
//---------------------------------------------------------------------------
wire [31:0] and_result;
wire [31:0] nor_result;
wire [31:0] or_result;
wire [31:0] xor_result;
wire [31:0] lui_result;
wire [31:0] sll_result;
wire [63:0] sr64_result;   //only [31:0] is used below
wire [31:0] sr_result;

assign and_result = src1 & src2;
assign or_result  = src1 | src2;
assign nor_result = ~or_result;
assign xor_result = src1 ^ src2;
assign lui_result = src2;

// SLL result
assign sll_result = src1 << src2[4:0];

// SRL, SRA result
assign sr64_result = {{32{op_sra & src1[31]}}, src1[31:0]} >> src2[4:0];
assign sr_result   = sr64_result[31:0];

// The early candidates are masked and pre-ORed here, while the adders work.
// The OR-of-masked-candidates form is kept exactly, so a non one-hot alu_op
// still produces the same value as before.
wire [31:0] early_result;

assign early_result = ({32{op_and         }} & and_result)
                    | ({32{op_nor         }} & nor_result)
                    | ({32{op_or          }} & or_result)
                    | ({32{op_xor         }} & xor_result)
                    | ({32{op_lui         }} & lui_result)
                    | ({32{op_sll         }} & sll_result)
                    | ({32{op_srl | op_sra}} & sr_result);

//---------------------------------------------------------------------------
// add / sub / compare: two bare operators, no lookup tree
//    sum_add         = a + b
//    {ltu32,sum_sub} = {1'b0,a} - {1'b0,b}      (bit 32 = borrow = a <u b)
//---------------------------------------------------------------------------
wire [31:0] sum_add;
wire [31:0] sum_sub;
wire [32:0] sub_c;
wire        ltu32;      //a <u b, = the borrow out of a - b

assign sum_add = src1 + src2;
assign sub_c   = {1'b0, src1} - {1'b0, src2};
assign sum_sub = sub_c[31:0];
assign ltu32   = sub_c[32];

//---------------------------------------------------------------------------
// one-hot OR stage
//---------------------------------------------------------------------------
wire sub_form;
wire addsub_grp;
wire add_grp_sel;
wire sub_grp_sel;
wire cmp_bit;

assign sub_form    = op_sub | op_slt | op_sltu;
assign addsub_grp  = op_add | op_sub;
assign add_grp_sel = addsub_grp & ~sub_form;
assign sub_grp_sel = addsub_grp &  sub_form;

// Compare contributes to bit 0 only.  Both slt and sltu reduce to ltu32, so
// this is one 5-input LUT (a31, b31, ltu32, op_slt, op_sltu).
assign cmp_bit = (op_slt  & ((src1[31] & ~src2[31])
                          | ((src1[31] ~^ src2[31]) & ltu32)))
               | (op_sltu & ltu32);

assign alu_result = ({32{add_grp_sel}} & sum_add)
                  | ({32{sub_grp_sel}} & sum_sub)
                  | {31'b0, cmp_bit}
                  | early_result;

endmodule
