// Slot 1 decoder of the dual-issue core: only the ALU and MUL subsets.
//
// Slot 1 never issues anything else (a load/store, branch, CSR, TLB, cacop, div
// or an illegal instruction just waits there and falls into slot 0), so this
// decoder is much smaller than the full one in mycpu_top:
//   * no mem / csr / tlb / branch / exception decode at all,
//   * every instruction it accepts writes rd (gr_we is implied by `legal`),
//   * src2 is never rd (no `src_reg_is_rd` instruction is in the set),
//   * no "src2 = 4" (that is jirl / bl only).
//
// alu_op / md_op keep exactly the bit assignment of mycpu_top / alu.v / mul.v,
// so slot 0 and slot 1 can share the ALU and mul modules.
module id_alu_mul_decoder(
    input  wire [31:0] inst,
    output wire        legal,        // alu or mul: slot 1 may issue it

    output wire [11:0] alu_op,
    output wire [ 6:0] md_op,        // only the MUL_* bits are ever set

    output wire [ 4:0] dest,
    output wire [ 4:0] rf_raddr1,
    output wire [ 4:0] rf_raddr2,
    output wire        src1_used,
    output wire        src2_used,
    output wire        src1_is_pc,
    output wire        src2_is_imm,
    output wire [31:0] imm
);

// md_op bit positions (same as the top / mul)
localparam MUL_W    = 0;
localparam MULH_W   = 1;
localparam MULH_WU  = 2;

wire [ 5:0] op_31_26;
wire [ 3:0] op_25_22;
wire [ 1:0] op_21_20;
wire [ 4:0] op_19_15;

wire [63:0] op_31_26_d;
wire [15:0] op_25_22_d;
wire [ 3:0] op_21_20_d;
wire [31:0] op_19_15_d;

assign op_31_26 = inst[31:26];
assign op_25_22 = inst[25:22];
assign op_21_20 = inst[21:20];
assign op_19_15 = inst[19:15];

decoder_6_64 u_dec0(.in(op_31_26), .out(op_31_26_d));
decoder_4_16 u_dec1(.in(op_25_22), .out(op_25_22_d));
decoder_2_4  u_dec2(.in(op_21_20), .out(op_21_20_d));
decoder_5_32 u_dec3(.in(op_19_15), .out(op_19_15_d));

// ---------------------------------------------------------------- ALU group
wire inst_add_w, inst_sub_w, inst_slt, inst_sltu, inst_nor, inst_and, inst_or, inst_xor;
wire inst_slli_w, inst_srli_w, inst_srai_w, inst_addi_w;
wire inst_slti, inst_sltui, inst_andi, inst_ori, inst_xori;
wire inst_sll_w, inst_srl_w, inst_sra_w, inst_lu12i_w, inst_pcaddu12i;

assign inst_add_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h00];
assign inst_sub_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h02];
assign inst_slt    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h04];
assign inst_sltu   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h05];
assign inst_nor    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h08];
assign inst_and    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h09];
assign inst_or     = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0a];
assign inst_xor    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0b];
assign inst_sll_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0e];
assign inst_srl_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0f];
assign inst_sra_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h10];

assign inst_slli_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h01];
assign inst_srli_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h09];
assign inst_srai_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h11];

assign inst_addi_w = op_31_26_d[6'h00] & op_25_22_d[4'ha];
assign inst_slti   = op_31_26_d[6'h00] & op_25_22_d[4'h8];
assign inst_sltui  = op_31_26_d[6'h00] & op_25_22_d[4'h9];
assign inst_andi   = op_31_26_d[6'h00] & op_25_22_d[4'hd];
assign inst_ori    = op_31_26_d[6'h00] & op_25_22_d[4'he];
assign inst_xori   = op_31_26_d[6'h00] & op_25_22_d[4'hf];

assign inst_lu12i_w   = op_31_26_d[6'h05] & ~ inst[25];
assign inst_pcaddu12i = op_31_26_d[6'h07];

// ---------------------------------------------------------------- MUL group
wire inst_mul_w, inst_mulh_w, inst_mulh_wu;

assign inst_mul_w   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h18];
assign inst_mulh_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h19];
assign inst_mulh_wu = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h1a];

// ---------------------------------------------------------------- outputs
wire alu_rr;     // R-type ALU: both sources are registers

assign alu_rr = inst_add_w | inst_sub_w | inst_slt | inst_sltu | inst_nor | inst_and
              | inst_or | inst_xor | inst_sll_w | inst_srl_w | inst_sra_w;

assign legal = alu_rr
             | inst_slli_w | inst_srli_w | inst_srai_w | inst_addi_w
             | inst_slti | inst_sltui | inst_andi | inst_ori | inst_xori
             | inst_lu12i_w | inst_pcaddu12i
             | inst_mul_w | inst_mulh_w | inst_mulh_wu;

assign alu_op[ 0] = inst_add_w | inst_addi_w | inst_pcaddu12i;
assign alu_op[ 1] = inst_sub_w;
assign alu_op[ 2] = inst_slt | inst_slti;
assign alu_op[ 3] = inst_sltu | inst_sltui;
assign alu_op[ 4] = inst_and | inst_andi;
assign alu_op[ 5] = inst_nor;
assign alu_op[ 6] = inst_or | inst_ori;
assign alu_op[ 7] = inst_xor | inst_xori;
assign alu_op[ 8] = inst_slli_w | inst_sll_w;
assign alu_op[ 9] = inst_srli_w | inst_srl_w;
assign alu_op[10] = inst_srai_w | inst_sra_w;
assign alu_op[11] = inst_lu12i_w;

assign md_op[MUL_W]   = inst_mul_w;
assign md_op[MULH_W]  = inst_mulh_w;
assign md_op[MULH_WU] = inst_mulh_wu;
assign md_op[6:3]     = 4'b0;

assign dest      = inst[4:0];
assign rf_raddr1 = inst[9:5];
assign rf_raddr2 = inst[14:10];

// lu12i takes only the immediate, pcaddu12i only pc + immediate
assign src1_used    = legal & ~ (inst_lu12i_w | inst_pcaddu12i);
assign src2_used    = alu_rr | inst_mul_w | inst_mulh_w | inst_mulh_wu;
assign src1_is_pc   = inst_pcaddu12i;
assign src2_is_imm  = legal & ~ src2_used;

wire [11:0] i12;
wire [19:0] i20;
wire        need_ui12;
wire        need_si20;

assign i12 = inst[21:10];
assign i20 = inst[24: 5];
assign need_ui12 = inst_andi | inst_ori | inst_xori;
assign need_si20 = inst_lu12i_w | inst_pcaddu12i;

assign imm = need_si20 ? {i20[19:0], 12'b0}        :
             need_ui12 ? {20'b0, i12[11:0]}        :
                         {{20{i12[11]}}, i12[11:0]};

endmodule
