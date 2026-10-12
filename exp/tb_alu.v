// Standalone directional + random testbench for alu.v (not part of the CPU build).
//
//   iverilog -g2005 -o /tmp/tb_alu exp/myCPU/alu.v exp/tb_alu.v
//   timeout 60 vvp /tmp/tb_alu
//
// The reference model is written from the ALU/LoongArch semantics, not from the
// RTL: a 33-bit  a + (sub ? ~b : b) + sub  adder plus an explicit OR of every
// candidate masked by its one-hot opcode bit.  For one-hot opcodes that is
// exactly what alu.v computes.
//
// Coverage:
//   * all 12 one-hot opcodes (add/sub/slt/sltu/and/nor/or/xor/lui/sll/srl/sra)
//     over a corner-value cross product: 0, 1, 2, 0x7fffffff, 0x80000000,
//     0x80000001, 0xffffffff, 0xfffffffe, random-looking patterns, and shift
//     amounts 0 / 31 / 32(+garbage high bits) / 63
//   * signed and unsigned compares in both directions, including the
//     -2^31 vs 2^31-1 and 0xffffffff vs 0x80000000 cases
//   * sra on negative operands, srl with the sign bit set (no sign leak)
//   * shifts whose amount field b[4:0] is 0 because b has garbage in [31:5]
//   * 20000 random (op, a, b) triples from a fixed seed
//
// alu_op is one-hot by construction (mycpu_top builds alu_op_id from the one-hot
// instruction decode), so only one-hot opcodes are driven here; multi-hot
// vectors are not part of the contract and the RTL does not decode them.
`timescale 1ns/1ps

module tb_alu;

reg  [11:0] op;
reg  [31:0] a, b;
wire [31:0] y;

alu u_alu(
    .alu_op     (op),
    .alu_src1   (a ),
    .alu_src2   (b ),
    .alu_result (y )
);

localparam ADD  = 0;
localparam SUB  = 1;
localparam SLT  = 2;
localparam SLTU = 3;
localparam AND  = 4;
localparam NOR  = 5;
localparam OR   = 6;
localparam XOR  = 7;
localparam SLL  = 8;
localparam SRL  = 9;
localparam SRA  = 10;
localparam LUI  = 11;

integer checks = 0;
integer errors = 0;
integer i, j, k;

//-----------------------------------------------------------------------------
// independent reference model
//-----------------------------------------------------------------------------
function [31:0] ref_alu;
    input [11:0] o;
    input [31:0] x, z;
    reg   [32:0] diff;
    reg   [63:0] sr;
    reg   [31:0] r;
    reg          sub;
    begin
        sub  = o[SUB] | o[SLT] | o[SLTU];
        diff = {1'b0, x} + {1'b0, (sub ? ~z : z)} + {32'b0, sub};
        r = 32'b0;
        if (o[ADD] | o[SUB]) r = r | diff[31:0];
        if (o[SLT])  r = r | {31'b0, (x[31] & ~z[31]) | ((x[31] ~^ z[31]) & diff[31])};
        if (o[SLTU]) r = r | {31'b0, ~diff[32]};
        if (o[AND])  r = r | (x & z);
        if (o[NOR])  r = r | ~(x | z);
        if (o[OR])   r = r | (x | z);
        if (o[XOR])  r = r | (x ^ z);
        if (o[LUI])  r = r | z;
        if (o[SLL])  r = r | (x << z[4:0]);
        if (o[SRL] | o[SRA]) begin
            sr = {{32{o[SRA] & x[31]}}, x} >> z[4:0];
            r  = r | sr[31:0];
        end
        ref_alu = r;
    end
endfunction

//-----------------------------------------------------------------------------
// one comparison
//-----------------------------------------------------------------------------
task chk;
    input [11:0] o;
    input [31:0] x, z;
    reg   [31:0] e;
    begin
        op = o; a = x; b = z;
        #1;
        e = ref_alu(o, x, z);
        checks = checks + 1;
        if (y !== e) begin
            errors = errors + 1;
            if (errors <= 20)
                $display("ALU ERROR: op=%b a=%h b=%h got=%h exp=%h", o, x, z, y, e);
        end
    end
endtask

// corner values: signs, all-ones, all-zeros, value 1/2, shift boundaries
reg [31:0] vals [0:15];

initial begin
    vals[ 0] = 32'h0000_0000;
    vals[ 1] = 32'h0000_0001;
    vals[ 2] = 32'h0000_0002;
    vals[ 3] = 32'h7fff_ffff;
    vals[ 4] = 32'h8000_0000;
    vals[ 5] = 32'h8000_0001;
    vals[ 6] = 32'hffff_ffff;
    vals[ 7] = 32'hffff_fffe;
    vals[ 8] = 32'h0000_001f;   // shift amount 31
    vals[ 9] = 32'h0000_0020;   // shift amount 32 -> uses b[4:0] = 0
    vals[10] = 32'hffff_ffe0;   // garbage in [31:5], b[4:0] = 0
    vals[11] = 32'h0000_003f;   // shift amount 63 -> uses b[4:0] = 31
    vals[12] = 32'h1234_5678;
    vals[13] = 32'hdead_beef;
    vals[14] = 32'h0000_8000;
    vals[15] = 32'h7fff_0000;

    //------------------------------------------------------------------ reset
    op = 12'b0; a = 32'b0; b = 32'b0;
    #5;

    //-------------------------------------- every one-hot opcode x corner pair
    for (k = 0; k < 12; k = k + 1) begin
        for (i = 0; i < 16; i = i + 1) begin
            for (j = 0; j < 16; j = j + 1) begin
                chk(12'b1 << k, vals[i], vals[j]);
            end
        end
    end

    //-------------------------------------- explicit compare / shift corners
    // signed compare: the classic overflow pairs
    chk(12'b1 << SLT,  32'h7fff_ffff, 32'h8000_0000);   //  2^31-1 <  -2^31 ? yes
    chk(12'b1 << SLT,  32'h8000_0000, 32'h7fff_ffff);   // -2^31   <   2^31-1 ? yes
    chk(12'b1 << SLT,  32'hffff_ffff, 32'h0000_0000);   // -1 < 0
    chk(12'b1 << SLT,  32'h0000_0000, 32'hffff_ffff);   // 0 < -1 ? no
    chk(12'b1 << SLT,  32'h8000_0000, 32'h8000_0001);   // -2^31 < -2^31+1
    chk(12'b1 << SLTU, 32'hffff_ffff, 32'h0000_0001);   // unsigned: no
    chk(12'b1 << SLTU, 32'h0000_0001, 32'hffff_ffff);   // unsigned: yes
    chk(12'b1 << SLTU, 32'h8000_0000, 32'h7fff_ffff);   // unsigned: no
    chk(12'b1 << SLTU, 32'h7fff_ffff, 32'h8000_0000);   // unsigned: yes
    chk(12'b1 << SLTU, 32'hffff_ffff, 32'hffff_ffff);   // equal: no

    // sra negative / srl with the sign bit set
    chk(12'b1 << SRA,  32'h8000_0000, 32'd1);
    chk(12'b1 << SRA,  32'hffff_ffff, 32'd31);
    chk(12'b1 << SRA,  32'h8000_0001, 32'd4);
    chk(12'b1 << SRL,  32'h8000_0000, 32'd1);
    chk(12'b1 << SRL,  32'hffff_ffff, 32'd31);
    chk(12'b1 << SLL,  32'h0000_0001, 32'd31);
    chk(12'b1 << SLL,  32'hffff_ffff, 32'hffff_ff20);  // shift 0
    chk(12'b1 << SRL,  32'hffff_ffff, 32'h0000_0020);  // shift 32 -> 0
    chk(12'b1 << SRA,  32'hffff_ffff, 32'h0000_0040);  // shift 64 -> b[4:0]=0
    chk(12'b1 << SLL,  32'h1234_5678, 32'hdead_beef);  // b[4:0]=15

    //------------------------------------------------------------------ random
    for (k = 0; k < 20000; k = k + 1) begin
        i = ({$random} % 12);
        if (i < 0) i = i + 12;
        chk(12'b1 << i, {$random}, {$random});
    end

    $display("==========================================");
    if (errors == 0) $display("PASS: %0d checks, 0 errors", checks);
    else             $display("FAIL: %0d checks, %0d errors", checks, errors);
    $display("==========================================");
    $finish;
end

endmodule
