// 2-cycle 32x32 multiplier: radix-4 modified Booth + Wallace CSA tree
//
//   src1 / src2 / md_op   (the caller's stage-0 operands)
//        -> s0: Booth encode (17 digits) + partial products + the correction
//               constant + the WHOLE Wallace tree L1..L6 (18 -> 2 vectors),
//               registered as (ws, wc)
//        -> combinational tail: 64-bit CPA + high/low select
//
// The tree is nothing but wide XOR/OR LUT chains; the only deep structure left
// is the final carry chain.  Keeping that CPA combinational after the (ws, wc)
// register is what lets the two halves live in two pipeline stages: the tree is
// computed in the stage-0 cycle (now ID, on the operands ID has resolved) and
// the CPA in the stage-1 cycle (MEM), where the result mux also picks the
// product.
//
// allow: the single internal register stage is frozen while allow is low.
// There is no valid here - the caller tracks positions (mul_pos).
//
// The result is combinational: it is ready while the caller's stage-1 registers
// hold the instruction, and the top's MEM result mux selects it.  mul is
// therefore never forwarded from MEM; consumers see it through WB / the RF
// write-first bypass.
//
// md_op is the one-hot mul/div select shared with the top:
//   MUL_W    : low  32 bits of the product         (mul.w)
//   MULH_W   : high 32 bits of the signed product   (mulh.w)
//   MULH_WU  : high 32 bits of the unsigned product (mulh.wu)
// signed/unsigned is unified by a single extension bit, so one Booth array
// serves all three:
//   signed_op = md_op[MULH_W]
//   Aext = signed_op ? sign_extend(a) : zero_extend(a)   (64-bit)
//   B33  = {signed_op & b[31], b}                        (33-bit signed multiplier)
// A 33-bit signed multiplier needs 17 radix-4 digits, so the unsigned MSB
// contributes a 17th digit and no separate correction path is needed.
//
// Modified Booth digit (group of 3 overlapping bits p2 p1 p0):
//   000/111 -> 0 | 001/010 -> +1 | 011 -> +2 | 100 -> -2 | 101/110 -> -1
// A negative digit inverts the operand FIRST and then shifts it, so its
// two's-complement +1 rides up with the operand and lands on that digit's own
// weight column (2i).  The +1s of the 17 digits therefore occupy 17 distinct
// columns and never collide: they are ORed into one 64-bit constant, with no
// adder and no carry chain.  (Inverting after the shift instead pins every +1
// to bit 0, which then has to be summed - that version needed a 5-bit adder
// tree here.)
//
// The final 64-bit CPA (ws + wc) is a single bare `+`, carry chain left to the
// synthesizer.  Measured faster than every hand-written variant tried here -
// see docs/ex_opt_attempt/mul_carry_chain.md.
module mul(
    input  wire        clk,
    input  wire        reset,
    input  wire        allow,
    input  wire [31:0] src1,
    input  wire [31:0] src2,
    input  wire [ 6:0] md_op,
    output wire [31:0] result
);

// md_op bit positions, same encoding as the top
localparam MUL_W   = 0;
localparam MULH_W  = 1;
localparam MULH_WU = 2;

// the single internal register stage: the tree collapsed to two vectors
reg [63:0] ws, wc;
reg [ 6:0] md1;               // mul/div select, delayed with the tree
reg [63:0] nneg;

// ---------------------------------------------------------------- output

// Booth encoding
wire        signed_op;
wire [63:0] Aext;
wire        sb;
wire [34:0] bx;
wire [63:0] pp  [0:16];
wire [16:0] negc;

// Wallace levels
wire [63:0] L1 [0:11];
wire [63:0] L2 [0:7];
wire [63:0] L3 [0:5];
wire [63:0] L4 [0:3];
wire [63:0] L5 [0:2];
wire [63:0] wsum, wcarry;

genvar  g;
integer k;

// operand extension: the only bit that distinguishes signed from unsigned
assign signed_op = md_op[MULH_W];
assign Aext      = signed_op ? {{32{src1[31]}}, src1[31:0]} : {32'b0, src1[31:0]};
assign sb        = signed_op & src2[31];

// bx[0] = x[-1] = 0, bx[1..32] = b[0..31], bx[33] = x[32] = sb, bx[34] = x[33] = sb
assign bx = {sb, sb, src2, 1'b0};

// Booth digits + partial products
generate
    for (g = 0; g < 17; g = g + 1) begin : g_booth
        wire        p0, p1, p2, dbl, sgl, nz, neg;
        wire [63:0] mag;

        assign p0  = bx[2*g];
        assign p1  = bx[2*g+1];
        assign p2  = bx[2*g+2];
        assign dbl = (p1 & p0 & ~p2) | (~p1 & ~p0 & p2);
        assign sgl = (p1 ^ p0) & ~dbl;
        assign nz  = dbl | sgl;
        assign neg = p2;
        assign mag = dbl ? {Aext[62:0], 1'b0} : Aext;   // 1x or 2x
        // invert first, then shift: the +1 rides up to column 2*g
        assign pp[g]  = nz ? (neg ? ((~mag) << (2*g)) : (mag << (2*g))) : 64'b0;
        assign negc[g] = neg & nz;
    end
endgenerate

always @(*) begin
    nneg = 64'b0;
    for (k = 0; k < 17; k = k + 1) nneg[2*k] = negc[k];
end

// Wallace levels L1..L6: 18 -> 12 -> 8 -> 6 -> 4 -> 3 -> 2 (all combinational)
generate
    for (g = 0; g < 5; g = g + 1) begin : g_l1
        csa64 u(.a(pp[3*g]), .b(pp[3*g+1]), .c(pp[3*g+2]),
                .s(L1[2*g]), .co(L1[2*g+1]));
    end
endgenerate
csa64 u_l1f(.a(pp[15]), .b(pp[16]), .c(nneg),
            .s(L1[10]), .co(L1[11]));

generate
    for (g = 0; g < 4; g = g + 1) begin : g_l2
        csa64 u(.a(L1[3*g]), .b(L1[3*g+1]), .c(L1[3*g+2]),
                .s(L2[2*g]), .co(L2[2*g+1]));
    end
endgenerate

// L3..L6, all in the same cycle as L1/L2
generate
    for (g = 0; g < 2; g = g + 1) begin : g_l3
        csa64 u(.a(L2[3*g]), .b(L2[3*g+1]), .c(L2[3*g+2]),
                .s(L3[2*g]), .co(L3[2*g+1]));
    end
endgenerate
assign L3[4] = L2[6];
assign L3[5] = L2[7];

generate
    for (g = 0; g < 2; g = g + 1) begin : g_l4
        csa64 u(.a(L3[3*g]), .b(L3[3*g+1]), .c(L3[3*g+2]),
                .s(L4[2*g]), .co(L4[2*g+1]));
    end
endgenerate

csa64 u_l5(.a(L4[0]), .b(L4[1]), .c(L4[2]), .s(L5[0]), .co(L5[1]));
assign L5[2] = L4[3];

csa64 u_l6(.a(L5[0]), .b(L5[1]), .c(L5[2]), .s(wsum), .co(wcarry));

// the only register stage: the two vectors the tree collapsed to
always @(posedge clk) begin
    if (reset) begin
        ws  <= 64'b0;
        wc  <= 64'b0;
        md1 <= 7'b0;
    end
    else if (allow) begin
        ws  <= wsum;
        wc  <= wcarry;
        md1 <= md_op;
    end
end

//--------------------------------------------------------------- final CPA
(* keep = "true", dont_touch = "true" *) wire [63:0] prod;
wire        hi_sel;
wire [31:0] sel_w;

assign prod = ws + wc;

// high/low select: mul.w wins, then mulh.w / mulh.wu, else low
assign hi_sel = ~md1[MUL_W] & (md1[MULH_W] | md1[MULH_WU]);

assign sel_w = hi_sel ? prod[63:32] : prod[31:0];

// combinational output (the caller / MEM registers it)
assign result = sel_w;

endmodule
