// 3-stage pipelined 32x32 multiplier: radix-4 modified Booth + Wallace CSA tree
//
//   src1 / src2 / md_op   (already the caller's stage-0 registered operands)
//        -> s0: Booth encode (17 digits) + partial products + Wallace L1..L2
//        -> s1: Wallace L3..L6                              (8 -> 2 vectors)
//        -> s2: 64-bit carry-select CPA + high/low select, registered
//
// allow[2:0]: stage k is frozen while allow[k] is low; one bit per internal
// stage.  There is no valid here - the caller tracks positions (mul_pos).
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
// A negative digit is formed as ~(mag << 2i) plus a +1; the +1s of all negative
// digits are collected into one small constant added to the tree.
//
// The final 64-bit CPA (ws2 + wc2) is a four-segment carry-select instead of one
// ripple: segment 0 is a true add whose carry out picks the segment-1 copy, and
// segments 1..3 are computed twice (carry-in 0 / carry-in 1) so no carry chain
// is longer than 16 bits.  The segment carries are merged with a flat lookahead
// (2 LUT levels) rather than a serial select chain, so the whole 64-bit add is
// ~4 CARRY4 + 2 LUT levels instead of 16 CARRY4.  The duplicated adds carry
// (* keep *)/(* dont_touch *): a full-data carry-select is exactly the shape an
// area-first synthesizer folds back into a ripple (docs/exp13_div.md section 10).
module mul(
    input  wire        clk,
    input  wire        reset,
    input  wire [ 2:0] allow,
    input  wire [31:0] src1,
    input  wire [31:0] src2,
    input  wire [ 6:0] md_op,
    output wire [31:0] result
);

// md_op bit positions, same encoding as the top
localparam MUL_W   = 0;
localparam MULH_W  = 1;
localparam MULH_WU = 2;

// ---------------------------------------------------------------- stage 0
reg [63:0] L2r [0:7];         // 18 vectors reduced to 8
reg [ 6:0] md1;
reg [ 4:0] nneg;

// ---------------------------------------------------------------- stage 1
reg [63:0] ws2, wc2;          // reduced to (sum, carry)
reg [ 6:0] md2;

// ---------------------------------------------------------------- stage 2
reg [31:0] result_r;          // registered carry-select result
reg [31:0] sel;               // combinational high/low select

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
        wire p0 = bx[2*g];
        wire p1 = bx[2*g+1];
        wire p2 = bx[2*g+2];
        wire dbl = (p1 & p0 & ~p2) | (~p1 & ~p0 & p2);
        wire sgl = (p1 ^ p0) & ~dbl;
        wire nz  = dbl | sgl;
        wire neg = p2;
        wire [63:0] mag = dbl ? {Aext[62:0], 1'b0} : Aext;   // 1x or 2x
        assign pp[g]  = nz ? ((mag << (2*g)) ^ {64{neg}}) : 64'b0;
        assign negc[g] = neg & nz;
    end
endgenerate

// +1 per negative digit completes its two's complement
always @(*) begin
    nneg = 5'd0;
    for (k = 0; k < 17; k = k + 1)
        nneg = nneg + {4'b0, negc[k]};
end

// Wallace levels L1..L2: 18 -> 12 -> 8   (stage 0)
generate
    for (g = 0; g < 5; g = g + 1) begin : g_l1
        csa64 u(.a(pp[3*g]), .b(pp[3*g+1]), .c(pp[3*g+2]),
                .s(L1[2*g]), .co(L1[2*g+1]));
    end
endgenerate
csa64 u_l1f(.a(pp[15]), .b(pp[16]), .c({59'b0, nneg}),
            .s(L1[10]), .co(L1[11]));

generate
    for (g = 0; g < 4; g = g + 1) begin : g_l2
        csa64 u(.a(L1[3*g]), .b(L1[3*g+1]), .c(L1[3*g+2]),
                .s(L2[2*g]), .co(L2[2*g+1]));
    end
endgenerate

always @(posedge clk) begin
    if (reset) begin
        L2r[0] <= 64'b0;
        L2r[1] <= 64'b0;
        L2r[2] <= 64'b0;
        L2r[3] <= 64'b0;
        L2r[4] <= 64'b0;
        L2r[5] <= 64'b0;
        L2r[6] <= 64'b0;
        L2r[7] <= 64'b0;
        md1    <= 7'b0;
    end
    else if (allow[0]) begin
        L2r[0] <= L2[0];
        L2r[1] <= L2[1];
        L2r[2] <= L2[2];
        L2r[3] <= L2[3];
        L2r[4] <= L2[4];
        L2r[5] <= L2[5];
        L2r[6] <= L2[6];
        L2r[7] <= L2[7];
        md1    <= md_op;
    end
end

// Wallace levels L3..L6: 8 -> 6 -> 4 -> 3 -> 2   (stage 1)
generate
    for (g = 0; g < 2; g = g + 1) begin : g_l3
        csa64 u(.a(L2r[3*g]), .b(L2r[3*g+1]), .c(L2r[3*g+2]),
                .s(L3[2*g]), .co(L3[2*g+1]));
    end
endgenerate
assign L3[4] = L2r[6];
assign L3[5] = L2r[7];

generate
    for (g = 0; g < 2; g = g + 1) begin : g_l4
        csa64 u(.a(L3[3*g]), .b(L3[3*g+1]), .c(L3[3*g+2]),
                .s(L4[2*g]), .co(L4[2*g+1]));
    end
endgenerate

csa64 u_l5(.a(L4[0]), .b(L4[1]), .c(L4[2]), .s(L5[0]), .co(L5[1]));
assign L5[2] = L4[3];

csa64 u_l6(.a(L5[0]), .b(L5[1]), .c(L5[2]), .s(wsum), .co(wcarry));

always @(posedge clk) begin
    if (reset) begin
        ws2 <= 64'b0;
        wc2 <= 64'b0;
        md2 <= 7'b0;
    end
    else if (allow[1]) begin
        ws2 <= wsum;
        wc2 <= wcarry;
        md2 <= md1;
    end
end

//--------------------------------------------------------------- stage 2 CPA
// Four 16-bit segments.  cs0 is the true low add; cs1/cs2/cs3 exist in a
// carry-in-0 and a carry-in-1 flavour and the right one is selected by the
// carry out of the segment below.  For a segment, carry-out(cin=0) = G and
// carry-out(cin=1) = G|P, so the two copies also hand out G = cs_0[16] and
// P = cs_1[16] & ~cs_0[16]; the carry chain then closes with a flat lookahead
// instead of a serial mux chain:
//     c1 = G0,  c2 = G1 | P1&c1,  c3 = G2 | P2&G1 | P2&P1&c1
wire [16:0] cs0;
(* keep = "true", dont_touch = "true" *) wire [16:0] cs1_0, cs1_1;
(* keep = "true", dont_touch = "true" *) wire [16:0] cs2_0, cs2_1;
// segment 3 has no carry consumer, so it stays 16 bits wide
(* keep = "true", dont_touch = "true" *) wire [15:0] cs3_0, cs3_1;

assign cs0   = {1'b0, ws2[15: 0]} + {1'b0, wc2[15: 0]};
assign cs1_0 = {1'b0, ws2[31:16]} + {1'b0, wc2[31:16]};
assign cs1_1 = {1'b0, ws2[31:16]} + {1'b0, wc2[31:16]} + 17'd1;
assign cs2_0 = {1'b0, ws2[47:32]} + {1'b0, wc2[47:32]};
assign cs2_1 = {1'b0, ws2[47:32]} + {1'b0, wc2[47:32]} + 17'd1;
assign cs3_0 = ws2[63:48] + wc2[63:48];
assign cs3_1 = ws2[63:48] + wc2[63:48] + 16'd1;

wire g1, p1, g2, p2;
wire c1, c2, c3;

assign c1 = cs0[16];
assign g1 = cs1_0[16];
assign p1 = cs1_1[16] & ~ cs1_0[16];
assign g2 = cs2_0[16];
assign p2 = cs2_1[16] & ~ cs2_0[16];
assign c2 = g1 | (p1 & c1);
assign c3 = g2 | (p2 & g1) | (p2 & p1 & c1);

wire [63:0] prod;
assign prod = { c3 ? cs3_1 : cs3_0,
                c2 ? cs2_1[15:0] : cs2_0[15:0],
                c1 ? cs1_1[15:0] : cs1_0[15:0],
                cs0[15:0] };

// high/low select, then one register (stage 2)
always @(*) begin
    case (1'b1)
        md2[MUL_W]   : sel = prod[31:0];
        md2[MULH_W]  : sel = prod[63:32];
        md2[MULH_WU] : sel = prod[63:32];
        default      : sel = prod[31:0];
    endcase
end

always @(posedge clk) begin
    if (reset)         result_r <= 32'b0;
    else if (allow[2]) result_r <= sel;
end

assign result = result_r;

endmodule
