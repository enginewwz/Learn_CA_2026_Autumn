// Branch unit for EX stage 0: decide taken and build the target.
//
// The branch conditions do NOT re-implement a comparator.  The ALU already
// evaluates its borrow prefix tree on the very same operand pair
// (alu_src1 / alu_src2 == br src1 / src2), so blt/bge/bltu/bgeu read the two
// compare taps the ALU exports (lt_i = signed <, ltu_i = unsigned <) and only
// invert them for the "or equal" forms.  Equality has no such tap, so beq/bne
// still fold src1 ^ src2 here (one XOR per bit + a reduction OR: 3 LUT levels
// for 32 bits, and it starts from the registered operands).
//
// jirl has no comparison at all: br_op[JIRL] alone decides taken, and the
// target is base + offs for every branch (base = pc for a conditional branch,
// the forwarded rj for jirl).
module br(
    input  wire [31:0] src1,     // compare operands (same pair the ALU uses)
    input  wire [31:0] src2,
    input  wire        lt_i,     // signed   src1 < src2   (ALU borrow tree)
    input  wire        ltu_i,    // unsigned src1 < src2   (ALU borrow tree)
    input  wire [31:0] base,     // target base: pc (conditional) / rj (jirl)
    input  wire [31:0] offs,     // i16 << 2
    input  wire [ 6:0] br_op,    // one-hot branch select
    input  wire        valid,    // this stage really holds a branch
    output wire        taken,
    output wire [31:0] target
);

// br_op bit positions: one-hot branch select
localparam BEQ  = 0;
localparam BNE  = 1;
localparam BLT  = 2;
localparam BGE  = 3;
localparam BLTU = 4;
localparam BGEU = 5;
localparam JIRL = 6;

wire eq;

assign eq = ~| (src1 ^ src2);

// bge = ~lt / bgeu = ~ltu are the ALU taps inverted here, not a second compare
assign taken  = valid & ( (br_op[BEQ]  &   eq   ) | (br_op[BNE]  & ~ eq   )
                        | (br_op[BLT]  &   lt_i ) | (br_op[BGE]  & ~ lt_i )
                        | (br_op[BLTU] &   ltu_i) | (br_op[BGEU] & ~ ltu_i)
                        |  br_op[JIRL] );

assign target = base + offs;

endmodule
