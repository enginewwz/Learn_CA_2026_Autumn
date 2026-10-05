// Branch unit for EX stage 0: decide taken and build the target.
//
// A conditional branch compares the operands that ID already registered for the
// ALU (alu_src1 / alu_src2), and jirl is always taken.  The target is
// base + offs, where ID picks the base: pc for a conditional branch, the
// (forwarded) rj for jirl.  Both inputs are registers, so no part of the
// ID forwarding chain reaches this adder combinationally.
module br(
    input  wire [31:0] src1,     // compare operands
    input  wire [31:0] src2,
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
wire lt;
wire ltu;

assign eq  = (src1 == src2);
assign lt  = ($signed(src1) < $signed(src2));
assign ltu = (src1 < src2);

assign taken  = valid & ( (br_op[BEQ]  &   eq ) | (br_op[BNE]  & ~ eq )
                        | (br_op[BLT]  &   lt ) | (br_op[BGE]  & ~ lt )
                        | (br_op[BLTU] &   ltu) | (br_op[BGEU] & ~ ltu)
                        |  br_op[JIRL] );

assign target = base + offs;

endmodule
