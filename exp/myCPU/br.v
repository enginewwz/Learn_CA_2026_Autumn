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

// carry-lookahead combine of two adjacent (G, P) groups:
//   {G,P} = {Ghi | (Phi & Glo), Phi & Plo}
// The subtraction borrow tree uses g_i = ~a_i & b_i, p_i = a_i ~^ b_i,
// brw_0 = 0, so the root is (a <u b).
function [1:0] cla;
    input ghi, phi, glo, plo;
    begin
        cla[1] = ghi | (phi & glo);
        cla[0] = phi & plo;
    end
endfunction

genvar g;

wire [1:0] brw2 [0:15];    //aligned 2-bit groups
wire [1:0] brw4 [0: 7];
wire [1:0] brw8 [0: 3];
wire [1:0] brw16_1;

generate
    for (g = 0; g < 16; g = g + 1) begin : g_brw2
        assign brw2[g] = cla(~src1[2*g+1] & src2[2*g+1],
                              src1[2*g+1] ~^ src2[2*g+1],
                             ~src1[2*g  ] & src2[2*g  ],
                              src1[2*g  ] ~^ src2[2*g  ]);
    end
    for (g = 0; g < 8; g = g + 1) begin : g_brw4
        assign brw4[g] = cla(brw2[2*g+1][1], brw2[2*g+1][0],
                             brw2[2*g  ][1], brw2[2*g  ][0]);
    end
    for (g = 0; g < 4; g = g + 1) begin : g_brw8
        assign brw8[g] = cla(brw4[2*g+1][1], brw4[2*g+1][0],
                             brw4[2*g  ][1], brw4[2*g  ][0]);
    end
endgenerate

wire brw_lo16;   //G(0..15): borrow out of the low half
wire ltu32;      //G(0..31): a <u b

assign brw_lo16 = brw8[1][1] | (brw8[1][0] & brw8[0][1]);
assign brw16_1  = cla(brw8[3][1], brw8[3][0], brw8[2][1], brw8[2][0]);
assign ltu32    = brw16_1[1] | (brw16_1[0] & brw_lo16);

wire lt;
wire ltu;
wire eq;

assign ltu = ltu32;
assign lt  = (src1[31] & ~src2[31])
           | ((src1[31] ~^ src2[31]) & ltu32);
assign eq  = ~| (src1 ^ src2);

assign taken  = valid & ( (br_op[BEQ]  &   eq   ) | (br_op[BNE]  & ~ eq   )
                        | (br_op[BLT]  &   lt   ) | (br_op[BGE]  & ~ lt   )
                        | (br_op[BLTU] &   ltu  ) | (br_op[BGEU] & ~ ltu  )
                        |  br_op[JIRL] );

assign target = base + offs;

endmodule
