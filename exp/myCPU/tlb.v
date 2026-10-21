module tlb
#(
    parameter TLBNUM = 16
)
(
    input  wire                      clk,
    input  wire                      reset,

    // search port 0 (for fetch)
    input  wire [              18:0] s0_vppn,
    input  wire                      s0_va_bit12,
    input  wire [               9:0] s0_asid,
    output wire                      s0_found,
    output wire [$clog2(TLBNUM)-1:0] s0_index,
    output wire [              19:0] s0_ppn,
    output wire [               5:0] s0_ps,
    output wire [               1:0] s0_plv,
    output wire [               1:0] s0_mat,
    output wire                      s0_d,
    output wire                      s0_v,

    // search port 1 (for load/store)
    input  wire [              18:0] s1_vppn,
    input  wire                      s1_va_bit12,
    input  wire [               9:0] s1_asid,
    output wire                      s1_found,
    output wire [$clog2(TLBNUM)-1:0] s1_index,
    output wire [              19:0] s1_ppn,
    output wire [               5:0] s1_ps,
    output wire [               1:0] s1_plv,
    output wire [               1:0] s1_mat,
    output wire                      s1_d,
    output wire                      s1_v,

    // search port 2 (TLBSRCH: TLBEHI + ASID, index only)
    input  wire [              18:0] s2_vppn,
    input  wire [               9:0] s2_asid,
    output wire                      s2_found,
    output wire [$clog2(TLBNUM)-1:0] s2_index,

    // invtlb opcode (keys come in on the write port: w_asid, w_vppn)
    input  wire                      invtlb_valid,
    input  wire [               4:0] invtlb_op,

    // write port
    input  wire                      we,     //w(rite) e(nable)
    input  wire [$clog2(TLBNUM)-1:0] w_index,
    input  wire                      w_e,
    input  wire [              18:0] w_vppn,       //TLBEHI[31:13], invtlb: rk[31:13]
    input  wire [               5:0] w_ps,
    input  wire [               9:0] w_asid,       //ASID, invtlb: rj[9:0]
    input  wire                      w_g,
    input  wire [              19:0] w_ppn0,
    input  wire [               1:0] w_plv0,
    input  wire [               1:0] w_mat0,
    input  wire                      w_d0,
    input  wire                      w_v0,
    input  wire [              19:0] w_ppn1,
    input  wire [               1:0] w_plv1,
    input  wire [               1:0] w_mat1,
    input  wire                      w_d1,
    input  wire                      w_v1,

    // read port
    input  wire [$clog2(TLBNUM)-1:0] r_index,
    output wire                      r_e,
    output wire [              18:0] r_vppn,
    output wire [               5:0] r_ps,
    output wire [               9:0] r_asid,
    output wire                      r_g,
    output wire [              19:0] r_ppn0,
    output wire [               1:0] r_plv0,
    output wire [               1:0] r_mat0,
    output wire                      r_d0,
    output wire                      r_v0,
    output wire [              19:0] r_ppn1,
    output wire [               1:0] r_plv1,
    output wire [               1:0] r_mat1,
    output wire                      r_d1,
    output wire                      r_v1
);

reg  [TLBNUM-1:0] tlb_e;
reg  [TLBNUM-1:0] tlb_ps4MB;    //pagesize 1:4MB, 0:4KB
reg  [      18:0] tlb_vppn     [TLBNUM-1:0];
reg  [       9:0] tlb_asid     [TLBNUM-1:0];
reg               tlb_g        [TLBNUM-1:0];
reg  [      19:0] tlb_ppn0     [TLBNUM-1:0];
reg  [       1:0] tlb_plv0     [TLBNUM-1:0];
reg  [       1:0] tlb_mat0     [TLBNUM-1:0];
reg               tlb_d0       [TLBNUM-1:0];
reg               tlb_v0       [TLBNUM-1:0];
reg  [      19:0] tlb_ppn1     [TLBNUM-1:0];
reg  [       1:0] tlb_plv1     [TLBNUM-1:0];
reg  [       1:0] tlb_mat1     [TLBNUM-1:0];
reg               tlb_d1       [TLBNUM-1:0];
reg               tlb_v1       [TLBNUM-1:0];

wire [TLBNUM-1:0] s0_hit, s1_hit, s2_hit;
wire [TLBNUM-1:0] inv_asid_match, inv_va_match, inv_clear;

// Search: a 4 level 2:1 mux tournament (16 -> 8 -> 4 -> 2 -> 1) instead of a 16
// deep priority chain.  Inside a pair the lower entry wins, so the winner is
// still the first matching entry.  s0/s1 carry the winner's flags (index, ps4MB,
// plv, mat, d, v) so the exception check does not sit behind a 16:1 mux, while
// the ppn keeps the plain [index] read; s2 only needs the index.
localparam SW = 4 + 1 + 2 + 2 + 1 + 1;           // index, ps4MB, plv, mat, d, v
wire [SW-1:0] sw0 [0:TLBNUM-1];
wire [SW-1:0] sw1 [0:TLBNUM-1];

wire [ 7:0] s0_h1, s1_h1, s2_h1;
wire [ 3:0] s0_h2, s1_h2, s2_h2;
wire [ 1:0] s0_h3, s1_h3, s2_h3;
wire [SW-1:0] s0_d1 [0:7], s1_d1 [0:7];
wire [SW-1:0] s0_d2 [0:3], s1_d2 [0:3];
wire [SW-1:0] s0_d3 [0:1], s1_d3 [0:1];
wire [SW-1:0] s0_d4,       s1_d4;
wire [ 3:0] s2_ix1 [0:7], s2_ix2 [0:3], s2_ix3 [0:1], s2_ix4;

wire inv_all, inv_g1, inv_g0, inv_asid, inv_va, inv_gva;
wire w_ps4mb;

genvar i;
genvar j;

assign inv_all  = (invtlb_op == 5'd0) | (invtlb_op == 5'd1);
assign inv_g1   = (invtlb_op == 5'd2);
assign inv_g0   = (invtlb_op == 5'd3);
assign inv_asid = (invtlb_op == 5'd4);
assign inv_va   = (invtlb_op == 5'd5);
assign inv_gva  = (invtlb_op == 5'd6);

generate
    for (i = 0; i < TLBNUM; i = i + 1) begin : search_hit
        assign s0_hit [i] = tlb_e[i]    & (tlb_g[i] | tlb_asid[i] == s0_asid)
                                        & (tlb_vppn[i][18:9] == s0_vppn[18:9])
                                        & (tlb_ps4MB[i] | (tlb_vppn[i][8:0] == s0_vppn[8:0]));
        assign s1_hit [i] = tlb_e[i]    & (tlb_g[i] | tlb_asid[i] == s1_asid)
                                        & (tlb_vppn[i][18:9] == s1_vppn[18:9])
                                        & (tlb_ps4MB[i] | (tlb_vppn[i][8:0] == s1_vppn[8:0]));

        assign s2_hit [i] = tlb_e[i]    & (tlb_g[i] | tlb_asid[i] == s2_asid)
                                        & (tlb_vppn[i][18:9] == s2_vppn[18:9])
                                        & (tlb_ps4MB[i] | (tlb_vppn[i][8:0] == s2_vppn[8:0]));

        // page pick: ps4MB comes from the entry, the page bit from the searched
        // VA, so this mux runs in parallel with the compare
        localparam [3:0] IDX = i;
        wire s0_pg1 = tlb_ps4MB[i] ? s0_vppn[8] : s0_va_bit12;
        wire s1_pg1 = tlb_ps4MB[i] ? s1_vppn[8] : s1_va_bit12;

        assign sw0[i] = {IDX, tlb_ps4MB[i],
                         s0_pg1 ? tlb_plv1[i] : tlb_plv0[i],
                         s0_pg1 ? tlb_mat1[i] : tlb_mat0[i],
                         s0_pg1 ? tlb_d1[i]   : tlb_d0[i],
                         s0_pg1 ? tlb_v1[i]   : tlb_v0[i]};
        assign sw1[i] = {IDX, tlb_ps4MB[i],
                         s1_pg1 ? tlb_plv1[i] : tlb_plv0[i],
                         s1_pg1 ? tlb_mat1[i] : tlb_mat0[i],
                         s1_pg1 ? tlb_d1[i]   : tlb_d0[i],
                         s1_pg1 ? tlb_v1[i]   : tlb_v0[i]};

        assign inv_asid_match[i] = (tlb_asid[i] == w_asid);
        assign inv_va_match[i]   = (tlb_vppn[i][18:9] == w_vppn[18:9])
                                 & (tlb_ps4MB[i] | (tlb_vppn[i][8:0] == w_vppn[8:0]));

        // op 6 is "(G or ASID matches) and VA matches"
        assign inv_clear[i] = invtlb_valid &
                              ( inv_all
                              | inv_g1   &  tlb_g[i]
                              | inv_g0   & ~tlb_g[i]
                              | inv_asid & ~tlb_g[i] &  inv_asid_match[i]
                              | inv_va   & ~tlb_g[i] &  inv_asid_match[i] & inv_va_match[i]
                              | inv_gva  & (~tlb_g[i] | inv_asid_match[i]) & inv_va_match[i]);
    end
endgenerate

assign s0_found = |s0_hit;
assign s1_found = |s1_hit;
assign s2_found = |s2_hit;

generate
    for (j = 0; j < 8; j = j + 1) begin : ix1
        assign s0_h1[j]  = s0_hit[2*j] | s0_hit[2*j+1];
        assign s1_h1[j]  = s1_hit[2*j] | s1_hit[2*j+1];
        assign s2_h1[j]  = s2_hit[2*j] | s2_hit[2*j+1];
        assign s0_d1[j]  = s0_hit[2*j] ? sw0[2*j] : sw0[2*j+1];
        assign s1_d1[j]  = s1_hit[2*j] ? sw1[2*j] : sw1[2*j+1];
        assign s2_ix1[j] = s2_hit[2*j] ? 2 * j : 2 * j + 1;
    end
    for (j = 0; j < 4; j = j + 1) begin : ix2
        assign s0_h2[j]  = s0_h1[2*j] | s0_h1[2*j+1];
        assign s1_h2[j]  = s1_h1[2*j] | s1_h1[2*j+1];
        assign s2_h2[j]  = s2_h1[2*j] | s2_h1[2*j+1];
        assign s0_d2[j]  = s0_h1[2*j] ? s0_d1[2*j] : s0_d1[2*j+1];
        assign s1_d2[j]  = s1_h1[2*j] ? s1_d1[2*j] : s1_d1[2*j+1];
        assign s2_ix2[j] = s2_h1[2*j] ? s2_ix1[2*j] : s2_ix1[2*j+1];
    end
    for (j = 0; j < 2; j = j + 1) begin : ix3
        assign s0_h3[j]  = s0_h2[2*j] | s0_h2[2*j+1];
        assign s1_h3[j]  = s1_h2[2*j] | s1_h2[2*j+1];
        assign s2_h3[j]  = s2_h2[2*j] | s2_h2[2*j+1];
        assign s0_d3[j]  = s0_h2[2*j] ? s0_d2[2*j] : s0_d2[2*j+1];
        assign s1_d3[j]  = s1_h2[2*j] ? s1_d2[2*j] : s1_d2[2*j+1];
        assign s2_ix3[j] = s2_h2[2*j] ? s2_ix2[2*j] : s2_ix2[2*j+1];
    end
endgenerate

assign s0_d4 = s0_h3[0] ? s0_d3[0] : s0_d3[1];
assign s1_d4 = s1_h3[0] ? s1_d3[0] : s1_d3[1];
assign s2_ix4 = s2_h3[0] ? s2_ix3[0] : s2_ix3[1];

wire [ 3:0] s0_idx, s1_idx;
wire        s0_ps4, s1_ps4;
wire [ 1:0] s0_lvw, s1_lvw, s0_mtw, s1_mtw;
wire        s0_dtw, s1_dtw, s0_vdw, s1_vdw;
wire        s0_dual_page_sel, s1_dual_page_sel;

assign {s0_idx, s0_ps4, s0_lvw, s0_mtw, s0_dtw, s0_vdw} = s0_d4;
assign {s1_idx, s1_ps4, s1_lvw, s1_mtw, s1_dtw, s1_vdw} = s1_d4;

// found comes straight off the hit vector: a 16 bit OR is two LUT levels and it
// gates the address mux, so it should not wait for the tree root
assign s0_found = | s0_hit;
assign s1_found = | s1_hit;
assign s2_found = | s2_hit;

assign s0_index = s0_idx;
assign s1_index = s1_idx;
assign s2_index = s2_ix4;

// va[21] for a 4MB entry is already inside vppn (bit 8); va[12] comes in on its own port
assign s0_dual_page_sel = s0_ps4 ? s0_vppn[8] : s0_va_bit12;
assign s1_dual_page_sel = s1_ps4 ? s1_vppn[8] : s1_va_bit12;

assign s0_ppn = s0_dual_page_sel ? tlb_ppn1[s0_index] : tlb_ppn0[s0_index];
assign s1_ppn = s1_dual_page_sel ? tlb_ppn1[s1_index] : tlb_ppn0[s1_index];
assign s0_plv = s0_lvw;
assign s1_plv = s1_lvw;
assign s0_mat = s0_mtw;
assign s1_mat = s1_mtw;
assign s0_d   = s0_dtw;
assign s1_d   = s1_dtw;
assign s0_v   = s0_vdw;
assign s1_v   = s1_vdw;

assign s0_ps  = s0_ps4 ? 6'd21 : 6'd12;
assign s1_ps  = s1_ps4 ? 6'd21 : 6'd12;

assign w_ps4mb = (w_ps == 6'd21);

always @(posedge clk) begin
    if (reset) begin
        tlb_e <= {(TLBNUM){1'b0}};
    end
    else begin
        if (invtlb_valid)
            tlb_e <= tlb_e & ~inv_clear;

        if (we) begin
            tlb_e    [w_index] <= w_e;
            tlb_ps4MB[w_index] <= w_ps4mb;
            tlb_vppn [w_index] <= w_vppn;
            tlb_asid [w_index] <= w_asid;
            tlb_g    [w_index] <= w_g;
            tlb_ppn0 [w_index] <= w_ppn0;
            tlb_plv0 [w_index] <= w_plv0;
            tlb_mat0 [w_index] <= w_mat0;
            tlb_d0   [w_index] <= w_d0;
            tlb_v0   [w_index] <= w_v0;
            tlb_ppn1 [w_index] <= w_ppn1;
            tlb_plv1 [w_index] <= w_plv1;
            tlb_mat1 [w_index] <= w_mat1;
            tlb_d1   [w_index] <= w_d1;
            tlb_v1   [w_index] <= w_v1;
        end
    end
end

// an E=0 entry still holds its old fields, TLBRD zeroes what it writes back
assign r_e    = tlb_e    [r_index];
assign r_vppn = tlb_vppn [r_index];
assign r_ps   = tlb_ps4MB[r_index] ? 6'd21 : 6'd12;
assign r_asid = tlb_asid [r_index];
assign r_g    = tlb_g    [r_index];
assign r_ppn0 = tlb_ppn0 [r_index];
assign r_plv0 = tlb_plv0 [r_index];
assign r_mat0 = tlb_mat0 [r_index];
assign r_d0   = tlb_d0   [r_index];
assign r_v0   = tlb_v0   [r_index];
assign r_ppn1 = tlb_ppn1 [r_index];
assign r_plv1 = tlb_plv1 [r_index];
assign r_mat1 = tlb_mat1 [r_index];
assign r_d1   = tlb_d1   [r_index];
assign r_v1   = tlb_v1   [r_index];

endmodule
