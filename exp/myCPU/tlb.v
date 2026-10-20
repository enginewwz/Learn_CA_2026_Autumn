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

wire [TLBNUM-1:0] s0_hit, s1_hit;
wire [TLBNUM-1:0] inv_asid_match, inv_va_match, inv_clear;

reg                       s0_ps_sel, s1_ps_sel;
reg  [$clog2(TLBNUM)-1:0] s0_index_sel, s1_index_sel;
wire                      s0_dual_page_sel, s1_dual_page_sel;

wire inv_all, inv_g1, inv_g0, inv_asid, inv_va, inv_gva;
wire w_ps4mb;

genvar i;
integer k;

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

always @(*) begin
    s0_ps_sel  = 1'b0;
    s0_index_sel = 0;
    for (k = TLBNUM-1; k >= 0; k = k - 1)
        if (s0_hit[k]) begin
            s0_ps_sel  = tlb_ps4MB[k];
            s0_index_sel = k [$clog2(TLBNUM)-1:0];
        end
end

always @(*) begin
    s1_ps_sel  = 1'b0;
    s1_index_sel = 0;
    for (k = TLBNUM-1; k >= 0; k = k - 1)
        if (s1_hit[k]) begin
            s1_ps_sel  = tlb_ps4MB[k];
            s1_index_sel = k [$clog2(TLBNUM)-1:0];
        end
end

// va[21] for a 4MB entry is already inside vppn (bit 8); va[12] comes in on its own port
assign s0_dual_page_sel = s0_ps_sel ? s0_vppn[8] : s0_va_bit12;
assign s1_dual_page_sel = s1_ps_sel ? s1_vppn[8] : s1_va_bit12;

assign s0_index = s0_index_sel;
assign s1_index = s1_index_sel;

assign s0_ps  = s0_ps_sel ? 6'd21 : 6'd12;
assign s1_ps  = s1_ps_sel ? 6'd21 : 6'd12;

assign s0_ppn = s0_dual_page_sel ? tlb_ppn1[s0_index_sel] : tlb_ppn0[s0_index_sel];
assign s1_ppn = s1_dual_page_sel ? tlb_ppn1[s1_index_sel] : tlb_ppn0[s1_index_sel];

assign s0_plv = s0_dual_page_sel ? tlb_plv1[s0_index_sel] : tlb_plv0[s0_index_sel];
assign s1_plv = s1_dual_page_sel ? tlb_plv1[s1_index_sel] : tlb_plv0[s1_index_sel];

assign s0_mat = s0_dual_page_sel ? tlb_mat1[s0_index_sel] : tlb_mat0[s0_index_sel];
assign s1_mat = s1_dual_page_sel ? tlb_mat1[s1_index_sel] : tlb_mat0[s1_index_sel];

assign s0_d   = s0_dual_page_sel ? tlb_d1[s0_index_sel] : tlb_d0[s0_index_sel];
assign s1_d   = s1_dual_page_sel ? tlb_d1[s1_index_sel] : tlb_d0[s1_index_sel];

assign s0_v   = s0_dual_page_sel ? tlb_v1[s0_index_sel] : tlb_v0[s0_index_sel];
assign s1_v   = s1_dual_page_sel ? tlb_v1[s1_index_sel] : tlb_v0[s1_index_sel];

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
