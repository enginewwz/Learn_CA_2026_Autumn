// Control and status register file
//
// one asynchronous read port and one clocked write port.
//
// both the read and the write happen in the same pipeline point (WB)
//
// The storage is *field* based, not word based, because the fields do not
// share a management style; the read port composes the word from the current
// register values.
//
//   field class          | management
//   ---------------------|---------------------------------------------------
//   software RW          | masked write: `CSR_FIELD(sel, cur, wdata, wmask)
//   read-only constant   | CRMD.DA/PG/DATF/DATM, fixed while the core has no
//                        | address translation
//   hardware written     | Ecode/EsubCode/ERA/PRMD/CRMD on `excp_i`, BADV on
//                        | `excp_badv_en_i`
//   set/clear            | ESTAT.IS[11] (TI): set by the timer, cleared by TICLR
//   line driven          | ESTAT.IS[9:2] = hwi_i, IS[12] = ipi_i
//   counter              | TVAL (down), `timer` (free-running 64-bit for
//                        | rdcntvl.w / rdcntvh.w)
//   write-only           | TICLR: reads 0

module csr(
    input  wire        clk,
    input  wire        reset,

    // read port
    // asynchronous read of the registered value; undefined CSRs read 0
    input  wire [13:0] raddr_i,
    output reg  [31:0] rdata_o,

    // write port
    // clocked on the posedge; `wmask_i` is all-ones for csrwr, rj for csrxchg
    input  wire        we_i,
    input  wire [13:0] waddr_i,
    input  wire [31:0] wdata_i,
    input  wire [31:0] wmask_i,

    // exception entry
    // hardware write, highest priority.  excp_badv_en_i is only set for the
    // address exceptions (ADE/ALE); syscall/break/INE leave BADV alone.
    input  wire        excp_i,
    input  wire [31:0] excp_pc_i,
    input  wire [ 5:0] excp_ecode_i,
    input  wire [ 8:0] excp_esubcode_i,
    input  wire        excp_badv_en_i,
    input  wire [31:0] excp_badv_i,
    input  wire        excp_tlb_i,      // this exception also reloads TLBEHI.VPPN

    // ertn
    input  wire        ertn_i,

    // interrupts
    input  wire [ 7:0] hwi_i,           // hardware
    input  wire        ipi_i,           // inter-processor

    // TLB instruction writes (they commit at WB, so no pass-through is needed)
    // sel: 1 = TLBSRCH, 2 = TLBRD
    input  wire        tlb_we_i,
    input  wire [ 1:0] tlb_sel_i,
    input  wire        tlb_srch_hit_i,
    input  wire [ 3:0] tlb_srch_index_i,
    input  wire        tlb_rd_e_i,
    input  wire [18:0] tlb_rd_vppn_i,
    input  wire [ 5:0] tlb_rd_ps_i,
    input  wire [ 9:0] tlb_rd_asid_i,
    input  wire [31:0] tlb_rd_lo0_i,
    input  wire [31:0] tlb_rd_lo1_i,

    // TLB / address-translation side band
    output wire [ 3:0] tlbidx_index_o,
    output wire [ 5:0] tlbidx_ps_o,
    output wire        tlbidx_ne_o,
    output wire [18:0] tlbehi_vppn_o,
    output wire [31:0] tlbelo0_o,
    output wire [31:0] tlbelo1_o,
    output wire [ 9:0] asid_o,
    output wire [31:0] tlbrentry_o,
    output wire [ 5:0] estat_ecode_o,
    output wire        crmd_da_o,
    output wire        crmd_pg_o,

    // pipeline side-band
    output wire [ 1:0] crmd_plv_o,
    output wire        crmd_ie_o,
    output wire [31:0] dmw0_o,
    output wire [31:0] dmw1_o,
    output wire [31:0] eentry_o,
    output wire [31:0] era_o,
    output wire        int_o,           // |(ECFG.LIE & ESTAT.IS) & CRMD.IE
    output wire [63:0] timer_o          // rdcntvl.w / rdcntvh.w
);

// address map
localparam [13:0] CSR_CRMD    = 14'h000;
localparam [13:0] CSR_PRMD    = 14'h001;
localparam [13:0] CSR_ECFG    = 14'h004;
localparam [13:0] CSR_ESTAT   = 14'h005;
localparam [13:0] CSR_ERA     = 14'h006;
localparam [13:0] CSR_BADV    = 14'h007;
localparam [13:0] CSR_EENTRY  = 14'h00c;
localparam [13:0] CSR_SAVE0   = 14'h030;
localparam [13:0] CSR_SAVE1   = 14'h031;
localparam [13:0] CSR_SAVE2   = 14'h032;
localparam [13:0] CSR_SAVE3   = 14'h033;
localparam [13:2] CSR_SAVE_H  = 12'b000000001100;   // head
localparam [13:0] CSR_TID     = 14'h040;
localparam [13:0] CSR_TCFG    = 14'h041;
localparam [13:0] CSR_TVAL    = 14'h042;    // read-only
localparam [13:0] CSR_TICLR   = 14'h044;    // write-only, reads 0
localparam [13:0] CSR_TLBIDX  = 14'h010;
localparam [13:0] CSR_TLBEHI  = 14'h011;
localparam [13:0] CSR_TLBELO0 = 14'h012;
localparam [13:0] CSR_TLBELO1 = 14'h013;
localparam [13:0] CSR_ASID    = 14'h018;
localparam [13:0] CSR_TLBRENTRY = 14'h088;
localparam [13:0] CSR_DMW0    = 14'h180;
localparam [13:0] CSR_DMW1    = 14'h181;
// EUEN (0x02) / CPUID (0x20) and anything else read 0 and ignore writes

// masked software write of one field: take wdata on the selected bits
`define CSR_FIELD(sel, cur, val, msk) ((sel) ? (((val) & (msk)) | ((cur) & ~(msk))) : (cur))

// definitions

// write select, one per implemented CSR
wire        wr_crmd;
wire        wr_prmd;
wire        wr_ecfg;
wire        wr_estat;
wire        wr_era;
wire        wr_badv;
wire        wr_eentry;
wire        wr_tid;
wire        wr_tcfg;
wire        wr_ticlr;
wire        wr_tlbidx;
wire        wr_tlbehi;
wire        wr_tlbelo0;
wire        wr_tlbelo1;
wire        wr_asid;
wire        wr_tlbrentry;
wire        wr_dmw0;
wire        wr_dmw1;

// CRMD
// DA/PG/DATF/DATM are real state now: the core has address translation.
reg         csr_crmd_da;
reg         csr_crmd_pg;
reg  [1:0]  csr_crmd_datf;
reg  [1:0]  csr_crmd_datm;

reg  [1:0]  crmd_plv;
reg         crmd_ie;

wire        crmd_da_sw;
wire        crmd_pg_sw;
wire [1:0]  crmd_datf_sw;
wire [1:0]  crmd_datm_sw;
wire [1:0]  crmd_plv_sw;    // software
wire        crmd_ie_sw;
reg         tlb_excp_r;     // the exception being handled is a tlb refill
wire        tlb_excp_nxt;
wire        crmd_da_nxt;
wire        crmd_pg_nxt;
wire [1:0]  crmd_datf_nxt;
wire [1:0]  crmd_datm_nxt;
wire [1:0]  crmd_plv_nxt;
wire        crmd_ie_nxt;

// PRMD
reg  [1:0]  prmd_pplv;
reg         prmd_pie;

wire [1:0]  prmd_pplv_sw;
wire        prmd_pie_sw;
wire [1:0]  prmd_pplv_nxt;
wire        prmd_pie_nxt;

// ESTAT
// IS[0]/IS[1] software writable, IS[9:2] from hwi_i, IS[11] timer, IS[12] ipi,
// IS[10] reserved.  Ecode/EsubCode come from the exception unit only.
reg         estat_is0;
reg         estat_is1;
reg         estat_ti;
reg  [ 5:0] estat_ecode;
reg  [ 8:0] estat_esubcode;

wire        estat_is0_nxt;
wire        estat_is1_nxt;
wire [ 5:0] estat_ecode_nxt;
wire [ 8:0] estat_esubcode_nxt;
wire        ti_set;
wire        ti_set_wr;
wire        ti_clr;
wire        estat_ti_nxt;
wire [12:0] estat_is;

// ERA / BADV
reg  [31:0] era;
reg  [31:0] badv;

wire [31:0] era_sw;
wire [31:0] era_nxt;
wire [31:0] badv_nxt;

// EENTRY
// VA[31:6], [5:0] read as zero
reg  [25:0] eentry_va;
wire [25:0] eentry_va_nxt;

// TLBIDX   = {NE, 1'b0, PS[5:0], 20'h0, Index[3:0]}
// TLBEHI   = {VPPN[18:0], 13'h0}
// TLBELO   = {4'h0, PPN[19:0], 1'b0, G, MAT[1:0], PLV[1:0], D, V}
// ASID     = {8'h0, 8'd10, 6'h0, ASID[9:0]}
// TLBRENTRY= {PA[31:6], 6'h0}
reg  [ 3:0] tlbidx_index;
reg  [ 5:0] tlbidx_ps;
reg         tlbidx_ne;
reg  [18:0] tlbehi_vppn;
reg  [27:0] tlbelo0;
reg  [27:0] tlbelo1;
reg  [ 9:0] asid;
reg  [25:0] tlbrentry_pa;

wire [ 3:0] tlbidx_index_sw;
wire [ 5:0] tlbidx_ps_sw;
wire        tlbidx_ne_sw;
wire [18:0] tlbehi_vppn_sw;
wire [27:0] tlbelo0_sw, tlbelo1_sw;
wire [ 9:0] asid_sw;
wire [25:0] tlbrentry_pa_sw;

wire [ 3:0] tlbidx_index_nxt;
wire [ 5:0] tlbidx_ps_nxt;
wire        tlbidx_ne_nxt;
wire [18:0] tlbehi_vppn_nxt;
wire [27:0] tlbelo0_nxt, tlbelo1_nxt;
wire [ 9:0] asid_nxt;
wire [25:0] tlbrentry_pa_nxt;

// DMW0/1 = {VSEG[2:0], 1'h0, PSEG[2:0], 19'h0, MAT[1:0], PLV3, 2'h0, PLV0}
// only VSEG/PSEG/MAT/PLV3/PLV0 are writable, the reserved bits stay 0
reg  [31:0] dmw0, dmw1;
wire [31:0] dmw0_sw, dmw1_sw;
wire [31:0] dmw0_wval, dmw1_wval;
wire [31:0] dmw0_wmsk, dmw1_wmsk;

// SAVE0..3
reg  [31:0] save     [0:3];
reg  [31:0] save_nxt [0:3];
integer     si;

wire        wr_save;
wire [ 1:0] save_sel;
wire [31:0] save_w;

// ECFG
// LIE[12:0]; bit 10 is reserved and never becomes writable
reg  [12:0] ecfg_lie;
wire [12:0] ecfg_lie_msk;
wire [12:0] ecfg_lie_nxt;

// TID
reg  [31:0] tid;
wire [31:0] tid_nxt;

// timer
// tcfg_en/periodic/initval are fields of TCFG; tval is the down counter behind
// TVAL.
reg         tcfg_en;
reg         tcfg_periodic;
reg  [29:0] tcfg_initval;
reg  [31:0] tval;

wire [31:0] tcfg_cur;
wire [31:0] tcfg_w;
wire        tcfg_en_nxt;
wire        tcfg_periodic_nxt;
wire [29:0] tcfg_initval_nxt;
wire [31:0] tcfg_reload;
wire [31:0] tval_free;
wire [31:0] tval_nxt;

// free-running 64-bit counter for rdcntvl.w / rdcntvh.w
reg  [63:0] timer;
wire [63:0] timer_nxt;

// assignments

// write select
assign wr_crmd   = we_i & (waddr_i == CSR_CRMD  );
assign wr_prmd   = we_i & (waddr_i == CSR_PRMD  );
assign wr_ecfg   = we_i & (waddr_i == CSR_ECFG  );
assign wr_estat  = we_i & (waddr_i == CSR_ESTAT );
assign wr_era    = we_i & (waddr_i == CSR_ERA   );
assign wr_badv   = we_i & (waddr_i == CSR_BADV  );
assign wr_eentry = we_i & (waddr_i == CSR_EENTRY);
assign wr_tid    = we_i & (waddr_i == CSR_TID   );
assign wr_tcfg   = we_i & (waddr_i == CSR_TCFG  );
assign wr_ticlr  = we_i & (waddr_i == CSR_TICLR );
assign wr_tlbidx    = we_i & (waddr_i == CSR_TLBIDX   );
assign wr_tlbehi    = we_i & (waddr_i == CSR_TLBEHI   );
assign wr_tlbelo0   = we_i & (waddr_i == CSR_TLBELO0  );
assign wr_tlbelo1   = we_i & (waddr_i == CSR_TLBELO1  );
assign wr_asid      = we_i & (waddr_i == CSR_ASID     );
assign wr_tlbrentry = we_i & (waddr_i == CSR_TLBRENTRY);
assign wr_dmw0      = we_i & (waddr_i == CSR_DMW0     );
assign wr_dmw1      = we_i & (waddr_i == CSR_DMW1     );

// CRMD translation-mode fields
// exception entry drops to DA=1/PG=0 for a refill, so the handler itself runs
// on physical addresses; ertn only restores PRMD.PPLV/PIE, so software has to
// switch back to the mapped mode itself
assign crmd_da_sw   = `CSR_FIELD(wr_crmd, csr_crmd_da,   wdata_i[3],   wmask_i[3]);
assign crmd_pg_sw   = `CSR_FIELD(wr_crmd, csr_crmd_pg,   wdata_i[4],   wmask_i[4]);
assign crmd_datf_sw = `CSR_FIELD(wr_crmd, csr_crmd_datf, wdata_i[6:5], wmask_i[6:5]);
assign crmd_datm_sw = `CSR_FIELD(wr_crmd, csr_crmd_datm, wdata_i[8:7], wmask_i[8:7]);

// only a tlb refill switches the core back to direct addressing; any other
// exception leaves DA/PG alone (a syscall must not unmap the core)
// ertn out of a refill handler goes back to the mapped mode: a refill can only
// be taken while mapping is on, and the handler ran physical
assign tlb_excp_nxt  = excp_i ? (excp_ecode_i == 6'h3f) : (ertn_i ? 1'b0 : tlb_excp_r);

assign crmd_da_nxt   = excp_i ? ((excp_ecode_i == 6'h3f) ? 1'b1 : csr_crmd_da) :
                       (ertn_i & tlb_excp_r)              ? 1'b0 : crmd_da_sw;
assign crmd_pg_nxt   = excp_i ? ((excp_ecode_i == 6'h3f) ? 1'b0 : csr_crmd_pg) :
                       (ertn_i & tlb_excp_r)              ? 1'b1 : crmd_pg_sw;
assign crmd_datf_nxt = crmd_datf_sw;
assign crmd_datm_nxt = crmd_datm_sw;

// Software-write view of CRMD / PRMD, before the exception/ertn override.  The
// exception / ertn entry is serviced one stage before the instruction reaches
// WB, so it takes these views: the entry is ordered after a CSR write that is
// still in flight, and the write itself lands on the edge into WB.
assign crmd_plv_sw = `CSR_FIELD(wr_crmd, crmd_plv, wdata_i[1:0], wmask_i[1:0]);
assign crmd_ie_sw  = `CSR_FIELD(wr_crmd, crmd_ie,  wdata_i[2],   wmask_i[2]);
assign prmd_pplv_sw = `CSR_FIELD(wr_prmd, prmd_pplv, wdata_i[1:0], wmask_i[1:0]);
assign prmd_pie_sw  = `CSR_FIELD(wr_prmd, prmd_pie,  wdata_i[2],   wmask_i[2]);

// exception entry drops to kernel mode with interrupts off; ertn restores PRMD
assign crmd_plv_nxt = excp_i ? 2'b00          :
                      ertn_i ? prmd_pplv_sw   :
                               crmd_plv_sw;
assign crmd_ie_nxt  = excp_i ? 1'b0           :
                      ertn_i ? prmd_pie_sw    :
                               crmd_ie_sw;

// PRMD
// exception entry saves the interrupted PLV/IE; software may write it too
assign prmd_pplv_nxt = excp_i ? crmd_plv_sw : prmd_pplv_sw;
assign prmd_pie_nxt  = excp_i ? crmd_ie_sw  : prmd_pie_sw;

// ESTAT
assign estat_is0_nxt = `CSR_FIELD(wr_estat, estat_is0, wdata_i[0], wmask_i[0]);
assign estat_is1_nxt = `CSR_FIELD(wr_estat, estat_is1, wdata_i[1], wmask_i[1]);
assign estat_ecode_nxt    = excp_i ? excp_ecode_i    : estat_ecode;
assign estat_esubcode_nxt = excp_i ? excp_esubcode_i : estat_esubcode;

// TI is raised when the count reaches zero, and cleared by TICLR[0].  A TCFG
// write that enables the timer with InitVal zero loads a count of zero, so that
// write raises TI in the same cycle.
assign ti_set    = tcfg_en & (tval == 32'd1);
assign ti_set_wr = wr_tcfg & tcfg_w[0] & (tcfg_w[31:2] == 30'd0);
assign ti_clr    = wr_ticlr & wdata_i[0] & wmask_i[0];
assign estat_ti_nxt = ti_clr ? 1'b0 : (ti_set | ti_set_wr | estat_ti);

assign estat_is = {ipi_i, estat_ti, 1'b0, hwi_i[7:0], estat_is1, estat_is0};

// ERA / BADV
// `era_sw` is the software-write view: that is what an ertn must jump to, and it
// may follow a csrwr ERA closely enough that the write is still in flight
assign era_sw   = `CSR_FIELD(wr_era, era, wdata_i, wmask_i);
assign era_nxt  = excp_i ? excp_pc_i : era_sw;
assign badv_nxt = excp_badv_en_i ? excp_badv_i :
                  `CSR_FIELD(wr_badv, badv, wdata_i, wmask_i);

// EENTRY
assign eentry_va_nxt = `CSR_FIELD(wr_eentry, eentry_va, wdata_i[31:6], wmask_i[31:6]);

// TLB CSRs
// A TLB instruction commits at WB, i.e. after its flush has emptied IF..EX, so
// its reads already see the software writes and no pass-through is needed here.
assign tlbidx_index_sw = `CSR_FIELD(wr_tlbidx, tlbidx_index, wdata_i[3:0],   wmask_i[3:0]);
assign tlbidx_ps_sw    = `CSR_FIELD(wr_tlbidx, tlbidx_ps,    wdata_i[29:24], wmask_i[29:24]);
assign tlbidx_ne_sw    = `CSR_FIELD(wr_tlbidx, tlbidx_ne,    wdata_i[31],    wmask_i[31]);
assign tlbehi_vppn_sw  = `CSR_FIELD(wr_tlbehi, tlbehi_vppn,  wdata_i[31:13], wmask_i[31:13]);
assign tlbelo0_sw      = `CSR_FIELD(wr_tlbelo0, tlbelo0,
                                    {wdata_i[27:8], 1'b0, wdata_i[6:0]}, wmask_i[27:0]);
assign tlbelo1_sw      = `CSR_FIELD(wr_tlbelo1, tlbelo1,
                                    {wdata_i[27:8], 1'b0, wdata_i[6:0]}, wmask_i[27:0]);
assign asid_sw         = `CSR_FIELD(wr_asid, asid, wdata_i[9:0], wmask_i[9:0]);
assign tlbrentry_pa_sw = `CSR_FIELD(wr_tlbrentry, tlbrentry_pa, wdata_i[31:6], wmask_i[31:6]);

wire tlb_srch_we;
wire tlb_rd_we;

assign tlb_srch_we = tlb_we_i & (tlb_sel_i == 2'd1);
assign tlb_rd_we   = tlb_we_i & (tlb_sel_i == 2'd2);

assign tlbidx_index_nxt = (tlb_srch_we & tlb_srch_hit_i) ? tlb_srch_index_i : tlbidx_index_sw;
assign tlbidx_ps_nxt    = tlb_rd_we ? (tlb_rd_e_i ? tlb_rd_ps_i : 6'h0)      : tlbidx_ps_sw;
assign tlbidx_ne_nxt    = tlb_srch_we ? ~ tlb_srch_hit_i :
                          tlb_rd_we   ? ~ tlb_rd_e_i     : tlbidx_ne_sw;
assign tlbehi_vppn_nxt  = excp_tlb_i ? excp_badv_i[31:13] :
                          tlb_rd_we   ? (tlb_rd_e_i ? tlb_rd_vppn_i : 19'h0)   : tlbehi_vppn_sw;
assign tlbelo0_nxt      = tlb_rd_we ? (tlb_rd_e_i ? tlb_rd_lo0_i[27:0] : 28'h0) : tlbelo0_sw;
assign tlbelo1_nxt      = tlb_rd_we ? (tlb_rd_e_i ? tlb_rd_lo1_i[27:0] : 28'h0) : tlbelo1_sw;
assign asid_nxt         = tlb_rd_we ? (tlb_rd_e_i ? tlb_rd_asid_i : 10'h0)  : asid_sw;
assign tlbrentry_pa_nxt = tlbrentry_pa_sw;

// DMW windows: only VSEG/PSEG/MAT/PLV3/PLV0 are writable, the reserved bits stay 0
assign dmw0_wval = {wdata_i[31:29], 1'b0, wdata_i[27:25], 19'b0, wdata_i[5:4], wdata_i[3], 2'b0, wdata_i[0]};
assign dmw1_wval = {wdata_i[31:29], 1'b0, wdata_i[27:25], 19'b0, wdata_i[5:4], wdata_i[3], 2'b0, wdata_i[0]};
assign dmw0_wmsk = {wmask_i[31:29], 1'b0, wmask_i[27:25], 19'b0, wmask_i[5:4], wmask_i[3], 2'b0, wmask_i[0]};
assign dmw1_wmsk = {wmask_i[31:29], 1'b0, wmask_i[27:25], 19'b0, wmask_i[5:4], wmask_i[3], 2'b0, wmask_i[0]};

assign dmw0_sw = wr_dmw0 ? ((dmw0_wval & dmw0_wmsk) | (dmw0 & ~dmw0_wmsk)) : dmw0;
assign dmw1_sw = wr_dmw1 ? ((dmw1_wval & dmw1_wmsk) | (dmw1 & ~dmw1_wmsk)) : dmw1;

// SAVE0..3
assign wr_save  = we_i & (waddr_i[13:2] == CSR_SAVE_H);
assign save_sel = waddr_i[1:0];
assign save_w   = (wdata_i & wmask_i) | (save[save_sel] & ~wmask_i);

always @(*) begin
    for (si = 0; si < 4; si = si + 1) begin
        save_nxt[si] = (wr_save & (save_sel == si[1:0])) ? save_w : save[si];
    end
end

// ECFG
assign ecfg_lie_msk = wmask_i[12:0] & 13'h1bff;
assign ecfg_lie_nxt = `CSR_FIELD(wr_ecfg, ecfg_lie, wdata_i[12:0], ecfg_lie_msk);

// TID
assign tid_nxt = `CSR_FIELD(wr_tid, tid, wdata_i, wmask_i);

// timer
assign tcfg_cur = {tcfg_initval, tcfg_periodic, tcfg_en};
assign tcfg_w   = (wdata_i & wmask_i) | (tcfg_cur & ~wmask_i);

assign tcfg_en_nxt       = wr_tcfg ? tcfg_w[0]    : tcfg_en;
assign tcfg_periodic_nxt = wr_tcfg ? tcfg_w[1]    : tcfg_periodic;
assign tcfg_initval_nxt  = wr_tcfg ? tcfg_w[31:2] : tcfg_initval;

// the countdown runs on the current configuration; a TCFG write with En = 1
assign tcfg_reload = {tcfg_initval, 2'b00};
assign tval_free   = tcfg_en
                   ? ((tval != 32'd0) ? (tval - 32'd1)
                                      : (tcfg_periodic ? tcfg_reload : tval))
                   : tval;
assign tval_nxt = (wr_tcfg & tcfg_w[0]) ? {tcfg_w[31:2], 2'b00} : tval_free;

// free-running 64-bit counter
assign timer_nxt = timer + 64'd1;

// registers
always @(posedge clk) begin
    if (reset) begin
        tlb_excp_r    <= 1'b0;
        csr_crmd_da   <= 1'b1;
        csr_crmd_pg   <= 1'b0;
        csr_crmd_datf <= 2'b0;
        csr_crmd_datm <= 2'b0;
        crmd_plv      <= 2'b0;
        crmd_ie       <= 1'b0;
        prmd_pplv     <= 2'b0;
        prmd_pie      <= 1'b0;
        estat_is0     <= 1'b0;
        estat_is1     <= 1'b0;
        estat_ti      <= 1'b0;
        estat_ecode   <= 6'b0;
        estat_esubcode<= 9'b0;
        era           <= 32'b0;
        badv          <= 32'b0;
        eentry_va     <= 26'b0;
        tlbidx_index  <= 4'b0;
        tlbidx_ps     <= 6'b0;
        tlbidx_ne     <= 1'b0;
        tlbehi_vppn   <= 19'b0;
        tlbelo0       <= 28'b0;
        tlbelo1       <= 28'b0;
        asid          <= 10'b0;
        tlbrentry_pa  <= 26'b0;
        dmw0          <= 32'b0;
        dmw1          <= 32'b0;
        save[0]       <= 32'b0;
        save[1]       <= 32'b0;
        save[2]       <= 32'b0;
        save[3]       <= 32'b0;
        ecfg_lie      <= 13'b0;
        tid           <= 32'b0;
        tcfg_en       <= 1'b0;
        tcfg_periodic <= 1'b0;
        tcfg_initval  <= 30'b0;
        tval          <= 32'b0;
        timer         <= 64'b0;
    end
    else begin
        tlb_excp_r    <= tlb_excp_nxt;
        csr_crmd_da   <= crmd_da_nxt;
        csr_crmd_pg   <= crmd_pg_nxt;
        csr_crmd_datf <= crmd_datf_nxt;
        csr_crmd_datm <= crmd_datm_nxt;
        crmd_plv      <= crmd_plv_nxt;
        crmd_ie       <= crmd_ie_nxt;
        prmd_pplv     <= prmd_pplv_nxt;
        prmd_pie      <= prmd_pie_nxt;
        estat_is0     <= estat_is0_nxt;
        estat_is1     <= estat_is1_nxt;
        estat_ti      <= estat_ti_nxt;
        estat_ecode   <= estat_ecode_nxt;
        estat_esubcode<= estat_esubcode_nxt;
        era           <= era_nxt;
        badv          <= badv_nxt;
        eentry_va     <= eentry_va_nxt;
        tlbidx_index  <= tlbidx_index_nxt;
        tlbidx_ps     <= tlbidx_ps_nxt;
        tlbidx_ne     <= tlbidx_ne_nxt;
        tlbehi_vppn   <= tlbehi_vppn_nxt;
        tlbelo0       <= tlbelo0_nxt;
        tlbelo1       <= tlbelo1_nxt;
        asid          <= asid_nxt;
        tlbrentry_pa  <= tlbrentry_pa_nxt;
        dmw0          <= dmw0_sw;
        dmw1          <= dmw1_sw;
        save[0]       <= save_nxt[0];
        save[1]       <= save_nxt[1];
        save[2]       <= save_nxt[2];
        save[3]       <= save_nxt[3];
        ecfg_lie      <= ecfg_lie_nxt;
        tid           <= tid_nxt;
        tcfg_en       <= tcfg_en_nxt;
        tcfg_periodic <= tcfg_periodic_nxt;
        tcfg_initval  <= tcfg_initval_nxt;
        tval          <= tval_nxt;
        timer         <= timer_nxt;
    end
end

// read port
// Plain asynchronous read of the registered state. 
always @(*) begin
    case (raddr_i)
        CSR_CRMD   : rdata_o = {23'b0, csr_crmd_datm, csr_crmd_datf, csr_crmd_pg,
                                csr_crmd_da, crmd_ie, crmd_plv};
        CSR_PRMD   : rdata_o = {29'b0, prmd_pie, prmd_pplv};
        CSR_ECFG   : rdata_o = {19'b0, ecfg_lie};
        CSR_ESTAT  : rdata_o = {1'b0, estat_esubcode, estat_ecode, 3'b0, estat_is};
        CSR_ERA    : rdata_o = era;
        CSR_BADV   : rdata_o = badv;
        CSR_EENTRY : rdata_o = {eentry_va, 6'b0};
        CSR_SAVE0  : rdata_o = save[0];
        CSR_SAVE1  : rdata_o = save[1];
        CSR_SAVE2  : rdata_o = save[2];
        CSR_SAVE3  : rdata_o = save[3];
        CSR_TID    : rdata_o = tid;
        CSR_TCFG   : rdata_o = {tcfg_initval, tcfg_periodic, tcfg_en};
        CSR_TVAL   : rdata_o = tval;
        CSR_TICLR  : rdata_o = 32'b0;       // write-only
        CSR_TLBIDX : rdata_o = {tlbidx_ne, 1'b0, tlbidx_ps, 20'b0, tlbidx_index};
        CSR_TLBEHI : rdata_o = {tlbehi_vppn, 13'b0};
        CSR_TLBELO0: rdata_o = {4'b0, tlbelo0};
        CSR_TLBELO1: rdata_o = {4'b0, tlbelo1};
        CSR_ASID   : rdata_o = {8'b0, 8'd10, 6'b0, asid};
        CSR_TLBRENTRY: rdata_o = {tlbrentry_pa, 6'b0};
        CSR_DMW0   : rdata_o = dmw0;
        CSR_DMW1   : rdata_o = dmw1;
        default    : rdata_o = 32'b0;       // EUEN / CPUID / unimplemented
    endcase
end

assign crmd_plv_o = crmd_plv;

assign crmd_ie_o  = crmd_ie_nxt;

// nextpc is stored when entering if
assign eentry_o   = {eentry_va, 6'b0};
assign era_o      = era;
assign timer_o    = timer;

assign int_o      = (| (ecfg_lie & estat_is)) & crmd_ie;

assign tlbidx_index_o = tlbidx_index;
assign tlbidx_ps_o    = tlbidx_ps;
assign tlbidx_ne_o    = tlbidx_ne;
assign tlbehi_vppn_o  = tlbehi_vppn;
assign tlbelo0_o      = {4'b0, tlbelo0};
assign tlbelo1_o      = {4'b0, tlbelo1};
assign asid_o         = asid;
assign tlbrentry_o    = {tlbrentry_pa, 6'b0};
assign estat_ecode_o  = estat_ecode;
assign crmd_da_o      = csr_crmd_da;
assign crmd_pg_o      = csr_crmd_pg;
assign dmw0_o         = dmw0;
assign dmw1_o         = dmw1;

`undef CSR_FIELD

endmodule
