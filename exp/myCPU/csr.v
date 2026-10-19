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

    // ertn
    input  wire        ertn_i,

    // interrupts
    input  wire [ 7:0] hwi_i,           // hardware
    input  wire        ipi_i,           // inter-processor

    // pipeline side-band
    // CRMD as it will be for the *next* instruction: includes the software write
    // in this slot and the exception / ertn override.  This is the value the
    // fetch issued by this slot's redirect has to be tagged with.
    output wire [ 1:0] crmd_plv_o,
    output wire        crmd_ie_o,
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
// EUEN (0x02) / CPUID (0x20) and anything else read 0 and ignore writes

// masked software write of one field: take wdata on the selected bits
`define CSR_FIELD(sel, cur, val, msk) \
    ((sel) ? (((val) & (msk)) | ((cur) & ~(msk))) : (cur))

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

// CRMD
// PLV/IE are real state; the translation-mode fields are fixed while the core
// has no address translation.
wire        csr_crmd_da;
wire        csr_crmd_pg;
wire [1:0]  csr_crmd_datf;
wire [1:0]  csr_crmd_datm;

reg  [1:0]  crmd_plv;
reg         crmd_ie;

wire [1:0]  crmd_plv_sw;    // software
wire        crmd_ie_sw;
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

// CRMD translation-mode fields
assign csr_crmd_da   = 1'b1;
assign csr_crmd_pg   = 1'b0;
assign csr_crmd_datf = 2'b00;
assign csr_crmd_datm = 2'b00;

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
        default    : rdata_o = 32'b0;       // EUEN / CPUID / unimplemented
    endcase
end

// side-band
// NB: the exception / ertn override must be part of this view.  A redirect is
// issued in the same cycle the entry/return lands, and the fetch it starts must
// already carry the *new* PLV; crmd_plv_sw alone would still show the old one
// for an exception (-> 0) or an ertn (-> PRMD.PPLV).
assign crmd_plv_o = crmd_plv_nxt;
assign crmd_ie_o  = crmd_ie_nxt;
assign eentry_o   = {eentry_va_nxt, 6'b0};
assign era_o      = era_sw;
assign timer_o    = timer;

assign int_o      = (| (ecfg_lie & estat_is)) & crmd_ie;

`undef CSR_FIELD

endmodule
