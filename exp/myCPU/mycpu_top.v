// ===========================================================================
// Dual-issue (2-wide) in-order core.
//
//   IF  : preif sends nextpc to the mmu; mmu / icache return 1 or 2 words
//         (both inside the same 16B line), parked into the ID queue.
//   ID  : 3-slot compacting queue.  Slot 0 = full decode, slot 1 = alu/mul
//         only, slot 2 = storage only (no decode).  Up to 2 instructions are
//         issued per cycle; slot 0 is always the older one.
//   MEM : the pair moves as one unit.  Load / store, CSR, TLB, cacop, div and
//         every exception live on slot 0 only.
//   WB  : the pair moves as one unit, 2 write ports.  Slot 1 is the younger
//         instruction, so it wins a WAW inside a pair.
// ===========================================================================
module mycpu_top(
    input  wire        aclk,
    input  wire        aresetn,      // low active

    // AXI read address channel
    output wire [ 3:0] arid,
    output wire [31:0] araddr,
    output wire [ 7:0] arlen,
    output wire [ 2:0] arsize,
    output wire [ 1:0] arburst,
    output wire [ 1:0] arlock,
    output wire [ 3:0] arcache,
    output wire [ 2:0] arprot,
    output wire        arvalid,
    input  wire        arready,

    // AXI read data channel
    input  wire [ 3:0] rid,
    input  wire [31:0] rdata,
    input  wire [ 1:0] rresp,
    input  wire        rlast,
    input  wire        rvalid,
    output wire        rready,

    // AXI write address channel
    output wire [ 3:0] awid,
    output wire [31:0] awaddr,
    output wire [ 7:0] awlen,
    output wire [ 2:0] awsize,
    output wire [ 1:0] awburst,
    output wire [ 1:0] awlock,
    output wire [ 3:0] awcache,
    output wire [ 2:0] awprot,
    output wire        awvalid,
    input  wire        awready,

    // AXI write data channel
    output wire [ 3:0] wid,
    output wire [31:0] wdata,
    output wire [ 3:0] wstrb,
    output wire        wlast,
    output wire        wvalid,
    input  wire        wready,

    // AXI write response channel
    input  wire [ 3:0] bid,
    input  wire [ 1:0] bresp,
    input  wire        bvalid,
    output wire        bready,

    // trace debug interface: slot 0 (the older instruction of the pair)
    output wire [31:0] debug_wb_pc,
    output wire [ 3:0] debug_wb_rf_we,
    output wire [ 4:0] debug_wb_rf_wnum,
    output wire [31:0] debug_wb_rf_wdata,
    // trace debug interface: slot 1 (the younger one, never valid alone)
    output wire [31:0] debug_wb_pc_1,
    output wire [ 3:0] debug_wb_rf_we_1,
    output wire [ 4:0] debug_wb_rf_wnum_1,
    output wire [31:0] debug_wb_rf_wdata_1
);

wire clk;
wire resetn;

assign clk    = aclk;
assign resetn = aresetn;

localparam DUAL_EN = 1'b1;      // 0: slot 1 never issues (single-issue bring-up)

reg         reset;
always @(posedge clk) reset <= ~resetn;

localparam IF                = 0;
localparam ID                = 1;
localparam MEM               = 2;
localparam WB                = 3;
localparam TOTAL_STAGE_WIDTH = 4;

// md_op bit positions: one-hot mul/div operation select
localparam MUL_W             = 0;
localparam MULH_W            = 1;
localparam MULH_WU           = 2;
localparam DIV_W             = 3;
localparam MOD_W             = 4;
localparam DIV_WU            = 5;
localparam MOD_WU            = 6;

// mem_size bit positions: one-hot access size
localparam SIZE_BYTE         = 0;
localparam SIZE_HALF         = 1;
localparam SIZE_WORD         = 2;

// csr_ctl bit positions
localparam CSR_RD            = 0;
localparam CSR_WE            = 1;
localparam CSR_ERTN          = 2;
localparam CSR_EXCP          = 3;
localparam CSR_RDCNTVL       = 4;
localparam CSR_RDCNTVH       = 5;
localparam CSR_RDCNTID       = 6;
localparam CSR_WB            = 7;
localparam CSR_PRIV          = 8;

// excp_cause bit positions
localparam EXCP_SYS          = 0;
localparam EXCP_BRK          = 1;
localparam EXCP_INE          = 2;
localparam EXCP_ADEF         = 3;
localparam EXCP_ALE          = 4;
localparam EXCP_INT          = 5;
localparam EXCP_IPE          = 6;
localparam EXCP_TLBR         = 7;
localparam EXCP_PIF          = 8;
localparam EXCP_PIL          = 9;
localparam EXCP_PIS          = 10;
localparam EXCP_PME          = 11;
localparam EXCP_PPI          = 12;

localparam [13:0] CSR_CRMD   = 14'h000;
localparam [13:0] CSR_DMW0   = 14'h180;
localparam [13:0] CSR_DMW1   = 14'h181;
localparam [13:0] CSR_ASID   = 14'h018;

// TLB instruction select, one hot per slot
localparam TLB_SRCH = 0;
localparam TLB_RD   = 1;
localparam TLB_WR   = 2;
localparam TLB_FILL = 3;
localparam TLB_INV  = 4;

// declarations

// mmu: the fetch / memory-access stage controller.
wire        preif_req;         // a fetch enters IF
wire [31:0] inst_rdata_0;      // first word, returns after an arbitrary latency
wire [31:0] inst_rdata_1;      // second word of the pair (nextpc + 4)
wire        premem_req;        // a load / store in ID that enters MEM at this edge
wire [31:0] mem_va;            // its virtual address (= slot 0 alu_result_mem)
wire        mem_abort;         // this access has been found illegal
wire [ 3:0] mem_wstrb;         // byte strobes; 4'b0 means read
wire [31:0] mem_stdata;        // store data, already spread over the byte lanes
wire [31:0] mem_rdata;
wire        if_pending;        // from the mmu: this fetch should wait
wire        mem_pending;       // from the mmu: this access should wait

// unit position: the instruction born in ID carries its mul / div flag into
// MEM, where the product / quotient appears.
reg         mul_pos_0, mul_pos_1, div_pos_0;
wire        cur_mul_pos_0, cur_mul_pos_1, cur_div_pos_0;
wire        next_mul_pos_0, next_mul_pos_1, next_div_pos_0;

// pc per slot, indexed by stage: pc_0[IF] is the fetch address; slot 2 only exists in ID.
reg  [31:0] pc_0 [WB:IF] /* verilator split_var */;
reg  [31:0] pc_1 [WB:ID] /* verilator split_var */;
reg  [31:0] pc_2;

// fetch / redirect
wire [31:0] nextpc, seq_pc;
wire        preif_adef, if_fetch_single, if_excp, if_abort;
wire [12:0] if_excp_cause, if_fetch_cause;
reg         adef_if;  // store preif adef
wire        br_taken_id_0;
wire        pc_redirect;
wire [31:0] br_target_id_0;
reg         pc_redirected;
reg  [31:0] redirected_pc_reg;
wire        sel_era, sel_eentry, sel_wb_seq, sel_br, sel_cur_br, sel_seq;

// ID queue
reg  [31:0] inst_id_0, inst_id_1, inst_id_2;
reg  [12:0] excp_id_0, excp_id_1, excp_id_2;
wire [ 1:0] id_deq, id_occ, id_tail, id_fill_req, id_fill_cnt;
wire        id_fill, id_fill_0, id_fill_1, id_room, id_head_move;
wire        id_next_valid_0, id_next_valid_1, id_next_valid_2;
wire [ 2:0] id_shift_valid;
wire [31:0] id_shift_pc_0,   id_shift_pc_1,   id_shift_pc_2;
wire [31:0] id_shift_inst_0, id_shift_inst_1, id_shift_inst_2;
wire [12:0] id_shift_excp_0, id_shift_excp_1, id_shift_excp_2;
wire [31:0] id_fill_pc_0,    id_fill_pc_1;
wire [31:0] id_fill_inst_0,  id_fill_inst_1;
wire [12:0] id_fill_excp_0,  id_fill_excp_1;
wire        id_issue_0, id_issue_1, id_issue_1_ok;

// ID decode, slot 0
wire [11:0] id_alu_op_0;
wire [MOD_WU:0] id_md_op_0;
wire        id_src1_used_0, id_src2_used_0, id_res_from_mem_0;
wire        id_mem_we_0, id_mem_en_0, id_mem_zext_0, id_gr_we_0;
wire [ 2:0] id_mem_size_0;
wire [ 4:0] id_dest_0, id_tlb_op_0, id_invtlb_op_0, id_cacop_code_0;
wire [13:0] id_csr_num_0;
wire [ 8:0] id_csr_ctl_0;
wire [12:0] id_excp_decode_0, id_excp_cause_0;
wire        id_src1_is_pc_0, id_src2_is_imm_0, id_csr_mask_ones_0;
wire [ 4:0] id_rf_raddr1_0, id_rf_raddr2_0;
wire [31:0] id_imm_0, id_src1_0, id_src2_0, alu_result_id_0;
wire        inst_cacop_0;

// ID decode, slot 1 (alu / mul only) --> id_alu_mul_decoder module
wire        id_legal_1;
wire [11:0] id_alu_op_1;
wire [MOD_WU:0] id_md_op_1;
wire [ 4:0] id_dest_1, id_rf_raddr1_1, id_rf_raddr2_1;
wire        id_src1_used_1, id_src2_used_1, id_src1_is_pc_1, id_src2_is_imm_1;
wire [31:0] id_imm_1, id_src1_1, id_src2_1, alu_result_id_1;

// ID issue helpers
wire        id_csr_block_0;

// MEM stage registers
reg  [31:0] alu_result_mem_0, alu_result_mem_1;
reg  [MOD_WU:0] md_op_mem_0, md_op_mem_1;
reg  [31:0] div_src1_mem, div_src2_mem;
reg         res_from_mem_0, mem_we_0, mem_en_0, mem_zext_0, cacop_0;
reg  [ 2:0] mem_size_0;
reg  [31:0] mem_wdata_mem_0;
reg  [ 4:0] cacop_code_0;

reg  [ 4:0] dest_0 [WB:MEM] /* verilator split_var */;
reg  [ 4:0] dest_1 [WB:MEM] /* verilator split_var */;
reg         gr_we_0 [WB:MEM] /* verilator split_var */;
reg         gr_we_1 [WB:MEM] /* verilator split_var */;
reg  [13:0] csr_num_0 [WB:MEM] /* verilator split_var */;
reg  [ 8:0] csr_ctl_0 [WB:MEM] /* verilator split_var */;
reg  [ 4:0] tlb_op_0 [WB:MEM] /* verilator split_var */;
reg  [ 4:0] invtlb_op_0 [WB:MEM] /* verilator split_var */;
reg  [31:0] csr_wdata_0 [WB:MEM] /* verilator split_var */;
reg  [31:0] csr_mask_0 [WB:MEM] /* verilator split_var */;
reg  [12:0] excp_cause_0 [WB:MEM] /* verilator split_var */;

// ID -> MEM combinational
wire        premem_ale_0, premem_int_take, premem_excp_0;
wire [ 8:0] premem_csr_ctl_0;
wire [12:0] premem_excp_cause_0;

// MEM
wire        mem_byte, mem_half, mem_word;
wire [ 3:0] st_we_byte, st_we_half, st_we_word, st_we;
wire [ 7:0] ld_byte;
wire [15:0] ld_half;
wire [31:0] ld_byte_ext, ld_half_ext, mem_result;
wire        mem_excp_0, mem_ipe_0;
wire [ 8:0] mem_csr_ctl_v;
wire [12:0] mem_excp_cause_0;
wire        mem_tlb, mem_cacop, mem_cacop_ic, mem_abort_i;
wire [ 4:0] mem_cacop_code;
wire        mem_tlb_chk, mem_map_wr, mem_crmd_tlb_cacop;
wire [12:0] mem_tlb_cause;

// WB
wire [31:0] result_mem_0, result_mem_1;
reg  [31:0] result_wb_0, result_wb_1;
wire        rf_we_0, rf_we_1, rf_we_csr;
wire [ 4:0] rf_waddr_0, rf_waddr_1;
wire [31:0] rf_wdata_0, rf_wdata_1, rf_wdata_csr;
reg         wb_excp_ertn, wb_excp, wb_ertn, wb_crmd_tlb_cacop;
reg  [31:0] wb_excp_badv;
wire [ 5:0] excp_ecode;
wire        excp_badv_en, excp_tlb;
wire [31:0] excp_entry;

// div / mul
wire [31:0] mul_result_0, mul_result_1;
wire [31:0] div_result, div_quot, div_rem;
wire        div_busy, div_is_signed, div_is_mod, div_start, div_reset;
reg         div_started;

// forwarding
wire        fw_valid_mem_0, fw_valid_mem_1;
wire [31:0] fw_data_mem_0, fw_data_mem_1;
wire [31:0] id_fw1_data_0, id_fw2_data_0, id_fw1_data_1, id_fw2_data_1;
wire        id_fw1_pend_0, id_fw2_pend_0, id_fw1_pend_1, id_fw2_pend_1;
wire        id_fw_pending_0, id_fw_pending_1;
wire        id_fw1_hit_mem_slot1_0, id_fw1_hit_mem_slot0_0, id_fw2_hit_mem_slot1_0, id_fw2_hit_mem_slot0_0;
wire        id_fw1_hit_id_slot0_1, id_fw2_hit_id_slot0_1;  
wire        id_fw1_hit_mem_slot1_1, id_fw1_hit_mem_slot0_1, id_fw2_hit_mem_slot1_1, id_fw2_hit_mem_slot0_1;
wire [31:0] id_rf_rdata1_0, id_rf_rdata2_0, id_rf_rdata1_1, id_rf_rdata2_1;
reg         csr_wb_flying;

// branch
wire [ 6:0] id_br_op_0;
wire [31:0] id_br_base_0, id_br_offs_0;
wire        id_br_valid;
reg         id_br_sent;

// CSR file
wire [31:0] csr_rdata, eentry_o, era_o;
wire [63:0] timer_o;
wire        csr_we, int_o;
wire [13:0] csr_raddr, csr_waddr;
wire [31:0] csr_wdata_w;
wire [ 1:0] crmd_plv_o;
wire        crmd_ie_o;

// translation
wire        map_mode, if_xlat, mem_xlat;
wire        if_dmw_hit, mem_dmw_hit;
wire [31:0] if_dmw_pa, mem_dmw_pa;
wire        dmw0_hit_if, dmw1_hit_if, dmw0_hit_mem, dmw1_hit_mem;
wire [31:0] dmw0_o, dmw1_o;
wire [ 1:0] if_mat, mem_mat, tlb_s0_mat, tlb_s1_mat;
wire        tlb_s0_found, tlb_s1_found, tlb_s2_found;
wire [19:0] tlb_s0_ppn, tlb_s1_ppn;
wire [ 5:0] tlb_s0_ps, tlb_s1_ps;
wire [ 3:0] tlb_s2_index;
wire        tlb_s0_v, tlb_s1_v, tlb_s1_d;
wire [ 1:0] tlb_s0_plv, tlb_s1_plv;
wire        tlb_s0_va_bit12, tlb_s1_va_bit12;
wire [18:0] tlb_s0_vppn, tlb_s1_vppn;
wire        tlb_r_e, tlb_r_g, tlb_r_d0, tlb_r_d1, tlb_r_v0, tlb_r_v1;
wire [18:0] tlb_r_vppn;
wire [ 5:0] tlb_r_ps;
wire [ 9:0] tlb_r_asid;
wire [19:0] tlb_r_ppn0, tlb_r_ppn1;
wire [ 1:0] tlb_r_plv0, tlb_r_plv1, tlb_r_mat0, tlb_r_mat1;
wire [31:0] tlb_rd_lo0, tlb_rd_lo1;
wire [ 1:0] tlb_csr_sel;
wire        wb_tlb_commit, wb_tlb_srch, wb_tlb_rd, wb_tlb_wr, wb_tlb_inv;
wire [ 3:0] tlb_w_index;
wire        tlb_w_e, tlb_w_g, tlb_w_d0, tlb_w_d1, tlb_w_v0, tlb_w_v1;
wire [18:0] tlb_w_vppn;
wire [ 9:0] tlb_w_asid;
wire [19:0] tlb_w_ppn0, tlb_w_ppn1;
wire [ 1:0] tlb_w_plv0, tlb_w_plv1, tlb_w_mat0, tlb_w_mat1;
wire [ 3:0] tlbidx_index_o;
wire [ 5:0] tlbidx_ps_o;
wire        tlbidx_ne_o;
wire [18:0] tlbehi_vppn_o;
wire [31:0] tlbelo0_o, tlbelo1_o;
wire [ 9:0] asid_o;
wire [31:0] tlbrentry_o;
wire [ 5:0] estat_ecode_o;
wire        crmd_da_o, crmd_pg_o;
wire [ 1:0] crmd_datf_o, crmd_datm_o;

// ID decode helpers (slot 0)
wire [ 5:0] op_31_26;
wire [ 3:0] op_25_22;
wire [ 1:0] op_21_20;
wire [ 4:0] op_19_15;
wire [ 4:0] rd, rj, rk;
wire [11:0] i12;
wire [19:0] i20;
wire [15:0] i16;
wire [25:0] i26;
wire [63:0] op_31_26_d;
wire [15:0] op_25_22_d;
wire [ 3:0] op_21_20_d;
wire [31:0] op_19_15_d;

wire        inst_add_w, inst_sub_w, inst_slt, inst_sltu, inst_nor, inst_and, inst_or, inst_xor;
wire        inst_slli_w, inst_srli_w, inst_srai_w, inst_addi_w, inst_ld_w, inst_st_w;
wire        inst_jirl, inst_b, inst_bl, inst_beq, inst_bne, inst_lu12i_w;
wire        inst_slti, inst_sltui, inst_andi, inst_ori, inst_xori;
wire        inst_sll_w, inst_srl_w, inst_sra_w, inst_pcaddu12i;
wire        inst_mul_w, inst_mulh_w, inst_mulh_wu, inst_div_w, inst_div_wu, inst_mod_w, inst_mod_wu;
wire        inst_blt, inst_bge, inst_bltu, inst_bgeu;
wire        inst_ld_b, inst_ld_h, inst_ld_bu, inst_ld_hu, inst_st_b, inst_st_h;
wire        inst_csr_any, inst_csrrd, inst_csrwr, inst_csrxchg, inst_ertn;
wire        inst_syscall, inst_break, inst_idle;
wire        inst_rdcntvl_w, inst_rdcntvh_w, inst_rdcntid, inst_ine;
wire        inst_tlbsrch, inst_tlbrd, inst_tlbwr, inst_tlbfill, inst_invtlb, inst_tlb_any;
wire        need_ui12, need_si20, src2_is_4;
wire [31:0] br_offs_i26, br_offs_i16;
wire        src_reg_is_rd;

// ===========================================================================
// pipeline control
// ---------------------------------------------------------------------------
//   valid_0 / valid_1 : lane 0 (slot 0) and lane 1 (slot 1) of a pair
//   valid_2           : the ID-only storage slot (slot 2)
//   stall / allow / flush : one stage vector each
//   cur_valid_k = valid_k & ~flush_k        next_valid_k = the stage shift
// ===========================================================================

reg  [TOTAL_STAGE_WIDTH-1:0] valid_0;    // [WB:IF]  slot 0 (always present)
reg  [TOTAL_STAGE_WIDTH-1:0] valid_1;    // [WB:IF]  slot 1 (IF bit never set)
reg  [TOTAL_STAGE_WIDTH-1:0] valid_2;    // [WB:IF]  slot 2 (ID bit used only)

wire        wb_flush;                   // slot 0 in WB redirects (see MEM / WB below)

// allow[IF] consumes allow[ID] only, while stall[IF] consumes the ID dequeue
// (which in turn uses allow[ID] / stall[ID] / stall[MEM])
wire [TOTAL_STAGE_WIDTH-1:0] stall /* verilator split_var */;
wire [TOTAL_STAGE_WIDTH-1:0] allow /* verilator split_var */;
wire [TOTAL_STAGE_WIDTH-1:0] flush, flush_slot_1_2;
wire [TOTAL_STAGE_WIDTH-1:0] cur_valid_0, cur_valid_1, cur_valid_2;
wire [TOTAL_STAGE_WIDTH-1:0] next_valid_0, next_valid_1, next_valid_2;

// ---- flush: only slot 0 can raise one -------------------------------------
//   wb_flush      : slot 0 in WB reports an exception / ertn / tlb / cacop
//   br_taken_id_0 : slot 0 in ID takes a branch
assign flush   = wb_flush      ? 4'b0111 :    // IF / ID / MEM die, WB keeps going
                 br_taken_id_0 ? 4'b0001 :    // only the wrong-path fetch dies
                                 4'b0000;

// slot 1 / slot 2 should flush when slot0 flushs at wb/id --> as they are younger than slot 0
assign flush_slot_1_2 = flush | {wb_flush, 1'b0, br_taken_id_0, 1'b0};

assign cur_valid_0 = valid_0 & ~ flush;
assign cur_valid_1 = valid_1 & ~ flush_slot_1_2;
assign cur_valid_2 = valid_2 & ~ flush_slot_1_2;

// ---- stall: this stage cannot advance -------------------------------------
//   IF  : the fetch has not come back / the ID queue cannot take a whole pair
//   ID/MEM : only slot 0 can hold the pipeline
assign stall[IF]  = if_pending | ~ id_room;
assign stall[ID]  = cur_valid_0[ID] & id_fw_pending_0;
assign stall[MEM] = mem_pending | (cur_div_pos_0 & (div_busy | ~ div_started));
assign stall[WB]  = 1'b0;

// ---- allow ----------------------------------------------------------------
assign allow[WB]  = 1'b1;
assign allow[MEM] = ~ stall[MEM] & allow[WB];
assign allow[ID]  = ~ stall[ID]  & allow[MEM];
assign allow[IF]  = ~ stall[IF]  & allow[ID];

// ---- issue = dequeue from the ID queue ------------------------------------
// Slot 0 leaves ID when neither ID (its operands) nor MEM blocks; slot 1 also
// needs its own qualification.
assign id_issue_0 = cur_valid_0[ID] & ~ id_fw_pending_0 & ~ stall[MEM];
assign id_issue_1 = cur_valid_1[ID] & ~ id_fw_pending_0 & ~ stall[MEM] & id_issue_1_ok;

// ---- next_valid -----------------------------------------------------------
// The source of the ID bit is the ID queue fill (id_next_valid_k) instead of
// cur_valid & stall
assign next_valid_0 = {cur_valid_0[MEM:ID] & ~ stall[MEM:ID],  // -> WB, MEM
                       id_next_valid_0,                        // -> ID
                       1'b1}                                   // -> IF: a new fetch starts
                    & allow | (~ allow & cur_valid_0);

// lane 1: the MEM bit also needs the slot 1 issue qualification, so a slot 1
// that does not issue sends a bubble instead
assign next_valid_1 = {cur_valid_1[MEM] & ~ stall[MEM],                 // -> WB
                       cur_valid_1[ID]  & ~ stall[ID] & id_issue_1_ok,  // -> MEM (or bubble)
                       id_next_valid_1,                                 // -> ID
                       1'b0}                                            // -> IF
                    & allow | (~ allow & cur_valid_1);

// slot 2 is storage only: it never leaves ID
assign next_valid_2 = {2'b0, id_next_valid_2, 1'b0} & allow
                    | (~allow & cur_valid_2);

always @(posedge clk) begin
    if (reset) begin
        valid_0 <= 4'b0;
        valid_1 <= 4'b0;
        valid_2 <= 4'b0;
    end
    else begin
        valid_0 <= next_valid_0;
        valid_1 <= next_valid_1;
        valid_2 <= next_valid_2;
    end
end

// fetch: preif / IF / redirect
assign preif_adef = | nextpc[1:0];
assign preif_req  = ~ preif_adef;

// can the fetch in flight return two words?  It must not cross the 64B icache
// line (word 15 is the last one) and it must be cacheable (an uncached fetch is
// one bridge beat, so it can never return the second word)
assign if_fetch_single = (& pc_0[IF][5:2]) | ~ (if_mat == 2'b01) | if_excp;

assign seq_pc = pc_0[IF] + (if_fetch_single ? 32'h4 : 32'h8);

assign sel_era      = wb_ertn;
assign sel_eentry   = wb_excp;
assign sel_wb_seq   = ~ wb_excp_ertn & wb_crmd_tlb_cacop;
assign sel_br       = ~ wb_excp_ertn & ~ wb_crmd_tlb_cacop & pc_redirected;
assign sel_cur_br   = ~ wb_excp_ertn & ~ wb_crmd_tlb_cacop & ~ pc_redirected & br_taken_id_0;
assign sel_seq      = ~ (wb_excp_ertn | wb_crmd_tlb_cacop | pc_redirected | br_taken_id_0);

assign nextpc       = ({32{sel_era    }} & era_o                ) |
                      ({32{sel_eentry }} & excp_entry           ) |
                      ({32{sel_wb_seq }} & (pc_0[WB] + 32'h4)   ) |
                      ({32{sel_br     }} & redirected_pc_reg    ) |
                      ({32{sel_cur_br }} & br_target_id_0       ) |
                      ({32{sel_seq    }} & seq_pc               );

assign pc_redirect = br_taken_id_0 | wb_excp_ertn | wb_crmd_tlb_cacop;

always @(posedge clk) begin
    if (reset) begin
        pc_redirected     <= 1'b0;
        redirected_pc_reg <= 32'b0;
    end
    else begin
        if (allow[IF]) begin
            pc_redirected     <= 1'b0;
            redirected_pc_reg <= 32'b0;
        end
        else if (pc_redirect && ~ allow[IF]) begin
            pc_redirected     <= 1'b1;
            redirected_pc_reg <= nextpc;
        end
    end
end

always @(posedge clk) begin
    if (reset) begin
        pc_0[IF]    <= 32'h1bfffffc; // trick: first fetched pc is 0x1c000000 --> use single fetch route
        adef_if     <= 1'b0;
    end
    else begin
        if (allow[IF]) begin
            pc_0[IF]    <= nextpc;
            adef_if     <= preif_adef;
        end
    end
end

// a fetch that carries an exception (ADEF or a TLB class one) drops the whole
// pair: only one nop carrying the exception enters the queue
assign if_excp_cause = ({12'b0, adef_if} << EXCP_ADEF) | if_fetch_cause;
assign if_excp       = | if_excp_cause;
assign if_abort      = | if_fetch_cause | flush[IF];

// ===========================================================================
// ID: 3-slot compacting queue
// ---------------------------------------------------------------------------
// The three slots only hold {pc, inst, excp} and they are always compacted
// (the occupancy can only be 000 / 001 / 011 / 111).  Every cycle the head
// dequeues id_deq instructions (0/1/2), the rest slides down by id_deq, and
// then the newly fetched instruction(s) are written from the tail id_tail:
//   slot 0 empty                     -> fill slot 0 / slot 1
//   slot 0 occupied, slot 1 empty    -> fill slot 1 / slot 2
//   otherwise                        -> stall[IF] = 1, IF is blocked
// ===========================================================================

assign id_deq  = {1'b0, id_issue_0} + {1'b0, id_issue_1};
assign id_occ  = {1'b0, cur_valid_0[ID]} + {1'b0, cur_valid_1[ID]} + {1'b0, cur_valid_2[ID]};
assign id_tail = id_occ - id_deq;              // enqueue base = occupancy after the dequeue

// IF always hands ID a pair (two slots).  The second slot carries a bubble
// (valid = 0) when the fetch really returned one word only: an exception, an
// uncached fetch or a fetch that would cross the 16B line.  ID needs room for
// the whole pair, so slot 1 being occupied already blocks IF.
assign id_room          = ~ id_tail[1]; // id_tail + 2 <= 3
assign id_fill          = allow[IF] & cur_valid_0[IF];      // do fill
assign id_fill_0        = id_fill;                          // slot t   : always real
assign id_fill_1        = id_fill & ~ if_fetch_single;      // slot t+1 : real or bubble
assign id_head_move     = id_issue_0 | (id_fill & (id_tail == 2'd0));

// shift: slot k takes slot k + id_deq
assign id_shift_pc_0   = (id_deq == 2'd0) ? pc_0[ID]  : (id_deq == 2'd1) ? pc_1[ID]  : pc_2;
assign id_shift_pc_1   = (id_deq == 2'd0) ? pc_1[ID]  : (id_deq == 2'd1) ? pc_2      : 32'b0;
assign id_shift_pc_2   = (id_deq == 2'd0) ? pc_2      : 32'b0;
assign id_shift_inst_0 = (id_deq == 2'd0) ? inst_id_0 : (id_deq == 2'd1) ? inst_id_1 : inst_id_2;
assign id_shift_inst_1 = (id_deq == 2'd0) ? inst_id_1 : (id_deq == 2'd1) ? inst_id_2 : 32'b0;
assign id_shift_inst_2 = (id_deq == 2'd0) ? inst_id_2 : 32'b0;
assign id_shift_excp_0 = (id_deq == 2'd0) ? excp_id_0 : (id_deq == 2'd1) ? excp_id_1 : excp_id_2;
assign id_shift_excp_1 = (id_deq == 2'd0) ? excp_id_1 : (id_deq == 2'd1) ? excp_id_2 : 13'b0;
assign id_shift_excp_2 = (id_deq == 2'd0) ? excp_id_2 : 13'b0;

assign id_shift_valid = (id_deq == 2'd0) ? {cur_valid_2[ID], cur_valid_1[ID], cur_valid_0[ID]} :
                        (id_deq == 2'd1) ? {1'b0, cur_valid_2[ID], cur_valid_1[ID]} :
                                           {1'b0, 1'b0, cur_valid_2[ID]};

// the incoming pair (for an exception only one nop = andi $r0, $r0, 0)
assign id_fill_pc_0   = pc_0[IF];
assign id_fill_inst_0 = adef_if ? 32'h0340_0000 : inst_rdata_0; // only adef should overwrite inst
assign id_fill_excp_0 = if_excp_cause;
assign id_fill_pc_1   = pc_0[IF] + 32'h4;
assign id_fill_inst_1 = inst_rdata_1;
assign id_fill_excp_1 = 13'h0;

// slot next_valid: everything from id_tail up is the freed tail, filled with
// the fetched pair (the second half may be the bubble)
assign id_next_valid_0 = (id_tail == 2'd0) ? id_fill_0 : id_shift_valid[0];
assign id_next_valid_1 = (id_tail == 2'd0) ? id_fill_1 :
                         (id_tail == 2'd1) ? id_fill_0 : id_shift_valid[1];
assign id_next_valid_2 = (id_tail == 2'd0) ? 1'b0 :
                         (id_tail == 2'd1) ? id_fill_1 : id_shift_valid[2];

always @(posedge clk) begin
    if (reset) begin
        pc_0[ID]   <= 32'b0;
        pc_1[ID]   <= 32'b0;
        pc_2   <= 32'b0;
        inst_id_0 <= 32'h0340_0000;
        inst_id_1 <= 32'h0340_0000;
        inst_id_2 <= 32'h0340_0000;
        excp_id_0 <= 13'b0;
        excp_id_1 <= 13'b0;
        excp_id_2 <= 13'b0;
    end
    else begin
        case (id_tail)
            2'd0: begin
                pc_0[ID]  <= id_fill_pc_0;
                inst_id_0 <= id_fill_inst_0;
                excp_id_0 <= id_fill_excp_0;

                pc_1[ID]  <= id_fill_pc_1;
                inst_id_1 <= id_fill_inst_1;
                excp_id_1 <= id_fill_excp_1;
                
                pc_2      <= 32'b0;
                inst_id_2 <= 32'h0340_0000; // nop
                excp_id_2 <= 13'b0;
            end
            2'd1: begin
                pc_0[ID]  <= id_shift_pc_0;
                inst_id_0 <= id_shift_inst_0;
                excp_id_0 <= id_shift_excp_0;

                pc_1[ID]  <= id_fill_pc_0;
                inst_id_1 <= id_fill_inst_0;
                excp_id_1 <= id_fill_excp_0;
                
                pc_2      <= id_fill_pc_1;
                inst_id_2 <= id_fill_inst_1;
                excp_id_2 <= id_fill_excp_1;
            end
            default: begin  // 2'd2 or 2'd3
                pc_0[ID]  <= id_shift_pc_0;
                inst_id_0 <= id_shift_inst_0;
                excp_id_0 <= id_shift_excp_0;

                pc_1[ID]  <= id_shift_pc_1;
                inst_id_1 <= id_shift_inst_1;
                excp_id_1 <= id_shift_excp_1;

                pc_2      <= id_shift_pc_2;
                inst_id_2 <= id_shift_inst_2;
                excp_id_2 <= id_shift_excp_2;
            end
        endcase
    end
end

// ===========================================================================
// mul/div unit position flags: born in ID, used in MEM
// ===========================================================================

assign cur_mul_pos_0 = mul_pos_0 & ~flush[MEM];
assign cur_mul_pos_1 = mul_pos_1 & ~flush[MEM];
assign cur_div_pos_0 = div_pos_0 & ~flush[MEM];

assign next_mul_pos_0 = (id_issue_0 & (| id_md_op_0[2:0])) | (~allow[MEM] & cur_mul_pos_0);
assign next_mul_pos_1 = (id_issue_1 & (| id_md_op_1[2:0])) | (~allow[MEM] & cur_mul_pos_1);
assign next_div_pos_0 = (id_issue_0 & (| id_md_op_0[6:3])) | (~allow[MEM] & cur_div_pos_0);

always @(posedge clk) begin
    if (reset) begin
        mul_pos_0 <= 1'b0;
        mul_pos_1 <= 1'b0;
        div_pos_0 <= 1'b0;
    end
    else begin
        mul_pos_0 <= next_mul_pos_0;
        mul_pos_1 <= next_mul_pos_1;
        div_pos_0 <= next_div_pos_0;
    end
end

// ===========================================================================
// ID: slot 0, full decode
// ===========================================================================

assign id_excp_cause_0 = excp_id_0 | id_excp_decode_0;

assign op_31_26  = inst_id_0[31:26];
assign op_25_22  = inst_id_0[25:22];
assign op_21_20  = inst_id_0[21:20];
assign op_19_15  = inst_id_0[19:15];

assign rd   = inst_rdcntid ? inst_id_0[9:5] : inst_id_0[4:0];
assign rj   = inst_id_0[ 9: 5];
assign rk   = inst_id_0[14:10];

assign i12  = inst_id_0[21:10];
assign i20  = inst_id_0[24: 5];
assign i16  = inst_id_0[25:10];
assign i26  = {inst_id_0[ 9: 0], inst_id_0[25:10]};

decoder_6_64 u_dec0(.in(op_31_26 ), .out(op_31_26_d ));
decoder_4_16 u_dec1(.in(op_25_22 ), .out(op_25_22_d ));
decoder_2_4  u_dec2(.in(op_21_20 ), .out(op_21_20_d ));
decoder_5_32 u_dec3(.in(op_19_15 ), .out(op_19_15_d ));

assign inst_add_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h00];
assign inst_sub_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h02];
assign inst_slt    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h04];
assign inst_sltu   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h05];
assign inst_nor    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h08];
assign inst_and    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h09];
assign inst_or     = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0a];
assign inst_xor    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0b];
assign inst_slli_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h01];
assign inst_srli_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h09];
assign inst_srai_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h11];
assign inst_addi_w = op_31_26_d[6'h00] & op_25_22_d[4'ha];
assign inst_ld_w   = op_31_26_d[6'h0a] & op_25_22_d[4'h2];
assign inst_st_w   = op_31_26_d[6'h0a] & op_25_22_d[4'h6];
assign inst_jirl   = op_31_26_d[6'h13];
assign inst_b      = op_31_26_d[6'h14];
assign inst_bl     = op_31_26_d[6'h15];
assign inst_beq    = op_31_26_d[6'h16];
assign inst_bne    = op_31_26_d[6'h17];
assign inst_lu12i_w= op_31_26_d[6'h05] & ~ inst_id_0[25];

assign inst_slti     = op_31_26_d[6'h00] & op_25_22_d[4'h8];
assign inst_sltui    = op_31_26_d[6'h00] & op_25_22_d[4'h9];
assign inst_andi     = op_31_26_d[6'h00] & op_25_22_d[4'hd];
assign inst_ori      = op_31_26_d[6'h00] & op_25_22_d[4'he];
assign inst_xori     = op_31_26_d[6'h00] & op_25_22_d[4'hf];
assign inst_sll_w    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0e];
assign inst_srl_w    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0f];
assign inst_sra_w    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h10];
assign inst_pcaddu12i= op_31_26_d[6'h07];

assign inst_mul_w    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h18];
assign inst_mulh_w   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h19];
assign inst_mulh_wu  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h1a];
assign inst_div_w    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h00];
assign inst_mod_w    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h01];
assign inst_div_wu   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h02];
assign inst_mod_wu   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h03];

assign inst_blt   = op_31_26_d[6'h18];
assign inst_bge   = op_31_26_d[6'h19];
assign inst_bltu  = op_31_26_d[6'h1a];
assign inst_bgeu  = op_31_26_d[6'h1b];
assign inst_ld_b  = op_31_26_d[6'h0a] & op_25_22_d[4'h0];
assign inst_ld_h  = op_31_26_d[6'h0a] & op_25_22_d[4'h1];
assign inst_st_b  = op_31_26_d[6'h0a] & op_25_22_d[4'h4];
assign inst_st_h  = op_31_26_d[6'h0a] & op_25_22_d[4'h5];
assign inst_ld_bu = op_31_26_d[6'h0a] & op_25_22_d[4'h8];
assign inst_ld_hu = op_31_26_d[6'h0a] & op_25_22_d[4'h9];

assign inst_csr_any = op_31_26_d[6'h01] & ~ inst_id_0[25] & ~ inst_id_0[24];
assign inst_csrrd   = inst_csr_any & (inst_id_0[9:5] == 5'd0);
assign inst_csrwr   = inst_csr_any & (inst_id_0[9:5] == 5'd1);
assign inst_csrxchg = inst_csr_any & ~ inst_csrrd & ~ inst_csrwr;
assign inst_ertn    = op_31_26_d[6'h01] & inst_id_0[25] & ~ inst_id_0[24] & op_19_15_d[5'h10]
                    & (inst_id_0[14:10] == 5'b01110);
assign inst_idle    = op_31_26_d[6'h01] & inst_id_0[25] & ~ inst_id_0[24] & op_19_15_d[5'h11];
assign inst_tlbsrch = op_31_26_d[6'h01] & inst_id_0[25] & ~ inst_id_0[24] & op_19_15_d[5'h10]
                    & (inst_id_0[14:10] == 5'b01010);
assign inst_tlbrd   = op_31_26_d[6'h01] & inst_id_0[25] & ~ inst_id_0[24] & op_19_15_d[5'h10]
                    & (inst_id_0[14:10] == 5'b01011);
assign inst_tlbwr   = op_31_26_d[6'h01] & inst_id_0[25] & ~ inst_id_0[24] & op_19_15_d[5'h10]
                    & (inst_id_0[14:10] == 5'b01100);
assign inst_tlbfill = op_31_26_d[6'h01] & inst_id_0[25] & ~ inst_id_0[24] & op_19_15_d[5'h10]
                    & (inst_id_0[14:10] == 5'b01101);
assign inst_invtlb  = op_31_26_d[6'h01] & inst_id_0[25] & ~ inst_id_0[24] & op_19_15_d[5'h13];
assign inst_tlb_any = inst_tlbsrch | inst_tlbrd | inst_tlbwr | inst_tlbfill | inst_invtlb;

// cacop code, rj, si12  (opcode 0000 0110 00; code[2:0]: 0 icache / 1 dcache,
// code[4:3]: 0 store tag / 1 index invalidate / 2 hit invalidate)
assign inst_cacop_0    = op_31_26_d[6'h01] & op_25_22_d[4'h8];
assign id_cacop_code_0 = inst_id_0[4:0];

assign inst_syscall = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h16];
assign inst_break   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h14];

assign inst_rdcntvl_w = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h0] & op_19_15_d[5'h00]
                      & (inst_id_0[14:10] == 5'b11000) & (inst_id_0[9:5] == 5'd0);
assign inst_rdcntvh_w = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h0] & op_19_15_d[5'h00]
                      & (inst_id_0[14:10] == 5'b11001) & (inst_id_0[9:5] == 5'd0);
assign inst_rdcntid   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h0] & op_19_15_d[5'h00]
                      & (inst_id_0[14:10] == 5'b11000) & (inst_id_0[4:0] == 5'd0) & ~ inst_rdcntvl_w;

assign inst_ine = ~ (inst_lu12i_w | inst_add_w | inst_sub_w | inst_slt | inst_sltu
                   | inst_nor | inst_and | inst_or | inst_xor
                   | inst_slli_w | inst_srli_w | inst_srai_w
                   | inst_addi_w | inst_ld_w | inst_st_w | inst_jirl | inst_b | inst_bl
                   | inst_beq | inst_bne
                   | inst_slti | inst_sltui | inst_andi | inst_ori | inst_xori
                   | inst_sll_w | inst_srl_w | inst_sra_w | inst_pcaddu12i
                   | inst_mul_w | inst_mulh_w | inst_mulh_wu
                   | inst_div_w | inst_mod_w | inst_div_wu | inst_mod_wu
                   | inst_blt | inst_bge | inst_bltu | inst_bgeu
                   | inst_ld_b | inst_ld_h | inst_ld_bu | inst_ld_hu
                   | inst_st_b | inst_st_h
                   | inst_csr_any | inst_ertn | inst_syscall | inst_break | inst_idle
                   | inst_tlb_any | inst_cacop_0
                   | inst_rdcntvl_w | inst_rdcntvh_w | inst_rdcntid);

assign id_src1_used_0 = inst_add_w | inst_sub_w | inst_slt | inst_sltu
                  | inst_and | inst_or | inst_nor | inst_xor
                  | inst_slli_w | inst_srli_w | inst_srai_w | inst_addi_w
                  | inst_ld_w | inst_st_w | inst_beq | inst_bne | inst_jirl
                  | inst_slti | inst_sltui | inst_andi | inst_ori | inst_xori
                  | inst_sll_w | inst_srl_w | inst_sra_w
                  | inst_mul_w | inst_mulh_w | inst_mulh_wu
                  | inst_div_w | inst_mod_w | inst_div_wu | inst_mod_wu
                  | inst_blt | inst_bge | inst_bltu | inst_bgeu
                  | inst_ld_b | inst_ld_h | inst_ld_bu | inst_ld_hu
                  | inst_st_b | inst_st_h
                  | inst_csrxchg
                  | inst_invtlb | inst_cacop_0;
assign id_src2_used_0 = inst_add_w | inst_sub_w | inst_slt | inst_sltu
                  | inst_and | inst_or | inst_nor | inst_xor
                  | inst_st_w | inst_beq | inst_bne
                  | inst_sll_w | inst_srl_w | inst_sra_w
                  | inst_mul_w | inst_mulh_w | inst_mulh_wu
                  | inst_div_w | inst_mod_w | inst_div_wu | inst_mod_wu
                  | inst_blt | inst_bge | inst_bltu | inst_bgeu
                  | inst_st_b | inst_st_h
                  | inst_csrwr | inst_csrxchg
                  | inst_invtlb;

assign id_alu_op_0[ 0] = inst_add_w | inst_addi_w | inst_ld_w | inst_st_w
                  | inst_jirl | inst_bl | inst_pcaddu12i
                  | inst_ld_b | inst_ld_h | inst_ld_bu | inst_ld_hu
                  | inst_st_b | inst_st_h
                  | inst_cacop_0;                          // cacop address = rj + si12
assign id_alu_op_0[ 1] = inst_sub_w;
assign id_alu_op_0[ 2] = inst_slt | inst_slti;
assign id_alu_op_0[ 3] = inst_sltu | inst_sltui;
assign id_alu_op_0[ 4] = inst_and | inst_andi;
assign id_alu_op_0[ 5] = inst_nor;
assign id_alu_op_0[ 6] = inst_or | inst_ori;
assign id_alu_op_0[ 7] = inst_xor | inst_xori;
assign id_alu_op_0[ 8] = inst_slli_w | inst_sll_w;
assign id_alu_op_0[ 9] = inst_srli_w | inst_srl_w;
assign id_alu_op_0[10] = inst_srai_w | inst_sra_w;
assign id_alu_op_0[11] = inst_lu12i_w;

assign id_md_op_0[MUL_W]   = inst_mul_w;
assign id_md_op_0[MULH_W]  = inst_mulh_w;
assign id_md_op_0[MULH_WU] = inst_mulh_wu;
assign id_md_op_0[DIV_W]   = inst_div_w;
assign id_md_op_0[MOD_W]   = inst_mod_w;
assign id_md_op_0[DIV_WU]  = inst_div_wu;
assign id_md_op_0[MOD_WU]  = inst_mod_wu;

assign id_res_from_mem_0 = inst_ld_w | inst_ld_b | inst_ld_h | inst_ld_bu | inst_ld_hu;
assign id_mem_we_0       = inst_st_w | inst_st_b | inst_st_h;
assign id_mem_en_0       = id_mem_we_0 | id_res_from_mem_0;
assign id_mem_size_0[SIZE_BYTE] = inst_ld_b | inst_ld_bu | inst_st_b;
assign id_mem_size_0[SIZE_HALF] = inst_ld_h | inst_ld_hu | inst_st_h;
assign id_mem_size_0[SIZE_WORD] = inst_ld_w | inst_st_w;
assign id_mem_zext_0     = inst_ld_bu | inst_ld_hu;

assign id_csr_num_0 = inst_rdcntid ? 14'h040 : inst_id_0[23:10];
// hit-class cacop (code[4:3]=2, i.e. 0x10 / 0x11) is allowed at PLV3, every
// other cacop is privileged
assign id_csr_ctl_0[CSR_PRIV] = inst_csr_any | inst_ertn | inst_idle | inst_tlb_any
                              | (inst_cacop_0 & ~ (inst_id_0[4] & ~ inst_id_0[3]));
assign id_tlb_op_0 = {inst_invtlb, inst_tlbfill, inst_tlbwr, inst_tlbrd, inst_tlbsrch};
assign id_csr_ctl_0[CSR_RD] = inst_csrrd | inst_csrwr | inst_csrxchg;
assign id_csr_ctl_0[CSR_WE] = inst_csrwr | inst_csrxchg;
assign id_csr_ctl_0[CSR_ERTN] = inst_ertn;
assign id_csr_ctl_0[CSR_RDCNTVL] = inst_rdcntvl_w;
assign id_csr_ctl_0[CSR_RDCNTVH] = inst_rdcntvh_w;
assign id_csr_ctl_0[CSR_RDCNTID] = inst_rdcntid;
assign id_csr_ctl_0[CSR_WB] = inst_csrrd | inst_csrwr | inst_csrxchg
                            | inst_rdcntvl_w | inst_rdcntvh_w | inst_rdcntid;
assign id_csr_ctl_0[CSR_EXCP] = | id_excp_cause_0;
assign id_invtlb_op_0 = inst_id_0[4:0];
assign id_csr_mask_ones_0 = inst_csrwr;

assign id_excp_decode_0[EXCP_SYS]  = inst_syscall;
assign id_excp_decode_0[EXCP_BRK]  = inst_break;
assign id_excp_decode_0[EXCP_INE]  = inst_ine | (inst_invtlb & (inst_id_0[4:0] > 5'd6));
assign id_excp_decode_0[EXCP_ADEF] = 1'b0;
assign id_excp_decode_0[EXCP_ALE]  = 1'b0;
assign id_excp_decode_0[EXCP_INT]  = 1'b0;
assign id_excp_decode_0[EXCP_IPE]  = 1'b0;
assign id_excp_decode_0[EXCP_TLBR] = 1'b0;
assign id_excp_decode_0[EXCP_PIF]  = 1'b0;
assign id_excp_decode_0[EXCP_PIL]  = 1'b0;
assign id_excp_decode_0[EXCP_PIS]  = 1'b0;
assign id_excp_decode_0[EXCP_PME]  = 1'b0;
assign id_excp_decode_0[EXCP_PPI]  = 1'b0;

assign id_gr_we_0 = ~inst_st_w & ~inst_beq & ~inst_bne & ~inst_b
              & ~inst_st_b & ~inst_st_h
              & ~inst_blt & ~inst_bge & ~inst_bltu & ~inst_bgeu
              & ~inst_ertn & ~inst_syscall & ~inst_break & ~inst_idle
              & ~inst_tlb_any & ~inst_cacop_0;
assign id_dest_0 = inst_bl ? 5'd1 : rd;

assign need_ui12  = inst_andi | inst_ori | inst_xori;
assign need_si20  = inst_lu12i_w | inst_pcaddu12i;
assign src2_is_4  = inst_jirl | inst_bl;

assign id_imm_0 = src2_is_4 ? 32'h4                      :
                  need_si20 ? {i20[19:0], 12'b0}         :
                  need_ui12 ? {20'b0, i12[11:0]}         :
                              {{20{i12[11]}}, i12[11:0]} ;

assign br_offs_i26 = {{ 4{i26[25]}}, i26[25:0], 2'b0};
assign br_offs_i16 = {{14{i16[15]}}, i16[15:0], 2'b0};

assign src_reg_is_rd = inst_beq | inst_bne | inst_st_w | inst_st_b | inst_st_h
                     | inst_blt | inst_bge | inst_bltu | inst_bgeu
                     | inst_csrwr | inst_csrxchg;

assign id_src1_is_pc_0  = inst_jirl | inst_bl | inst_pcaddu12i;
assign id_src2_is_imm_0 = inst_slli_w | inst_srli_w | inst_srai_w
                        | inst_addi_w | inst_slti | inst_sltui
                        | inst_andi | inst_ori | inst_xori
                        | inst_ld_w | inst_st_w
                        | inst_ld_b | inst_ld_h | inst_ld_bu | inst_ld_hu
                        | inst_st_b | inst_st_h
                        | inst_lu12i_w | inst_pcaddu12i
                        | inst_jirl | inst_bl | inst_cacop_0;

assign id_rf_raddr1_0 = rj;
assign id_rf_raddr2_0 = src_reg_is_rd ? rd : rk;

// ===========================================================================
// ID: slot 1, docode alu / mul only
// ===========================================================================

id_alu_mul_decoder u_decoder_slot1(
    .inst        (inst_id_1       ),
    .legal       (id_legal_1      ),
    .alu_op      (id_alu_op_1     ),
    .md_op       (id_md_op_1      ),
    .dest        (id_dest_1       ),
    .rf_raddr1   (id_rf_raddr1_1  ),
    .rf_raddr2   (id_rf_raddr2_1  ),
    .src1_used   (id_src1_used_1  ),
    .src2_used   (id_src2_used_1  ),
    .src1_is_pc  (id_src1_is_pc_1 ),
    .src2_is_imm (id_src2_is_imm_1),
    .imm         (id_imm_1        )
);

// ===========================================================================
// ID: is slot 1 co-issue ok signal generate
// ===========================================================================

// a CSR-class instruction is never executed in parallel with anything else
assign id_csr_block_0 = inst_csr_any | inst_ertn | inst_idle | inst_tlb_any | inst_cacop_0
                      | inst_rdcntvl_w | inst_rdcntvh_w | inst_rdcntid;

assign id_issue_1_ok = DUAL_EN
                     & id_legal_1
                     & ~ id_fw_pending_1        // its own operands are not ready
                     & ~ (| excp_id_1)          // carries an exception: handle it in slot 0
                     & ~ premem_excp_0          // slot 0 raises an exception / takes the interrupt
                     & ~ id_csr_block_0;        // CSR class never pairs

// ===========================================================================
// ID: branch unit (slot 0 only)
// ===========================================================================
// b / bl / jirl are unconditional: they use the br unit's JIRL (taken) bit,
// so the target is built by the same base+offs adder as the conditional
// branches.  b/bl use pc + i26<<2; jirl uses rj + i16<<2.
assign id_br_op_0    = {inst_jirl | inst_b | inst_bl,
                        inst_bgeu, inst_bltu, inst_bge, inst_blt, inst_bne, inst_beq};
assign id_br_base_0  = inst_jirl ? id_fw1_data_0 : pc_0[ID];
assign id_br_offs_0  = (inst_b | inst_bl) ? br_offs_i26 : br_offs_i16;
assign id_br_valid   = valid_0[ID] & ~ id_br_sent & ~ id_fw_pending_0;   // raw valid: no flush loop

// a taken branch already redirected the fetch; do not resolve it again while it
// sits in slot 0 waiting for MEM (the head moving clears the flag)
always @(posedge clk) begin
    if (reset | flush[ID])       id_br_sent <= 1'b0;
    else if (id_head_move)      id_br_sent <= 1'b0;
    else if (~ id_fw_pending_0) id_br_sent <= 1'b1;
end

br u_br(
    .src1   (id_fw1_data_0  ),
    .src2   (id_fw2_data_0  ),
    .base   (id_br_base_0   ),
    .offs   (id_br_offs_0   ),
    .br_op  (id_br_op_0     ),
    .valid  (id_br_valid    ),
    .taken  (br_taken_id_0  ),
    .target (br_target_id_0 )
);

// ===========================================================================
// ID: forwarding (MEM slot 0 / slot 1 -> ID)
// ===========================================================================
// fw_valid_mem_*: this MEM slot holds a result that is ready to be handed to
// ID.  Only a plain ALU result qualifies: a load needs its data, mul/div
// results are only selected at the MEM/WB edge, and a CSR write goes through
// the dedicated RF port (ID waits for it via csr_wb_flying).
assign fw_valid_mem_0 = ~ mem_en_0 & ~ csr_ctl_0[MEM][CSR_WB] & ~ mul_pos_0 & ~ div_pos_0;
assign fw_data_mem_0  = alu_result_mem_0;
assign fw_valid_mem_1 = ~ mul_pos_1;            // slot 1 is alu or mul, nothing else
assign fw_data_mem_1  = alu_result_mem_1;

// CSR-writes-RF dependency: a csrrd/csrwr/... writes the RF through the CSR
// data port at WB with no read bypass, so ID waits until the write has landed.
always @(posedge clk) begin
    if (reset | flush[ID])                          csr_wb_flying <= 1'b0;
    else if (id_issue_0 & id_csr_ctl_0[CSR_WB])     csr_wb_flying <= 1'b1;
    else if (valid_0[WB] & csr_ctl_0[WB][CSR_WB])   csr_wb_flying <= 1'b0;
end

// slot 0, source 1
assign id_fw1_hit_mem_slot1_0 = valid_1[MEM] & gr_we_1[MEM] & id_src1_used_0
                       & (id_rf_raddr1_0 != 5'd0) & (id_rf_raddr1_0 == dest_1[MEM]);
assign id_fw1_hit_mem_slot0_0 = valid_0[MEM] & gr_we_0[MEM] & id_src1_used_0
                       & (id_rf_raddr1_0 != 5'd0) & (id_rf_raddr1_0 == dest_0[MEM]);

assign id_fw1_pend_0   = (id_fw1_hit_mem_slot1_0 & ~ fw_valid_mem_1)
                       | (id_fw1_hit_mem_slot0_0 & ~ fw_valid_mem_0) | csr_wb_flying;
assign id_fw1_data_0   = ({32{id_fw1_hit_mem_slot1_0}} & fw_data_mem_1) |
                         ({32{id_fw1_hit_mem_slot0_0}} & fw_data_mem_0) |
                         ({32{~ id_fw1_hit_mem_slot1_0 & ~ id_fw1_hit_mem_slot0_0}} & id_rf_rdata1_0);

// slot 0, source 2
assign id_fw2_hit_mem_slot1_0 = valid_1[MEM] & gr_we_1[MEM] & id_src2_used_0
                       & (id_rf_raddr2_0 != 5'd0) & (id_rf_raddr2_0 == dest_1[MEM]);
assign id_fw2_hit_mem_slot0_0 = valid_0[MEM] & gr_we_0[MEM] & id_src2_used_0
                       & (id_rf_raddr2_0 != 5'd0) & (id_rf_raddr2_0 == dest_0[MEM]);

assign id_fw2_pend_0   = (id_fw2_hit_mem_slot1_0 & ~ fw_valid_mem_1)
                       | (id_fw2_hit_mem_slot0_0 & ~ fw_valid_mem_0) | csr_wb_flying;
assign id_fw2_data_0   = ({32{id_fw2_hit_mem_slot1_0}} & fw_data_mem_1) |
                         ({32{id_fw2_hit_mem_slot0_0}} & fw_data_mem_0) |
                         ({32{~ id_fw2_hit_mem_slot1_0 & ~ id_fw2_hit_mem_slot0_0}} & id_rf_rdata2_0);

assign id_fw_pending_0 = id_fw1_pend_0 | id_fw2_pend_0;

// slot 1, source 1
assign id_fw1_hit_id_slot0_1 = valid_0[ID] & id_gr_we_0 & id_src1_used_1 
                            & (id_dest_0 != 5'd0) & (id_rf_raddr1_1 == id_dest_0);  // RAW
assign id_fw1_hit_mem_slot1_1 = valid_1[MEM] & gr_we_1[MEM] & id_src1_used_1
                       & (id_rf_raddr1_1 != 5'd0) & (id_rf_raddr1_1 == dest_1[MEM]);
assign id_fw1_hit_mem_slot0_1 = valid_0[MEM] & gr_we_0[MEM] & id_src1_used_1
                       & (id_rf_raddr1_1 != 5'd0) & (id_rf_raddr1_1 == dest_0[MEM]);

assign id_fw1_pend_1   = id_fw1_hit_id_slot0_1
                       | (id_fw1_hit_mem_slot1_1 & ~ fw_valid_mem_1)
                       | (id_fw1_hit_mem_slot0_1 & ~ fw_valid_mem_0) | csr_wb_flying;
assign id_fw1_data_1   = ({32{id_fw1_hit_mem_slot1_1}} & fw_data_mem_1) |
                         ({32{id_fw1_hit_mem_slot0_1}} & fw_data_mem_0) |
                         ({32{~ id_fw1_hit_mem_slot1_1 & ~ id_fw1_hit_mem_slot0_1}} & id_rf_rdata1_1);

// slot 1, source 2
assign id_fw2_hit_id_slot0_1 = valid_0[ID] & id_gr_we_0 & id_src2_used_1 
                            & (id_dest_0 != 5'd0) & (id_rf_raddr2_1 == id_dest_0);  // RAW
assign id_fw2_hit_mem_slot1_1 = valid_1[MEM] & gr_we_1[MEM] & id_src2_used_1
                       & (id_rf_raddr2_1 != 5'd0) & (id_rf_raddr2_1 == dest_1[MEM]);
assign id_fw2_hit_mem_slot0_1 = valid_0[MEM] & gr_we_0[MEM] & id_src2_used_1
                       & (id_rf_raddr2_1 != 5'd0) & (id_rf_raddr2_1 == dest_0[MEM]);

assign id_fw2_pend_1   = id_fw2_hit_id_slot0_1
                       | (id_fw2_hit_mem_slot1_1 & ~ fw_valid_mem_1)
                       | (id_fw2_hit_mem_slot0_1 & ~ fw_valid_mem_0) | csr_wb_flying;
assign id_fw2_data_1   = ({32{id_fw2_hit_mem_slot1_1}} & fw_data_mem_1) |
                         ({32{id_fw2_hit_mem_slot0_1}} & fw_data_mem_0) |
                         ({32{~ id_fw2_hit_mem_slot1_1 & ~ id_fw2_hit_mem_slot0_1}} & id_rf_rdata2_1);

assign id_fw_pending_1 = id_fw1_pend_1 | id_fw2_pend_1;

// ID datapath: the operands are already resolved by the ID forwarding network
assign id_src1_0 = id_src1_is_pc_0  ? pc_0[ID] : id_fw1_data_0;
assign id_src2_0 = id_src2_is_imm_0 ? id_imm_0 : id_fw2_data_0;
assign id_src1_1 = id_src1_is_pc_1  ? pc_1[ID] : id_fw1_data_1;
assign id_src2_1 = id_src2_is_imm_1 ? id_imm_1 : id_fw2_data_1;

// ID units: the ALU computes the result / the load-store address here, and the
// multiplier's Booth + Wallace tree collapses the product to (ws, wc) here.
// Both results are registered into MEM at the ID/MEM edge.
alu u_alu_0(
    .alu_op     (id_alu_op_0   ),
    .src1       (id_src1_0     ),
    .src2       (id_src2_0     ),
    .alu_result (alu_result_id_0)
);

alu u_alu_1(
    .alu_op     (id_alu_op_1   ),
    .src1       (id_src1_1     ),
    .src2       (id_src2_1     ),
    .alu_result (alu_result_id_1)
);

mul u_mul_0(
    .clk    (clk            ),
    .allow  (allow[ID]      ),
    .src1   (id_src1_0      ),
    .src2   (id_src2_0      ),
    .md_op  (id_md_op_0     ),
    .result (mul_result_0   )
);

mul u_mul_1(
    .clk    (clk            ),
    .allow  (allow[ID]      ),
    .src1   (id_src1_1      ),
    .src2   (id_src2_1      ),
    .md_op  (id_md_op_1     ),
    .result (mul_result_1   )
);

// ===========================================================================
// ID -> MEM
// ===========================================================================
assign premem_ale_0   = id_mem_en_0 & ((id_mem_size_0[SIZE_HALF] &    alu_result_id_0[0]   |
                                        id_mem_size_0[SIZE_WORD] & (| alu_result_id_0[1:0])));
assign premem_int_take = int_o;
assign premem_excp_0   = premem_ale_0 | premem_int_take | id_csr_ctl_0[CSR_EXCP];
assign premem_csr_ctl_0 = id_csr_ctl_0 | (premem_excp_0 ? (9'h1 << CSR_EXCP) : 9'h0);
assign premem_excp_cause_0 = id_excp_cause_0
                           | (premem_ale_0 ? (13'h1 << EXCP_ALE) : 13'h0)
                           | (premem_int_take ? (13'h1 << EXCP_INT) : 13'h0);
// mem_pre is "the access that will enter MEM next cycle", and the mmu uses it
// to keep its state machine running even while MEM is held.
assign premem_req   = cur_valid_0[ID] & ~ id_fw_pending_0
                    & (id_mem_en_0 | inst_cacop_0) & ~ premem_excp_0;

always @(posedge clk) begin
    if (allow[MEM]) begin
        pc_0[MEM]          <= pc_0[ID];
        pc_1[MEM]          <= pc_1[ID];
        alu_result_mem_0  <= alu_result_id_0;
        alu_result_mem_1  <= alu_result_id_1;
        md_op_mem_0       <= id_md_op_0;
        md_op_mem_1       <= id_md_op_1;
        div_src1_mem      <= id_src1_0;
        div_src2_mem      <= id_src2_0;
        res_from_mem_0    <= id_res_from_mem_0;
        mem_we_0          <= id_mem_we_0;
        mem_en_0          <= id_mem_en_0;
        mem_wdata_mem_0   <= id_fw2_data_0;
        mem_size_0        <= id_mem_size_0;
        mem_zext_0        <= id_mem_zext_0;
        gr_we_0[MEM]      <= id_gr_we_0;
        gr_we_1[MEM]      <= 1'b1;              // slot 1 is alu / mul: always writes rd
        dest_0[MEM]       <= id_dest_0;
        dest_1[MEM]       <= id_dest_1;
        csr_num_0[MEM]    <= id_csr_num_0;
        csr_ctl_0[MEM]    <= premem_csr_ctl_0;
        tlb_op_0[MEM]     <= id_tlb_op_0;
        invtlb_op_0[MEM]  <= id_invtlb_op_0;
        cacop_0           <= inst_cacop_0;
        cacop_code_0      <= id_cacop_code_0;
        excp_cause_0[MEM] <= premem_excp_cause_0;
        csr_wdata_0[MEM]  <= id_fw2_data_0;
        csr_mask_0[MEM]   <= id_csr_mask_ones_0 ? 32'hffffffff : id_fw1_data_0;
    end
end

// ===========================================================================
// MEM
// ===========================================================================
// the address / byte lanes / store data were all computed in ID (slot 0)
assign mem_va         = alu_result_mem_0;
assign mem_cacop      = cacop_0 & valid_0[MEM] & ~ mem_excp_0;
assign mem_cacop_code = cacop_code_0;
assign mem_abort      = mem_excp_0 | flush[MEM];

assign mem_byte = mem_size_0[SIZE_BYTE];
assign mem_half = mem_size_0[SIZE_HALF];
assign mem_word = mem_size_0[SIZE_WORD];

assign st_we_byte = 4'b0001 << alu_result_mem_0[1:0];
assign st_we_half = alu_result_mem_0[1] ? 4'b1100 : 4'b0011;
assign st_we_word = 4'b1111;

assign st_we = ({4{mem_byte}} & st_we_byte) | ({4{mem_half}} & st_we_half) | ({4{mem_word}} & st_we_word);

assign mem_wstrb  = {4{mem_we_0 && valid_0[MEM] && ~ mem_excp_0}} & st_we;
assign mem_stdata = ({32{mem_byte}} & {4{mem_wdata_mem_0[ 7: 0]}}) |
                    ({32{mem_half}} & {2{mem_wdata_mem_0[15: 0]}}) |
                    ({32{mem_word}} &    mem_wdata_mem_0);

assign ld_byte = (alu_result_mem_0[1:0] == 2'd0) ? mem_rdata[ 7: 0] :
                 (alu_result_mem_0[1:0] == 2'd1) ? mem_rdata[15: 8] :
                 (alu_result_mem_0[1:0] == 2'd2) ? mem_rdata[23:16] :
                                                   mem_rdata[31:24];
assign ld_half = alu_result_mem_0[1] ? mem_rdata[31:16] : mem_rdata[15:0];

assign ld_byte_ext = mem_zext_0 ? {24'b0, ld_byte} : {{24{ld_byte[7]}}, ld_byte};
assign ld_half_ext = mem_zext_0 ? {16'b0, ld_half} : {{16{ld_half[15]}}, ld_half};

assign mem_result = ({32{mem_byte}} & ld_byte_ext)  |
                    ({32{mem_half}} & ld_half_ext)  |
                    ({32{mem_word}} & mem_rdata);

assign mem_ipe_0  = csr_ctl_0[MEM][CSR_PRIV] & (crmd_plv_o != 2'b0);
assign mem_excp_0 = (csr_ctl_0[MEM][CSR_EXCP] | mem_ipe_0 | (| mem_tlb_cause));

assign mem_csr_ctl_v    = csr_ctl_0[MEM] | ((mem_ipe_0 | (| mem_tlb_cause)) ? (9'h1 << CSR_EXCP) : 9'h0);
assign mem_excp_cause_0 = excp_cause_0[MEM]
                        | (mem_ipe_0 ? (13'h1 << EXCP_IPE) : 13'h0)
                        | mem_tlb_cause;

assign mem_tlb = (| tlb_op_0[MEM]) & ~ tlb_op_0[MEM][TLB_SRCH];

assign mem_map_wr   = csr_ctl_0[MEM][CSR_WE] & (csr_num_0[MEM] == CSR_CRMD
                                              | csr_num_0[MEM] == CSR_DMW0
                                              | csr_num_0[MEM] == CSR_DMW1
                                              | csr_num_0[MEM] == CSR_ASID);

assign mem_cacop_ic = mem_cacop & (cacop_code_0[2:0] == 3'd0) & allow[MEM];

assign mem_crmd_tlb_cacop = mem_map_wr | mem_tlb | mem_cacop_ic;

always @(posedge clk) begin
    if (reset) begin
        wb_crmd_tlb_cacop <= 1'b0;
        wb_excp           <= 1'b0;
        wb_ertn           <= 1'b0;
        wb_excp_ertn      <= 1'b0;
    end
    else begin
        wb_crmd_tlb_cacop <= mem_crmd_tlb_cacop & cur_valid_0[MEM];
        wb_excp           <= mem_excp_0 & cur_valid_0[MEM];
        wb_ertn           <= csr_ctl_0[MEM][CSR_ERTN] & ~ mem_excp_0 & cur_valid_0[MEM];
        wb_excp_ertn      <= (csr_ctl_0[MEM][CSR_ERTN] | mem_excp_0) & cur_valid_0[MEM];
    end
end

// ===========================================================================
// WB
// ===========================================================================
assign wb_flush = wb_excp_ertn | wb_crmd_tlb_cacop;

// div handshake: entirely in MEM.  The operands were latched from ID, start is
// asserted for the first MEM cycle, and MEM waits until the divider drops busy.
assign div_is_signed = md_op_mem_0[DIV_W] | md_op_mem_0[MOD_W];
assign div_is_mod    = md_op_mem_0[MOD_W] | md_op_mem_0[MOD_WU];
assign div_result    = div_is_mod ? div_rem : div_quot;
assign div_start     = cur_div_pos_0 & ~ div_started & ~ div_busy;
assign div_reset     = reset | flush[MEM];

always @(posedge clk) begin
    if (div_reset)                     div_started <= 1'b0;
    else if (div_start & ~ allow[MEM]) div_started <= 1'b1;
    else if (allow[MEM])               div_started <= 1'b0;
end

div u_div(
    .clk       (clk            ),
    .reset     (div_reset      ),
    .start     (div_start      ),
    .is_signed (div_is_signed  ),
    .src1      (div_src1_mem   ),
    .src2      (div_src2_mem   ),
    .busy      (div_busy       ),
    .done      (               ),
    .quot      (div_quot       ),
    .rem       (div_rem        )
);

// WB result mux: the ID ALU result latched at the ID/MEM edge, the mul result
// (the CPA finishes here), the div result (ready when busy drops) or the load
// data (slot 0 only).  The ALU / load values are also forwarded from MEM, while
// mul / div are only forwarded from WB (see fw_valid_mem_*).
assign result_mem_0 = ({32{cur_mul_pos_0}} & mul_result_0) |
                      ({32{cur_div_pos_0}} & div_result) |
                      ({32{~ cur_mul_pos_0 & ~ cur_div_pos_0}} &
                       (res_from_mem_0 ? mem_result : alu_result_mem_0));

assign result_mem_1 = ({32{cur_mul_pos_1}} & mul_result_1) |
                      ({32{~ cur_mul_pos_1}} & alu_result_mem_1);

// The WB payload always mirrors MEM (allow[WB] is always 1); valid_wb_* is the
// shadow of valid_mem_* (see next_valid_wb_*), so an instruction that has to
// wait in MEM is written back exactly once.
always @(posedge clk) begin
    pc_0[WB]          <= pc_0[MEM];
    pc_1[WB]          <= pc_1[MEM];
    result_wb_0      <= result_mem_0;
    result_wb_1      <= result_mem_1;
    gr_we_0[WB]      <= gr_we_0[MEM];
    gr_we_1[WB]      <= gr_we_1[MEM];
    dest_0[WB]       <= dest_0[MEM];
    dest_1[WB]       <= dest_1[MEM];
    csr_num_0[WB]    <= csr_num_0[MEM];
    csr_ctl_0[WB]    <= mem_csr_ctl_v;
    tlb_op_0[WB]     <= tlb_op_0[MEM];
    invtlb_op_0[WB]  <= invtlb_op_0[MEM];
    excp_cause_0[WB] <= mem_excp_cause_0;
    csr_wdata_0[WB]  <= csr_wdata_0[MEM];
    csr_mask_0[WB]   <= csr_mask_0[MEM];
    wb_excp_badv     <= (excp_cause_0[MEM][EXCP_ALE] | (| mem_tlb_cause))
                        ? alu_result_mem_0 : pc_0[MEM];
end

// The write enables use the raw valid_*[WB] plus the registered wb_flush, not
// cur_valid_*[WB] / flush_slot_1_2[WB]: the RF write-first bypass is combinational into
// the ID branch compare, so rf_we must not depend on the flush vector (the
// single-issue core keeps rf_we register-only for the same reason).  The values
// are identical: cur_valid_1[WB] == valid_1[WB] & ~wb_flush.
assign rf_we_0    = gr_we_0[WB] && valid_0[WB] && ~ csr_ctl_0[WB][CSR_EXCP]
                  && ~ csr_ctl_0[WB][CSR_WB];
assign rf_waddr_0 = dest_0[WB];
assign rf_wdata_0 = result_wb_0;

// slot 1 is only alu / mul.  wb_flush is raised by slot 0 of this very pair, so
// slot 1 must not commit (the WB bit of flush_slot_1_2 says the same thing).
assign rf_we_1    = gr_we_1[WB] && valid_1[WB] && ~ wb_flush;
assign rf_waddr_1 = dest_1[WB];
assign rf_wdata_1 = result_wb_1;

// CSR GPR write: same address port as the pipeline write (the address is the
// registered dest[WB]), separate data port, no write-first read bypass.  The
// forwarding networks mark it pending and never forward csr_rdata.
assign rf_we_csr    = gr_we_0[WB] && valid_0[WB] && ~ csr_ctl_0[WB][CSR_EXCP]
                    && csr_ctl_0[WB][CSR_WB];
assign rf_wdata_csr = csr_ctl_0[WB][CSR_RD]      ? csr_rdata      :
                      csr_ctl_0[WB][CSR_RDCNTID] ? csr_rdata      :
                      csr_ctl_0[WB][CSR_RDCNTVL] ? timer_o[31: 0] :
                      csr_ctl_0[WB][CSR_RDCNTVH] ? timer_o[63:32] :
                                                 csr_rdata;

assign excp_ecode    = excp_cause_0[WB][EXCP_INT]  ? 6'h0 :
                       excp_cause_0[WB][EXCP_ALE]  ? 6'h9 :
                       excp_cause_0[WB][EXCP_ADEF] ? 6'h8 :
                       excp_cause_0[WB][EXCP_TLBR] ? 6'h3f :
                       excp_cause_0[WB][EXCP_PIF]  ? 6'h3 :
                       excp_cause_0[WB][EXCP_PIL]  ? 6'h1 :
                       excp_cause_0[WB][EXCP_PIS]  ? 6'h2 :
                       excp_cause_0[WB][EXCP_PPI]  ? 6'h7 :
                       excp_cause_0[WB][EXCP_PME]  ? 6'h4 :
                       excp_cause_0[WB][EXCP_IPE]  ? 6'he :
                       excp_cause_0[WB][EXCP_INE]  ? 6'hd :
                       excp_cause_0[WB][EXCP_BRK]  ? 6'hc :
                                                   6'hb;

assign excp_tlb      = excp_cause_0[WB][EXCP_TLBR] | excp_cause_0[WB][EXCP_PIF]
                     | excp_cause_0[WB][EXCP_PIL]  | excp_cause_0[WB][EXCP_PIS]
                     | excp_cause_0[WB][EXCP_PPI]  | excp_cause_0[WB][EXCP_PME];

assign excp_badv_en  = wb_excp & ~ excp_cause_0[WB][EXCP_INT]
                                & (excp_cause_0[WB][EXCP_ALE] | excp_cause_0[WB][EXCP_ADEF] | excp_tlb);

assign csr_we      = valid_0[WB] & csr_ctl_0[WB][CSR_WE] & ~ csr_ctl_0[WB][CSR_EXCP];
assign csr_raddr   = csr_num_0[WB];
assign csr_waddr   = csr_num_0[WB];
assign csr_wdata_w = csr_wdata_0[WB];

// ===========================================================================
// va --> pa translation
// ===========================================================================
assign tlb_s0_vppn     = pc_0[IF][31:13];
assign tlb_s0_va_bit12 = pc_0[IF][12];
assign tlb_s1_vppn     = alu_result_mem_0[31:13];
assign tlb_s1_va_bit12 = alu_result_mem_0[12];

assign tlb_rd_lo0 = {4'b0, tlb_r_ppn0, 1'b0, tlb_r_g, tlb_r_mat0, tlb_r_plv0, tlb_r_d0, tlb_r_v0};
assign tlb_rd_lo1 = {4'b0, tlb_r_ppn1, 1'b0, tlb_r_g, tlb_r_mat1, tlb_r_plv1, tlb_r_d1, tlb_r_v1};

assign tlb_csr_sel = tlb_op_0[WB][TLB_SRCH] ? 2'd1 :
                     tlb_op_0[WB][TLB_RD]   ? 2'd2 : 2'd0;

assign map_mode = crmd_pg_o & ~ crmd_da_o;

assign dmw0_hit_if  = map_mode & ((crmd_plv_o == 2'd0) ? dmw0_o[0] : dmw0_o[3]) & (pc_0[IF][31:29] == dmw0_o[31:29]);
assign dmw1_hit_if  = map_mode & ((crmd_plv_o == 2'd0) ? dmw1_o[0] : dmw1_o[3]) & (pc_0[IF][31:29] == dmw1_o[31:29]);
assign dmw0_hit_mem = map_mode & ((crmd_plv_o == 2'd0) ? dmw0_o[0] : dmw0_o[3]) & (alu_result_mem_0[31:29] == dmw0_o[31:29]);
assign dmw1_hit_mem = map_mode & ((crmd_plv_o == 2'd0) ? dmw1_o[0] : dmw1_o[3]) & (alu_result_mem_0[31:29] == dmw1_o[31:29]);

// MAT of the access: the dmw window wins, then the tlb entry, and when nothing
// is translated (PG=0 or DA=1: the va *is* the pa) the CRMD DATF / DATM field
// decides.  That last case is the one the func tests run in (CRMD=0x28:
// DA=1, DATF=01 cached fetch, DATM=00 uncached data), and it is what makes the
// confreg / UART window uncached -- without it those writes sit in the dcache.
assign if_mat  = if_dmw_hit  ? (dmw0_hit_if  ? dmw0_o[5:4] : dmw1_o[5:4])
                             : (if_xlat  ? tlb_s0_mat
                                         : (map_mode ? 2'b01 : crmd_datf_o));
assign mem_mat = mem_dmw_hit ? (dmw0_hit_mem ? dmw0_o[5:4] : dmw1_o[5:4])
                             : (mem_xlat ? tlb_s1_mat
                                         : (map_mode ? 2'b01 : crmd_datm_o));

assign if_dmw_hit  = dmw0_hit_if  | dmw1_hit_if;
assign mem_dmw_hit = dmw0_hit_mem | dmw1_hit_mem;
assign if_dmw_pa   = {(dmw0_hit_if  ? dmw0_o[27:25] : dmw1_o[27:25]), pc_0[IF][28:0]};
assign mem_dmw_pa  = {(dmw0_hit_mem ? dmw0_o[27:25] : dmw1_o[27:25]), alu_result_mem_0[28:0]};

assign if_xlat  = tlb_s0_found & map_mode;
assign mem_xlat = tlb_s1_found & map_mode;

assign if_fetch_cause = (if_xlat | (~ if_dmw_hit & map_mode)) ?
                        ((map_mode & ~ if_dmw_hit & ~ tlb_s0_found)            ? (13'h1 << EXCP_TLBR) :
                         (map_mode & ~ if_dmw_hit & ~ tlb_s0_v)                ? (13'h1 << EXCP_PIF ) :
                         (map_mode & ~ if_dmw_hit & (crmd_plv_o > tlb_s0_plv)) ? (13'h1 << EXCP_PPI ) : 13'h0)
                        : 13'h0;

// a hit-class cacop (code[4:3]=2) is translated like a load, so it may raise
// the tlb exceptions; store-tag / index invalidate use the va as it is
assign mem_tlb_chk = valid_0[MEM] & (mem_en_0 | (cacop_0 & cacop_code_0[4]))
                   & ~ csr_ctl_0[MEM][CSR_EXCP];
assign mem_tlb_cause = ({13{mem_tlb_chk}} &
                        ((map_mode & ~ mem_dmw_hit & ~ tlb_s1_found)            ? (13'h1 << EXCP_TLBR) :
                        (map_mode & ~ mem_dmw_hit & ~ tlb_s1_v)                 ? (13'h1 << (mem_we_0 ? EXCP_PIS : EXCP_PIL)) :
                        (map_mode & ~ mem_dmw_hit & (crmd_plv_o > tlb_s1_plv))  ? (13'h1 << EXCP_PPI ) :
                        (map_mode & ~ mem_dmw_hit & mem_we_0 & ~ tlb_s1_d)      ? (13'h1 << EXCP_PME ) :
                                                                                   13'h0));

assign excp_entry = excp_cause_0[WB][EXCP_TLBR] ? tlbrentry_o : eentry_o;

// ===========================================================================
// instances
// ===========================================================================
regfile u_regfile(
    .clk      (clk           ),
    .raddr1_0 (id_rf_raddr1_0),
    .rdata1_0 (id_rf_rdata1_0),
    .raddr2_0 (id_rf_raddr2_0),
    .rdata2_0 (id_rf_rdata2_0),
    .raddr1_1 (id_rf_raddr1_1),
    .rdata1_1 (id_rf_rdata1_1),
    .raddr2_1 (id_rf_raddr2_1),
    .rdata2_1 (id_rf_rdata2_1),
    .we_0     (rf_we_0       ),
    .waddr_0  (rf_waddr_0    ),
    .wdata_0  (rf_wdata_0    ),
    .we_1     (rf_we_1       ),
    .waddr_1  (rf_waddr_1    ),
    .wdata_1  (rf_wdata_1    ),
    .we_csr   (rf_we_csr     ),
    .wdata_csr(rf_wdata_csr  )
);

tlb u_tlb(
    .clk          (clk            ),
    .reset        (reset          ),

    .s0_vppn      (tlb_s0_vppn    ),
    .s0_va_bit12  (tlb_s0_va_bit12),
    .s0_asid      (asid_o         ),
    .s0_found     (tlb_s0_found   ),
    .s0_index     (               ),
    .s0_ppn       (tlb_s0_ppn     ),
    .s0_ps        (tlb_s0_ps      ),
    .s0_plv       (tlb_s0_plv     ),
    .s0_mat       (tlb_s0_mat     ),
    .s0_d         (               ),
    .s0_v         (tlb_s0_v       ),

    .s1_vppn      (tlb_s1_vppn    ),
    .s1_va_bit12  (tlb_s1_va_bit12),
    .s1_asid      (asid_o         ),
    .s1_found     (tlb_s1_found   ),
    .s1_index     (               ),
    .s1_ppn       (tlb_s1_ppn     ),
    .s1_ps        (tlb_s1_ps      ),
    .s1_plv       (tlb_s1_plv     ),
    .s1_mat       (tlb_s1_mat     ),
    .s1_d         (tlb_s1_d       ),
    .s1_v         (tlb_s1_v       ),

    .s2_vppn      (tlbehi_vppn_o  ),
    .s2_asid      (asid_o         ),
    .s2_found     (tlb_s2_found   ),
    .s2_index     (tlb_s2_index   ),

    .invtlb_valid (wb_tlb_inv     ),
    .invtlb_op    (invtlb_op_0[WB]),

    .we           (wb_tlb_wr      ),
    .w_index      (tlb_w_index    ),
    .w_e          (tlb_w_e        ),
    .w_vppn       (tlb_w_vppn     ),
    .w_ps         (tlbidx_ps_o    ),
    .w_asid       (tlb_w_asid     ),
    .w_g          (tlb_w_g        ),
    .w_ppn0       (tlb_w_ppn0     ),
    .w_plv0       (tlb_w_plv0     ),
    .w_mat0       (tlb_w_mat0     ),
    .w_d0         (tlb_w_d0       ),
    .w_v0         (tlb_w_v0       ),
    .w_ppn1       (tlb_w_ppn1     ),
    .w_plv1       (tlb_w_plv1     ),
    .w_mat1       (tlb_w_mat1     ),
    .w_d1         (tlb_w_d1       ),
    .w_v1         (tlb_w_v1       ),

    .r_index      (tlbidx_index_o ),
    .r_e          (tlb_r_e        ),
    .r_vppn       (tlb_r_vppn     ),
    .r_ps         (tlb_r_ps       ),
    .r_asid       (tlb_r_asid     ),
    .r_g          (tlb_r_g        ),
    .r_ppn0       (tlb_r_ppn0     ),
    .r_plv0       (tlb_r_plv0     ),
    .r_mat0       (tlb_r_mat0     ),
    .r_d0         (tlb_r_d0       ),
    .r_v0         (tlb_r_v0       ),
    .r_ppn1       (tlb_r_ppn1     ),
    .r_plv1       (tlb_r_plv1     ),
    .r_mat1       (tlb_r_mat1     ),
    .r_d1         (tlb_r_d1       ),
    .r_v1         (tlb_r_v1       )
);

csr u_csr(
    .clk             (clk               ),
    .reset           (reset             ),
    .raddr_i         (csr_raddr         ),
    .rdata_o         (csr_rdata         ),
    .we_i            (csr_we            ),
    .waddr_i         (csr_waddr         ),
    .wdata_i         (csr_wdata_w       ),
    .wmask_i         (csr_mask_0[WB]    ),
    .excp_i          (wb_excp           ),
    .excp_pc_i       (pc_0[WB]          ),
    .excp_ecode_i    (excp_ecode        ),
    .excp_esubcode_i (9'h0              ),
    .excp_badv_en_i  (excp_badv_en      ),
    .excp_badv_i     (wb_excp_badv      ),
    .excp_tlb_i      (wb_excp & excp_tlb),
    .ertn_i          (wb_ertn           ),
    .hwi_i           (8'h0              ),
    .ipi_i           (1'b0              ),
    .tlb_we_i        (wb_tlb_commit     ),
    .tlb_sel_i       (tlb_csr_sel       ),
    .tlb_srch_hit_i  (tlb_s2_found      ),
    .tlb_srch_index_i(tlb_s2_index      ),
    .tlb_rd_e_i      (tlb_r_e           ),
    .tlb_rd_vppn_i   (tlb_r_vppn        ),
    .tlb_rd_ps_i     (tlb_r_ps          ),
    .tlb_rd_asid_i   (tlb_r_asid        ),
    .tlb_rd_lo0_i    (tlb_rd_lo0        ),
    .tlb_rd_lo1_i    (tlb_rd_lo1        ),

    .tlbidx_index_o  (tlbidx_index_o),
    .tlbidx_ps_o     (tlbidx_ps_o   ),
    .tlbidx_ne_o     (tlbidx_ne_o   ),
    .tlbehi_vppn_o   (tlbehi_vppn_o ),
    .tlbelo0_o       (tlbelo0_o     ),
    .tlbelo1_o       (tlbelo1_o     ),
    .asid_o          (asid_o        ),
    .tlbrentry_o     (tlbrentry_o   ),
    .estat_ecode_o   (estat_ecode_o ),
    .crmd_da_o       (crmd_da_o     ),
    .crmd_datf_o     (crmd_datf_o   ),
    .crmd_datm_o     (crmd_datm_o   ),
    .crmd_pg_o       (crmd_pg_o     ),

    .crmd_plv_o      (crmd_plv_o   ),
    .dmw0_o          (dmw0_o       ),
    .dmw1_o          (dmw1_o       ),
    .crmd_ie_o       (crmd_ie_o    ),
    .eentry_o        (eentry_o     ),
    .era_o           (era_o        ),
    .int_o           (int_o        ),
    .timer_o         (timer_o      )
);

assign wb_tlb_commit = valid_0[WB] & (| tlb_op_0[WB]) & ~ csr_ctl_0[WB][CSR_EXCP];
assign wb_tlb_srch   = wb_tlb_commit & tlb_op_0[WB][TLB_SRCH];
assign wb_tlb_rd     = wb_tlb_commit & tlb_op_0[WB][TLB_RD];
assign wb_tlb_wr     = wb_tlb_commit & (tlb_op_0[WB][TLB_WR] | tlb_op_0[WB][TLB_FILL]);
assign wb_tlb_inv    = wb_tlb_commit & tlb_op_0[WB][TLB_INV];

assign tlb_w_index = tlb_op_0[WB][TLB_FILL] ? timer_o[3:0] : tlbidx_index_o;
assign tlb_w_e     = (estat_ecode_o == 6'h3f) ? 1'b1 : ~ tlbidx_ne_o;
assign tlb_w_vppn  = wb_tlb_inv ? csr_wdata_0[WB][31:13] : tlbehi_vppn_o;
assign tlb_w_asid  = wb_tlb_inv ? csr_mask_0[WB][9:0]    : asid_o;
assign tlb_w_g     = tlbelo0_o[6] & tlbelo1_o[6];
assign tlb_w_ppn0  = tlbelo0_o[27:8];
assign tlb_w_plv0  = tlbelo0_o[3:2];
assign tlb_w_mat0  = tlbelo0_o[5:4];
assign tlb_w_d0    = tlbelo0_o[1];
assign tlb_w_v0    = tlbelo0_o[0];
assign tlb_w_ppn1  = tlbelo1_o[27:8];
assign tlb_w_plv1  = tlbelo1_o[3:2];
assign tlb_w_mat1  = tlbelo1_o[5:4];
assign tlb_w_d1    = tlbelo1_o[1];
assign tlb_w_v1    = tlbelo1_o[0];

mmu u_mmu(
    .clk            (clk            ),
    .reset          (reset          ),

    .if_pre         (preif_req      ),
    .if_va          (pc_0[IF]          ),
    .if_allow       (allow[IF]       ),
    .if_pending     (if_pending     ),
    .if_rdata_0     (inst_rdata_0   ),
    .if_rdata_1     (inst_rdata_1   ),

    .mem_pre        (premem_req     ),
    .mem_va         (mem_va         ),
    .mem_wstrb      (mem_wstrb      ),
    .mem_stdata     (mem_stdata     ),
    .mem_abort      (mem_abort      ),
    .mem_allow      (allow[MEM]      ),
    .mem_pending    (mem_pending    ),
    .mem_rdata      (mem_rdata      ),

    .crmd_plv       (crmd_plv_o     ),

    .if_xlat        (if_xlat        ),
    .if_ppn         (tlb_s0_ppn     ),
    .if_ps          (tlb_s0_ps      ),
    .if_dmw_hit     (if_dmw_hit     ),
    .if_dmw_pa      (if_dmw_pa      ),
    .if_abort       (if_abort       ),
    .mem_xlat       (mem_xlat       ),
    .mem_ppn        (tlb_s1_ppn     ),
    .mem_ps         (tlb_s1_ps      ),
    .mem_dmw_hit    (mem_dmw_hit    ),
    .mem_dmw_pa     (mem_dmw_pa     ),

    .if_pre_addr        (nextpc             ),   // VIPT index, one cycle early
    .mem_pre_addr       (alu_result_id_0    ),
    .mem_cacop          (mem_cacop          ),
    .mem_cacop_code     (mem_cacop_code     ),
    .if_mat             (if_mat             ),
    .mem_mat            (mem_mat            ),

    .arid           (arid           ),
    .araddr         (araddr         ),
    .arlen          (arlen          ),
    .arsize         (arsize         ),
    .arburst        (arburst        ),
    .arlock         (arlock         ),
    .arcache        (arcache        ),
    .arprot         (arprot         ),
    .arvalid        (arvalid        ),
    .arready        (arready        ),

    .rid            (rid            ),
    .rdata          (rdata          ),
    .rresp          (rresp          ),
    .rlast          (rlast          ),
    .rvalid         (rvalid         ),
    .rready         (rready         ),

    .awid           (awid           ),
    .awaddr         (awaddr         ),
    .awlen          (awlen          ),
    .awsize         (awsize         ),
    .awburst        (awburst        ),
    .awlock         (awlock         ),
    .awcache        (awcache        ),
    .awprot         (awprot         ),
    .awvalid        (awvalid        ),
    .awready        (awready        ),

    .wid            (wid            ),
    .wdata          (wdata          ),
    .wstrb          (wstrb          ),
    .wlast          (wlast          ),
    .wvalid         (wvalid         ),
    .wready         (wready         ),

    .bid            (bid            ),
    .bresp          (bresp          ),
    .bvalid         (bvalid         ),
    .bready         (bready         )
);

// ===========================================================================
// debug / trace
// ===========================================================================

assign debug_wb_pc       = pc_0[WB];
// the CSR GPR write uses the dedicated CSR data port
assign debug_wb_rf_we    = {4{rf_we_0 | rf_we_csr}};
assign debug_wb_rf_wnum  = rf_waddr_0;
assign debug_wb_rf_wdata = rf_we_csr ? rf_wdata_csr : rf_wdata_0;

assign debug_wb_pc_1       = pc_1[WB];
assign debug_wb_rf_we_1    = {4{rf_we_1}};
assign debug_wb_rf_wnum_1  = rf_waddr_1;
assign debug_wb_rf_wdata_1 = rf_wdata_1;

endmodule
