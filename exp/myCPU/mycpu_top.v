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

    // trace debug interface
    output wire [31:0] debug_wb_pc,
    output wire [ 3:0] debug_wb_rf_we,
    output wire [ 4:0] debug_wb_rf_wnum,
    output wire [31:0] debug_wb_rf_wdata
);


wire clk    = aclk;
wire resetn = aresetn;

// mmu: the fetch / memory-access stage controller.
wire        preif_req;         // a fetch enters IF
wire [31:0] inst_rdata;        // returns after an arbitrary latency
wire        premem_req;        // a load / store in ID that enters MEM at this edge
wire [31:0] mem_va;            // its virtual address (= alu_result_mem)
wire        mem_abort;         // this access has been found illegal
wire [ 3:0] mem_wstrb;         // byte strobes; 4'b0 means read
wire [31:0] mem_stdata;        // store data, already spread over the byte lanes
wire [31:0] mem_rdata;
wire        if_pending;        // from the mmu: this fetch should wait
wire        mem_pending;       // from the mmu: this access should wait

// pipeline: IF -> ID -> MEM -> WB
//
// ID and EX are merged: the ALU (and the stage-0 Booth/Wallace tree of the
// multiplier) runs in ID on the operands the ID forwarding network has already
// resolved, and the ALU result is registered into MEM.  MEM is therefore both
// the memory-access stage and the "result select" stage (ALU / mul CPA / div /
// load data), exactly as it used to be one cycle after EX.  The divider is
// started from MEM and its result is consumed there, so it no longer has any
// handshake with an EX slot.

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

// excp_cause bit positions: one-hot
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

// pipeline
reg  [TOTAL_STAGE_WIDTH - 1: 0] valid /* verilator split_var */;
wire [TOTAL_STAGE_WIDTH - 1: 0] stall;
wire [TOTAL_STAGE_WIDTH - 1: 0] allow;
wire [TOTAL_STAGE_WIDTH - 1: 0] flush;
wire [TOTAL_STAGE_WIDTH - 1: 0] cur_valid;
wire [TOTAL_STAGE_WIDTH - 1: 0] next_valid;

// unit position: the instruction born in ID carries its mul / div flag into
// MEM, where the product / quotient appears.  The ALU flag is no longer needed
// (an ALU result is simply alu_result_mem at MEM).
reg             mul_pos;
reg             div_pos;
wire            cur_mul_pos;
wire            cur_div_pos;
wire            next_mul_pos;
wire            next_div_pos;

wire [31:0] nextpc;
(* keep = "true" *) wire [31:0] seq_pc;
wire        br_taken_id;    // any taken branch / jump, decided at ID
(* keep = "true" *) wire [31:0] br_target_id;
wire        br_redirect;

// stage payload registers
reg  [31:0] pc [WB:IF] /* verilator split_var */;
reg  [31:0] inst;
reg  [31:0] br_pc_reg;
reg         br_reg;

// ID -> MEM: combinational at ID
wire [11:0] id_alu_op;
wire [MOD_WU:0] id_md_op;
wire        id_src1_used, id_src2_used, id_res_from_mem;
wire        id_mem_we, id_mem_en, id_mem_zext, id_gr_we;
wire [ 2:0] id_mem_size;
wire [ 4:0] id_dest, id_tlb_op, id_invtlb_op;
wire [ 4:0] id_cacop_code;
wire [13:0] id_csr_num;
wire [ 8:0] id_csr_ctl;
wire [12:0] id_excp_cause;
wire        id_src1_is_pc, id_src2_is_imm, id_csr_mask_ones;
wire [31:0] id_rf_rdata1, id_rf_rdata2;
wire [ 4:0] id_rf_raddr1, id_rf_raddr2;
wire [31:0] id_imm;
wire [31:0] id_src1, id_src2;
wire [31:0] alu_result_id;

// MEM stage registers: the ID ALU result and the mul/div select travel with the
// instruction into MEM, where the result mux below picks the final value
reg  [31:0] alu_result_mem;
reg  [MOD_WU:0] md_op_mem;

// div / mod operands are resolved in ID (like the branch operands) and latched
// into MEM together with the instruction; the divider is a MEM-stage unit now
reg  [31:0] div_src1_mem, div_src2_mem;

// payload arrays (birth stage .. last use).  With EX merged into ID the
// mul/div/ALU payload below has a single registered slot (MEM).
reg         res_from_mem [MEM:MEM] /* verilator split_var */;
reg         mem_we [MEM:MEM] /* verilator split_var */;
reg         mem_en [MEM:MEM] /* verilator split_var */;
reg  [31:0] mem_wdata_mem /* verilator split_var */;
reg  [ 2:0] mem_size [MEM:MEM] /* verilator split_var */;
reg         mem_zext [MEM:MEM] /* verilator split_var */;
reg         gr_we [WB:MEM] /* verilator split_var */;
reg  [ 4:0] dest [WB:MEM] /* verilator split_var */;
reg  [13:0] csr_num [WB:MEM] /* verilator split_var */;
reg  [ 8:0] csr_ctl [WB:MEM] /* verilator split_var */;
reg  [ 4:0] tlb_op [WB:MEM] /* verilator split_var */;
reg  [ 4:0] invtlb_op [WB:MEM] /* verilator split_var */;
reg         cacop [MEM:MEM] /* verilator split_var */;
reg  [ 4:0] cacop_code [MEM:MEM] /* verilator split_var */;
reg  [31:0] csr_wdata [WB:MEM] /* verilator split_var */;
reg  [31:0] csr_mask [WB:MEM] /* verilator split_var */;
reg  [12:0] excp_cause [WB:IF] /* verilator split_var */;

// MEM result mux (ALU / mul / div / load data): feeds WB and the forwarding
// networks; result_wb is what the WB stage actually holds.
wire [31:0] result_mem;
reg  [31:0] result_wb;

// CSR file interconnect
wire [31:0] csr_rdata;
wire        csr_we;
wire [13:0] csr_raddr;
wire [13:0] csr_waddr;
wire [31:0] csr_wdata_w;
wire [31:0] eentry_o;
wire [31:0] era_o;
wire [63:0] timer_o;

/* verilator lint_off UNUSEDSIGNAL */
wire [ 1:0] crmd_plv_o;
wire        crmd_ie_o;
/* verilator lint_on UNUSEDSIGNAL */
wire        int_o;
wire        preif_adef;

// exception / ertn / interrupt
wire        mem_excp;
wire        wb_flush;
wire        mem_ipe;
wire [ 8:0] mem_csr_ctl;
wire [12:0] mem_excp_cause;
reg         wb_excp_ertn;
reg         wb_excp;
reg         wb_ertn;
reg         wb_crmd_tlb_cacop;
wire        mem_crmd_tlb_cacop;
wire        mem_tlb;
wire        mem_cacop_ic;
wire [ 5:0] excp_ecode;
wire        excp_badv_en;
reg  [31:0] wb_excp_badv;

// TLB / DMW translation and the exceptions it can raise
wire        map_mode;
wire        if_xlat, mem_xlat;
wire        inst_cacop;
wire        mem_cacop;
wire [ 4:0] mem_cacop_code;
wire [ 1:0] if_mat, mem_mat;
wire [ 1:0] tlb_s0_mat, tlb_s1_mat;
wire        if_dmw_hit, mem_dmw_hit;
wire [31:0] if_dmw_pa, mem_dmw_pa;
wire        dmw0_hit_if, dmw1_hit_if, dmw0_hit_mem, dmw1_hit_mem;
wire [31:0] dmw0_o, dmw1_o;
wire [12:0] if_fetch_cause;
wire [12:0] if_excp_cause;
wire        if_abort;
wire        mem_tlb_chk;
wire [12:0] mem_tlb_cause;
wire        mem_map_wr;
wire [31:0] excp_entry;
wire        excp_tlb;

// ID -> MEM hand-off (combinational in ID, registered into MEM)
wire        premem_ale;
wire        premem_int_take;
wire        premem_excp;
wire [ 8:0] premem_csr_ctl;
wire [12:0] premem_excp_cause;

// fw src: the value MEM holds last cycle is the only register-level source a
// consumer in ID can see in time (WB is covered by the RF write-first bypass).
wire        fw_valid_mem;
wire [31:0] fw_data_mem;

// CSR-write-RF dependency: any in-flight stage writing the RF from CSR
reg         csr_dep;

// ID forwarding network: MEM -> ID.  Operands are resolved (and waited for)
// here because the ALU / mul / div operands are all consumed at ID now.
wire        id_fw1_hit_mem;
wire        id_fw1_pending;
wire        id_fw1_sel_mem, id_fw1_sel_rf;
wire [31:0] id_fw1_data;
wire        id_fw2_hit_mem;
wire        id_fw2_pending;
wire        id_fw2_sel_mem, id_fw2_sel_rf;
wire [31:0] id_fw2_data;
wire        id_fw_pending;
wire        id_br_valid;
reg         id_br_sent;

// unit results
wire [31:0] mul_result;
wire [31:0] div_result;
wire        div_busy;
wire [31:0] div_quot, div_rem;
wire        div_is_signed, div_is_mod;
wire        div_start;
reg         div_started;
wire        div_reset;

// branch unit at ID
wire [ 6:0] id_br_op;
wire [31:0] id_br_base, id_br_offs;

// memory / CSR helpers
wire        mem_byte, mem_half, mem_word;
wire [ 3:0] st_we_byte, st_we_half, st_we_word, st_we;
wire [ 7:0] ld_byte;
wire [15:0] ld_half;
wire [31:0] ld_byte_ext, ld_half_ext;
wire [31:0] mem_result;

// regfile
wire        rf_we, rf_we_csr;
wire [ 4:0] rf_waddr;
wire [31:0] rf_wdata, rf_wdata_csr;

// the top of the branch compare
wire        sel_era, sel_eentry, sel_wb_seq, sel_br, sel_cur_br, sel_seq;

// ID decode helpers
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
wire [12:0] excp_cause_id;

// tlb bus (kept from the original)
wire        tlb_s0_found, tlb_s1_found;
wire [19:0] tlb_s0_ppn,   tlb_s1_ppn;
wire [ 5:0] tlb_s0_ps,    tlb_s1_ps;
wire        tlb_s2_found;
wire [ 3:0] tlb_s2_index;
wire        tlb_s0_v, tlb_s1_v, tlb_s1_d;
wire [ 1:0] tlb_s0_plv, tlb_s1_plv;
wire        tlb_s0_va_bit12, tlb_s1_va_bit12;
wire [18:0] tlb_s0_vppn,     tlb_s1_vppn;
wire        tlb_r_e, tlb_r_g, tlb_r_d0, tlb_r_d1, tlb_r_v0, tlb_r_v1;
wire [18:0] tlb_r_vppn;
wire [ 5:0] tlb_r_ps;
wire [ 9:0] tlb_r_asid;
wire [19:0] tlb_r_ppn0, tlb_r_ppn1;
wire [ 1:0] tlb_r_plv0, tlb_r_plv1, tlb_r_mat0, tlb_r_mat1;
wire [ 3:0] tlb_w_index;
wire        tlb_w_e, tlb_w_g, tlb_w_d0, tlb_w_d1, tlb_w_v0, tlb_w_v1;
wire [18:0] tlb_w_vppn;
wire [ 9:0] tlb_w_asid;
wire [19:0] tlb_w_ppn0, tlb_w_ppn1;
wire [ 1:0] tlb_w_plv0, tlb_w_plv1, tlb_w_mat0, tlb_w_mat1;
wire [31:0] tlb_rd_lo0, tlb_rd_lo1;
wire        wb_tlb_commit, wb_tlb_srch, wb_tlb_rd, wb_tlb_wr, wb_tlb_inv;
wire [ 1:0] tlb_csr_sel;
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


// redirect logic
assign seq_pc       = pc[IF] + 32'h4;

assign sel_era      = wb_ertn;
assign sel_eentry   = wb_excp;
assign sel_wb_seq   = ~ wb_excp_ertn & wb_crmd_tlb_cacop;
assign sel_br       = ~ wb_excp_ertn & ~ wb_crmd_tlb_cacop & br_reg;
assign sel_cur_br   = ~ wb_excp_ertn & ~ wb_crmd_tlb_cacop & ~ br_reg & br_taken_id;
assign sel_seq      = ~ (wb_excp_ertn | wb_crmd_tlb_cacop | br_reg | br_taken_id);

assign nextpc       = ({32{sel_era    }} & era_o       ) |
                      ({32{sel_eentry }} & excp_entry  ) |
                      ({32{sel_wb_seq }} & (pc[WB] + 32'h4)) |
                      ({32{sel_br     }} & br_pc_reg   ) |
                      ({32{sel_cur_br }} & br_target_id) |
                      ({32{sel_seq    }} & seq_pc      );

assign br_redirect = br_taken_id | wb_excp_ertn | wb_crmd_tlb_cacop;

always @(posedge clk) begin
    if (reset) begin
        br_reg <= 1'b0;
        br_pc_reg <= 32'b0;
    end
    else begin
        if (allow[IF]) begin
            br_reg <= 1'b0;
            br_pc_reg <= 32'b0;
        end
        else if (br_redirect && ~ allow[IF]) begin
            br_reg <= 1'b1;
            br_pc_reg <= nextpc;
        end
    end
end

always @(posedge clk) begin
    if (reset) begin
        pc[IF]  <= 32'h1bfffffc; // trick: first fetched pc is 0x1c000000
        pc[ID]  <= 32'h0;
        pc[MEM] <= 32'h0;
        pc[WB]  <= 32'h0;
    end
    else begin
        if (allow[IF] ) pc[IF]  <= nextpc;
        if (allow[ID] ) pc[ID]  <= pc[IF];
        if (allow[MEM]) pc[MEM] <= pc[ID];
        if (allow[WB] ) pc[WB]  <= pc[MEM];
    end
end

// pipeline control
assign cur_valid = valid & ~ flush;
assign next_valid = {cur_valid[MEM:IF] & ~ stall[MEM:IF], 1'b1} & allow | (~ allow & cur_valid);

always @(posedge clk) begin
    if (reset) valid <= 4'b0;
    else       valid <= next_valid;
end

assign flush    = wb_flush    ? 4'b0111 :
                  br_taken_id ? 4'b0001 :
                                4'b0000;

assign allow[WB]  = 1'b1;
assign allow[MEM] = ~ stall[MEM] & allow[WB];
assign allow[ID]  = ~ stall[ID]  & allow[MEM];
assign allow[IF]  = ~ stall[IF]  & allow[ID];

// ID now owns the ALU / mul / div operands, so it waits for every unresolved
// source (id_fw_pending already folds in the CSR-write dependency).  MEM waits
// for the mmu and for the divider, which both finish there.
assign stall[IF]  = if_pending;
assign stall[ID]  = cur_valid[ID] & id_fw_pending;
assign stall[MEM] = mem_pending | (cur_div_pos & (div_busy | ~ div_started));
assign stall[WB]  = 1'b0;

// unit position flags: born in ID, used in MEM
assign cur_mul_pos = mul_pos & ~ flush[MEM];
assign cur_div_pos = div_pos & ~ flush[MEM];

assign next_mul_pos = (cur_valid[ID] & ~ stall[ID] & (| id_md_op[2:0]) & allow[MEM])
                    | (~ allow[MEM] & cur_mul_pos);
assign next_div_pos = (cur_valid[ID] & ~ stall[ID] & (| id_md_op[6:3]) & allow[MEM])
                    | (~ allow[MEM] & cur_div_pos);

always @(posedge clk) begin
    if (reset) begin
        mul_pos <= 1'b0;
        div_pos <= 1'b0;
    end
    else begin
        mul_pos <= next_mul_pos;
        div_pos <= next_div_pos;
    end
end

// preif
assign preif_adef  = | nextpc[1:0];
assign preif_req   = ~ preif_adef;

// IF
assign if_excp_cause = excp_cause[IF] | if_fetch_cause;
assign if_abort    = | if_fetch_cause | flush[IF];

always @(posedge clk) begin
    if (allow[ID]) begin
        if (|if_excp_cause)  inst <= 32'h0340_0000; // nop = andi $r0, $r0, 0
        else                 inst <= inst_rdata;
    end
end

always @(posedge clk) begin
    if (reset) begin
        excp_cause[ID] <= 13'h0;
    end
    else if (allow[ID]) begin
        excp_cause[ID] <= if_excp_cause;
    end
end

always @(posedge clk) begin
    if (reset) begin
        excp_cause[IF] <= 13'h0;
    end
    else if (allow[IF]) begin
        excp_cause[IF] <= preif_adef ? (13'h1 << EXCP_ADEF) : 13'h0;
    end
end

// ID decode
assign op_31_26  = inst[31:26];
assign op_25_22  = inst[25:22];
assign op_21_20  = inst[21:20];
assign op_19_15  = inst[19:15];

assign rd   = inst_rdcntid ? inst[9:5] : inst[4:0];
assign rj   = inst[ 9: 5];
assign rk   = inst[14:10];

assign i12  = inst[21:10];
assign i20  = inst[24: 5];
assign i16  = inst[25:10];
assign i26  = {inst[ 9: 0], inst[25:10]};

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
assign inst_lu12i_w= op_31_26_d[6'h05] & ~inst[25];

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

assign inst_csr_any = op_31_26_d[6'h01] & ~ inst[25] & ~ inst[24];
assign inst_csrrd   = inst_csr_any & (inst[9:5] == 5'd0);
assign inst_csrwr   = inst_csr_any & (inst[9:5] == 5'd1);
assign inst_csrxchg = inst_csr_any & ~ inst_csrrd & ~ inst_csrwr;
assign inst_ertn    = op_31_26_d[6'h01] & inst[25] & ~ inst[24] & op_19_15_d[5'h10]
                    & (inst[14:10] == 5'b01110);
assign inst_idle    = op_31_26_d[6'h01] & inst[25] & ~ inst[24] & op_19_15_d[5'h11];
assign inst_tlbsrch = op_31_26_d[6'h01] & inst[25] & ~ inst[24] & op_19_15_d[5'h10]
                    & (inst[14:10] == 5'b01010);
assign inst_tlbrd   = op_31_26_d[6'h01] & inst[25] & ~ inst[24] & op_19_15_d[5'h10]
                    & (inst[14:10] == 5'b01011);
assign inst_tlbwr   = op_31_26_d[6'h01] & inst[25] & ~ inst[24] & op_19_15_d[5'h10]
                    & (inst[14:10] == 5'b01100);
assign inst_tlbfill = op_31_26_d[6'h01] & inst[25] & ~ inst[24] & op_19_15_d[5'h10]
                    & (inst[14:10] == 5'b01101);
assign inst_invtlb  = op_31_26_d[6'h01] & inst[25] & ~ inst[24] & op_19_15_d[5'h13];
assign inst_tlb_any = inst_tlbsrch | inst_tlbrd | inst_tlbwr | inst_tlbfill | inst_invtlb;

// cacop code, rj, si12  (opcode 0000 0110 00; code[2:0]: 0 icache / 1 dcache,
// code[4:3]: 0 store tag / 1 index invalidate / 2 hit invalidate)
assign inst_cacop    = op_31_26_d[6'h01] & op_25_22_d[4'h8];
assign id_cacop_code = inst[4:0];

assign inst_syscall = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h16];
assign inst_break   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h14];

assign inst_rdcntvl_w = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h0] & op_19_15_d[5'h00]
                      & (inst[14:10] == 5'b11000) & (inst[9:5] == 5'd0);
assign inst_rdcntvh_w = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h0] & op_19_15_d[5'h00]
                      & (inst[14:10] == 5'b11001) & (inst[9:5] == 5'd0);
assign inst_rdcntid   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h0] & op_19_15_d[5'h00]
                      & (inst[14:10] == 5'b11000) & (inst[4:0] == 5'd0) & ~ inst_rdcntvl_w;

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
                   | inst_tlb_any | inst_cacop
                   | inst_rdcntvl_w | inst_rdcntvh_w | inst_rdcntid);

assign id_src1_used = inst_add_w | inst_sub_w | inst_slt | inst_sltu
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
                  | inst_invtlb | inst_cacop;
assign id_src2_used = inst_add_w | inst_sub_w | inst_slt | inst_sltu
                  | inst_and | inst_or | inst_nor | inst_xor
                  | inst_st_w | inst_beq | inst_bne
                  | inst_sll_w | inst_srl_w | inst_sra_w
                  | inst_mul_w | inst_mulh_w | inst_mulh_wu
                  | inst_div_w | inst_mod_w | inst_div_wu | inst_mod_wu
                  | inst_blt | inst_bge | inst_bltu | inst_bgeu
                  | inst_st_b | inst_st_h
                  | inst_csrwr | inst_csrxchg
                  | inst_invtlb;

assign id_alu_op[ 0] = inst_add_w | inst_addi_w | inst_ld_w | inst_st_w
                  | inst_jirl | inst_bl | inst_pcaddu12i
                  | inst_ld_b | inst_ld_h | inst_ld_bu | inst_ld_hu
                  | inst_st_b | inst_st_h
                  | inst_cacop;                            // cacop address = rj + si12
assign id_alu_op[ 1] = inst_sub_w;
assign id_alu_op[ 2] = inst_slt | inst_slti;
assign id_alu_op[ 3] = inst_sltu | inst_sltui;
assign id_alu_op[ 4] = inst_and | inst_andi;
assign id_alu_op[ 5] = inst_nor;
assign id_alu_op[ 6] = inst_or | inst_ori;
assign id_alu_op[ 7] = inst_xor | inst_xori;
assign id_alu_op[ 8] = inst_slli_w | inst_sll_w;
assign id_alu_op[ 9] = inst_srli_w | inst_srl_w;
assign id_alu_op[10] = inst_srai_w | inst_sra_w;
assign id_alu_op[11] = inst_lu12i_w;

assign id_md_op[MUL_W]   = inst_mul_w;
assign id_md_op[MULH_W]  = inst_mulh_w;
assign id_md_op[MULH_WU] = inst_mulh_wu;
assign id_md_op[DIV_W]   = inst_div_w;
assign id_md_op[MOD_W]   = inst_mod_w;
assign id_md_op[DIV_WU]  = inst_div_wu;
assign id_md_op[MOD_WU]  = inst_mod_wu;

assign id_res_from_mem = inst_ld_w | inst_ld_b | inst_ld_h | inst_ld_bu | inst_ld_hu;
assign id_mem_we       = inst_st_w | inst_st_b | inst_st_h;
assign id_mem_en       = id_mem_we | id_res_from_mem;
assign id_mem_size[SIZE_BYTE] = inst_ld_b | inst_ld_bu | inst_st_b;
assign id_mem_size[SIZE_HALF] = inst_ld_h | inst_ld_hu | inst_st_h;
assign id_mem_size[SIZE_WORD] = inst_ld_w | inst_st_w;
assign id_mem_zext     = inst_ld_bu | inst_ld_hu;

assign id_csr_num = inst_rdcntid ? 14'h040 : inst[23:10];
assign id_excp_cause = excp_cause[ID] | excp_cause_id;
// hit-class cacop (code[4:3]=2, i.e. 0x10 / 0x11) is allowed at PLV3, every
// other cacop is privileged
assign id_csr_ctl[CSR_PRIV] = inst_csr_any | inst_ertn | inst_idle | inst_tlb_any
                            | (inst_cacop & ~ (inst[4] & ~ inst[3]));
assign id_tlb_op = {inst_invtlb, inst_tlbfill, inst_tlbwr, inst_tlbrd, inst_tlbsrch};
assign id_csr_ctl[CSR_RD] = inst_csrrd | inst_csrwr | inst_csrxchg;
assign id_csr_ctl[CSR_WE] = inst_csrwr | inst_csrxchg;
assign id_csr_ctl[CSR_ERTN] = inst_ertn;
assign id_csr_ctl[CSR_RDCNTVL] = inst_rdcntvl_w;
assign id_csr_ctl[CSR_RDCNTVH] = inst_rdcntvh_w;
assign id_csr_ctl[CSR_RDCNTID] = inst_rdcntid;
assign id_csr_ctl[CSR_WB] = inst_csrrd | inst_csrwr | inst_csrxchg
                           | inst_rdcntvl_w | inst_rdcntvh_w | inst_rdcntid;
assign id_csr_ctl[CSR_EXCP] = | id_excp_cause;
assign id_invtlb_op = inst[4:0];
assign id_csr_mask_ones = inst_csrwr;

assign excp_cause_id[EXCP_SYS]  = inst_syscall;
assign excp_cause_id[EXCP_BRK]  = inst_break;
assign excp_cause_id[EXCP_INE]  = inst_ine | (inst_invtlb & (inst[4:0] > 5'd6));
assign excp_cause_id[EXCP_ADEF] = 1'b0;
assign excp_cause_id[EXCP_ALE]  = 1'b0;
assign excp_cause_id[EXCP_INT]  = 1'b0;
assign excp_cause_id[EXCP_IPE]  = 1'b0;
assign excp_cause_id[EXCP_TLBR] = 1'b0;
assign excp_cause_id[EXCP_PIF]  = 1'b0;
assign excp_cause_id[EXCP_PIL]  = 1'b0;
assign excp_cause_id[EXCP_PIS]  = 1'b0;
assign excp_cause_id[EXCP_PME]  = 1'b0;
assign excp_cause_id[EXCP_PPI]  = 1'b0;

assign id_gr_we = ~inst_st_w & ~inst_beq & ~inst_bne & ~inst_b
              & ~inst_st_b & ~inst_st_h
              & ~inst_blt & ~inst_bge & ~inst_bltu & ~inst_bgeu
              & ~inst_ertn & ~inst_syscall & ~inst_break & ~inst_idle
              & ~inst_tlb_any & ~inst_cacop;
assign id_dest = inst_bl ? 5'd1 : rd;

assign need_ui12  = inst_andi | inst_ori | inst_xori;
assign need_si20  = inst_lu12i_w | inst_pcaddu12i;
assign src2_is_4  = inst_jirl | inst_bl;

assign id_imm = src2_is_4 ? 32'h4                      :
                need_si20 ? {i20[19:0], 12'b0}         :
                need_ui12 ? {20'b0, i12[11:0]}         :
                            {{20{i12[11]}}, i12[11:0]} ;

assign br_offs_i26 = {{ 4{i26[25]}}, i26[25:0], 2'b0};
assign br_offs_i16 = {{14{i16[15]}}, i16[15:0], 2'b0};

assign src_reg_is_rd = inst_beq | inst_bne | inst_st_w | inst_st_b | inst_st_h
                     | inst_blt | inst_bge | inst_bltu | inst_bgeu
                     | inst_csrwr | inst_csrxchg;

assign id_src1_is_pc  = inst_jirl | inst_bl | inst_pcaddu12i;
assign id_src2_is_imm = inst_slli_w | inst_srli_w | inst_srai_w
                      | inst_addi_w | inst_slti | inst_sltui
                      | inst_andi | inst_ori | inst_xori
                      | inst_ld_w | inst_st_w
                      | inst_ld_b | inst_ld_h | inst_ld_bu | inst_ld_hu
                      | inst_st_b | inst_st_h
                      | inst_lu12i_w | inst_pcaddu12i
                      | inst_jirl | inst_bl | inst_cacop;

assign id_rf_raddr1 = rj;
assign id_rf_raddr2 = src_reg_is_rd ? rd : rk;

// ID branch
// b / bl / jirl are unconditional: they use the br unit's JIRL (taken) bit,
// so the target is built by the same base+offs adder as the conditional
// branches.  b/bl use pc + i26<<2; jirl uses rj + i16<<2.
assign id_br_op      = {inst_jirl | inst_b | inst_bl,
                        inst_bgeu, inst_bltu, inst_bge, inst_blt, inst_bne, inst_beq};
assign id_br_base    = inst_jirl ? id_fw1_data : pc[ID];
assign id_br_offs    = (inst_b | inst_bl) ? br_offs_i26 : br_offs_i16;
assign id_fw_pending = id_fw1_pending | id_fw2_pending;
assign id_br_valid   = valid[ID] & ~ id_br_sent & ~ id_fw_pending;

always @(posedge clk) begin
    if (reset)                  id_br_sent <= 1'b0;
    else if (allow[ID])         id_br_sent <= 1'b0;
    else if (~ id_fw_pending)   id_br_sent <= 1'b1;
end

br u_br(
    .src1   (id_fw1_data    ),
    .src2   (id_fw2_data    ),
    .base   (id_br_base     ),
    .offs   (id_br_offs     ),
    .br_op  (id_br_op       ),
    .valid  (id_br_valid    ),
    .taken  (br_taken_id    ),
    .target (br_target_id   )
);

// fw src
// fw_valid_mem: MEM holds a result that is ready to be handed to ID.  Only the
// plain ALU result qualifies: a load needs its data, mul/div results are only
// selected at the MEM/WB edge, and a CSR write goes through the dedicated RF
// port (ID waits for it via csr_dep).  Everything that is not forwarded here
// reaches ID through the RF write-first bypass one cycle later (WB).
assign fw_valid_mem  = ~ mem_en[MEM] & ~ csr_ctl[MEM][CSR_WB] & ~ mul_pos & ~ div_pos;

assign fw_data_mem = alu_result_mem;

// CSR-writes-RF dependency: a csrrd/csrwr/... writes the RF through the CSR
// data port at WB with no read bypass, so ID waits until the write has landed.
always @(posedge clk) begin
    if (reset | flush[ID])                      csr_dep <= 1'b0;
    else if (allow[ID] & valid[ID] & ~ csr_dep & id_csr_ctl[CSR_WB])
                                                csr_dep <= 1'b1;
    else if (valid[WB] & csr_ctl[WB][CSR_WB])   csr_dep <= 1'b0;
end

// id_fw(dst): the (only) register source a consumer in ID can see in time.
// A hit whose value is not ready yet (load / mul / div / CSR) makes ID wait;
// the producer then reaches WB and the RF read port covers it.
assign id_fw1_hit_mem = valid[MEM] & gr_we[MEM] & id_src1_used & (id_rf_raddr1 != 5'd0) & (id_rf_raddr1 == dest[MEM]);
assign id_fw1_pending = (id_fw1_sel_mem & ~ fw_valid_mem) | csr_dep;

assign id_fw1_sel_mem = id_fw1_hit_mem;
assign id_fw1_sel_rf  = ~ id_fw1_hit_mem;

assign id_fw1_data = ({32{id_fw1_sel_mem}} & fw_data_mem) |
                     ({32{id_fw1_sel_rf }} & id_rf_rdata1);

assign id_fw2_hit_mem = valid[MEM] & gr_we[MEM] & id_src2_used & (id_rf_raddr2 != 5'd0) & (id_rf_raddr2 == dest[MEM]);
assign id_fw2_pending = (id_fw2_sel_mem & ~ fw_valid_mem) | csr_dep;

assign id_fw2_sel_mem = id_fw2_hit_mem;
assign id_fw2_sel_rf  = ~ id_fw2_hit_mem;

assign id_fw2_data = ({32{id_fw2_sel_mem}} & fw_data_mem) |
                     ({32{id_fw2_sel_rf }} & id_rf_rdata2);

// ID -> MEM
always @(posedge clk) begin
    if (allow[MEM]) begin
        alu_result_mem    <= alu_result_id;
        md_op_mem         <= id_md_op;
        div_src1_mem      <= id_src1;
        div_src2_mem      <= id_src2;
        res_from_mem[MEM] <= id_res_from_mem;
        mem_we[MEM]       <= id_mem_we;
        mem_en[MEM]       <= id_mem_en;
        mem_wdata_mem     <= id_fw2_data;
        mem_size[MEM]     <= id_mem_size;
        mem_zext[MEM]     <= id_mem_zext;
        gr_we[MEM]        <= id_gr_we;
        dest[MEM]         <= id_dest;
        csr_num[MEM]      <= id_csr_num;
        csr_ctl[MEM]      <= premem_csr_ctl;
        tlb_op[MEM]       <= id_tlb_op;
        invtlb_op[MEM]    <= id_invtlb_op;
        cacop[MEM]        <= inst_cacop;
        cacop_code[MEM]   <= id_cacop_code;
        excp_cause[MEM]   <= premem_excp_cause;
        csr_wdata[MEM]    <= id_fw2_data;
        csr_mask[MEM]     <= id_csr_mask_ones ? 32'hffffffff : id_fw1_data;
    end
    if (allow[WB]) begin
        result_wb         <= result_mem;
        gr_we[WB]         <= gr_we[MEM];
        dest[WB]          <= dest[MEM];
        csr_num[WB]       <= csr_num[MEM];
        csr_ctl[WB]       <= mem_csr_ctl;
        tlb_op[WB]        <= tlb_op[MEM];
        invtlb_op[WB]     <= invtlb_op[MEM];
        excp_cause[WB]    <= mem_excp_cause;
        csr_wdata[WB]     <= csr_wdata[MEM];
        csr_mask[WB]      <= csr_mask[MEM];
        wb_excp_badv      <= (excp_cause[MEM][EXCP_ALE] | (| mem_tlb_cause))
                             ? alu_result_mem : pc[MEM];
    end
end

// ID datapath: the operands are already resolved by the ID forwarding network
assign id_src1 = id_src1_is_pc  ? pc[ID] : id_fw1_data;
assign id_src2 = id_src2_is_imm ? id_imm : id_fw2_data;

// ID units: the ALU computes the result / the load-store address here, and the
// multiplier's Booth + Wallace tree collapses the product to (ws, wc) here.
// Both results are registered into MEM at the ID/MEM edge.
alu u_alu_id(
    .alu_op     (id_alu_op     ),
    .src1       (id_src1       ),
    .src2       (id_src2       ),
    .alu_result (alu_result_id )
);

mul u_mul(
    .clk    (clk            ),
    .reset  (reset          ),
    .allow  (allow[ID]      ),
    .src1   (id_src1        ),
    .src2   (id_src2        ),
    .md_op  (id_md_op       ),
    .result (mul_result     )
);

// div handshake: entirely in MEM.  The operands were latched from ID, start is
// asserted for the first MEM cycle, and MEM waits until the divider drops busy.
assign div_is_signed   = md_op_mem[DIV_W] | md_op_mem[MOD_W];
assign div_is_mod      = md_op_mem[MOD_W] | md_op_mem[MOD_WU];
assign div_result      = div_is_mod ? div_rem : div_quot;
assign div_start       = cur_div_pos & ~ div_started & ~ div_busy;
assign div_reset       = reset | flush[MEM];

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

// MEM result mux: the ID ALU result latched at the ID/MEM edge, the mul result
// (the CPA finishes here), the div result (ready when busy drops) or the load
// data.  It is latched into WB; the ALU / load values are also forwarded from
// MEM, while mul / div are only forwarded from WB (see fw_valid_mem).
assign result_mem = ({32{mul_pos}} & mul_result) |
                    ({32{div_pos}} & div_result) |
                    ({32{~ mul_pos & ~ div_pos}} &
                     (res_from_mem[MEM] ? mem_result : alu_result_mem));

// ID -> MEM / MEM -> WB (pre-MEM signals are computed combinationally at ID)
assign premem_ale   = id_mem_en & ((id_mem_size[SIZE_HALF] &    alu_result_id[0]   |
                                    id_mem_size[SIZE_WORD] & (| alu_result_id[1:0])));
assign premem_int_take = int_o;
assign premem_excp  = premem_ale | premem_int_take | id_csr_ctl[CSR_EXCP];
assign premem_csr_ctl   = id_csr_ctl | (premem_excp ? (9'h1 << CSR_EXCP) : 9'h0);
assign premem_excp_cause = id_excp_cause
                         | (premem_ale ? (13'h1 << EXCP_ALE) : 13'h0)
                         | (premem_int_take ? (13'h1 << EXCP_INT) : 13'h0);
assign premem_req   = cur_valid[ID] & ~ stall[ID] & (id_mem_en | inst_cacop) & ~ premem_excp;

// MEM: the address / byte lanes / store data were all computed in ID
assign mem_va    = alu_result_mem;

// cacop: no translation, the address is used as a physical address by the mmu
assign mem_cacop        = cacop[MEM] & valid[MEM] & ~ mem_excp;
assign mem_cacop_code   = cacop_code[MEM];
assign mem_abort = mem_excp | flush[MEM];

assign mem_byte = mem_size[MEM][SIZE_BYTE];
assign mem_half = mem_size[MEM][SIZE_HALF];
assign mem_word = mem_size[MEM][SIZE_WORD];

assign st_we_byte = 4'b0001 << alu_result_mem[1:0];
assign st_we_half = alu_result_mem[1] ? 4'b1100 : 4'b0011;
assign st_we_word = 4'b1111;

assign st_we = ({4{mem_byte}} & st_we_byte) | ({4{mem_half}} & st_we_half) | ({4{mem_word}} & st_we_word);

assign mem_wstrb  = {4{mem_we[MEM] && valid[MEM] && ~ mem_excp}} & st_we;
assign mem_stdata = ({32{mem_byte}} & {4{mem_wdata_mem[ 7: 0]}}) |
                    ({32{mem_half}} & {2{mem_wdata_mem[15: 0]}}) |
                    ({32{mem_word}} &    mem_wdata_mem);

assign ld_byte = (alu_result_mem[1:0] == 2'd0) ? mem_rdata[ 7: 0] :
                 (alu_result_mem[1:0] == 2'd1) ? mem_rdata[15: 8] :
                 (alu_result_mem[1:0] == 2'd2) ? mem_rdata[23:16] :
                                                 mem_rdata[31:24];
assign ld_half = alu_result_mem[1] ? mem_rdata[31:16] : mem_rdata[15:0];

assign ld_byte_ext = mem_zext[MEM] ? {24'b0, ld_byte} : {{24{ld_byte[7]}}, ld_byte};
assign ld_half_ext = mem_zext[MEM] ? {16'b0, ld_half} : {{16{ld_half[15]}}, ld_half};

assign mem_result = ({32{mem_byte}} & ld_byte_ext)  |
                    ({32{mem_half}} & ld_half_ext)  |
                    ({32{mem_word}} & mem_rdata);

assign mem_ipe = csr_ctl[MEM][CSR_PRIV] & (crmd_plv_o != 2'b0);
assign mem_excp = (csr_ctl[MEM][CSR_EXCP] | mem_ipe | (| mem_tlb_cause));

assign mem_csr_ctl    = csr_ctl[MEM] | ((mem_ipe | (| mem_tlb_cause)) ? (9'h1 << CSR_EXCP) : 9'h0);
assign mem_excp_cause = excp_cause[MEM]
                      | (mem_ipe ? (13'h1 << EXCP_IPE) : 13'h0)
                      | mem_tlb_cause;

assign mem_tlb = (| tlb_op[MEM]) & ~ tlb_op[MEM][TLB_SRCH];

assign mem_map_wr   = csr_ctl[MEM][CSR_WE] & (csr_num[MEM] == CSR_CRMD
                                            | csr_num[MEM] == CSR_DMW0
                                            | csr_num[MEM] == CSR_DMW1
                                            | csr_num[MEM] == CSR_ASID);

assign mem_cacop_ic = mem_cacop & (cacop_code[MEM][2:0] == 3'd0) & allow[MEM];

assign mem_crmd_tlb_cacop = mem_map_wr | mem_tlb | mem_cacop_ic;

always @(posedge clk) begin
    if (reset) begin
        wb_crmd_tlb_cacop     <= 1'b0;
        wb_excp         <= 1'b0;
        wb_ertn         <= 1'b0;
        wb_excp_ertn    <= 1'b0;
    end
    else begin
        wb_crmd_tlb_cacop <= mem_crmd_tlb_cacop & cur_valid[MEM];
        wb_excp     <= mem_excp & cur_valid[MEM];
        wb_ertn     <= csr_ctl[MEM][CSR_ERTN] & ~ mem_excp & cur_valid[MEM];
        wb_excp_ertn<= (csr_ctl[MEM][CSR_ERTN] | mem_excp) & cur_valid[MEM];
    end
end

// WB
assign wb_flush = wb_excp_ertn | wb_crmd_tlb_cacop;

assign rf_we    = gr_we[WB] && valid[WB] && ~ csr_ctl[WB][CSR_EXCP] && ~ csr_ctl[WB][CSR_WB];
assign rf_waddr = dest[WB];
assign rf_wdata = result_wb;

// WB normal GPR writes reach ID / MEM through the RF write-first read bypass
// (and the MEM -> ID forward for the same-cycle case), so there is no explicit
// WB forwarding network any more.

// CSR GPR write: same address port as the pipeline write (the address is the
// registered dest[WB]), separate data port, no write-first read bypass.  The
// forwarding networks mark it pending and never forward csr_rdata.
assign rf_we_csr    = gr_we[WB] && valid[WB] && ~ csr_ctl[WB][CSR_EXCP]
                    && csr_ctl[WB][CSR_WB];
assign rf_wdata_csr = csr_ctl[WB][CSR_RD]      ? csr_rdata      :
                      csr_ctl[WB][CSR_RDCNTID] ? csr_rdata      :
                      csr_ctl[WB][CSR_RDCNTVL] ? timer_o[31: 0] :
                      csr_ctl[WB][CSR_RDCNTVH] ? timer_o[63:32] :
                                                 csr_rdata;

assign excp_ecode    = excp_cause[WB][EXCP_INT]  ? 6'h0 :
                       excp_cause[WB][EXCP_ALE]  ? 6'h9 :
                       excp_cause[WB][EXCP_ADEF] ? 6'h8 :
                       excp_cause[WB][EXCP_TLBR] ? 6'h3f :
                       excp_cause[WB][EXCP_PIF]  ? 6'h3 :
                       excp_cause[WB][EXCP_PIL]  ? 6'h1 :
                       excp_cause[WB][EXCP_PIS]  ? 6'h2 :
                       excp_cause[WB][EXCP_PPI]  ? 6'h7 :
                       excp_cause[WB][EXCP_PME]  ? 6'h4 :
                       excp_cause[WB][EXCP_IPE]  ? 6'he :
                       excp_cause[WB][EXCP_INE]  ? 6'hd :
                       excp_cause[WB][EXCP_BRK]  ? 6'hc :
                                                   6'hb;

assign excp_tlb      = excp_cause[WB][EXCP_TLBR] | excp_cause[WB][EXCP_PIF]
                     | excp_cause[WB][EXCP_PIL]  | excp_cause[WB][EXCP_PIS]
                     | excp_cause[WB][EXCP_PPI]  | excp_cause[WB][EXCP_PME];

assign excp_badv_en  = wb_excp & ~ excp_cause[WB][EXCP_INT]
                               & (excp_cause[WB][EXCP_ALE] | excp_cause[WB][EXCP_ADEF] | excp_tlb);

assign csr_we    = valid[WB] & csr_ctl[WB][CSR_WE] & ~ csr_ctl[WB][CSR_EXCP];
assign csr_raddr = csr_num[WB];
assign csr_waddr = csr_num[WB];
assign csr_wdata_w = csr_wdata[WB];

// va --> pa translation
assign tlb_s0_vppn     = pc[IF][31:13];
assign tlb_s0_va_bit12 = pc[IF][12];
assign tlb_s1_vppn     = alu_result_mem[31:13];
assign tlb_s1_va_bit12 = alu_result_mem[12];

assign tlb_rd_lo0 = {4'b0, tlb_r_ppn0, 1'b0, tlb_r_g, tlb_r_mat0, tlb_r_plv0, tlb_r_d0, tlb_r_v0};
assign tlb_rd_lo1 = {4'b0, tlb_r_ppn1, 1'b0, tlb_r_g, tlb_r_mat1, tlb_r_plv1, tlb_r_d1, tlb_r_v1};

assign tlb_csr_sel = tlb_op[WB][TLB_SRCH] ? 2'd1 :
                     tlb_op[WB][TLB_RD]   ? 2'd2 : 2'd0;

assign map_mode = crmd_pg_o & ~ crmd_da_o;

assign dmw0_hit_if  = map_mode & ((crmd_plv_o == 2'd0) ? dmw0_o[0] : dmw0_o[3]) & (pc[IF][31:29] == dmw0_o[31:29]);
assign dmw1_hit_if  = map_mode & ((crmd_plv_o == 2'd0) ? dmw1_o[0] : dmw1_o[3]) & (pc[IF][31:29] == dmw1_o[31:29]);
assign dmw0_hit_mem = map_mode & ((crmd_plv_o == 2'd0) ? dmw0_o[0] : dmw0_o[3]) & (alu_result_mem[31:29] == dmw0_o[31:29]);
assign dmw1_hit_mem = map_mode & ((crmd_plv_o == 2'd0) ? dmw1_o[0] : dmw1_o[3]) & (alu_result_mem[31:29] == dmw1_o[31:29]);

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
assign if_dmw_pa   = {(dmw0_hit_if  ? dmw0_o[27:25] : dmw1_o[27:25]), pc[IF][28:0]};
assign mem_dmw_pa  = {(dmw0_hit_mem ? dmw0_o[27:25] : dmw1_o[27:25]), alu_result_mem[28:0]};

assign if_xlat  = tlb_s0_found & map_mode;
assign mem_xlat = tlb_s1_found & map_mode;

assign if_fetch_cause = (if_xlat | (~ if_dmw_hit & map_mode)) ?
                        ((map_mode & ~ if_dmw_hit & ~ tlb_s0_found)             ? (13'h1 << EXCP_TLBR) :
                         (map_mode & ~ if_dmw_hit & ~ tlb_s0_v)                 ? (13'h1 << EXCP_PIF ) :
                         (map_mode & ~ if_dmw_hit & (crmd_plv_o > tlb_s0_plv))  ? (13'h1 << EXCP_PPI ) : 13'h0)
                        : 13'h0;

// a hit-class cacop (code[4:3]=2) is translated like a load, so it may raise
// the tlb exceptions; store-tag / index invalidate use the va as it is
assign mem_tlb_chk = valid[MEM] & (mem_en[MEM] | (cacop[MEM] & cacop_code[MEM][4]))
                   & ~ csr_ctl[MEM][CSR_EXCP];
assign mem_tlb_cause = ({13{mem_tlb_chk}} &
                        ((map_mode & ~ mem_dmw_hit & ~ tlb_s1_found)            ? (13'h1 << EXCP_TLBR) :
                        (map_mode & ~ mem_dmw_hit & ~ tlb_s1_v)                 ? (13'h1 << (mem_we[MEM] ? EXCP_PIS : EXCP_PIL)) :
                        (map_mode & ~ mem_dmw_hit & (crmd_plv_o > tlb_s1_plv))  ? (13'h1 << EXCP_PPI ) :
                        (map_mode & ~ mem_dmw_hit & mem_we[MEM] & ~ tlb_s1_d)   ? (13'h1 << EXCP_PME ) :
                                                                                   13'h0));

assign excp_entry = excp_cause[WB][EXCP_TLBR] ? tlbrentry_o : eentry_o;

regfile u_regfile(
    .clk    (clk         ),
    .raddr1 (id_rf_raddr1),
    .rdata1 (id_rf_rdata1),
    .raddr2 (id_rf_raddr2),
    .rdata2 (id_rf_rdata2),
    .we        (rf_we       ),
    .waddr     (rf_waddr    ),
    .wdata     (rf_wdata    ),
    .we_csr    (rf_we_csr    ),
    .wdata_csr (rf_wdata_csr )
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
    .s0_mat       (tlb_s0_mat      ),
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
    .s1_mat       (tlb_s1_mat      ),
    .s1_d         (tlb_s1_d       ),
    .s1_v         (tlb_s1_v       ),

    .s2_vppn      (tlbehi_vppn_o  ),
    .s2_asid      (asid_o         ),
    .s2_found     (tlb_s2_found   ),
    .s2_index     (tlb_s2_index   ),

    .invtlb_valid (wb_tlb_inv     ),
    .invtlb_op    (invtlb_op[WB]  ),

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
    .clk             (clk          ),
    .reset           (reset        ),
    .raddr_i         (csr_raddr    ),
    .rdata_o         (csr_rdata    ),
    .we_i            (csr_we       ),
    .waddr_i         (csr_waddr    ),
    .wdata_i         (csr_wdata_w  ),
    .wmask_i         (csr_mask[WB] ),
    .excp_i          (wb_excp      ),
    .excp_pc_i       (pc[WB]       ),
    .excp_ecode_i    (excp_ecode   ),
    .excp_esubcode_i (9'h0         ),
    .excp_badv_en_i  (excp_badv_en ),
    .excp_badv_i     (wb_excp_badv ),
    .excp_tlb_i      (wb_excp & excp_tlb),
    .ertn_i          (wb_ertn      ),
    .hwi_i           (8'h0         ),
    .ipi_i           (1'b0         ),
    .tlb_we_i        (wb_tlb_commit),
    .tlb_sel_i       (tlb_csr_sel   ),
    .tlb_srch_hit_i  (tlb_s2_found ),
    .tlb_srch_index_i(tlb_s2_index ),
    .tlb_rd_e_i      (tlb_r_e      ),
    .tlb_rd_vppn_i   (tlb_r_vppn   ),
    .tlb_rd_ps_i     (tlb_r_ps     ),
    .tlb_rd_asid_i   (tlb_r_asid   ),
    .tlb_rd_lo0_i    (tlb_rd_lo0   ),
    .tlb_rd_lo1_i    (tlb_rd_lo1   ),

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

assign wb_tlb_commit = valid[WB] & (| tlb_op[WB]) & ~ csr_ctl[WB][CSR_EXCP];
assign wb_tlb_srch   = wb_tlb_commit & tlb_op[WB][TLB_SRCH];
assign wb_tlb_rd     = wb_tlb_commit & tlb_op[WB][TLB_RD];
assign wb_tlb_wr     = wb_tlb_commit & (tlb_op[WB][TLB_WR] | tlb_op[WB][TLB_FILL]);
assign wb_tlb_inv    = wb_tlb_commit & tlb_op[WB][TLB_INV];

assign tlb_w_index = tlb_op[WB][TLB_FILL] ? timer_o[3:0] : tlbidx_index_o;
assign tlb_w_e     = (estat_ecode_o == 6'h3f) ? 1'b1 : ~ tlbidx_ne_o;
assign tlb_w_vppn  = wb_tlb_inv ? csr_wdata[WB][31:13] : tlbehi_vppn_o;
assign tlb_w_asid  = wb_tlb_inv ? csr_mask[WB][9:0]   : asid_o;
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
    .if_va          (pc[IF]         ),
    .if_allow       (allow[IF]      ),
    .if_pending     (if_pending     ),
    .if_rdata       (inst_rdata     ),

    .mem_pre        (premem_req     ),
    .mem_va         (mem_va         ),
    .mem_wstrb      (mem_wstrb      ),
    .mem_stdata     (mem_stdata     ),
    .mem_abort      (mem_abort      ),
    .mem_allow      (allow[MEM]     ),
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
    .mem_pre_addr       (alu_result_id      ),
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

assign debug_wb_pc       = pc[WB];
// the CSR GPR write uses the dedicated CSR data port, so the debug WB
// interface must report both writes (the SoC testbench checks this output)
assign debug_wb_rf_we    = {4{rf_we | rf_we_csr}};
assign debug_wb_rf_wnum  = rf_waddr;
assign debug_wb_rf_wdata = rf_we_csr ? rf_wdata_csr : rf_wdata;

endmodule
