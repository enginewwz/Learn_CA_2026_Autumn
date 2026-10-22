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

// internal clock / reset names kept as before
wire clk    = aclk;
wire resetn = aresetn;

// mmu: the fetch / memory-access stage controller.
wire        inst_req;          // the IF stage has a word to fetch at pc[IF]
wire [31:0] inst_rdata;        // returns after an arbitrary latency
wire        mem_req;           // the MEM slot holds a real access
wire [31:0] mem_va;            // its virtual address (= result[MEM])
wire        mem_abort;         // this access has been found illegal
wire [ 3:0] mem_wstrb;         // byte strobes; 4'b0 means read
wire [31:0] mem_stdata;        // store data, already spread over the byte lanes
wire [31:0] mem_rdata;
wire        mem_pre;           // a load / store in EX that enters MEM at this edge
wire        if_pending;        // from the mmu: this fetch should wait
wire        mem_pending;      // from the mmu: this access should wait

// pipeline: IF -> ID -> EX -> MEM -> WB

// feat: we use vectors to show which state the signal is in

reg         reset;
always @(posedge clk) reset <= ~resetn;

localparam IF                = 0;
localparam ID                = 1;
localparam EX                = 2;
localparam MEM               = 3;
localparam WB                = 4;
localparam TOTAL_STAGE_WIDTH = 5;

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
localparam CSR_RD            = 0;   // csrrd/csrwr/csrxchg: rd <- CSR[csr_num]
localparam CSR_WE            = 1;   // csrwr/csrxchg: write the CSR
localparam CSR_ERTN          = 2;   // ertn
localparam CSR_EXCP          = 3;   // this slot carries an exception; kind is in excp_cause
localparam CSR_RDCNTVL       = 4;   // rdcntvl.w
localparam CSR_RDCNTVH       = 5;   // rdcntvh.w
localparam CSR_RDCNTID       = 6;   // rdcntid
localparam CSR_WB            = 7;   // rd comes from csr.v at WB
localparam CSR_PRIV          = 8;   // csrrd/csrwr/csrxchg/ertn/idle: IPE when PLV != 0

// excp_cause bit positions: one-hot, registered from IF (ADEF is born there) all
// the way to WB.  WB is where the exception entry ports are driven from, i.e. the
// same point as every other CSR access.
localparam EXCP_SYS          = 0;   // syscall
localparam EXCP_BRK          = 1;   // break
localparam EXCP_INE          = 2;   // instruction not exist
localparam EXCP_ADEF         = 3;   // fetch address not 4-byte aligned
localparam EXCP_ALE          = 4;   // load / store not naturally aligned (judged in MEM)
localparam EXCP_INT          = 5;   // interrupt (judged in MEM)
localparam EXCP_IPE          = 6;   // privileged instruction at PLV != 0 (judged in MEM)
localparam EXCP_TLBR         = 7;   // tlb refill
localparam EXCP_PIF          = 8;   // fetch: page invalid
localparam EXCP_PIL          = 9;   // load: page invalid
localparam EXCP_PIS          = 10;  // store: page invalid
localparam EXCP_PME          = 11;  // store: page not dirty
localparam EXCP_PPI          = 12;  // page privilege violation

localparam [13:0] CSR_CRMD   = 14'h000;
localparam [13:0] CSR_DMW0   = 14'h180;
localparam [13:0] CSR_DMW1   = 14'h181;
localparam [13:0] CSR_ASID   = 14'h018;

reg  [TOTAL_STAGE_WIDTH - 1: 0] valid /* verilator split_var */;
// when is valid used in leaf control signals? write back to rf / write mem / change pc

wire [TOTAL_STAGE_WIDTH - 1: 0] stall; // stall itself; the last stage (WB) would never stall it self; reset as 0
wire [TOTAL_STAGE_WIDTH - 1: 0] allow; // allow new inst; means that cur and later stage is ready for new inst; the last stage (WB) would always allow; reset as 1
wire [TOTAL_STAGE_WIDTH - 1: 0] flush;
wire [TOTAL_STAGE_WIDTH - 1: 0] cur_valid;
wire [TOTAL_STAGE_WIDTH - 1: 0] next_valid;

// only process in one stage
wire [31:0] nextpc;     // generate at IF stage

(* keep = "true" *) wire [31:0] seq_pc;     // generate at IF stage
wire        br_taken_id;    // b / bl, decided at ID
(* keep = "true" *) wire [31:0] br_target_id;
wire        br_taken_ex;    // conditional branches / jirl, decided in EX stage 0
wire [31:0] br_target_ex;
wire        br_redirect;    // a redirect is being issued this cycle
wire [31:0] inst_rd;    // receive in ID stage

// should be stored between stages.
// The [EX] elements are combinational aliases of the EX outputs
reg  [31:0] pc [WB:IF] /* verilator split_var */;
reg  [31:0] br_pc_reg;
reg         br_reg;

reg  [31:0] inst;

// ID: combinational, generated from inst; these only exist at ID
reg  [11:0] alu_op_id;
reg  [31:0] src1_id;
reg  [31:0] src2_id;
reg  [MOD_WU:0] md_op_id;

// stage fields: [ID] combinational, [EX] aliases EX, [MEM]/[WB] registered
reg         src1_used [MEM:ID] /* verilator split_var */;
reg         src2_used [MEM:ID] /* verilator split_var */;
reg         res_from_mem [MEM:ID] /* verilator split_var */;
reg         mem_we [MEM:ID] /* verilator split_var */;
reg         mem_en [MEM:ID] /* verilator split_var */;
reg  [31:0] mem_wdata [MEM:ID] /* verilator split_var */;
reg  [ 2:0] mem_size [MEM:ID] /* verilator split_var */;   // one-hot: SIZE_BYTE / SIZE_HALF / SIZE_WORD
reg         mem_zext [MEM:ID] /* verilator split_var */;   // the load zero-extends (ld.bu / ld.hu)
reg         gr_we [WB:ID] /* verilator split_var */;
reg  [4:0]  dest [WB:ID] /* verilator split_var */;


// CSR / exception
reg  [13:0] csr_num [WB:ID] /* verilator split_var */;
reg  [ 8:0] csr_ctl [WB:ID] /* verilator split_var */;
reg  [ 4:0] tlb_op [WB:ID] /* verilator split_var */;
reg  [ 4:0] invtlb_op [WB:ID] /* verilator split_var */;
reg  [12:0] excp_cause [WB:IF] /* verilator split_var */;
reg  [31:0] csr_wdata [WB:ID] /* verilator split_var */;
reg  [31:0] csr_mask [WB:ID] /* verilator split_var */;

// CSR file interconnect; the read port is used by the WB write-back mux
wire [31:0] csr_rdata;
wire        csr_we;
wire [13:0] csr_raddr;
wire [13:0] csr_waddr;
wire [31:0] csr_wdata_w;
wire [31:0] eentry_o;
wire [31:0] era_o;
wire [63:0] timer_o;

// CRMD view for the *next* slot: software write + exception/ertn override.  This
// is what both the IPE compare (MEM) and the fetch's arprot (bridge) need.
/* verilator lint_off UNUSEDSIGNAL */
wire [ 1:0] crmd_plv_o;
wire        crmd_ie_o;
/* verilator lint_on UNUSEDSIGNAL */
wire        int_o;
wire        next_if_adef;        // misaligned fetch address, gates the request

// exception / ertn / interrupt
wire        mem_excp;
wire        wb_flush;          // flush at WB: exception / ertn, or a write that changes the map
wire        mem_ipe;          // privileged instruction at PLV != 0, judged in MEM

wire        wb_excp_ertn;     // the WB instruction is entering the exception / ertn
wire        excp_i;
wire        ertn_i;
reg         wb_crmd_tlb;       // CRMD / TLB changed the map: restart the fetch
wire        mem_crmd_tlb;
wire        mem_tlb;
wire        tlb_op_map;      // this TLB op changes the mapping, so it must flush
wire [ 5:0] excp_ecode;
wire        excp_badv_en;
reg  [31:0] wb_excp_badv;   // BADV of the instruction now in WB

// TLB / DMW translation and the exceptions it can raise
wire        map_mode;           // PG mode: consult DMW / TLB
wire        if_xlat, mem_xlat;
wire        if_dmw_hit, mem_dmw_hit;
wire [31:0] if_dmw_pa, mem_dmw_pa;
wire        dmw0_hit_if, dmw1_hit_if, dmw0_hit_mem, dmw1_hit_mem;
wire [31:0] dmw0_o, dmw1_o;
wire [12:0] if_fetch_cause;
wire        if_abort;
reg  [12:0] if_excp_cause_r;
wire        mem_tlb_chk;
wire [12:0] mem_tlb_cause;
wire        mem_map_wr;
wire [31:0] excp_entry;
wire        excp_tlb;

wire        ex_mem_ale;     // load / store alignment, judged in EX (before MEM)
wire        ex_mem_excp;    // ALE / interrupt: folded into the slot at the EX -> MEM edge
wire        int_take;

// EX outputs, aliased into the [EX] elements of the stage fields above
wire [31:0] ex_pc_o;
wire        ex_src1_used_o;
wire        ex_src2_used_o;
wire        ex_res_from_mem_o;
wire        ex_mem_we_o;
wire        ex_mem_en_o;
wire [31:0] ex_mem_wdata_o;
wire [ 2:0] ex_mem_size_o;
wire        ex_mem_zext_o;
wire        ex_gr_we_o;
wire  [4:0] ex_dest_o;
wire [13:0] ex_csr_num_o;
wire [ 8:0] ex_csr_ctl_o;
wire [ 4:0] ex_tlb_op_o;
wire [ 4:0] ex_invtlb_op_o;
wire [12:0] ex_excp_cause_o;
wire [12:0] excp_cause_id;
wire [31:0] ex_csr_wdata_o;
wire [31:0] ex_csr_mask_o;
wire        ex_valid_o;
wire        ex_stall;

wire [31:0] ex_result;      // EX combinational output (this is the [EX] result)

// EX forwarding
wire        fw1_ex_hit_o, fw1_ex_valid_o, fw2_ex_hit_o, fw2_ex_valid_o;
wire [31:0] fw1_ex_data_o, fw2_ex_data_o;

reg  [31:0] result [WB:MEM];

wire        load_op;    // not used

// mid signals for ID stage
wire        src_reg_is_rd;
wire [31:0] rj_value;
wire [31:0] imm;
wire [31:0] br_offs_i26;
wire [31:0] br_offs_i16;
wire [ 6:0] br_op_id;
wire [31:0] br_base_id;
wire        src1_is_pc;
wire        src2_is_imm;

wire [ 5:0] op_31_26;
wire [ 3:0] op_25_22;
wire [ 1:0] op_21_20;
wire [ 4:0] op_19_15;
wire [ 4:0] rd;
wire [ 4:0] rj;
wire [ 4:0] rk;
wire [11:0] i12;
wire [19:0] i20;
wire [15:0] i16;
wire [25:0] i26;

wire [63:0] op_31_26_d;
wire [15:0] op_25_22_d;
wire [ 3:0] op_21_20_d;
wire [31:0] op_19_15_d;

wire        inst_add_w;
wire        inst_sub_w;
wire        inst_slt;
wire        inst_sltu;
wire        inst_nor;
wire        inst_and;
wire        inst_or;
wire        inst_xor;
wire        inst_slli_w;
wire        inst_srli_w;
wire        inst_srai_w;
wire        inst_addi_w;
wire        inst_ld_w;
wire        inst_st_w;
wire        inst_jirl;
wire        inst_b;
wire        inst_bl;
wire        inst_beq;
wire        inst_bne;
wire        inst_lu12i_w;

wire        inst_slti;
wire        inst_sltui;
wire        inst_andi;
wire        inst_ori;
wire        inst_xori;
wire        inst_sll_w;
wire        inst_srl_w;
wire        inst_sra_w;
wire        inst_pcaddu12i;

wire        inst_mul_w;
wire        inst_mulh_w;
wire        inst_mulh_wu;
wire        inst_div_w;
wire        inst_div_wu;
wire        inst_mod_w;
wire        inst_mod_wu;

wire        inst_blt;
wire        inst_bge;
wire        inst_bltu;
wire        inst_bgeu;
wire        inst_ld_b;
wire        inst_ld_h;
wire        inst_ld_bu;
wire        inst_ld_hu;
wire        inst_st_b;
wire        inst_st_h;

// CSR, system and counter instructions
wire        inst_csr_any;       // any of csrrd / csrwr / csrxchg
wire        inst_csrrd;
wire        inst_csrwr;
wire        inst_csrxchg;
wire        inst_ertn;
wire        inst_syscall;
wire        inst_break;
wire        inst_idle;
wire        inst_rdcntvl_w;
wire        inst_rdcntvh_w;
wire        inst_rdcntid;
wire        inst_ine;

// TLB instructions
wire        inst_tlbsrch;
wire        inst_tlbrd;
wire        inst_tlbwr;
wire        inst_tlbfill;
wire        inst_invtlb;
wire        inst_tlb_any;

// TLB instruction select, one hot per slot
localparam TLB_SRCH = 0;
localparam TLB_RD   = 1;
localparam TLB_WR   = 2;
localparam TLB_FILL = 3;
localparam TLB_INV  = 4;

wire        need_ui5;
wire        need_ui12;
wire        need_si12;
wire        need_si16;
wire        need_si20;
wire        src2_is_4;

wire [ 4:0] rf_raddr1;
wire [31:0] rf_rdata1;
wire [ 4:0] rf_raddr2;
wire [31:0] rf_rdata2;
wire        rf_we   ;
wire [ 4:0] rf_waddr;
wire [31:0] rf_wdata;

wire [31:0] mem_result;
wire        src_wait;

// byte / halfword / word memory access helpers (used in the MEM section)
wire        mem_byte, mem_half, mem_word;
wire        ex_mem_byte, ex_mem_half, ex_mem_word;
wire [ 3:0] st_we_byte, st_we_half, st_we_word, st_we;
wire [ 7:0] ld_byte;
wire [15:0] ld_half;
wire [31:0] ld_byte_ext, ld_half_ext;

// forwarding
wire [MEM:EX] fw1_hit     ;      // index 3..4: EX / MEM taps
wire [MEM:EX] fw2_hit     ;
wire [MEM:EX] fw1_valid   ;
wire [MEM:EX] fw2_valid   ;
wire [MEM:EX] fw1_pending ;
wire [MEM:EX] fw2_pending ;
wire        fw1_mem_sel,  fw2_mem_sel;
wire        fw1_rf_sel,   fw2_rf_sel;
wire [31:0] fw1_data,     fw2_data;

assign seq_pc       = pc[IF] + 32'h4;

wire sel_era    = ertn_i;                       // ertn -> era
wire sel_eentry = excp_i;                       // exception -> eentry / tlbrentry
wire sel_crmd   = ~ wb_excp_ertn & wb_crmd_tlb;  // CRMD / TLB write -> restart fetch at pc[WB]+4
wire sel_br     = ~ wb_excp_ertn & ~ wb_crmd_tlb & br_reg;  // latched redirect
wire sel_seq    = ~ (wb_excp_ertn | wb_crmd_tlb | br_reg | br_taken_ex | br_taken_id);

assign nextpc       = ({32{sel_era    }} & era_o       ) |
                      ({32{sel_eentry }} & excp_entry  ) |
                      ({32{sel_crmd   }} & (pc[WB] + 32'h4)) |
                      ({32{sel_br     }} & br_pc_reg   ) |
                      ({32{br_taken_ex}} & br_target_ex) |
                      ({32{br_taken_id}} & br_target_id) |
                      ({32{sel_seq    }} & seq_pc      );

assign br_redirect = br_taken_ex | br_taken_id | wb_excp_ertn | wb_crmd_tlb;

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
        pc[IF]  <= 32'h1bfffffc; // trick: to make nextpc be 0x1c000000 during reset
        pc[ID]  <= 32'h0;
        pc[MEM] <= 32'h0;
        pc[WB]  <= 32'h0;
    end
    else begin
        if (allow[IF] )     pc[IF]     <= nextpc;
        if (allow[ID] )     pc[ID]     <= pc[IF];
        if (allow[MEM])     pc[MEM]    <= pc[EX];
        if (allow[WB] )     pc[WB]     <= pc[MEM];
    end
end

// hazard unit
assign cur_valid = valid & ~ flush;
assign next_valid = {cur_valid[MEM:IF] & ~stall[MEM:IF], 1'b1} & allow | (~ allow & cur_valid);

// valid[EX] is registered inside EX
always @(posedge clk) begin
    if (reset) begin
        valid[IF]      <= 1'b0;
        valid[ID]      <= 1'b0;
        valid[MEM]     <= 1'b0;
        valid[WB]      <= 1'b0;
    end
    else begin
        valid[IF]      <= next_valid[IF];
        valid[ID]      <= next_valid[ID];
        valid[MEM]     <= next_valid[MEM];
        valid[WB]      <= next_valid[WB];
    end
end

assign flush    = wb_flush    ? 5'b01111 :
                  br_taken_ex ? 5'b00011 :
                  br_taken_id ? 5'b00001 :
                                5'b00000;

assign allow[WB]      = 1'b1;
assign allow[MEM]     = ~ (| stall[WB:MEM]);
assign allow[EX]      = ~ (| stall[WB:EX]);
assign allow[ID]      = ~ (| stall[WB:ID]);
assign allow[IF]      = ~ (| stall[WB:IF]);

// ID must wait when any tap of either source is still pending
assign src_wait = | (fw1_pending | fw2_pending);

assign stall[IF]      = if_pending;
assign stall[ID]      = src_wait && cur_valid[ID];
assign stall[EX]      = ex_stall;
assign stall[MEM]     = mem_pending;
assign stall[WB]      = 1'b0;

// forwarding
// EX part
assign fw1_hit[EX]   = fw1_ex_hit_o;
assign fw2_hit[EX]   = fw2_ex_hit_o;
assign fw1_valid[EX] = fw1_ex_valid_o;
assign fw2_valid[EX] = fw2_ex_valid_o;

// MEM part
assign fw1_hit[MEM]   = valid[MEM] && gr_we[MEM] && src1_used[ID] && (rf_raddr1 != 5'd0) && (rf_raddr1 == dest[MEM]);
assign fw2_hit[MEM]   = valid[MEM] && gr_we[MEM] && src2_used[ID] && (rf_raddr2 != 5'd0) && (rf_raddr2 == dest[MEM]);
assign fw1_valid[MEM] = fw1_hit[MEM] && ~ mem_en[MEM] && ~ csr_ctl[MEM][CSR_WB];
assign fw2_valid[MEM] = fw2_hit[MEM] && ~ mem_en[MEM] && ~ csr_ctl[MEM][CSR_WB];

assign fw1_pending  = fw1_hit & ~ fw1_valid;
assign fw2_pending  = fw2_hit & ~ fw2_valid;

assign fw1_mem_sel      = ~ fw1_valid[EX] &   fw1_valid[MEM];
assign fw1_rf_sel       = ~ fw1_valid[EX] & ~ fw1_valid[MEM];
assign fw2_mem_sel      = ~ fw2_valid[EX] &   fw2_valid[MEM];
assign fw2_rf_sel       = ~ fw2_valid[EX] & ~ fw2_valid[MEM];

assign fw1_data = ({32{fw1_valid[EX]}} & fw1_ex_data_o)     |
                  ({32{fw1_mem_sel}}  & result[MEM])        |
                  ({32{fw1_rf_sel}}   & rf_rdata1);

assign fw2_data = ({32{fw2_valid[EX]}} & fw2_ex_data_o)     |
                  ({32{fw2_mem_sel}}  & result[MEM])        |
                  ({32{fw2_rf_sel}}   & rf_rdata2);

assign next_if_adef  = | nextpc[1:0];

assign inst_req        = ~ next_if_adef;
assign inst_rd         = inst_rdata;

always @(posedge clk) begin
    if (allow[ID]) begin
        if (|if_excp_cause_r)  inst <= 32'h0340_0000; // nop = andi $r0, $r0, 0
        else                   inst <= inst_rdata;
    end
end

always @(posedge clk) begin
    if (reset) begin
        if_excp_cause_r <= 13'h0;
    end
    else if (allow[ID]) begin
        if_excp_cause_r <= excp_cause[IF] | if_fetch_cause;
    end
end

always @(posedge clk) begin
    if (reset) begin
        excp_cause[IF] <= 13'h0;
    end
    else if (allow[IF]) begin
        excp_cause[IF] <= next_if_adef ? (13'h1 << EXCP_ADEF) : 13'h0;
    end
end

// ID stage
assign op_31_26  = inst[31:26];
assign op_25_22  = inst[25:22];
assign op_21_20  = inst[21:20];
assign op_19_15  = inst[19:15];

assign rd   = inst_rdcntid ? inst[9:5] : inst[4:0];   // rdcntid keeps rd in [9:5]
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

// arithmetic / logic instructions
assign inst_slti     = op_31_26_d[6'h00] & op_25_22_d[4'h8];
assign inst_sltui    = op_31_26_d[6'h00] & op_25_22_d[4'h9];
assign inst_andi     = op_31_26_d[6'h00] & op_25_22_d[4'hd];
assign inst_ori      = op_31_26_d[6'h00] & op_25_22_d[4'he];
assign inst_xori     = op_31_26_d[6'h00] & op_25_22_d[4'hf];
assign inst_sll_w    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0e];
assign inst_srl_w    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0f];
assign inst_sra_w    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h10];
assign inst_pcaddu12i= op_31_26_d[6'h07];

// mul / div instructions
assign inst_mul_w    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h18];
assign inst_mulh_w   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h19];
assign inst_mulh_wu  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h1a];
assign inst_div_w    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h00];
assign inst_mod_w    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h01];
assign inst_div_wu   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h02];
assign inst_mod_wu   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h03];

// compare branches and byte / halfword memory
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

// CSR / system / counter instructions.
assign inst_csr_any = op_31_26_d[6'h01] & ~ inst[25] & ~ inst[24];
assign inst_csrrd   = inst_csr_any & (inst[9:5] == 5'd0);
assign inst_csrwr   = inst_csr_any & (inst[9:5] == 5'd1);
assign inst_csrxchg = inst_csr_any & ~ inst_csrrd & ~ inst_csrwr;
// TLB instructions share op_19_15 = 0x10 with ertn, so ertn also checks i14:10
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

assign inst_syscall = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h16];
assign inst_break   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h14];

// rdcntvl.w / rdcntvh.w
assign inst_rdcntvl_w = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h0] & op_19_15_d[5'h00]
                      & (inst[14:10] == 5'b11000) & (inst[9:5] == 5'd0);
assign inst_rdcntvh_w = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h0] & op_19_15_d[5'h00]
                      & (inst[14:10] == 5'b11001) & (inst[9:5] == 5'd0);
assign inst_rdcntid   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h0] & op_19_15_d[5'h00]
                      & (inst[14:10] == 5'b11000) & (inst[4:0] == 5'd0) & ~ inst_rdcntvl_w;

// instruction-not-existent: nothing above matched.
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
                   | inst_tlb_any
                   | inst_rdcntvl_w | inst_rdcntvh_w | inst_rdcntid);

// ID source usage includes branch operands and store data
always @(*) begin
    src1_used[ID] = inst_add_w | inst_sub_w | inst_slt | inst_sltu
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
                  | inst_csrxchg           // rj is the csrxchg write mask
                  | inst_invtlb;           // rj[9:0] is the ASID
    src2_used[ID] = inst_add_w | inst_sub_w | inst_slt | inst_sltu
                  | inst_and | inst_or | inst_nor | inst_xor
                  | inst_st_w | inst_beq | inst_bne
                  | inst_sll_w | inst_srl_w | inst_sra_w
                  | inst_mul_w | inst_mulh_w | inst_mulh_wu
                  | inst_div_w | inst_mod_w | inst_div_wu | inst_mod_wu
                  | inst_blt | inst_bge | inst_bltu | inst_bgeu
                  | inst_st_b | inst_st_h
                  | inst_csrwr | inst_csrxchg    // rd is the CSR write data
                  | inst_invtlb;                // rk is the VA
end

always @(posedge clk) begin
    if (allow[MEM]) begin
        src1_used[MEM] <= src1_used[EX];
        src2_used[MEM] <= src2_used[EX];
    end
end

// ALU operation
always @(*) begin
    alu_op_id[ 0] = inst_add_w | inst_addi_w | inst_ld_w | inst_st_w
                  | inst_jirl | inst_bl | inst_pcaddu12i
                  | inst_ld_b | inst_ld_h | inst_ld_bu | inst_ld_hu
                  | inst_st_b | inst_st_h;
    alu_op_id[ 1] = inst_sub_w;
    alu_op_id[ 2] = inst_slt | inst_slti;
    alu_op_id[ 3] = inst_sltu | inst_sltui;
    alu_op_id[ 4] = inst_and | inst_andi;
    alu_op_id[ 5] = inst_nor;
    alu_op_id[ 6] = inst_or | inst_ori;
    alu_op_id[ 7] = inst_xor | inst_xori;
    alu_op_id[ 8] = inst_slli_w | inst_sll_w;
    alu_op_id[ 9] = inst_srli_w | inst_srl_w;
    alu_op_id[10] = inst_srai_w | inst_sra_w;
    alu_op_id[11] = inst_lu12i_w;
end

// mul/div operation select
always @(*) begin
    md_op_id[MUL_W]   = inst_mul_w;
    md_op_id[MULH_W]  = inst_mulh_w;
    md_op_id[MULH_WU] = inst_mulh_wu;
    md_op_id[DIV_W]   = inst_div_w;
    md_op_id[MOD_W]   = inst_mod_w;
    md_op_id[DIV_WU]  = inst_div_wu;
    md_op_id[MOD_WU]  = inst_mod_wu;
end

// operands
always @(*) begin
    src1_id = src1_is_pc  ? pc[ID] : fw1_data;
    src2_id = src2_is_imm ? imm    : fw2_data;
end

// memory control and store data
always @(*) begin
    res_from_mem[ID]        = inst_ld_w | inst_ld_b | inst_ld_h | inst_ld_bu | inst_ld_hu;
    mem_we[ID]              = inst_st_w | inst_st_b | inst_st_h;
    mem_en[ID]              = mem_we[ID] | res_from_mem[ID];
    mem_wdata[ID]           = fw2_data;
    mem_size[ID][SIZE_BYTE] = inst_ld_b | inst_ld_bu | inst_st_b;
    mem_size[ID][SIZE_HALF] = inst_ld_h | inst_ld_hu | inst_st_h;
    mem_size[ID][SIZE_WORD] = inst_ld_w | inst_st_w;
    mem_zext[ID]            = inst_ld_bu | inst_ld_hu;
end

// CSR / exception payload
always @(*) begin
    // rdcntid has no CSR number in the instruction: it always reads TID (0x040).
    csr_num[ID] = inst_rdcntid ? 14'h040 : inst[23:10];

    csr_wdata[ID] = fw2_data;
    csr_mask[ID]  = inst_csrwr ? 32'hffffffff : fw1_data;

    csr_ctl[ID][CSR_RD]      = inst_csrrd | inst_csrwr | inst_csrxchg;
    csr_ctl[ID][CSR_WE]      = inst_csrwr | inst_csrxchg;
    csr_ctl[ID][CSR_ERTN]    = inst_ertn;
    csr_ctl[ID][CSR_RDCNTVL] = inst_rdcntvl_w;
    csr_ctl[ID][CSR_RDCNTVH] = inst_rdcntvh_w;
    csr_ctl[ID][CSR_RDCNTID] = inst_rdcntid;
    csr_ctl[ID][CSR_WB]      = inst_csrrd | inst_csrwr | inst_csrxchg
                             | inst_rdcntvl_w | inst_rdcntvh_w | inst_rdcntid;
    // all carried exceptions share one csr_ctl bit; excp_cause says which one
    excp_cause[ID]           = if_excp_cause_r | excp_cause_id;
    csr_ctl[ID][CSR_EXCP]    = | excp_cause[ID];
    // privileged instructions: IPE when the commit-point PLV is not 0
    csr_ctl[ID][CSR_PRIV]    = inst_csr_any | inst_ertn | inst_idle | inst_tlb_any;
    // TLB instruction payload
    tlb_op[ID]               = {inst_invtlb, inst_tlbfill, inst_tlbwr, inst_tlbrd, inst_tlbsrch};
    invtlb_op[ID]            = inst[4:0];
end

always @(posedge clk) begin
    if (allow[MEM]) begin
        tlb_op[MEM]     <= tlb_op[EX];
        invtlb_op[MEM]  <= invtlb_op[EX];
        csr_num[MEM]    <= csr_num[EX];
        // ALE / interrupt are born in EX and carried like any other cause
        csr_ctl[MEM]    <= csr_ctl[EX] | (ex_mem_excp ? (9'h1 << CSR_EXCP) : 9'h0);
        csr_wdata[MEM]  <= csr_wdata[EX];
        csr_mask[MEM]   <= csr_mask[EX];
        excp_cause[MEM] <= excp_cause[EX]
                           | (ex_mem_ale ? (13'h1 << EXCP_ALE) : 13'h0)
                           | (int_take   ? (13'h1 << EXCP_INT) : 13'h0);
    end
    if (allow[WB]) begin
        tlb_op[WB]     <= tlb_op[MEM];
        invtlb_op[WB]  <= invtlb_op[MEM];
        csr_num[WB]    <= csr_num[MEM];
        // IPE is judged in MEM (it needs the CRMD value after every older write),
        // but it takes effect at the same WB point as the CSR access it forbids
        csr_ctl[WB]    <= csr_ctl[MEM]
                        | ((mem_ipe | (| mem_tlb_cause)) ? (9'h1 << CSR_EXCP) : 9'h0);
        csr_wdata[WB]  <= csr_wdata[MEM];
        csr_mask[WB]   <= csr_mask[MEM];
        excp_cause[WB] <= excp_cause[MEM] | (mem_ipe ? (13'h1 << EXCP_IPE) : 13'h0)
                         | mem_tlb_cause;
        wb_excp_badv   <= (excp_cause[MEM][EXCP_ALE] | (| mem_tlb_cause))
                          ? result[MEM] : pc[MEM];
    end
end

// ADEF arrives from IF, the other three are decoded here
assign excp_cause_id[EXCP_SYS]  = inst_syscall;
assign excp_cause_id[EXCP_BRK]  = inst_break;
assign excp_cause_id[EXCP_INE]  = inst_ine | (inst_invtlb & (inst[4:0] > 5'd6));
assign excp_cause_id[EXCP_ADEF] = 1'b0;
assign excp_cause_id[EXCP_ALE]  = 1'b0;      // ALE / interrupt are judged in MEM
assign excp_cause_id[EXCP_INT]  = 1'b0;
assign excp_cause_id[EXCP_IPE]  = 1'b0;      // IPE is judged in MEM (needs crmd_plv_o)
assign excp_cause_id[EXCP_TLBR] = 1'b0;      // the TLB kinds are judged at the mmu's tlb cycle
assign excp_cause_id[EXCP_PIF]  = 1'b0;
assign excp_cause_id[EXCP_PIL]  = 1'b0;
assign excp_cause_id[EXCP_PIS]  = 1'b0;
assign excp_cause_id[EXCP_PME]  = 1'b0;
assign excp_cause_id[EXCP_PPI]  = 1'b0;

always @(posedge clk) begin
    if (allow[MEM]) begin
        res_from_mem[MEM] <= res_from_mem[EX];
        mem_we[MEM]       <= mem_we[EX];
        mem_en[MEM]       <= mem_en[EX];
        mem_wdata[MEM]    <= mem_wdata[EX];
        mem_size[MEM]     <= mem_size[EX];
        mem_zext[MEM]     <= mem_zext[EX];
    end
end

// write-back control
always @(*) begin
    gr_we[ID] = ~inst_st_w & ~inst_beq & ~inst_bne & ~inst_b
              & ~inst_st_b & ~inst_st_h
              & ~inst_blt & ~inst_bge & ~inst_bltu & ~inst_bgeu
              & ~inst_ertn & ~inst_syscall & ~inst_break & ~inst_idle
              & ~inst_tlb_any;
    dest[ID]  = inst_bl ? 5'd1 : rd;
end

always @(posedge clk) begin
    if (allow[MEM]) begin
        gr_we[MEM] <= gr_we[EX];
        dest[MEM]  <= dest[EX];
    end
    if (allow[WB]) begin
        gr_we[WB] <= gr_we[MEM];
        dest[WB]  <= dest[MEM];
    end
end

assign need_ui5   =  inst_slli_w | inst_srli_w | inst_srai_w;
assign need_ui12  =  inst_andi | inst_ori | inst_xori;
assign need_si12  =  inst_addi_w | inst_ld_w | inst_st_w | inst_slti | inst_sltui
                  |  inst_ld_b | inst_ld_h | inst_ld_bu | inst_ld_hu | inst_st_b | inst_st_h;
assign need_si16  =  inst_jirl | inst_beq | inst_bne;
assign need_si20  =  inst_lu12i_w | inst_pcaddu12i;
assign src2_is_4  =  inst_jirl | inst_bl;

assign imm = src2_is_4 ? 32'h4                      :
             need_si20 ? {i20[19:0], 12'b0}         :
             need_ui12 ? {20'b0, i12[11:0]}         :
/*need_ui5 || need_si12*/{{20{i12[11]}}, i12[11:0]} ;

// b / bl are pc-relative with an i26 offset and are resolved at ID; the
// conditional branches and jirl use the i16 offset and are resolved in EX.
assign br_offs_i26 = {{ 4{i26[25]}}, i26[25:0], 2'b0};
assign br_offs_i16 = {{14{i16[15]}}, i16[15:0], 2'b0};

assign src_reg_is_rd = inst_beq | inst_bne | inst_st_w | inst_st_b | inst_st_h
                     | inst_blt | inst_bge | inst_bltu | inst_bgeu
                     | inst_csrwr | inst_csrxchg;   // rf_raddr2 = rd (CSR write data)

assign src1_is_pc    = inst_jirl | inst_bl | inst_pcaddu12i;

assign src2_is_imm   = inst_slli_w |
                       inst_srli_w |
                       inst_srai_w |
                       inst_addi_w |
                       inst_slti   |
                       inst_sltui  |
                       inst_andi   |
                       inst_ori    |
                       inst_xori   |
                       inst_ld_w   |
                       inst_st_w   |
                       inst_ld_b   |
                       inst_ld_h   |
                       inst_ld_bu  |
                       inst_ld_hu  |
                       inst_st_b   |
                       inst_st_h   |
                       inst_lu12i_w|
                       inst_pcaddu12i |
                       inst_jirl   |
                       inst_bl     ;

assign rf_raddr1 = rj;
assign rf_raddr2 = src_reg_is_rd ? rd :rk;

assign rj_value  = fw1_data;

assign br_taken_id  = (inst_b | inst_bl) & cur_valid[ID];
assign br_target_id = pc[ID] + br_offs_i26;

assign br_op_id   = {inst_jirl, inst_bgeu, inst_bltu, inst_bge, inst_blt, inst_bne, inst_beq};
assign br_base_id = inst_jirl ? rj_value : pc[ID];

// EX stage
// [EX] stage view comes straight from EX
always @(*) begin
    pc[EX]           = ex_pc_o;
    src1_used[EX]    = ex_src1_used_o;
    src2_used[EX]    = ex_src2_used_o;
    res_from_mem[EX] = ex_res_from_mem_o;
    mem_we[EX]       = ex_mem_we_o;
    mem_en[EX]       = ex_mem_en_o;
    mem_wdata[EX]    = ex_mem_wdata_o;
    mem_size[EX]     = ex_mem_size_o;
    mem_zext[EX]     = ex_mem_zext_o;
    gr_we[EX]        = ex_gr_we_o;
    dest[EX]         = ex_dest_o;
    csr_num[EX]      = ex_csr_num_o;
    csr_ctl[EX]      = ex_csr_ctl_o;
    tlb_op[EX]       = ex_tlb_op_o;
    invtlb_op[EX]    = ex_invtlb_op_o;
    excp_cause[EX]   = ex_excp_cause_o;
    csr_wdata[EX]    = ex_csr_wdata_o;
    csr_mask[EX]     = ex_csr_mask_o;
    valid[EX]        = ex_valid_o;
end

assign ex_mem_ale  = mem_en[EX] & (ex_mem_half ? ex_result[0] :
                                   ex_mem_word ? | ex_result[1:0] : 1'b0);
assign int_take    = int_o;

assign ex_mem_excp = ex_mem_ale | int_take | csr_ctl[EX][CSR_EXCP];

// MEM stage
always @(posedge clk) begin
    if (allow[MEM])     result[MEM] <= ex_result;
    if (allow[WB])      result[WB]  <= res_from_mem[MEM] ? mem_result : result[MEM];
end

// a load and a store are both registered at the end of EX, so both issue in the
// first MEM cycle; a store will later gain its own state for the cache write
assign mem_pre   = cur_valid[EX] & mem_en[EX] & ~ ex_mem_excp;
assign mem_req   = cur_valid[MEM] & mem_en[MEM];
assign mem_va    = result[MEM];
// MEM found the access illegal, or the whole pipeline is being flushed at WB: either
// way the mmu must not issue this access
assign mem_abort = mem_excp | flush[MEM];

// byte / halfword / word store: broadcast the stored data to every byte lane
// and let the write-enable mask pick the addressed lane(s).
// mem_byte / mem_half / mem_word are exclusive, so every select below is a flat
// masked-OR instead of a priority chain.
assign mem_byte = mem_size[MEM][SIZE_BYTE];
assign mem_half = mem_size[MEM][SIZE_HALF];
assign mem_word = mem_size[MEM][SIZE_WORD];

assign ex_mem_byte = mem_size[EX][SIZE_BYTE];   // size as seen in EX (ALE judge)
assign ex_mem_half = mem_size[EX][SIZE_HALF];
assign ex_mem_word = mem_size[EX][SIZE_WORD];

assign st_we_byte = 4'b0001 << result[MEM][1:0];
assign st_we_half = result[MEM][1] ? 4'b1100 : 4'b0011;
assign st_we_word = 4'b1111;

assign st_we = ({4{mem_byte}} & st_we_byte) | ({4{mem_half}} & st_we_half) | ({4{mem_word}} & st_we_word);

assign mem_wstrb  = {4{mem_we[MEM] && valid[MEM] && ~ mem_excp}} & st_we;
assign mem_stdata = ({32{mem_byte}} & {4{mem_wdata[MEM][ 7: 0]}}) |
                    ({32{mem_half}} & {2{mem_wdata[MEM][15: 0]}}) |
                    ({32{mem_word}} &    mem_wdata[MEM]);

// byte / halfword / word load: take the addressed lane and extend it
assign ld_byte = (result[MEM][1:0] == 2'd0) ? mem_rdata[ 7: 0] :
                 (result[MEM][1:0] == 2'd1) ? mem_rdata[15: 8] :
                 (result[MEM][1:0] == 2'd2) ? mem_rdata[23:16] :
                                              mem_rdata[31:24];
assign ld_half = result[MEM][1] ? mem_rdata[31:16] : mem_rdata[15:0];

assign ld_byte_ext = mem_zext[MEM] ? {24'b0, ld_byte} : {{24{ld_byte[7]}}, ld_byte};
assign ld_half_ext = mem_zext[MEM] ? {16'b0, ld_half} : {{16{ld_half[15]}}, ld_half};

assign mem_result = ({32{mem_byte}} & ld_byte_ext)  |
                    ({32{mem_half}} & ld_half_ext)  |
                    ({32{mem_word}} & mem_rdata);

assign mem_ipe = valid[MEM] & csr_ctl[MEM][CSR_PRIV] & (crmd_plv_o != 2'b0);

assign mem_excp = csr_ctl[MEM][CSR_EXCP] | mem_ipe | (| mem_tlb_cause);

// tlbsrch only reports where the entry lives (TLBIDX.Index/NE), which is not
// part of the translation, so it neither flushes nor re-fetches
assign tlb_op_map = (| tlb_op[MEM]) & ~ tlb_op[MEM][TLB_SRCH];
assign mem_tlb    = valid[MEM] & tlb_op_map;

// CRMD / DMW / ASID all feed the translation, so writing any of them must restart
// the fetch: a younger access would otherwise use a mix of old and new state
assign mem_map_wr   = csr_ctl[MEM][CSR_WE] & (csr_num[MEM] == CSR_CRMD
                                            | csr_num[MEM] == CSR_DMW0
                                            | csr_num[MEM] == CSR_DMW1
                                            | csr_num[MEM] == CSR_ASID);

assign mem_crmd_tlb = (mem_map_wr & valid[MEM]) | mem_tlb;


always @(posedge clk) begin
    if (reset) wb_crmd_tlb <= 1'b0;
    else       wb_crmd_tlb <= mem_crmd_tlb;
end

// WB stage
assign wb_flush = wb_excp_ertn | wb_crmd_tlb;

assign rf_we    = gr_we[WB] && valid[WB] && ~ csr_ctl[WB][CSR_EXCP];
assign rf_waddr = dest[WB];
assign rf_wdata = csr_ctl[WB][CSR_RD]      ? csr_rdata      :
                  csr_ctl[WB][CSR_RDCNTID] ? csr_rdata      :
                  csr_ctl[WB][CSR_RDCNTVL] ? timer_o[31: 0] :
                  csr_ctl[WB][CSR_RDCNTVH] ? timer_o[63:32] :
                                             result[WB];

// redirect signal at WB stage
assign excp_i       = valid[WB] & csr_ctl[WB][CSR_EXCP];
// Exception wins over ertn
assign ertn_i       = valid[WB] & csr_ctl[WB][CSR_ERTN] & ~ excp_i;
assign wb_excp_ertn = excp_i | ertn_i;

assign excp_ecode    = excp_cause[WB][EXCP_INT]  ? 6'h0 :
                       excp_cause[WB][EXCP_IPE]  ? 6'he :
                       excp_cause[WB][EXCP_ALE]  ? 6'h9 :
                       excp_cause[WB][EXCP_ADEF] ? 6'h8 :
                       excp_cause[WB][EXCP_TLBR] ? 6'h3f :
                       excp_cause[WB][EXCP_PIF]  ? 6'h3 :
                       excp_cause[WB][EXCP_PIL]  ? 6'h1 :
                       excp_cause[WB][EXCP_PIS]  ? 6'h2 :
                       excp_cause[WB][EXCP_PPI]  ? 6'h7 :
                       excp_cause[WB][EXCP_PME]  ? 6'h4 :
                       excp_cause[WB][EXCP_INE]  ? 6'hd :
                       excp_cause[WB][EXCP_BRK]  ? 6'hc :
                                                   6'hb;   // syscall

assign excp_tlb      = excp_cause[WB][EXCP_TLBR] | excp_cause[WB][EXCP_PIF]
                     | excp_cause[WB][EXCP_PIL]  | excp_cause[WB][EXCP_PIS]
                     | excp_cause[WB][EXCP_PPI]  | excp_cause[WB][EXCP_PME];

assign excp_badv_en  = excp_i & ~ excp_cause[WB][EXCP_INT]
                              & (excp_cause[WB][EXCP_ALE] | excp_cause[WB][EXCP_ADEF]
                                 | excp_tlb);

assign csr_we    = valid[WB] & csr_ctl[WB][CSR_WE] & ~ csr_ctl[WB][CSR_EXCP];
assign csr_raddr = csr_num[WB];
assign csr_waddr = csr_num[WB];
assign csr_wdata_w = csr_wdata[WB];

regfile u_regfile(
    .clk    (clk      ),
    .raddr1 (rf_raddr1),
    .rdata1 (rf_rdata1),
    .raddr2 (rf_raddr2),
    .rdata2 (rf_rdata2),
    .we     (rf_we    ),
    .waddr  (rf_waddr ),
    .wdata  (rf_wdata )
);

EX u_ex(
    .clk            (clk              ),
    .reset          (reset            ),
    .flush_i        (flush[EX]        ),
    .allow_next     (allow[MEM]       ),
  
    .valid_i        (next_valid[EX]   ),
    .pc_i           (pc[ID]           ),
    .src1_used_i    (src1_used[ID]    ),
    .src2_used_i    (src2_used[ID]    ),
    .alu_op_i       (alu_op_id        ),
    .src1_i         (src1_id          ),
    .src2_i         (src2_id          ),
    .br_op_i        (br_op_id         ),
    .br_base_i      (br_base_id       ),
    .br_offs_i      (br_offs_i16      ),
    .md_op_i        (md_op_id         ),
    .csr_num_i      (csr_num[ID]      ),
    .csr_ctl_i      (csr_ctl[ID]      ),
    .excp_cause_i   (excp_cause[ID]   ),
    .csr_wdata_i    (csr_wdata[ID]    ),
    .csr_mask_i     (csr_mask[ID]     ),
    .res_from_mem_i (res_from_mem[ID] ),
    .mem_we_i       (mem_we[ID]       ),
    .mem_en_i       (mem_en[ID]       ),
    .mem_wdata_i    (mem_wdata[ID]    ),
    .mem_size_i     (mem_size[ID]     ),
    .mem_zext_i     (mem_zext[ID]     ),
    .gr_we_i        (gr_we[ID]        ),
    .dest_i         (dest[ID]         ),
 
    .pc_o           (ex_pc_o          ),
    .src1_used_o    (ex_src1_used_o   ),
    .src2_used_o    (ex_src2_used_o   ),
    .res_from_mem_o (ex_res_from_mem_o),
    .mem_we_o       (ex_mem_we_o      ),
    .mem_en_o       (ex_mem_en_o      ),
    .mem_wdata_o    (ex_mem_wdata_o   ),
    .mem_size_o     (ex_mem_size_o    ),
    .mem_zext_o     (ex_mem_zext_o    ),
    .gr_we_o        (ex_gr_we_o       ),
    .dest_o         (ex_dest_o        ),
    .csr_num_o      (ex_csr_num_o     ),
    .csr_ctl_o      (ex_csr_ctl_o     ),
    .excp_cause_o   (ex_excp_cause_o  ),
    .csr_wdata_o    (ex_csr_wdata_o   ),
    .csr_mask_o     (ex_csr_mask_o    ),
    .tlb_op_i       (tlb_op[ID]       ),
    .tlb_op_o       (ex_tlb_op_o      ),
    .invtlb_op_i    (invtlb_op[ID]    ),
    .invtlb_op_o    (ex_invtlb_op_o   ),
 
    .ex_result      (ex_result        ),
    .valid_o        (ex_valid_o       ),
    .stall_o        (ex_stall         ),
    .br_taken_o     (br_taken_ex      ),
    .br_target_o    (br_target_ex     ),

    .fw_raddr1_i    (rf_raddr1        ),
    .fw_raddr2_i    (rf_raddr2        ),
    .fw1_hit_o      (fw1_ex_hit_o     ),
    .fw1_valid_o    (fw1_ex_valid_o   ),
    .fw1_data_o     (fw1_ex_data_o    ),
    .fw2_hit_o      (fw2_ex_hit_o     ),
    .fw2_valid_o    (fw2_ex_valid_o   ),
    .fw2_data_o     (fw2_ex_data_o    )
);

// The TLB instructions write/read/invalidate the tlb at WB.  The fetch (s0) and
// the MEM access (s1) search it; a TLBSRCH in WB takes s1 over, which is safe
// because by then its own flush has turned the MEM slot into a bubble.
wire        tlb_s0_found, tlb_s1_found;
wire [19:0] tlb_s0_ppn,   tlb_s1_ppn;
wire [ 5:0] tlb_s0_ps,    tlb_s1_ps;
wire [ 3:0] tlb_s1_index;
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

assign wb_tlb_commit = valid[WB] & (| tlb_op[WB]) & ~ csr_ctl[WB][CSR_EXCP];
assign wb_tlb_srch   = wb_tlb_commit & tlb_op[WB][TLB_SRCH];
assign wb_tlb_rd     = wb_tlb_commit & tlb_op[WB][TLB_RD];
assign wb_tlb_wr     = wb_tlb_commit & (tlb_op[WB][TLB_WR] | tlb_op[WB][TLB_FILL]);
assign wb_tlb_inv    = wb_tlb_commit & tlb_op[WB][TLB_INV];

// write / invalidate keys: invtlb reuses the write port (rj -> ASID, rk -> VPPN)
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

// search keys: s0 = the fetch address, s1 = the MEM address (TLBSRCH in WB takes s1)
assign tlb_s0_vppn     = pc[IF][31:13];
assign tlb_s0_va_bit12 = pc[IF][12];
// s1 always searches the MEM address; TLBSRCH has its own port (s2), so no
// WB stage signal leaks into the MEM exception path any more
assign tlb_s1_vppn     = result[MEM][31:13];
assign tlb_s1_va_bit12 = result[MEM][12];

// TLBRD hands the entry back to the CSRs
assign tlb_rd_lo0 = {4'b0, tlb_r_ppn0, 1'b0, tlb_r_g, tlb_r_mat0, tlb_r_plv0, tlb_r_d0, tlb_r_v0};
assign tlb_rd_lo1 = {4'b0, tlb_r_ppn1, 1'b0, tlb_r_g, tlb_r_mat1, tlb_r_plv1, tlb_r_d1, tlb_r_v1};

assign tlb_csr_sel = tlb_op[WB][TLB_SRCH] ? 2'd1 :
                     tlb_op[WB][TLB_RD]   ? 2'd2 : 2'd0;

// dmw / tlb exceptions
// mapping is only active in PG mode; a DMW hit wins over the TLB and also
// suppresses every TLB exception
assign map_mode = crmd_pg_o & ~ crmd_da_o;

assign dmw0_hit_if  = map_mode & ((crmd_plv_o == 2'd0) ? dmw0_o[0] : dmw0_o[3]) & (pc[IF][31:29] == dmw0_o[31:29]);
assign dmw1_hit_if  = map_mode & ((crmd_plv_o == 2'd0) ? dmw1_o[0] : dmw1_o[3]) & (pc[IF][31:29] == dmw1_o[31:29]);
assign dmw0_hit_mem = map_mode & ((crmd_plv_o == 2'd0) ? dmw0_o[0] : dmw0_o[3]) & (result[MEM][31:29] == dmw0_o[31:29]);
assign dmw1_hit_mem = map_mode & ((crmd_plv_o == 2'd0) ? dmw1_o[0] : dmw1_o[3]) & (result[MEM][31:29] == dmw1_o[31:29]);

assign if_dmw_hit  = dmw0_hit_if  | dmw1_hit_if;
assign mem_dmw_hit = dmw0_hit_mem | dmw1_hit_mem;
assign if_dmw_pa   = {(dmw0_hit_if  ? dmw0_o[27:25] : dmw1_o[27:25]), pc[IF][28:0]};
assign mem_dmw_pa  = {(dmw0_hit_mem ? dmw0_o[27:25] : dmw1_o[27:25]), result[MEM][28:0]};

assign if_xlat  = tlb_s0_found & map_mode;
assign mem_xlat = tlb_s1_found & map_mode;

// fetch: ADEF is born on nextpc, these three are checked when the mmu issues
assign if_fetch_cause = (if_xlat | (~ if_dmw_hit & map_mode)) ?
                        ((map_mode & ~ if_dmw_hit & ~ tlb_s0_found)             ? (13'h1 << EXCP_TLBR) :
                         (map_mode & ~ if_dmw_hit & ~ tlb_s0_v)                 ? (13'h1 << EXCP_PIF ) :
                         (map_mode & ~ if_dmw_hit & (crmd_plv_o > tlb_s0_plv))  ? (13'h1 << EXCP_PPI ) : 13'h0)
                        : 13'h0;
assign if_abort       = | if_fetch_cause;

// mem: ALE / INT are already folded into the slot, so they win
assign mem_tlb_chk = valid[MEM] & mem_en[MEM] & ~ csr_ctl[MEM][CSR_EXCP];
assign mem_tlb_cause = ({13{mem_tlb_chk}} &
     ( (map_mode & ~ mem_dmw_hit & ~ tlb_s1_found)                        ? (13'h1 << EXCP_TLBR) :
       (map_mode & ~ mem_dmw_hit & ~ tlb_s1_v)                            ? (13'h1 << (mem_we[MEM] ? EXCP_PIS : EXCP_PIL)) :
       (map_mode & ~ mem_dmw_hit & (crmd_plv_o > tlb_s1_plv))             ? (13'h1 << EXCP_PPI ) :
       (map_mode & ~ mem_dmw_hit & mem_we[MEM] & ~ tlb_s1_d)              ? (13'h1 << EXCP_PME ) :
                                                                           13'h0));

// a refill goes to TLBRENTRY (physical), everything else to EENTRY
assign excp_entry = excp_cause[WB][EXCP_TLBR] ? tlbrentry_o : eentry_o;

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
    .s0_mat       (               ),
    .s0_d         (               ),   // D only guards stores (PME), a fetch never looks at it
    .s0_v         (tlb_s0_v       ),

    .s1_vppn      (tlb_s1_vppn    ),
    .s1_va_bit12  (tlb_s1_va_bit12),
    .s1_asid      (asid_o         ),
    .s1_found     (tlb_s1_found   ),
    .s1_index     (               ),
    .s1_ppn       (tlb_s1_ppn     ),
    .s1_ps        (tlb_s1_ps      ),
    .s1_plv       (tlb_s1_plv     ),
    .s1_mat       (               ),
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
    .excp_i          (excp_i       ),
    .excp_pc_i       (pc[WB]       ),
    .excp_ecode_i    (excp_ecode   ),
    .excp_esubcode_i (9'h0         ),
    .excp_badv_en_i  (excp_badv_en ),
    .excp_badv_i     (wb_excp_badv ),
    .excp_tlb_i      (excp_i & excp_tlb),
    .ertn_i          (ertn_i       ),
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

mmu u_mmu(
    .clk            (clk            ),
    .reset          (reset          ),

    .if_req         (inst_req       ),
    .if_va          (pc[IF]         ),
    .if_allow       (allow[IF]      ),
    .if_flush       (flush[IF]      ),
    .if_pending     (if_pending     ),
    .if_rdata       (inst_rdata     ),

    .mem_pre        (mem_pre        ),
    .mem_req        (mem_req        ),
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
assign debug_wb_rf_we    = {4{rf_we}};  // only support word read
assign debug_wb_rf_wnum  = dest[WB];
assign debug_wb_rf_wdata = rf_wdata;

endmodule
