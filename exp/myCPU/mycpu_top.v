module mycpu_top(
    input  wire        clk,
    input  wire        resetn,
    // inst sram interface
    output wire        inst_sram_en,
    output wire [3:0]  inst_sram_we,
    output wire [31:0] inst_sram_addr,
    output wire [31:0] inst_sram_wdata,
    input  wire [31:0] inst_sram_rdata,
    // data sram interface
    output wire        data_sram_en,
    output wire [3:0]  data_sram_we,
    output wire [31:0] data_sram_addr,
    output wire [31:0] data_sram_wdata,
    input  wire [31:0] data_sram_rdata,
    // trace debug interface
    output wire [31:0] debug_wb_pc,
    output wire [ 3:0] debug_wb_rf_we,
    output wire [ 4:0] debug_wb_rf_wnum,
    output wire [31:0] debug_wb_rf_wdata
);

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

// csr_ctl bit positions (exp12/13)
localparam CSR_RD            = 0;   // csrrd/csrwr/csrxchg: rd <- CSR[csr_num]
localparam CSR_WE            = 1;   // csrwr/csrxchg: write the CSR
localparam CSR_ERTN          = 2;   // ertn
localparam CSR_EXCP          = 3;   // this slot carries an exception; kind is in excp_cause
localparam CSR_RDCNTVL       = 4;   // rdcntvl.w
localparam CSR_RDCNTVH       = 5;   // rdcntvh.w
localparam CSR_RDCNTID       = 6;   // rdcntid
localparam CSR_WB            = 7;   // rd comes from csr.v at WB

// excp_cause bit positions: one-hot, registered from IF (ADEF is born there) all
// the way to WB.  WB is where the exception entry ports are driven from, i.e. the
// same point as every other CSR access.
localparam EXCP_SYS          = 0;   // syscall
localparam EXCP_BRK          = 1;   // break
localparam EXCP_INE          = 2;   // instruction not exist
localparam EXCP_ADEF         = 3;   // fetch address not 4-byte aligned
localparam EXCP_ALE          = 4;   // load / store not naturally aligned (judged in MEM)
localparam EXCP_INT          = 5;   // interrupt (judged in MEM)

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
// The [EX] elements are combinational aliases of the EX.v outputs
reg  [31:0] pc [WB:IF] /* verilator split_var */;
reg  [31:0] br_pc_reg;
reg         br_reg;

// IF -> ID
reg  [31:0] inst [ID:IF];

// ID: combinational, generated from inst[ID]; these only exist at ID
reg  [11:0] alu_op_id;
reg  [31:0] alu_src1_id;
reg  [31:0] alu_src2_id;
reg  [MOD_WU:0] md_op_id;

// stage fields: [ID] combinational, [EX] aliases EX.v, [MEM]/[WB] registered
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
reg  [ 7:0] csr_ctl [WB:ID] /* verilator split_var */;
reg  [ 5:0] excp_cause [WB:IF] /* verilator split_var */;
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

// write-first CRMD view
/* verilator lint_off UNUSEDSIGNAL */
wire [ 1:0] crmd_plv_o;
wire        crmd_ie_o;
/* verilator lint_on UNUSEDSIGNAL */
wire        int_o;
wire        if_adef;

// exception / ertn / interrupt
wire        mem_ale;        // this load / store is not naturally aligned
wire        int_take;       // an interrupt is pending
wire        mem_excp;       // the instruction in MEM must not access memory
wire        mem_excp_ertn;
wire        wb_excp_ertn;     // the WB instruction is entering the exception / ertn
wire        excp_i;
wire        ertn_i;
wire [ 5:0] excp_ecode;
wire        excp_badv_en;
reg  [31:0] excp_badv_wb;   // BADV of the instruction now in WB

// EX.v outputs, aliased into the [EX] elements of the stage fields above
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
wire [ 7:0] ex_csr_ctl_o;
wire [ 5:0] ex_excp_cause_o;
wire [ 5:0] excp_cause_id;
wire [31:0] ex_csr_wdata_o;
wire [31:0] ex_csr_mask_o;
wire        ex_valid_o;
wire        ex_stall;

wire [31:0] ex_result;      // EX.v combinational output (this is the [EX] result)

// EX.v forwarding
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
wire [ 3:0] st_we_byte, st_we_half, st_we_word, st_we;
wire [ 7:0] ld_byte;
wire [15:0] ld_half;
wire [31:0] ld_byte_ext, ld_half_ext;

// forwarding
wire [MEM:EX] fw1_hit     ;
wire [MEM:EX] fw2_hit     ;
wire [MEM:EX] fw1_valid   ;
wire [MEM:EX] fw2_valid   ;
wire [MEM:EX] fw1_pending ;
wire [MEM:EX] fw2_pending ;
wire        fw1_mem_sel,  fw2_mem_sel;  // final mux: take the operand from MEM
wire        fw1_rf_sel,   fw2_rf_sel;   // final mux: no tap hit, plain register file read
wire [31:0] fw1_data,     fw2_data;     // final operand


// fake axi path: handshake state
reg  IF_req_valid;
wire fake_IF_req_ready;
wire IF_req_fire;

wire fake_IF_rsp_valid;
reg  IF_rsp_ready;
wire IF_rsp_fire_wire;
reg  IF_rsp_fire_reg;
wire IF_rsp_fire;

reg  MEM_req_valid;
wire fake_MEM_req_ready;
wire MEM_req_fire_wire;
reg  MEM_req_fire_reg;
wire MEM_req_fire;

wire fake_MEM_rsp_valid;
reg  MEM_rsp_ready;
wire MEM_rsp_fire_wire;
reg  MEM_rsp_fire_reg;
wire MEM_rsp_fire;

// fake axi path
always @(posedge clk) begin
    if (reset)                          IF_req_valid <= 1'b0;
    else if (allow[IF])                 IF_req_valid <= 1'b1;
    else if (fake_IF_req_ready)         IF_req_valid <= 1'b0;
    else                                IF_req_valid <= IF_req_valid; 
end

assign IF_req_fire = IF_req_valid && fake_IF_req_ready;

assign fake_IF_req_ready = IF_req_valid;

always @(posedge clk) begin
    if (reset)                          IF_rsp_fire_reg <= 1'b1;    // to start without stalling IF
    else if (allow[IF])                 IF_rsp_fire_reg <= 1'b0;
    else if (IF_rsp_fire_wire)          IF_rsp_fire_reg <= 1'b1;
    else                                IF_rsp_fire_reg <= IF_rsp_fire_reg;
end

always @(posedge clk) begin
    if (reset)                          IF_rsp_ready    <= 1'b0;
    else if (allow[IF])                 IF_rsp_ready    <= 1'b0;
    else if (IF_req_fire)               IF_rsp_ready    <= 1'b1;
    else                                IF_rsp_ready    <= IF_rsp_ready;
end

assign IF_rsp_fire_wire = IF_rsp_ready && fake_IF_rsp_valid;

assign IF_rsp_fire = IF_rsp_fire_reg | IF_rsp_fire_wire;

assign fake_IF_rsp_valid = IF_rsp_ready;

always @(posedge clk) begin
    if (reset)                          MEM_req_valid <= 1'b0;
    else if (allow[MEM])                MEM_req_valid <= next_valid[MEM] && mem_en[EX];
    else if (fake_MEM_req_ready)        MEM_req_valid <= 1'b0;
    else                                MEM_req_valid <= MEM_req_valid; 
end

always @(posedge clk) begin
    if (reset)                          MEM_req_fire_reg <= 1'b0;
    else if (allow[MEM])                MEM_req_fire_reg <= 1'b0;
    else if (MEM_req_fire_wire)         MEM_req_fire_reg <= 1'b1;
    else                                MEM_req_fire_reg <= MEM_req_fire_reg;
end

assign MEM_req_fire_wire = MEM_req_valid && fake_MEM_req_ready;

assign MEM_req_fire = MEM_req_fire_reg | MEM_req_fire_wire;

assign fake_MEM_req_ready = MEM_req_valid;

always @(posedge clk) begin
    if (reset)                          MEM_rsp_fire_reg <= 1'b0;
    else if (allow[MEM])                MEM_rsp_fire_reg <= 1'b0;
    else if (MEM_rsp_fire)              MEM_rsp_fire_reg <= 1'b1;
    else                                MEM_rsp_fire_reg <= MEM_rsp_fire_reg;
end

always @(posedge clk) begin
    if (reset)                          MEM_rsp_ready    <= 1'b0;
    else if (allow[MEM])                MEM_rsp_ready    <= 1'b0;
    else if (MEM_req_fire)              MEM_rsp_ready    <= ~ mem_we[MEM];
    else                                MEM_rsp_ready    <= MEM_rsp_ready;
end

assign MEM_rsp_fire_wire = MEM_rsp_ready && fake_MEM_rsp_valid;

assign MEM_rsp_fire = MEM_rsp_fire_reg | MEM_rsp_fire_wire;

assign fake_MEM_rsp_valid = MEM_rsp_ready;


assign seq_pc       = pc[IF] + 32'h4;

wire sel_era    = ertn_i;                       // ertn -> era
wire sel_eentry = wb_excp_ertn & ~ ertn_i;      // exception -> eentry (== excp_i)
wire sel_br     = ~ wb_excp_ertn & br_reg;      // latched redirect, loses only to the WB redirect
wire sel_seq    = ~ (wb_excp_ertn | br_reg | br_taken_ex | br_taken_id);

assign nextpc       = ({32{sel_era    }} & era_o       ) |
                      ({32{sel_eentry }} & eentry_o    ) |
                      ({32{sel_br     }} & br_pc_reg   ) |
                      ({32{br_taken_ex}} & br_target_ex) |
                      ({32{br_taken_id}} & br_target_id) |
                      ({32{sel_seq    }} & seq_pc      );

assign br_redirect = br_taken_ex | br_taken_id | wb_excp_ertn;

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
        pc[IF] <= 32'h1bfffffc;     // trick: to make nextpc be 0x1c000000 during reset
        pc[ID] <= 32'h0;
        pc[MEM] <= 32'h0;
        pc[WB] <= 32'h0;
    end
    else begin
        if (allow[IF] ) pc[IF]  <= nextpc;
        if (allow[ID] ) pc[ID]  <= pc[IF];
        if (allow[MEM]) pc[MEM] <= pc[EX];
        if (allow[WB] ) pc[WB]  <= pc[MEM];
    end
end

// hazard unit
assign cur_valid = valid & ~ flush;
assign next_valid = {cur_valid[MEM:IF] & ~stall[MEM:IF], 1'b1} & allow | (~ allow & cur_valid);

// valid[EX] is registered inside EX.v
always @(posedge clk) begin
    if (reset) begin
        valid[IF]  <= 1'b0;
        valid[ID]  <= 1'b0;
        valid[MEM] <= 1'b0;
        valid[WB]  <= 1'b0;
    end
    else begin
        valid[IF]  <= next_valid[IF];
        valid[ID]  <= next_valid[ID];
        valid[MEM] <= next_valid[MEM];
        valid[WB]  <= next_valid[WB];
    end
end

assign flush = mem_excp_ertn ? 5'b00111 :
               br_taken_ex   ? 5'b00011 :
               br_taken_id   ? 5'b00001 :
                               5'b00000;

// allow logic --> generate with stall logic
assign allow[WB]  = 1'b1;
assign allow[MEM] = ~ stall[MEM];
assign allow[EX]  = ~ stall[EX] && allow[MEM];
assign allow[ID]  = ~ stall[ID] && allow[EX];
assign allow[IF]  = ~ stall[IF] && allow[ID];

// ID must wait when any tap of either source is still pending
assign src_wait = | (fw1_pending | fw2_pending);

// stall for IF & MEM
assign stall[IF]  = ~ IF_rsp_fire | mem_excp_ertn;  // should not include valid[IF]
assign stall[ID] = src_wait && cur_valid[ID];
assign stall[EX] = ex_stall;
assign stall[MEM] = valid[MEM] && mem_en[MEM] && ~ mem_excp && (mem_we[MEM] ? ~MEM_req_fire : ~MEM_rsp_fire);
assign stall[WB] = 1'b0;

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

assign fw1_mem_sel = ~ fw1_valid[EX] &   fw1_valid[MEM];
assign fw1_rf_sel  = ~ fw1_valid[EX] & ~ fw1_valid[MEM];
assign fw2_mem_sel = ~ fw2_valid[EX] &   fw2_valid[MEM];
assign fw2_rf_sel  = ~ fw2_valid[EX] & ~ fw2_valid[MEM];

assign fw1_data = ({32{fw1_valid[EX]}} & fw1_ex_data_o) |
                  ({32{fw1_mem_sel}} & result[MEM]) |
                  ({32{fw1_rf_sel}}  & rf_rdata1);

assign fw2_data = ({32{fw2_valid[EX]}} & fw2_ex_data_o) |
                  ({32{fw2_mem_sel}} & result[MEM]) |
                  ({32{fw2_rf_sel}}  & rf_rdata2);

// IF stage
assign inst_sram_en    = IF_req_valid & ~ if_adef;
assign inst_sram_we    = 4'b0;
assign inst_sram_addr  = pc[IF];
assign inst_sram_wdata = 32'b0;
assign inst_rd         = inst_sram_rdata;

assign if_adef = | pc[IF][1:0];

always @(posedge clk) begin
    if (allow[ID]) begin
        if (if_adef)                inst[ID] <= 32'h0340_0000; // nop = andi $r0, $r0, 0
        else if (IF_rsp_fire_wire)  inst[ID] <= inst_rd;
        else                        inst[ID] <= inst[IF];
    end
end

always @(posedge clk) begin
    if (IF_rsp_fire) inst[IF] <= inst_rd;
end

always @(posedge clk) begin
    if (reset)          excp_cause[IF] <= 6'h0;
    else if (allow[ID]) excp_cause[IF] <= if_adef ? (6'h1 << EXCP_ADEF) : 6'h0;
end

// ID stage
assign op_31_26  = inst[ID][31:26];
assign op_25_22  = inst[ID][25:22];
assign op_21_20  = inst[ID][21:20];
assign op_19_15  = inst[ID][19:15];

assign rd   = inst_rdcntid ? inst[ID][9:5] : inst[ID][4:0];   // rdcntid keeps rd in [9:5]
assign rj   = inst[ID][ 9: 5];
assign rk   = inst[ID][14:10];

assign i12  = inst[ID][21:10];
assign i20  = inst[ID][24: 5];
assign i16  = inst[ID][25:10];
assign i26  = {inst[ID][ 9: 0], inst[ID][25:10]};

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
assign inst_lu12i_w= op_31_26_d[6'h05] & ~inst[ID][25];

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
assign inst_csr_any = op_31_26_d[6'h01] & ~ inst[ID][25] & ~ inst[ID][24];
assign inst_csrrd   = inst_csr_any & (inst[ID][9:5] == 5'd0);
assign inst_csrwr   = inst_csr_any & (inst[ID][9:5] == 5'd1);
assign inst_csrxchg = inst_csr_any & ~ inst_csrrd & ~ inst_csrwr;
assign inst_ertn    = op_31_26_d[6'h01] & inst[ID][25] & ~ inst[ID][24] & op_19_15_d[5'h10];
assign inst_idle    = op_31_26_d[6'h01] & inst[ID][25] & ~ inst[ID][24] & op_19_15_d[5'h11];

assign inst_syscall = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h16];
assign inst_break   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h14];

// rdcntvl.w / rdcntvh.w
assign inst_rdcntvl_w = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h0] & op_19_15_d[5'h00]
                      & (inst[ID][14:10] == 5'b11000) & (inst[ID][9:5] == 5'd0);
assign inst_rdcntvh_w = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h0] & op_19_15_d[5'h00]
                      & (inst[ID][14:10] == 5'b11001) & (inst[ID][9:5] == 5'd0);
assign inst_rdcntid   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h0] & op_19_15_d[5'h00]
                      & (inst[ID][14:10] == 5'b11000) & (inst[ID][4:0] == 5'd0) & ~ inst_rdcntvl_w;

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
                  | inst_csrxchg;          // rj is the csrxchg write mask
    src2_used[ID] = inst_add_w | inst_sub_w | inst_slt | inst_sltu
                  | inst_and | inst_or | inst_nor | inst_xor
                  | inst_st_w | inst_beq | inst_bne
                  | inst_sll_w | inst_srl_w | inst_sra_w
                  | inst_mul_w | inst_mulh_w | inst_mulh_wu
                  | inst_div_w | inst_mod_w | inst_div_wu | inst_mod_wu
                  | inst_blt | inst_bge | inst_bltu | inst_bgeu
                  | inst_st_b | inst_st_h
                  | inst_csrwr | inst_csrxchg;   // rd is the CSR write data
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
    alu_src1_id = src1_is_pc  ? pc[ID] : fw1_data;
    alu_src2_id = src2_is_imm ? imm    : fw2_data;
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
    csr_num[ID] = inst_rdcntid ? 14'h040 : inst[ID][23:10];

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
    excp_cause[ID]           = excp_cause[IF] | excp_cause_id;
    csr_ctl[ID][CSR_EXCP]    = | excp_cause[ID];
end

always @(posedge clk) begin
    if (allow[MEM]) begin
        csr_num[MEM]    <= csr_num[EX];
        csr_ctl[MEM]    <= csr_ctl[EX];
        csr_wdata[MEM]  <= csr_wdata[EX];
        csr_mask[MEM]   <= csr_mask[EX];
        excp_cause[MEM] <= excp_cause[EX];
    end
    if (allow[WB]) begin
        csr_num[WB]    <= csr_num[MEM];
        csr_ctl[WB]    <= csr_ctl[MEM] | (mem_excp ? (8'h1 << CSR_EXCP) : 8'h0);
        csr_wdata[WB]  <= csr_wdata[MEM];
        csr_mask[WB]   <= csr_mask[MEM];
        // ALE / interrupt / ertn are judged in MEM
        excp_cause[WB] <= excp_cause[MEM]
                          | (mem_ale  ? (6'h1 << EXCP_ALE) : 6'h0)
                          | (int_take ? (6'h1 << EXCP_INT) : 6'h0);
        excp_badv_wb   <= mem_ale ? result[MEM] : pc[MEM];
    end
end

// ADEF arrives from IF, the other three are decoded here
assign excp_cause_id[EXCP_SYS]  = inst_syscall;
assign excp_cause_id[EXCP_BRK]  = inst_break;
assign excp_cause_id[EXCP_INE]  = inst_ine;
assign excp_cause_id[EXCP_ADEF] = 1'b0;
assign excp_cause_id[EXCP_ALE]  = 1'b0;      // ALE / interrupt are judged in MEM
assign excp_cause_id[EXCP_INT]  = 1'b0;

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
              & ~inst_ertn & ~inst_syscall & ~inst_break & ~inst_idle;
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

assign rj_value  = fw1_data;

assign br_taken_id  = (inst_b | inst_bl) & cur_valid[ID];
assign br_target_id = pc[ID] + br_offs_i26;

assign br_op_id   = {inst_jirl, inst_bgeu, inst_bltu, inst_bge, inst_blt, inst_bne, inst_beq};
assign br_base_id = inst_jirl ? rj_value : pc[ID];

// EX stage
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
    .alu_src1_i     (alu_src1_id      ),
    .alu_src2_i     (alu_src2_id      ),
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
 
    .pc_o           (ex_pc_o             ),
    .src1_used_o    (ex_src1_used_o      ),
    .src2_used_o    (ex_src2_used_o      ),
    .res_from_mem_o (ex_res_from_mem_o   ),
    .mem_we_o       (ex_mem_we_o         ),
    .mem_en_o       (ex_mem_en_o         ),
    .mem_wdata_o    (ex_mem_wdata_o      ),
    .mem_size_o     (ex_mem_size_o       ),
    .mem_zext_o     (ex_mem_zext_o       ),
    .gr_we_o        (ex_gr_we_o          ),
    .dest_o         (ex_dest_o           ),
    .csr_num_o      (ex_csr_num_o        ),
    .csr_ctl_o      (ex_csr_ctl_o        ),
    .excp_cause_o   (ex_excp_cause_o     ),
    .csr_wdata_o    (ex_csr_wdata_o      ),
    .csr_mask_o     (ex_csr_mask_o       ),
 
    .ex_result      (ex_result        ),
    .valid_o        (ex_valid_o       ),
    .stall_o        (ex_stall         ),
    .br_taken_o     (br_taken_ex      ),
    .br_target_o    (br_target_ex     ),

    .fw_raddr1_i    (rf_raddr1       ),
    .fw_raddr2_i    (rf_raddr2       ),
    .fw1_hit_o      (fw1_ex_hit_o    ),
    .fw1_valid_o    (fw1_ex_valid_o  ),
    .fw1_data_o     (fw1_ex_data_o   ),
    .fw2_hit_o      (fw2_ex_hit_o    ),
    .fw2_valid_o    (fw2_ex_valid_o  ),
    .fw2_data_o     (fw2_ex_data_o   )
    );

// [EX] stage view comes straight from EX.v
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
    excp_cause[EX]   = ex_excp_cause_o;
    csr_wdata[EX]    = ex_csr_wdata_o;
    csr_mask[EX]     = ex_csr_mask_o;
    valid[EX]        = ex_valid_o;
end

// MEM stage
always @(posedge clk) begin
    if (allow[MEM]) result[MEM] <= ex_result;
    if (allow[WB])  result[WB]  <= res_from_mem[MEM] ? mem_result : result[MEM];
end

assign data_sram_en    = mem_en[MEM] && valid[MEM] && MEM_req_valid && ~ mem_excp;
assign data_sram_addr  = result[MEM];

// byte / halfword / word store: broadcast the stored data to every byte lane
// and let the write-enable mask pick the addressed lane(s).
// mem_byte / mem_half / mem_word are exclusive, so every select below is a flat
// masked-OR instead of a priority chain.
assign mem_byte = mem_size[MEM][SIZE_BYTE];
assign mem_half = mem_size[MEM][SIZE_HALF];
assign mem_word = mem_size[MEM][SIZE_WORD];

assign st_we_byte = 4'b0001 << result[MEM][1:0];
assign st_we_half = result[MEM][1] ? 4'b1100 : 4'b0011;
assign st_we_word = 4'b1111;

assign st_we = ({4{mem_byte}} & st_we_byte) | ({4{mem_half}} & st_we_half) | ({4{mem_word}} & st_we_word);

assign data_sram_we    = {4{mem_we[MEM] && valid[MEM] && ~ mem_excp}} & st_we;
assign data_sram_wdata = ({32{mem_byte}} & {4{mem_wdata[MEM][ 7: 0]}}) |
                         ({32{mem_half}} & {2{mem_wdata[MEM][15: 0]}}) |
                         ({32{mem_word}} &    mem_wdata[MEM]);

// byte / halfword / word load: take the addressed lane and extend it
assign ld_byte = (result[MEM][1:0] == 2'd0) ? data_sram_rdata[ 7: 0] :
                 (result[MEM][1:0] == 2'd1) ? data_sram_rdata[15: 8] :
                 (result[MEM][1:0] == 2'd2) ? data_sram_rdata[23:16] :
                                              data_sram_rdata[31:24];
assign ld_half = result[MEM][1] ? data_sram_rdata[31:16] : data_sram_rdata[15:0];

assign ld_byte_ext = mem_zext[MEM] ? {24'b0, ld_byte} : {{24{ld_byte[7]}}, ld_byte};
assign ld_half_ext = mem_zext[MEM] ? {16'b0, ld_half} : {{16{ld_half[15]}}, ld_half};

assign mem_result = ({32{mem_byte}} & ld_byte_ext)  |
                    ({32{mem_half}} & ld_half_ext)  |
                    ({32{mem_word}} & data_sram_rdata);

// MEM excp judge
assign mem_ale   = mem_en[MEM] & (mem_half ? result[MEM][0] :
                                  mem_word ? | result[MEM][1:0] : 1'b0);
assign int_take  = int_o;
assign mem_excp  = csr_ctl[MEM][CSR_EXCP] | mem_ale | int_take;

assign mem_excp_ertn   = valid[MEM] & (mem_excp | csr_ctl[MEM][CSR_ERTN]);

// WB stage
assign rf_we    = gr_we[WB] && valid[WB] && ~ csr_ctl[WB][CSR_EXCP];
assign rf_waddr = dest[WB];
assign rf_wdata = csr_ctl[WB][CSR_RD]      ? csr_rdata      :
                  csr_ctl[WB][CSR_RDCNTID] ? csr_rdata      :
                  csr_ctl[WB][CSR_RDCNTVL] ? timer_o[31: 0] :
                  csr_ctl[WB][CSR_RDCNTVH] ? timer_o[63:32] :
                                             result[WB];

// redirect signal at WB stage
assign excp_i       = valid[WB] & csr_ctl[WB][CSR_EXCP];
assign ertn_i       = valid[WB] & csr_ctl[WB][CSR_ERTN];
assign wb_excp_ertn = excp_i | ertn_i;

assign excp_ecode    = excp_cause[WB][EXCP_INT]  ? 6'h0 :
                       excp_cause[WB][EXCP_ALE]  ? 6'h9 :
                       excp_cause[WB][EXCP_ADEF] ? 6'h8 :
                       excp_cause[WB][EXCP_INE]  ? 6'hd :
                       excp_cause[WB][EXCP_BRK]  ? 6'hc :
                                                   6'hb;   // syscall

assign excp_badv_en  = excp_i & ~ excp_cause[WB][EXCP_INT]
                              & (excp_cause[WB][EXCP_ALE] | excp_cause[WB][EXCP_ADEF]);

assign csr_we    = valid[WB] & csr_ctl[WB][CSR_WE];
assign csr_raddr = csr_num[WB];
assign csr_waddr = csr_num[WB];
assign csr_wdata_w = csr_wdata[WB];

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
    .excp_badv_i     (excp_badv_wb ),
    .ertn_i          (ertn_i       ),
    .hwi_i           (8'h0         ),
    .ipi_i           (1'b0         ),
    .crmd_plv_o      (crmd_plv_o   ),
    .crmd_ie_o       (crmd_ie_o    ),
    .eentry_o        (eentry_o     ),
    .era_o           (era_o        ),
    .int_o           (int_o        ),
    .timer_o         (timer_o      )
);

assign debug_wb_pc       = pc[WB];
assign debug_wb_rf_we    = {4{rf_we}};  // only support word read
assign debug_wb_rf_wnum  = dest[WB];
assign debug_wb_rf_wdata = rf_wdata;

endmodule
