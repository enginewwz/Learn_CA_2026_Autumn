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

localparam IF   = 0;
localparam ID   = 1;
localparam EX   = 2;
localparam MEM  = 3;
localparam WB   = 4;
localparam TOTAL_STAGE_WIDTH = 5;

// md_op bit positions: one-hot mul/div operation select
localparam MUL_W    = 0;
localparam MULH_W   = 1;
localparam MULH_WU  = 2;
localparam DIV_W    = 3;
localparam MOD_W    = 4;
localparam DIV_WU   = 5;
localparam MOD_WU   = 6;

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
wire        br_taken;   // generate at ID stage
(* keep = "true" *) wire [31:0] br_target;  // generate at ID stage
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
// [0]=byte access, [1]=halfword access, [2]=load zero-extends (ld.bu / ld.hu)
reg  [ 2:0] mem_size [MEM:ID] /* verilator split_var */;
reg         gr_we [WB:ID] /* verilator split_var */;
reg  [4:0]  dest [WB:ID] /* verilator split_var */;

// EX.v outputs, aliased into the [EX] elements of the stage fields above
wire [31:0] pc_o;
wire        src1_used_o;
wire        src2_used_o;
wire        res_from_mem_o;
wire        mem_we_o;
wire        mem_en_o;
wire [31:0] mem_wdata_o;
wire [ 2:0] mem_size_o;
wire        gr_we_o;
wire  [4:0] dest_o;
wire        valid_o;
wire        ex_stall;

wire [31:0] ex_result;      // EX.v combinational output (this is the [EX] result)

// EX.v forwarding, one group per stage (stage 0 is the youngest)
wire        fw_write_ex_stage0, fw_valid_ex_stage0, fw_write_ex_stage1, fw_valid_ex_stage1;
wire        fw_write_ex_stage2, fw_valid_ex_stage2;
wire [ 4:0] fw_dest_ex_stage0, fw_dest_ex_stage1, fw_dest_ex_stage2;
wire [31:0] fw_data_ex_stage1, fw_data_ex_stage2;

// the instruction's result, one entry per stage it is still live in.  [MEM] is
// the registered copy of EX.v's result: it doubles as the load/store address and
// as the MEM forwarding tap.  [WB] is the value written back, with the data read
// in MEM already folded in (a load only has a result from MEM onwards).
reg  [31:0] result [WB:MEM];

wire        load_op;    // not used

// [EX] stage view comes straight from EX.v
always @(*) begin
    pc[EX]           = pc_o;
    src1_used[EX]    = src1_used_o;
    src2_used[EX]    = src2_used_o;
    res_from_mem[EX] = res_from_mem_o;
    mem_we[EX]       = mem_we_o;
    mem_en[EX]       = mem_en_o;
    mem_wdata[EX]    = mem_wdata_o;
    mem_size[EX]     = mem_size_o;
    gr_we[EX]        = gr_we_o;
    dest[EX]         = dest_o;
    valid[EX]        = valid_o;
end

// mid signals for ID stage
wire        src_reg_is_rd;
wire [31:0] rj_value;
wire [31:0] rkd_value;
wire        rj_eq_rd;
wire        rj_lt_rd;
wire        rj_ltu_rd;
wire [31:0] imm;
wire [31:0] br_offs;
wire [31:0] jirl_offs;
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

// exp10: arithmetic / logic
wire        inst_slti;
wire        inst_sltui;
wire        inst_andi;
wire        inst_ori;
wire        inst_xori;
wire        inst_sll_w;
wire        inst_srl_w;
wire        inst_sra_w;
wire        inst_pcaddu12i;

// exp10: mul / div
wire        inst_mul_w;
wire        inst_mulh_w;
wire        inst_mulh_wu;
wire        inst_div_w;
wire        inst_div_wu;
wire        inst_mod_w;
wire        inst_mod_wu;

// exp11: compare branches and byte / halfword memory
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

wire        need_ui5;
wire        need_ui12;
wire        need_si12;
wire        need_si16;
wire        need_si20;
wire        need_si26;
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

// forwarding
wire [ 2:0] fw1_ex_hit,   fw2_ex_hit;   // per EX stage, bit 0 = the youngest
wire [ 2:0] fw1_ex_sel,   fw2_ex_sel;   // the youngest EX hit as a one-hot
wire [31:0] fw1_ex_data,  fw2_ex_data;  // value held by the selected EX hit
wire [MEM:EX] fw1_hit     ;
wire [MEM:EX] fw2_hit     ;
wire [MEM:EX] fw1_valid   ;
wire [MEM:EX] fw2_valid   ;
wire [MEM:EX] fw1_pending ;
wire [MEM:EX] fw2_pending ;
wire        fw1_mem_sel,  fw2_mem_sel;  // final mux: take the operand from MEM
wire        fw1_rf_sel,   fw2_rf_sel;   // final mux: no tap hit, plain register file read
wire [31:0] fw1_data,     fw2_data;     // final operand


// fake axi path
reg  IF_req_valid;
wire fake_IF_req_ready;
wire IF_req_fire;

wire fake_IF_rsp_valid;
reg  IF_rsp_ready;
wire IF_rsp_fire_wire;
reg  IF_rsp_fire_reg;
wire IF_rsp_fire;

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
assign nextpc       = br_reg ? br_pc_reg : br_taken ? br_target : seq_pc;

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
        else if (br_taken && ~ allow[IF]) begin
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
        if (allow[ID]) pc[ID]  <= pc[IF];
        if (allow[MEM]) pc[MEM] <= pc[EX];
        if (allow[WB]) pc[WB]  <= pc[MEM];
    end
end

// first cycle for new stage
assign cur_valid = valid & ~ flush;
assign next_valid = {cur_valid[TOTAL_STAGE_WIDTH -2: 0] & ~stall[MEM:IF], 1'b1} & allow | (~ allow & cur_valid);

// valid[EX] is registered inside EX.v (see the [EX] alias block), so only the
// other four stages are updated here
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

// flush logic --> only dectected at stage ID
assign flush = br_taken ? 5'b00001 : 5'b00000;

// allow logic --> generate with stall logic
assign allow[WB]  = 1'b1;
assign allow[MEM] = ~ stall[MEM];
assign allow[EX]  = ~ stall[EX] && allow[MEM];
assign allow[ID]  = ~ stall[ID] && allow[EX];
assign allow[IF]  = ~ stall[IF] && allow[ID];

// EX reports its own stall request (div waiting, ALU/mul/div back-pressure)
assign stall[EX] = ex_stall;
assign stall[WB] = 1'b0;

// forwarding logic
assign fw1_ex_hit = {
    src1_used[ID] && (rf_raddr1 != 5'd0) && (rf_raddr1 == fw_dest_ex_stage2) && fw_write_ex_stage2,
    src1_used[ID] && (rf_raddr1 != 5'd0) && (rf_raddr1 == fw_dest_ex_stage1) && fw_write_ex_stage1,
    src1_used[ID] && (rf_raddr1 != 5'd0) && (rf_raddr1 == fw_dest_ex_stage0) && fw_write_ex_stage0
};

assign fw2_ex_hit = {
    src2_used[ID] && (rf_raddr2 != 5'd0) && (rf_raddr2 == fw_dest_ex_stage2) && fw_write_ex_stage2,
    src2_used[ID] && (rf_raddr2 != 5'd0) && (rf_raddr2 == fw_dest_ex_stage1) && fw_write_ex_stage1,
    src2_used[ID] && (rf_raddr2 != 5'd0) && (rf_raddr2 == fw_dest_ex_stage0) && fw_write_ex_stage0
};

// several stages can hit at once, so resolve the youngest hit into a one-hot
// select first; the 32-bit value is then a flat masked-OR instead of a chain.
assign fw1_ex_sel = fw1_ex_hit[0] ? 3'b001 : fw1_ex_hit[1] ? 3'b010 : fw1_ex_hit[2] ? 3'b100 : 3'b000;
assign fw2_ex_sel = fw2_ex_hit[0] ? 3'b001 : fw2_ex_hit[1] ? 3'b010 : fw2_ex_hit[2] ? 3'b100 : 3'b000;

assign fw1_hit[EX]   = | fw1_ex_hit;
assign fw2_hit[EX]   = | fw2_ex_hit;
assign fw1_valid[EX] = (fw1_ex_sel[0] & fw_valid_ex_stage0) | (fw1_ex_sel[1] & fw_valid_ex_stage1) | (fw1_ex_sel[2] & fw_valid_ex_stage2);
assign fw2_valid[EX] = (fw2_ex_sel[0] & fw_valid_ex_stage0) | (fw2_ex_sel[1] & fw_valid_ex_stage1) | (fw2_ex_sel[2] & fw_valid_ex_stage2);
assign fw1_ex_data  = ({32{fw1_ex_sel[1]}} & fw_data_ex_stage1) | ({32{fw1_ex_sel[2]}} & fw_data_ex_stage2);   // stage 0 never supplies
assign fw2_ex_data  = ({32{fw2_ex_sel[1]}} & fw_data_ex_stage1) | ({32{fw2_ex_sel[2]}} & fw_data_ex_stage2);   // stage 0 never supplies

// MEM part
assign fw1_hit[MEM]   = valid[MEM] && gr_we[MEM] && src1_used[ID] && (rf_raddr1 != 5'd0) && (rf_raddr1 == dest[MEM]);
assign fw2_hit[MEM]   = valid[MEM] && gr_we[MEM] && src2_used[ID] && (rf_raddr2 != 5'd0) && (rf_raddr2 == dest[MEM]);
assign fw1_valid[MEM] = fw1_hit[MEM] && ~ mem_en[MEM];
assign fw2_valid[MEM] = fw2_hit[MEM] && ~ mem_en[MEM];

// a tap that hits but is not valid yet makes ID wait
assign fw1_pending  = fw1_hit & ~ fw1_valid;
assign fw2_pending  = fw2_hit & ~ fw2_valid;

assign fw1_mem_sel = ~ fw1_valid[EX] &   fw1_valid[MEM];
assign fw1_rf_sel  = ~ fw1_valid[EX] & ~ fw1_valid[MEM];
assign fw2_mem_sel = ~ fw2_valid[EX] &   fw2_valid[MEM];
assign fw2_rf_sel  = ~ fw2_valid[EX] & ~ fw2_valid[MEM];

assign fw1_data = ({32{fw1_valid[EX]}} & fw1_ex_data) |
                  ({32{fw1_mem_sel}} & result[MEM]) |
                  ({32{fw1_rf_sel}}  & rf_rdata1);

assign fw2_data = ({32{fw2_valid[EX]}} & fw2_ex_data) |
                  ({32{fw2_mem_sel}} & result[MEM]) |
                  ({32{fw2_rf_sel}}  & rf_rdata2);

// ID must wait when any tap of either source is still pending
assign src_wait = | (fw1_pending | fw2_pending);

assign stall[ID] = src_wait && valid[ID];

// stall for IF & MEM
assign stall[IF]  = ~ IF_rsp_fire;  // should not include valid[IF], for we need to manage the issue that we send request and were flushed then
assign stall[MEM] = valid[MEM] && mem_en[MEM] && (mem_we[MEM] ? ~MEM_req_fire : ~MEM_rsp_fire);

assign inst_sram_en    = IF_req_valid;
assign inst_sram_we    = 4'b0;
assign inst_sram_addr  = pc[IF];
assign inst_sram_wdata = 32'b0;
assign inst_rd         = inst_sram_rdata;

always @(posedge clk) begin
    if (allow[ID]) begin
        if (IF_rsp_fire_wire)   inst[ID] <= inst_rd;
        else                    inst[ID] <= inst[IF];
    end
end

always @(posedge clk) begin
    if (IF_rsp_fire) inst[IF] <= inst_rd;
end

assign op_31_26  = inst[ID][31:26];
assign op_25_22  = inst[ID][25:22];
assign op_21_20  = inst[ID][21:20];
assign op_19_15  = inst[ID][19:15];

assign rd   = inst[ID][ 4: 0];
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

// exp10 arithmetic / logic instructions
assign inst_slti     = op_31_26_d[6'h00] & op_25_22_d[4'h8];
assign inst_sltui    = op_31_26_d[6'h00] & op_25_22_d[4'h9];
assign inst_andi     = op_31_26_d[6'h00] & op_25_22_d[4'hd];
assign inst_ori      = op_31_26_d[6'h00] & op_25_22_d[4'he];
assign inst_xori     = op_31_26_d[6'h00] & op_25_22_d[4'hf];
assign inst_sll_w    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0e];
assign inst_srl_w    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0f];
assign inst_sra_w    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h10];
assign inst_pcaddu12i= op_31_26_d[6'h07];

// exp10 mul / div instructions
assign inst_mul_w    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h18];
assign inst_mulh_w   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h19];
assign inst_mulh_wu  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h1a];
assign inst_div_w    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h00];
assign inst_mod_w    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h01];
assign inst_div_wu   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h02];
assign inst_mod_wu   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h03];

// exp11 compare branches and byte / halfword memory
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

// ID source usage includes branch operands and store data, not only ALU inputs
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
                  | inst_st_b | inst_st_h;
    src2_used[ID] = inst_add_w | inst_sub_w | inst_slt | inst_sltu
                  | inst_and | inst_or | inst_nor | inst_xor
                  | inst_st_w | inst_beq | inst_bne
                  | inst_sll_w | inst_srl_w | inst_sra_w
                  | inst_mul_w | inst_mulh_w | inst_mulh_wu
                  | inst_div_w | inst_mod_w | inst_div_wu | inst_mod_wu
                  | inst_blt | inst_bge | inst_bltu | inst_bgeu
                  | inst_st_b | inst_st_h;
end

always @(posedge clk) begin
    if (allow[MEM]) begin
        src1_used[MEM] <= src1_used[EX];
        src2_used[MEM] <= src2_used[EX];
    end
end

// ID generates the ALU operation; EX.v holds it while the pipeline is blocked
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

// ID generates the mul/div operation select; EX.v holds it
always @(*) begin
    md_op_id[MUL_W]   = inst_mul_w;
    md_op_id[MULH_W]  = inst_mulh_w;
    md_op_id[MULH_WU] = inst_mulh_wu;
    md_op_id[DIV_W]   = inst_div_w;
    md_op_id[MOD_W]   = inst_mod_w;
    md_op_id[DIV_WU]  = inst_div_wu;
    md_op_id[MOD_WU]  = inst_mod_wu;
end

// ID selects the operands; EX.v consumes its registered copies
always @(*) begin
    alu_src1_id = src1_is_pc  ? pc[ID] : fw1_data;
    alu_src2_id = src2_is_imm ? imm    : fw2_data;
end

// ID generates memory control and store data; EX.v / MEM carry them together
always @(*) begin
    res_from_mem[ID] = inst_ld_w | inst_ld_b | inst_ld_h | inst_ld_bu | inst_ld_hu;
    mem_we[ID]       = inst_st_w | inst_st_b | inst_st_h;
    mem_en[ID]       = mem_we[ID] | res_from_mem[ID];
    mem_wdata[ID]    = fw2_data;
    // access size: [0]=byte, [1]=halfword, [2]=load zero-extends (ld.bu / ld.hu)
    mem_size[ID][0]  = inst_ld_b | inst_ld_bu | inst_st_b;
    mem_size[ID][1]  = inst_ld_h | inst_ld_hu | inst_st_h;
    mem_size[ID][2]  = inst_ld_bu | inst_ld_hu;
end

always @(posedge clk) begin
    if (allow[MEM]) begin
        res_from_mem[MEM] <= res_from_mem[EX];
        mem_we[MEM]       <= mem_we[EX];
        mem_en[MEM]       <= mem_en[EX];
        mem_wdata[MEM]    <= mem_wdata[EX];
        mem_size[MEM]     <= mem_size[EX];
    end
end

// ID generates write-back control; EX.v / MEM / WB carry it with the result
always @(*) begin
    gr_we[ID] = ~inst_st_w & ~inst_beq & ~inst_bne & ~inst_b
              & ~inst_st_b & ~inst_st_h
              & ~inst_blt & ~inst_bge & ~inst_bltu & ~inst_bgeu;
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
assign need_si26  =  inst_b | inst_bl;
assign src2_is_4  =  inst_jirl | inst_bl;

assign imm = src2_is_4 ? 32'h4                      :
             need_si20 ? {i20[19:0], 12'b0}         :
             need_ui12 ? {20'b0, i12[11:0]}         :
/*need_ui5 || need_si12*/{{20{i12[11]}}, i12[11:0]} ;

assign br_offs = need_si26 ? {{ 4{i26[25]}}, i26[25:0], 2'b0} :
                             {{14{i16[15]}}, i16[15:0], 2'b0} ;

assign jirl_offs = {{14{i16[15]}}, i16[15:0], 2'b0};

assign src_reg_is_rd = inst_beq | inst_bne | inst_st_w | inst_st_b | inst_st_h
                     | inst_blt | inst_bge | inst_bltu | inst_bgeu;

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
assign rkd_value = fw2_data;

assign rj_eq_rd  = (rj_value == rkd_value);
assign rj_lt_rd  = ($signed(rj_value) < $signed(rkd_value));
assign rj_ltu_rd = (rj_value < rkd_value);

assign br_taken = (   inst_beq  &&  rj_eq_rd
                   || inst_bne  && !rj_eq_rd
                   || inst_blt  &&  rj_lt_rd
                   || inst_bge  && !rj_lt_rd
                   || inst_bltu &&  rj_ltu_rd
                   || inst_bgeu && !rj_ltu_rd
                   || inst_jirl
                   || inst_bl
                   || inst_b
                  ) && cur_valid[ID] && allow[ID];   // = receive[EX]: the ID branch really moves in

assign br_target = (inst_beq || inst_bne || inst_blt || inst_bge
                 || inst_bltu || inst_bgeu || inst_bl || inst_b) ? (pc[ID] + br_offs) :
                                                /*inst_jirl*/ (rj_value + jirl_offs);

EX u_ex(
    .clk            (clk              ),
    .reset          (reset            ),
    .allow_cur      (allow[EX]        ),
    .allow_next     (allow[MEM]       ),
  
    .valid_i        (next_valid[EX]   ),
    .pc_i           (pc[ID]           ),
    .src1_used_i    (src1_used[ID]    ),
    .src2_used_i    (src2_used[ID]    ),
    .alu_op_i       (alu_op_id        ),
    .alu_src1_i     (alu_src1_id      ),
    .alu_src2_i     (alu_src2_id      ),
    .md_op_i        (md_op_id         ),
    .res_from_mem_i (res_from_mem[ID] ),
    .mem_we_i       (mem_we[ID]       ),
    .mem_en_i       (mem_en[ID]       ),
    .mem_wdata_i    (mem_wdata[ID]    ),
    .mem_size_i     (mem_size[ID]     ),
    .gr_we_i        (gr_we[ID]        ),
    .dest_i         (dest[ID]         ),
 
    .pc_o           (pc_o             ),
    .src1_used_o    (src1_used_o      ),
    .src2_used_o    (src2_used_o      ),
    .res_from_mem_o (res_from_mem_o   ),
    .mem_we_o       (mem_we_o         ),
    .mem_en_o       (mem_en_o         ),
    .mem_wdata_o    (mem_wdata_o      ),
    .mem_size_o     (mem_size_o       ),
    .gr_we_o        (gr_we_o          ),
    .dest_o         (dest_o           ),
 
    .ex_result      (ex_result        ),
    .valid_o        (valid_o          ),
    .stall_o        (ex_stall         ),

    .fw_valid_ex_stage0     (fw_valid_ex_stage0 ),
    .fw_write_ex_stage0     (fw_write_ex_stage0 ),
    .fw_dest_ex_stage0      (fw_dest_ex_stage0  ),
    .fw_valid_ex_stage1     (fw_valid_ex_stage1 ),
    .fw_write_ex_stage1     (fw_write_ex_stage1 ),
    .fw_dest_ex_stage1      (fw_dest_ex_stage1  ),
    .fw_data_ex_stage1      (fw_data_ex_stage1  ),
    .fw_valid_ex_stage2     (fw_valid_ex_stage2 ),
    .fw_write_ex_stage2     (fw_write_ex_stage2 ),    
    .fw_dest_ex_stage2      (fw_dest_ex_stage2  ),
    .fw_data_ex_stage2      (fw_data_ex_stage2  )
    );

always @(posedge clk) begin
    if (allow[MEM]) result[MEM] <= ex_result;
    if (allow[WB])  result[WB]  <= res_from_mem[MEM] ? mem_result : result[MEM];
end

assign data_sram_en    = mem_en[MEM] && valid[MEM] && MEM_req_valid;
assign data_sram_addr  = result[MEM];

// byte / halfword / word store: broadcast the stored data to every byte lane
// and let the write-enable mask pick the addressed lane(s).
// mem_byte / mem_half / mem_word are exclusive, so every select below is a flat
// masked-OR instead of a priority chain.
wire mem_byte = mem_size[MEM][0];
wire mem_half = mem_size[MEM][1];
wire mem_word = ~ mem_byte & ~ mem_half;

wire [3:0] st_we_byte = 4'b0001 << result[MEM][1:0];
wire [3:0] st_we_half = result[MEM][1] ? 4'b1100 : 4'b0011;
wire [3:0] st_we_word = 4'b1111;

wire [3:0] st_we = ({4{mem_byte}} & st_we_byte) | ({4{mem_half}} & st_we_half) | ({4{mem_word}} & st_we_word);

assign data_sram_we    = {4{mem_we[MEM] && valid[MEM]}} & st_we;
assign data_sram_wdata = ({32{mem_byte}} & {4{mem_wdata[MEM][ 7: 0]}}) |
                         ({32{mem_half}} & {2{mem_wdata[MEM][15: 0]}}) |
                         ({32{mem_word}} &    mem_wdata[MEM]);

// byte / halfword / word load: take the addressed lane and extend it
wire [ 7:0] ld_byte = (result[MEM][1:0] == 2'd0) ? data_sram_rdata[ 7: 0] :
                      (result[MEM][1:0] == 2'd1) ? data_sram_rdata[15: 8] :
                      (result[MEM][1:0] == 2'd2) ? data_sram_rdata[23:16] :
                                                     data_sram_rdata[31:24];
wire [15:0] ld_half = result[MEM][1] ? data_sram_rdata[31:16] : data_sram_rdata[15:0];

wire [31:0] ld_byte_ext = mem_size[MEM][2] ? {24'b0, ld_byte} : {{24{ld_byte[7]}}, ld_byte};
wire [31:0] ld_half_ext = mem_size[MEM][2] ? {16'b0, ld_half} : {{16{ld_half[15]}}, ld_half};

assign mem_result = ({32{mem_byte}} & ld_byte_ext)  |
                    ({32{mem_half}} & ld_half_ext)  |
                    ({32{mem_word}} & data_sram_rdata);

// WB stage: write back to regfile
assign rf_we    = gr_we[WB] && valid[WB];
assign rf_waddr = dest[WB];
assign rf_wdata = result[WB];

assign debug_wb_pc       = pc[WB];
assign debug_wb_rf_we    = {4{rf_we}};  // only support word read
assign debug_wb_rf_wnum  = dest[WB];
assign debug_wb_rf_wdata = result[WB];

endmodule
