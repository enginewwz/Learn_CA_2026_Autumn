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

reg  [TOTAL_STAGE_WIDTH - 1: 0] valid;
// when is valid used in leaf control signals? write back to rf / write mem / change pc
wire [TOTAL_STAGE_WIDTH - 1: 0] stall; // stall itself; the last stage (WB) would never stall it self; reset as 0
wire [TOTAL_STAGE_WIDTH - 1: 0] allow; // allow new inst; means that cur and later stage is ready for new inst; the last stage (WB) would always allow; reset as 1
wire [TOTAL_STAGE_WIDTH - 1: 0] flush;
wire [TOTAL_STAGE_WIDTH - 1: 0] cur_valid;
wire [TOTAL_STAGE_WIDTH - 1: 0] receive_new_inst;
wire [TOTAL_STAGE_WIDTH - 1: 0] next_valid;

// only process in one stage
wire [31:0] nextpc;     // generate at IF stage
wire [31:0] seq_pc;     // generate at IF stage
wire        br_taken;   // generate at ID stage
wire [31:0] br_target;  // generate at ID stage
wire [31:0] inst_rd;    // receive in ID stage

// should be stored between stages
reg  [31:0] pc [WB:IF];

// IF -> ID
reg  [31:0] inst [ID:IF];

// control signals for EX stage
reg  [11:0] alu_op [EX:ID];

reg  [31:0] alu_src1    [EX:ID]; // operands latched at ID, used at EX
reg  [31:0] alu_src2    [EX:ID];

// control signals for MEM stage
reg         res_from_mem [MEM:ID];
reg         mem_we [MEM:ID];
reg         mem_en [MEM:ID];
reg  [31:0] mem_wdata [MEM:ID]; // store data: latch at ID, write RAM at MEM

wire        load_op;    // not used

// control signals for WB stage
reg         gr_we [WB:ID];
reg  [4: 0] dest [WB:ID];

// EX -> MEM
wire [31:0] alu_result;      // ALU combinational output (EX stage)
reg  [31:0] alu_result_mem;

// MEM -> WB
wire [31:0] final_result;
reg  [31:0] final_result_wb;

// mid signals for ID stage
wire        src_reg_is_rd;
wire [31:0] rj_value;
wire [31:0] rkd_value;
wire        rj_eq_rd;
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

wire        need_ui5;
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
wire        load_use_hit;

wire [MEM:EX] forwarding_hit1;  // stage that hit forwarding addr
wire [31:0] forwarding_data1;

wire [MEM:EX] forwarding_hit2;  // stage that hit forwarding addr
wire [31:0] forwarding_data2;


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
    else if (receive_new_inst[IF])      IF_req_valid <= 1'b1;
    else if (fake_IF_req_ready)         IF_req_valid <= 1'b0;
    else                                IF_req_valid <= IF_req_valid; 
end

assign IF_req_fire = IF_req_valid && fake_IF_req_ready;

assign fake_IF_req_ready = IF_req_valid;

always @(posedge clk) begin
    if (reset)                          IF_rsp_fire_reg <= 1'b0;
    else if (receive_new_inst[IF])      IF_rsp_fire_reg <= 1'b0;
    else if (IF_rsp_fire_wire)          IF_rsp_fire_reg <= 1'b1;
    else                                IF_rsp_fire_reg <= IF_rsp_fire_reg;
end

always @(posedge clk) begin
    if (reset)                          IF_rsp_ready    <= 1'b0;
    else if (receive_new_inst[IF])      IF_rsp_ready    <= 1'b0;
    else if (IF_req_fire)               IF_rsp_ready    <= 1'b1;
    else                                IF_rsp_ready    <= IF_rsp_ready;
end

assign IF_rsp_fire_wire = IF_rsp_ready && fake_IF_rsp_valid;

assign IF_rsp_fire = IF_rsp_fire_reg | IF_rsp_fire_wire;

assign fake_IF_rsp_valid = IF_rsp_ready;

reg  MEM_req_valid;
wire fake_MEM_req_ready;
wire MEM_req_fire;

wire fake_MEM_rsp_valid;
reg  MEM_rsp_ready;
wire MEM_rsp_fire_wire;
reg  MEM_rsp_fire_reg;
wire MEM_rsp_fire;

always @(posedge clk) begin
    if (reset)                          MEM_req_valid <= 1'b0;
    else if (receive_new_inst[MEM])     MEM_req_valid <= valid[EX];
    else if (fake_MEM_req_ready)        MEM_req_valid <= 1'b0;
    else                                MEM_req_valid <= MEM_req_valid; 
end

assign MEM_req_fire = MEM_req_valid && fake_MEM_req_ready;

assign fake_IF_req_ready = IF_req_valid;

always @(posedge clk) begin
    if (reset)                          MEM_rsp_fire_reg <= 1'b0;
    else if (receive_new_inst[MEM])     MEM_rsp_fire_reg <= 1'b0;
    else if (MEM_req_fire)              MEM_rsp_fire_reg <= 1'b1;
    else                                MEM_rsp_fire_reg <= MEM_rsp_fire_reg;
end

always @(posedge clk) begin
    if (reset)                          MEM_rsp_ready    <= 1'b0;
    else if (receive_new_inst[MEM])     MEM_rsp_ready    <= 1'b0;
    else if (MEM_req_fire)              MEM_rsp_ready    <= 1'b1;
    else                                MEM_rsp_ready    <= MEM_rsp_ready;
end

assign MEM_rsp_fire_wire = MEM_rsp_ready && fake_MEM_rsp_valid;

assign MEM_rsp_fire = MEM_rsp_fire_reg | MEM_rsp_fire_wire;

assign fake_MEM_rsp_valid = MEM_rsp_ready;


assign seq_pc       = pc[IF] + 32'h4;
assign nextpc       = br_taken ? br_target : seq_pc;

always @(posedge clk) begin
    if (reset) begin
        pc[IF] <= 32'h1bfffffc;     // trick: to make nextpc be 0x1c000000 during reset
    end
    else begin
        if (receive_new_inst[IF] ) pc[IF]  <= nextpc;
        if (receive_new_inst[ID] ) pc[ID]  <= pc[IF];
        if (receive_new_inst[EX] ) pc[EX]  <= pc[ID];
        if (receive_new_inst[MEM]) pc[MEM] <= pc[EX];
        if (receive_new_inst[WB] ) pc[WB]  <= pc[MEM];
    end
end

// first cycle for new stage
assign cur_valid = valid & ~ flush;
assign receive_new_inst = {cur_valid[TOTAL_STAGE_WIDTH -2: 0], 1'b1} & allow;
assign next_valid = receive_new_inst | (~ allow & cur_valid);

always @(posedge clk) begin
    if (reset) begin
        valid <= 5'b0;
    end
    else begin
        valid <= next_valid;
    end
end

// flush logic --> only flushes at stage ID
assign flush = br_taken ? 5'b00011 : 5'b00000;

// allow logic --> generate with stall logic
assign allow[WB]  = 1'b1;
assign allow[MEM] = ~ stall[MEM];
assign allow[EX]  = ~ stall[EX] && allow[MEM];
assign allow[ID]  = ~ stall[ID] && allow[EX];
assign allow[IF]  = ~ stall[IF] && allow[ID];

// not used stall bits

assign stall[EX] = 1'b0;
assign stall[WB] = 1'b0;

// stall for load use hazard
assign load_use_hit =   (valid[EX]  && mem_en[EX]  && ~ mem_we[EX]  && ((rf_raddr1 == dest[EX])  || (rf_raddr2 == dest[EX])))  |
                        (valid[MEM] && mem_en[MEM] && ~ mem_we[MEM] && ((rf_raddr1 == dest[MEM]) || (rf_raddr2 == dest[MEM]))) ;

assign stall[ID] = load_use_hit;

// stall for IF & MEM
assign stall[IF]  = ~ IF_rsp_fire;
assign stall[MEM] = ~ MEM_rsp_fire || ~ valid[MEM];

// generate forwarding bits
assign forwarding_hit1[EX] = valid[EX] && ~ mem_en[EX] && gr_we[EX] && (rf_raddr1 == dest[EX]);
assign forwarding_hit2[EX] = valid[EX] && ~ mem_en[EX] && gr_we[EX] && (rf_raddr2 == dest[EX]);
assign forwarding_hit1[MEM] = valid[MEM] && ~ mem_en[MEM] && gr_we[MEM] && (rf_raddr1 == dest[MEM]);
assign forwarding_hit2[MEM] = valid[MEM] && ~ mem_en[MEM] && gr_we[MEM] && (rf_raddr2 == dest[MEM]);

assign forwarding_data1 = forwarding_hit1[MEM] ? alu_result_mem :
                          forwarding_hit1[EX]  ? alu_result     :
                          src1_is_pc ? pc[IF] : rj_value;

assign forwarding_data2 = forwarding_hit2[MEM] ? alu_result_mem :
                          forwarding_hit2[EX]  ? alu_result     :
                          src2_is_imm ? imm : rkd_value;

assign inst_sram_en    = IF_req_valid;
assign inst_sram_we    = 4'b0;
assign inst_sram_addr  = pc[IF];
assign inst_sram_wdata = 32'b0;
assign inst_rd         = inst_sram_rdata;

always @(posedge clk) begin
    if (receive_new_inst[ID] && IF_rsp_fire_wire) inst[ID] <= inst_rd;
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

always @(posedge clk) begin
    if (receive_new_inst[ID]) begin
        alu_op[ID][ 0] <= inst_add_w | inst_addi_w | inst_ld_w | inst_st_w
                    | inst_jirl | inst_bl;
        alu_op[ID][ 1] <= inst_sub_w;
        alu_op[ID][ 2] <= inst_slt;
        alu_op[ID][ 3] <= inst_sltu;
        alu_op[ID][ 4] <= inst_and;
        alu_op[ID][ 5] <= inst_nor;
        alu_op[ID][ 6] <= inst_or;
        alu_op[ID][ 7] <= inst_xor;
        alu_op[ID][ 8] <= inst_slli_w;
        alu_op[ID][ 9] <= inst_srli_w;
        alu_op[ID][10] <= inst_srai_w;
        alu_op[ID][11] <= inst_lu12i_w;
    end
    if (receive_new_inst[EX]) alu_op[EX] <= alu_op[ID];
end

// EX stage: operands selected at ID, latched to EX
always @(posedge clk) begin
    if (receive_new_inst[ID]) begin
        alu_src1[ID] <= forwarding_data1;
        alu_src2[ID] <= forwarding_data2;
    end
    if (receive_new_inst[EX]) begin
        alu_src1[EX] <= alu_src1[ID];
        alu_src2[EX] <= alu_src2[ID];
    end
end

// MEM stage: mem control and store data latched at ID, shifted to MEM
always @(posedge clk) begin
    if (receive_new_inst[ID]) begin
        res_from_mem[ID] <= inst_ld_w;
        mem_we[ID]       <= inst_st_w;
        mem_en[ID]       <= inst_st_w | inst_ld_w;
        mem_wdata[ID]    <= rkd_value;
    end
    if (receive_new_inst[EX]) begin
        res_from_mem[EX] <= res_from_mem[ID];
        mem_we[EX]       <= mem_we[ID];
        mem_en[EX]       <= mem_en[ID];
        mem_wdata[EX]    <= mem_wdata[ID];
    end
    if (receive_new_inst[MEM]) begin
        res_from_mem[MEM] <= res_from_mem[EX];
        mem_we[MEM]       <= mem_we[EX];
        mem_en[MEM]       <= mem_en[EX];
        mem_wdata[MEM]    <= mem_wdata[EX];
    end
end

// WB stage: write-back control latched at ID, shifted to WB
always @(posedge clk) begin
    if (receive_new_inst[ID]) begin
        gr_we[ID] <= ~inst_st_w & ~inst_beq & ~inst_bne & ~inst_b;
        dest[ID]  <= inst_bl ? 5'd1 : rd;
    end
    if (receive_new_inst[EX]) begin
        gr_we[EX] <= gr_we[ID];
        dest[EX]  <= dest[ID];
    end
    if (receive_new_inst[MEM]) begin
        gr_we[MEM] <= gr_we[EX];
        dest[MEM]  <= dest[EX];
    end
    if (receive_new_inst[WB]) begin
        gr_we[WB] <= gr_we[MEM];
        dest[WB]  <= dest[MEM];
    end
end

assign need_ui5   =  inst_slli_w | inst_srli_w | inst_srai_w;
assign need_si12  =  inst_addi_w | inst_ld_w | inst_st_w;
assign need_si16  =  inst_jirl | inst_beq | inst_bne;
assign need_si20  =  inst_lu12i_w;
assign need_si26  =  inst_b | inst_bl;
assign src2_is_4  =  inst_jirl | inst_bl;

assign imm = src2_is_4 ? 32'h4                      :
             need_si20 ? {i20[19:0], 12'b0}         :
/*need_ui5 || need_si12*/{{20{i12[11]}}, i12[11:0]} ;

assign br_offs = need_si26 ? {{ 4{i26[25]}}, i26[25:0], 2'b0} :
                             {{14{i16[15]}}, i16[15:0], 2'b0} ;

assign jirl_offs = {{14{i16[15]}}, i16[15:0], 2'b0};

assign src_reg_is_rd = inst_beq | inst_bne | inst_st_w;

assign src1_is_pc    = inst_jirl | inst_bl;

assign src2_is_imm   = inst_slli_w |
                       inst_srli_w |
                       inst_srai_w |
                       inst_addi_w |
                       inst_ld_w   |
                       inst_st_w   |
                       inst_lu12i_w|
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

assign rj_value  = rf_rdata1;
assign rkd_value = rf_rdata2;

assign rj_eq_rd = (rj_value == rkd_value);
assign br_taken = (   inst_beq  &&  rj_eq_rd
                   || inst_bne  && !rj_eq_rd
                   || inst_jirl
                   || inst_bl
                   || inst_b
                  ) && valid[ID];

assign br_target = (inst_beq || inst_bne || inst_bl || inst_b) ? (pc[ID] + br_offs) :
                                                /*inst_jirl*/ (rj_value + jirl_offs);

alu u_alu(
    .alu_op     (alu_op    [EX]),
    .alu_src1   (alu_src1  [EX]),
    .alu_src2   (alu_src2  [EX]),
    .alu_result (alu_result)
    );

always @(posedge clk) begin
    if (receive_new_inst[MEM]) alu_result_mem <= alu_result;
end

assign final_result = res_from_mem[MEM] ? mem_result : alu_result_mem;

always @(posedge clk) begin
    if (receive_new_inst[WB]) final_result_wb <= final_result;
end

assign data_sram_en    = mem_en[MEM] && valid[MEM] && MEM_req_valid;
assign data_sram_we    = {4{mem_we[MEM] && valid[MEM]}}; // only support word write
assign data_sram_addr  = alu_result_mem;
assign data_sram_wdata = mem_wdata[MEM];

assign mem_result = data_sram_rdata;

// WB stage: write back to regfile
assign rf_we    = gr_we[WB] && valid[WB];
assign rf_waddr = dest[WB];
assign rf_wdata = final_result;

assign debug_wb_pc       = pc[WB];
assign debug_wb_rf_we    = {4{rf_we}};  // only support word read
assign debug_wb_rf_wnum  = dest[WB];
assign debug_wb_rf_wdata = final_result;

endmodule
