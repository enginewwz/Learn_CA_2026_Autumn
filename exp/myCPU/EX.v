module EX(
    input  wire        clk,
    input  wire        reset,
    input  wire        flush_i,         // global flush
    input  wire        allow_next,      // allow[MEM]
    
    // ID -> EX: combinational at ID
    input  wire [11:0] alu_op_i,
    input  wire [31:0] src1_i,
    input  wire [31:0] src2_i,
    input  wire [ 6:0] br_op_i,         // branch payload
    input  wire [31:0] br_base_i,
    input  wire [31:0] br_offs_i,

    // should be stored through out the stages
    input  wire        valid_i,
    input  wire        src1_used_i,
    input  wire        src2_used_i,
    input  wire [31:0] pc_i,
    input  wire [ 6:0] md_op_i,    
    input  wire        res_from_mem_i,
    input  wire        mem_we_i,
    input  wire        mem_en_i,
    input  wire [31:0] mem_wdata_i,
    input  wire [ 2:0] mem_size_i,
    input  wire        mem_zext_i,
    input  wire        gr_we_i,
    input  wire [ 4:0] dest_i,

    // CSR / exception payload
    input  wire [13:0] csr_num_i,
    input  wire [ 8:0] csr_ctl_i,
    input  wire [12:0] excp_cause_i,
    input  wire [31:0] csr_wdata_i,
    input  wire [31:0] csr_mask_i,
    input  wire [ 2:0] tlb_op_i,
    input  wire [ 4:0] invtlb_op_i,

    // EX -> MEM: combinational, the MEM stage registers them on allow[MEM]
    output wire [31:0] pc_o,
    output wire        src1_used_o,
    output wire        src2_used_o,
    output wire        res_from_mem_o,
    output wire        mem_we_o,
    output wire        mem_en_o,
    output wire [31:0] mem_wdata_o,
    output wire [ 2:0] mem_size_o,
    output wire        mem_zext_o,
    output wire        gr_we_o,
    output wire [ 4:0] dest_o,

    // payload, aligned with the stage that drives EX -> MEM
    output wire [13:0] csr_num_o,
    output wire [ 8:0] csr_ctl_o,
    output wire [12:0] excp_cause_o,
    output wire [31:0] csr_wdata_o,
    output wire [31:0] csr_mask_o,
    output wire [ 2:0] tlb_op_o,
    output wire [ 4:0] invtlb_op_o,

    // execute result, valid and handshake
    output wire        valid_o,
    output wire [31:0] ex_result,
    output wire        stall_o,

    // branch decision and target, decided in stage 0;
    output wire        br_taken_o,
    output wire [31:0] br_target_o,

    // EX -> ID forwarding
    input  wire [ 4:0] fw_raddr1_i,   // the source registers the ID instruction reads
    input  wire [ 4:0] fw_raddr2_i,
    output wire        fw1_hit_o,     // ID src1 matches a register that EX will write
    output wire        fw1_valid_o,   // ... and the value is here already
    output wire [31:0] fw1_data_o,
    output wire        fw2_hit_o,
    output wire        fw2_valid_o,
    output wire [31:0] fw2_data_o
);

// md_op bit positions: one-hot mul/div operation select
localparam MUL_W    = 0;
localparam MULH_W   = 1;
localparam MULH_WU  = 2;
localparam DIV_W    = 3;
localparam MOD_W    = 4;
localparam DIV_WU   = 5;
localparam MOD_WU   = 6;

// csr_ctl[CSR_WB] means "the write-back value comes from the CSR file at WB"
localparam CSR_WB = 7;

// EX is two stages (0..1).  The multiplier folds its whole Wallace tree into the
// stage-0 cycle and registers only (sum, carry); the 64-bit CPA after that is
// combinational, so the product is ready while the instruction sits in stage 1,
// the last stage before MEM registers the result.  div is blocking and hands its
// result over at stage 1 as well.  mul / div are therefore NOT in the EX -> ID
// forwarding path -- their consumers wait for the MEM tap.
//
// Stage flags: exactly one of alu_pos / mul_pos / div_pos is set per stage, and
// flag <=> valid holds for every stage.
reg  [1:0] valid;
reg  [1:0] alu_pos;
reg  [1:0] mul_pos;
reg  [1:0] div_pos;
wire       alu_no_mul_div_shortcut;  // stage 0 may leave at once when nothing above it needs the bus
wire [1:0] cur_valid;
wire [1:0] next_valid;
wire [1:0] cur_alu_pos;
wire [1:0] cur_mul_pos;
wire [1:0] cur_div_pos;
wire [1:0] next_alu_pos;
wire [1:0] next_mul_pos;
wire [1:0] next_div_pos;
wire [1:0] stall;
wire [1:0] allow /* verilator split_var */;
wire       div_hold;        // the stage-0 div cannot start yet, so it must stay

// ID -> EX payload, registered in stage 0
reg  [11:0] alu_op;
reg  [31:0] src1;
reg  [31:0] src2;
reg  [ 6:0] br_op;
reg  [31:0] br_base;
reg  [31:0] br_offs;

reg  [31:0] alu_result [1:0];

wire [1:0] out_sel;

// stage fields: [0] is the ID/EX boundary, [1] drives EX -> MEM combinationally
reg         src1_used    [1:0];
reg         src2_used    [1:0];
reg  [31:0] pc           [1:0];
reg  [ 6:0] md_op        [1:0];
reg         res_from_mem [1:0];
reg         mem_we       [1:0];
reg         mem_en       [1:0];
reg  [31:0] mem_wdata    [1:0];
reg  [ 2:0] mem_size     [1:0];
reg         mem_zext     [1:0];
reg         gr_we        [1:0];
reg  [ 4:0] dest         [1:0];

reg  [13:0] csr_num      [1:0];
reg  [ 8:0] csr_ctl      [1:0];
reg  [12:0] excp_cause   [1:0];
reg  [31:0] csr_wdata    [1:0];
reg  [31:0] csr_mask     [1:0];
reg  [ 2:0] tlb_op       [1:0];
reg  [ 4:0] invtlb_op    [1:0];

// unit results
wire [31:0] alu_result_wire;
wire [31:0] mul_result_wire;
wire [31:0] div_result;
wire        sel_alu2, sel_mul2, sel_div2;
wire [31:0] result2;
wire        alu_lt, alu_ltu;

// div handshake
wire        div_busy;
wire [31:0] div_quot, div_rem;
wire        div_is_signed, div_is_mod;
wire        flush_reset;
reg         div_started;
wire        div_start, div_leave, stall_div;

// branch unit
wire        br_taken;
wire [31:0] br_target;
reg         br_valid;

// EX -> ID forwarding
wire [1:0] will_write;
wire [1:0] can_supply;
wire [1:0] fw1_hit, fw2_hit;
wire [1:0] fw1_sel, fw2_sel;

//--------------------------------------------------------------- allow / valid
// stage 1 leaves through allow_next; stage 0 waits for stage 1 and for a div
// that has not been able to start yet
assign stall[1] = stall_div;
assign stall[0] = div_hold;

assign allow[1] = (allow_next | ~ valid[1]) & ~ stall[1];
assign allow[0] = (allow[1]   | ~ valid[0]) & ~ stall[0];

assign stall_o = ~ allow[0];

// flush empties EX without touching the payload registers
assign cur_valid   = valid   & {2{~ flush_i}};
assign cur_alu_pos = alu_pos & {2{~ flush_i}};
assign cur_mul_pos = mul_pos & {2{~ flush_i}};
assign cur_div_pos = div_pos & {2{~ flush_i}};

assign next_valid = {cur_valid[0] & ~ stall[0] & ~ alu_no_mul_div_shortcut, valid_i}
                    & allow | (~ allow & cur_valid);

assign next_alu_pos = ({cur_alu_pos[0] & ~ stall[0] & ~ alu_no_mul_div_shortcut,
                        (~| md_op_i) & valid_i} & allow) | (~ allow & cur_alu_pos);
assign next_mul_pos = ({cur_mul_pos[0] & ~ stall[0],
                        (| md_op_i[2:0]) & valid_i} & allow) | (~ allow & cur_mul_pos);
assign next_div_pos = ({cur_div_pos[0] & ~ stall[0],
                        (| md_op_i[6:3]) & valid_i} & allow) | (~ allow & cur_div_pos);

always @(posedge clk) begin
    if (reset) begin
        valid   <= 2'b0;
        alu_pos <= 2'b0;
        mul_pos <= 2'b0;
        div_pos <= 2'b0;
    end
    else begin
        valid   <= next_valid;
        alu_pos <= next_alu_pos;
        mul_pos <= next_mul_pos;
        div_pos <= next_div_pos;
    end
end

// stage 0 can carry its ALU result straight out when nothing above it needs the
// bus.  "nothing above" keeps ALU instructions in order.
assign alu_no_mul_div_shortcut = (~ mul_pos[1]) & (~ div_pos[1]) & alu_pos[0] & (~| alu_pos[1]) & allow_next;

// stage 1 drives the output unless the div is still running
assign out_sel[1] = valid[1] & ~ stall_div;
assign out_sel[0] = alu_no_mul_div_shortcut;

assign valid_o = | out_sel;

//--------------------------------------------------------------- payload pipes
always @(posedge clk) begin
    if (allow[0]) begin
        alu_op           <= alu_op_i;
        src1         <= src1_i;
        src2         <= src2_i;
        br_op            <= br_op_i;
        br_base          <= br_base_i;
        br_offs          <= br_offs_i;
    end
end

always @(*) begin
    alu_result [0] = alu_result_wire;
end

always @(posedge clk) begin
    if (allow[1]) alu_result [1] <= alu_result [0];
end

always @(posedge clk) begin
    if (allow[0]) begin
        pc           [0] <= pc_i;
        src1_used    [0] <= src1_used_i;
        src2_used    [0] <= src2_used_i; 
        md_op        [0] <= md_op_i;
        res_from_mem [0] <= res_from_mem_i;
        mem_we       [0] <= mem_we_i;
        mem_en       [0] <= mem_en_i;
        mem_wdata    [0] <= mem_wdata_i;
        mem_size     [0] <= mem_size_i;
        mem_zext     [0] <= mem_zext_i;
        gr_we        [0] <= gr_we_i;
        dest         [0] <= dest_i;
        csr_num      [0] <= csr_num_i;
        csr_ctl      [0] <= csr_ctl_i;
        excp_cause   [0] <= excp_cause_i;
        csr_wdata    [0] <= csr_wdata_i;
        csr_mask     [0] <= csr_mask_i;
        tlb_op       [0] <= tlb_op_i;
        invtlb_op    [0] <= invtlb_op_i;
    end

    if (allow[1]) begin
        pc           [1] <= pc           [0];
        src1_used    [1] <= src1_used    [0];
        src2_used    [1] <= src2_used    [0];
        md_op        [1] <= md_op        [0];
        res_from_mem [1] <= res_from_mem [0];
        mem_we       [1] <= mem_we       [0];
        mem_en       [1] <= mem_en       [0];
        mem_wdata    [1] <= mem_wdata    [0];
        mem_size     [1] <= mem_size     [0];
        mem_zext     [1] <= mem_zext     [0];
        gr_we        [1] <= gr_we        [0];
        dest         [1] <= dest         [0];
        csr_num      [1] <= csr_num      [0];
        csr_ctl      [1] <= csr_ctl      [0];
        excp_cause   [1] <= excp_cause   [0];
        csr_wdata    [1] <= csr_wdata    [0];
        csr_mask     [1] <= csr_mask     [0];
        tlb_op       [1] <= tlb_op       [0];
        invtlb_op    [1] <= invtlb_op    [0];
    end

end

//--------------------------------------------------------------- output muxes
assign div_result = ({32{div_is_mod}} & div_rem) | ({32{~ div_is_mod}} & div_quot);
assign sel_alu2   = out_sel[1] & alu_pos[1];
assign sel_mul2   = out_sel[1] & mul_pos[1];
assign sel_div2   = out_sel[1] & div_pos[1];

assign result2 = ({32{sel_alu2}} & alu_result[1]) |
                 ({32{sel_mul2}} & mul_result_wire) |
                 ({32{sel_div2}} & div_result);

assign ex_result      = ({32{out_sel[0]}} & alu_result[0])   |
                        result2;

assign pc_o           = ({32{out_sel[0]}} & pc        [0]) | ({32{out_sel[1]}} & pc        [1]);
assign src1_used_o    = (out_sel[0] & src1_used       [0]) | (out_sel[1] & src1_used       [1]);
assign src2_used_o    = (out_sel[0] & src2_used       [0]) | (out_sel[1] & src2_used       [1]);
assign res_from_mem_o = (out_sel[0] & res_from_mem    [0]) | (out_sel[1] & res_from_mem    [1]);
assign mem_we_o       = (out_sel[0] & mem_we          [0]) | (out_sel[1] & mem_we          [1]);
assign mem_en_o       = (out_sel[0] & mem_en          [0]) | (out_sel[1] & mem_en          [1]);
assign mem_wdata_o    = ({32{out_sel[0]}} & mem_wdata [0]) | ({32{out_sel[1]}} & mem_wdata [1]);
assign mem_size_o     = ({ 3{out_sel[0]}} & mem_size  [0]) | ({ 3{out_sel[1]}} & mem_size  [1]);
assign mem_zext_o     = (out_sel[0] & mem_zext        [0]) | (out_sel[1] & mem_zext        [1]);
assign gr_we_o        = (out_sel[0] & gr_we           [0]) | (out_sel[1] & gr_we           [1]);
assign dest_o         = ({ 5{out_sel[0]}} & dest      [0]) | ({ 5{out_sel[1]}} & dest      [1]);

// payload rides along with whichever stage drives the output
assign csr_num_o      = ({14{out_sel[0]}} & csr_num  [0]) | ({14{out_sel[1]}} & csr_num  [1]);
assign csr_ctl_o      = ({ 9{out_sel[0]}} & csr_ctl  [0]) | ({ 9{out_sel[1]}} & csr_ctl  [1]);
assign tlb_op_o       = ({ 3{out_sel[0]}} & tlb_op   [0]) | ({ 3{out_sel[1]}} & tlb_op   [1]);
assign invtlb_op_o    = ({ 5{out_sel[0]}} & invtlb_op[0]) | ({ 5{out_sel[1]}} & invtlb_op[1]);
assign excp_cause_o   = ({13{out_sel[0]}} & excp_cause[0]) | ({13{out_sel[1]}} & excp_cause[1]);
assign csr_wdata_o    = ({32{out_sel[0]}} & csr_wdata[0]) | ({32{out_sel[1]}} & csr_wdata[1]);
assign csr_mask_o     = ({32{out_sel[0]}} & csr_mask [0]) | ({32{out_sel[1]}} & csr_mask [1]);

//--------------------------------------------------------------- branch (stage 0)
always @(posedge clk) begin
    if (reset)              br_valid <= 1'b0;
    else if (allow[0])      br_valid <= | br_op_i;   // a branch entered stage 0
    else if (allow[1])      br_valid <= 1'b0;        // it left without firing
    else if (br_taken)      br_valid <= 1'b0;        // held, but already fired
end

br u_br(
    .src1   (src1  ),
    .src2   (src2  ),
    .lt_i   (alu_lt    ),
    .ltu_i  (alu_ltu   ),
    .base   (br_base   ),
    .offs   (br_offs   ),
    .br_op  (br_op     ),
    .valid  (br_valid  ),
    .taken  (br_taken  ),
    .target (br_target )
);

// a flushed instruction still latches its payload, so valid[0] decides whether
// the branch in stage 0 is a real one
assign br_taken_o  = br_taken & valid[0];
assign br_target_o = br_target;

//--------------------------------------------------------------- EX -> ID fwd
// The three stages are matched here, so the top does not have to know them:
//   hit   : some stage of EX writes the register the ID instruction reads
//   valid : the youngest such stage can supply its value right now
//   data  : that value
assign will_write = {valid[1] & gr_we[1], valid[0] & gr_we[0]};

// Only ALU results are supplied inside EX.  mul / div / load consumers wait for
// the MEM tap, which also keeps result2 (the mul/div mux) out of this path.
// csr insts are seen as alu insts here, thus should not forward.
assign can_supply[1] = ~ mem_en[1] & alu_pos[1] & ~ csr_ctl[1][CSR_WB];
assign can_supply[0] = ~ mem_en[0] & alu_pos[0] & ~ csr_ctl[0][CSR_WB];

assign fw1_hit = {
    src1_used_i && (fw_raddr1_i != 5'd0) && (fw_raddr1_i == dest[1]) && will_write[1],
    src1_used_i && (fw_raddr1_i != 5'd0) && (fw_raddr1_i == dest[0]) && will_write[0]
};

assign fw2_hit = {
    src2_used_i && (fw_raddr2_i != 5'd0) && (fw_raddr2_i == dest[1]) && will_write[1],
    src2_used_i && (fw_raddr2_i != 5'd0) && (fw_raddr2_i == dest[0]) && will_write[0]
};

// the youngest hit wins; stage 0 supplies off the combinational ALU output
assign fw1_sel[0] = fw1_hit[0];
assign fw1_sel[1] = fw1_hit[1] & ~ fw1_hit[0];
assign fw2_sel[0] = fw2_hit[0];
assign fw2_sel[1] = fw2_hit[1] & ~ fw2_hit[0];

assign fw1_hit_o   = | fw1_hit;
assign fw2_hit_o   = | fw2_hit;

assign fw1_valid_o = | (fw1_sel & can_supply);
assign fw2_valid_o = | (fw2_sel & can_supply);

assign fw1_data_o  = ({32{fw1_sel[0]}} & alu_result[0]) |
                     ({32{fw1_sel[1]}} & alu_result[1]);
assign fw2_data_o  = ({32{fw2_sel[0]}} & alu_result[0]) |
                     ({32{fw2_sel[1]}} & alu_result[1]);

//--------------------------------------------------------------- units
alu u_alu(
    .alu_op     (alu_op         ),
    .src1   (src1       ),
    .src2   (src2       ),
    .alu_result (alu_result_wire),
    .lt_o       (alu_lt         ),
    .ltu_o      (alu_ltu        )
);

// mul: its whole Wallace tree is folded into the stage-0 cycle and registered as
// (sum, carry); the 64-bit CPA after it is combinational, so the product is ready
// while the instruction is in stage 1 and MEM registers it -- which is why mul is
// not in the EX -> ID forwarding mux.
mul u_mul(
    .clk    (clk                ),
    .reset  (reset              ),
    .allow  (allow[0]           ),
    .src1   (src1           ),
    .src2   (src2           ),
    .md_op  (md_op[0]           ),
    .result (mul_result_wire    )
);

// div: blocking, fed with the stage-0 operands, result read out at stage 1.
// done is left open on purpose: it is a pulse, ~busy is what stage 2 waits for.
div u_div(
    .clk       (clk         ),
    .reset     (flush_reset ),
    .start     (div_start   ),
    .is_signed (div_is_signed),
    .src1      (src1    ),
    .src2      (src2    ),
    .busy      (div_busy    ),
    .done      (            ),
    .quot      (div_quot    ),
    .rem       (div_rem     )
);

// div handshake: a fresh div starts as soon as stage 0 holds one and nothing
// older is still using the unit; while it cannot start, stage 0 must hold
assign flush_reset   = reset | flush_i;
assign div_is_signed = md_op[0][DIV_W] | md_op[0][MOD_W];
assign div_is_mod    = md_op[1][MOD_W] | md_op[1][MOD_WU];

assign stall_div = div_pos[1] & div_busy;
assign div_leave = div_pos[1] & ~ div_busy & allow_next;
assign div_start = div_pos[0] & ~ div_started & (~ div_pos[1] | div_leave);
assign div_hold  = div_pos[0] & ~ div_started & ~ div_start;

// one div in flight at a time: a following div waits in stage 0 until this one
// has carried its result out of EX
always @(posedge clk) begin
    if (reset)                              div_started <= 1'b0;
    else if (div_start & ~ allow[1])        div_started <= 1'b1;
    else if (allow[1])                      div_started <= 1'b0;
end

endmodule
