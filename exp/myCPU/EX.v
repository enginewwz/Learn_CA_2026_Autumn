module EX(
    input  wire        clk,
    input  wire        reset,
    input  wire        allow_cur,       // allow[EX]
    input  wire        allow_next,      // allow[MEM]
                                        
    // ID -> EX: combinational at ID
    input  wire [11:0] alu_op_i,
    input  wire [31:0] alu_src1_i,
    input  wire [31:0] alu_src2_i,
    input  wire [ 6:0] br_op_i,         // branch payload, registered with the ALU operands
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

    // EX -> MEM: combinational, the MEM stage registers them on allow_cur[MEM]
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

    // execute result, valid and handshake
    output wire        valid_o,
    output wire [31:0] ex_result,
    output wire        stall_o,

    // branch decision and target, decided in stage 0;
    output wire        br_taken_o,
    output wire [31:0] br_target_o,

    // EX -> ID forwarding.  EX resolves its own stages here; the top only sees,
    // per operand, whether the register the ID instruction reads is written by
    // something in EX (hit), whether that value is ready yet (valid) and the
    // value itself (a hit that is not valid yet makes ID wait for MEM).
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

// EX is three stages (0..2). The multiplier has two internal register stages, so
// with the stage-0 operand register its result is already combinational when the
// instruction is in stage 2, and MEM registers it itself (ex_result_mem)

// mul / div results are therefore NOT forwarded inside EX: their consumers wait
// until the value is in MEM.

// stage flags: exactly one of alu_pos / mul_pos / div_pos is set per stage, and
// flag <=> valid holds for every stage
reg  [2:0] valid;
reg  [2:0] alu_pos;
reg  [2:0] mul_pos;
reg  [2:0] div_pos;              // this instruction is a div/mod
wire [1:0] alu_no_mul_shortcut;  // if there is no mul in progress, alu should jump to stage[i]
wire [2:0] stall_pipeline /* verilator split_var */;

// ID -> EX payload, registered in stage 0
reg  [11:0] alu_op;
reg  [31:0] alu_src1;
reg  [31:0] alu_src2;
reg  [ 6:0] br_op;
reg  [31:0] br_base;
reg  [31:0] br_offs;

reg  [31:0] alu_result   [2:0];

// One-hot view of which EX stage drives the EX -> MEM interface this cycle.
// Normally that is stage 2; an ALU result that is already the oldest
// instruction in EX may leave from stage 1/0 instead (short-circuit).
wire [2:0] out_sel;

// the instruction's fields, one entry per stage it is still live in
reg  [ 2:0] src1_used         ;
reg  [ 2:0] src2_used         ;
reg  [31:0] pc           [2:0];
reg  [ 6:0] md_op        [2:0];
reg  [ 2:0] res_from_mem      ;
reg  [ 2:0] mem_we            ;
reg  [ 2:0] mem_en            ;
reg  [31:0] mem_wdata    [2:0];
reg  [ 2:0] mem_size     [2:0];   // one-hot: SIZE_BYTE / SIZE_HALF / SIZE_WORD
reg  [ 2:0] mem_zext          ;   // the load zero-extends (ld.bu / ld.hu)
reg  [ 2:0] gr_we             ;
reg  [ 4:0] dest         [2:0];

// unit results
wire [31:0] alu_result_wire;
wire [31:0] mul_result_wire;
wire [31:0] div_result;
wire        sel_alu2;
wire        sel_mul2;
wire        sel_div2;

// div handshake
wire        div_busy;
wire [31:0] div_quot, div_rem;
reg         div_started;         // stage-0 div has been started
wire        stall_div;           // stage-2 div that is still computing
wire        div_is_signed, div_is_mod;
wire        div_start, div_leave;

// branch unit (stage 0)
wire        br_taken;
wire [31:0] br_target;
reg         br_valid;

// EX -> ID forwarding, one group per stage (stage 0 is the youngest)
wire [2:0] will_write;
wire [2:1] can_supply;
wire [2:0] fw1_hit;
wire [2:0] fw2_hit;
wire [2:1] fw1_sel;
wire [2:1] fw2_sel;

// div is a blocking unit: it is started from EX stage 0 (the operands are
// registered there), the instruction walks to stage 2 and waits there until the
// unit goes idle, then its (registered) quotient/remainder go straight to MEM.
assign stall_div = div_pos[2] & div_busy;

assign stall_pipeline[2] = (valid[2] & ~ allow_next) | stall_div;
assign stall_pipeline[1] = (stall_pipeline[2] & valid[1]) | (div_pos[0] & ~ div_started & ~ div_start);
assign stall_pipeline[0] = stall_pipeline[1] & valid[0];
assign stall_o = stall_pipeline[0];

assign div_is_signed = md_op[0][DIV_W] | md_op[0][MOD_W];
assign div_is_mod    = md_op[2][MOD_W] | md_op[2][MOD_WU];
assign div_start     = div_pos[0] & ~ div_started & (~ div_pos[2] | div_leave) & ~ div_pos[1];
assign div_leave     = div_pos[2] & ~ div_busy & allow_next;

always @(posedge clk) begin
    if (allow_cur) begin
        alu_op           <= alu_op_i;
        alu_src1         <= alu_src1_i;
        alu_src2         <= alu_src2_i;
        br_op            <= br_op_i;
        br_base          <= br_base_i;
        br_offs          <= br_offs_i;
    end
end

always @(*) begin
    alu_result [0] = alu_result_wire;
end

always @(posedge clk) begin
    if (! stall_pipeline[1]) alu_result [1] <= alu_result [0];
    if (! stall_pipeline[2]) alu_result [2] <= alu_result [1];
end

always @(posedge clk) begin
    if (reset) begin
        valid           <= 3'b0;
    end
    else begin
        if (allow_cur)                valid [0] <= valid_i;
        else if (! stall_pipeline[1]) valid [0] <= 1'b0;

        // a short-circuited stage hands its instruction to MEM, not to stage+1
        if (! stall_pipeline[1]) valid [1] <= valid [0] & ~ alu_no_mul_shortcut[0];
        if (! stall_pipeline[2]) valid [2] <= valid [1] & ~ alu_no_mul_shortcut[1];
    end
end

always @(posedge clk) begin
    if (reset) begin
        alu_pos <= 3'b0;
        mul_pos <= 3'b0;
        div_pos <= 3'b0;
    end
    else begin
        // exactly one of the three is set for a real instruction, and they shift
        // with the same ~shortcut terms as valid[], so flag <=> valid always holds
        if (allow_cur) begin
            alu_pos [0] <= (~|     md_op_i) & valid_i;
            mul_pos [0] <= (| md_op_i[2:0]) & valid_i;
            div_pos [0] <= (| md_op_i[6:3]) & valid_i;
        end
        else if (! stall_pipeline[1]) begin
            alu_pos [0] <= 1'b0;
            mul_pos [0] <= 1'b0;
            div_pos [0] <= 1'b0;
        end
        if (! stall_pipeline[1]) begin
            alu_pos [1] <= alu_pos [0] & ~ alu_no_mul_shortcut[0];
            mul_pos [1] <= mul_pos [0] & ~ alu_no_mul_shortcut[0];
            div_pos [1] <= div_pos [0] & ~ alu_no_mul_shortcut[0];
        end
        if (! stall_pipeline[2]) begin
            alu_pos [2] <= alu_pos [1] & ~ alu_no_mul_shortcut[1];
            mul_pos [2] <= mul_pos [1] & ~ alu_no_mul_shortcut[1];
            div_pos [2] <= div_pos [1] & ~ alu_no_mul_shortcut[1];
        end
    end
    
end

always @(posedge clk) begin
    if (allow_cur) begin
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
    end


    if (! stall_pipeline[1]) begin
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
    end

    if (! stall_pipeline[2]) begin
        pc           [2] <= pc           [1];
        src1_used    [2] <= src1_used    [1];
        src2_used    [2] <= src2_used    [1];
        md_op        [2] <= md_op        [1];
        res_from_mem [2] <= res_from_mem [1];
        mem_we       [2] <= mem_we       [1];
        mem_en       [2] <= mem_en       [1];
        mem_wdata    [2] <= mem_wdata    [1];
        mem_size     [2] <= mem_size     [1];
        mem_zext     [2] <= mem_zext     [1];
        gr_we        [2] <= gr_we        [1];
        dest         [2] <= dest         [1];
    end
end

assign alu_no_mul_shortcut[1] = (~ mul_pos[2])   & (~ div_pos[2])   & alu_pos[1] & (~|alu_pos[  2]) & allow_next;
assign alu_no_mul_shortcut[0] = (~|mul_pos[2:1]) & (~|div_pos[2:1]) & alu_pos[0] & (~|alu_pos[2:1]) & allow_next;

// Stage 2 normally drives the outputs; when it is empty an ALU result that is
// already the oldest instruction short-circuits straight out of its stage.
// (valid[2] keeps priority so an older valid instruction is never overtaken.)
// A div waiting in stage 2 is still there, so it must not look valid to MEM.
assign out_sel[2]   = valid[2] & ~ stall_div;
assign out_sel[1:0] = alu_no_mul_shortcut;

// out_sel is one-hot, so the outputs are a plain wired-OR of masked stages
// (no implied priority chain: the terms are mutually exclusive by construction)
assign valid_o        = | out_sel;
assign pc_o           = ({32{out_sel[0]}} & pc        [0]) | ({32{out_sel[1]}} & pc        [1]) | ({32{out_sel[2]}} & pc        [2]);
assign src1_used_o    = (out_sel[0] & src1_used       [0]) | (out_sel[1] & src1_used       [1]) | (out_sel[2] & src1_used       [2]);
assign src2_used_o    = (out_sel[0] & src2_used       [0]) | (out_sel[1] & src2_used       [1]) | (out_sel[2] & src2_used       [2]);
assign res_from_mem_o = (out_sel[0] & res_from_mem    [0]) | (out_sel[1] & res_from_mem    [1]) | (out_sel[2] & res_from_mem    [2]);
assign mem_we_o       = (out_sel[0] & mem_we          [0]) | (out_sel[1] & mem_we          [1]) | (out_sel[2] & mem_we          [2]);
assign mem_en_o       = (out_sel[0] & mem_en          [0]) | (out_sel[1] & mem_en          [1]) | (out_sel[2] & mem_en          [2]);
assign mem_wdata_o    = ({32{out_sel[0]}} & mem_wdata [0]) | ({32{out_sel[1]}} & mem_wdata [1]) | ({32{out_sel[2]}} & mem_wdata [2]);
assign mem_size_o     = ({ 3{out_sel[0]}} & mem_size  [0]) | ({ 3{out_sel[1]}} & mem_size  [1]) | ({ 3{out_sel[2]}} & mem_size  [2]);
assign mem_zext_o     = (out_sel[0] & mem_zext        [0]) | (out_sel[1] & mem_zext        [1]) | (out_sel[2] & mem_zext        [2]);
assign gr_we_o        = (out_sel[0] & gr_we           [0]) | (out_sel[1] & gr_we           [1]) | (out_sel[2] & gr_we           [2]);
assign dest_o         = ({ 5{out_sel[0]}} & dest      [0]) | ({ 5{out_sel[1]}} & dest      [1]) | ({ 5{out_sel[2]}} & dest      [2]);

// stage 2: ALU (registered), mul (combinational CPA on top of its stage-1
// registers) or div (registered quotient/remainder, only once ~busy).
// Each option is qualified by out_sel[2], which makes all five terms exclusive.
assign div_result = ({32{div_is_mod}} & div_rem) | ({32{~ div_is_mod}} & div_quot);
assign sel_alu2   = out_sel[2] & alu_pos[2];
assign sel_mul2   = out_sel[2] & mul_pos[2];
assign sel_div2   = out_sel[2] & div_pos[2];

assign ex_result      = ({32{out_sel[0]}} & alu_result[0])   |
                        ({32{out_sel[1]}} & alu_result[1])   |
                        ({32{sel_alu2}}   & alu_result[2])   |
                        ({32{sel_mul2}}   & mul_result_wire) |
                        ({32{sel_div2}}   & div_result);


always @(posedge clk) begin
    if (reset)                    br_valid <= 1'b0;
    else if (allow_cur)           br_valid <= | br_op_i;   // a branch entered stage 0
    else if (! stall_pipeline[1]) br_valid <= 1'b0;        // it left without firing
    else if (br_taken)            br_valid <= 1'b0;        // held, but already fired
end

br u_br(
    .src1   (alu_src1  ),
    .src2   (alu_src2  ),
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

// EX -> ID forwarding.  The three stages are matched here, so the top does not
// have to know them:
//   hit   : some stage of EX writes the register the ID instruction reads, so
//           ID must not read the register file
//   valid : the youngest such stage can supply its value right now (ALU only)
//   data  : that value
assign will_write = valid & gr_we;

assign can_supply = gr_we[2:1] & ~ mem_en[2:1] & alu_pos[2:1];

assign fw1_hit = {
    src1_used_i && (fw_raddr1_i != 5'd0) && (fw_raddr1_i == dest[2]) && will_write[2],
    src1_used_i && (fw_raddr1_i != 5'd0) && (fw_raddr1_i == dest[1]) && will_write[1],
    src1_used_i && (fw_raddr1_i != 5'd0) && (fw_raddr1_i == dest[0]) && will_write[0]
};

assign fw2_hit = {
    src2_used_i && (fw_raddr2_i != 5'd0) && (fw_raddr2_i == dest[2]) && will_write[2],
    src2_used_i && (fw_raddr2_i != 5'd0) && (fw_raddr2_i == dest[1]) && will_write[1],
    src2_used_i && (fw_raddr2_i != 5'd0) && (fw_raddr2_i == dest[0]) && will_write[0]
};

// the youngest hit wins; stage 0 never supplies (its ALU result is not
// registered yet), so a stage-0 hit selects nothing and ID waits for MEM
assign fw1_sel = fw1_hit[0] ? 2'b00 : fw1_hit[1] ? 2'b01 : fw1_hit[2] ? 2'b10 : 2'b00;
assign fw2_sel = fw2_hit[0] ? 2'b00 : fw2_hit[1] ? 2'b01 : fw2_hit[2] ? 2'b10 : 2'b00;

assign fw1_hit_o   = | fw1_hit;
assign fw2_hit_o   = | fw2_hit;
assign fw1_valid_o = (fw1_sel[1] & can_supply[1]) | (fw1_sel[2] & can_supply[2]);
assign fw2_valid_o = (fw2_sel[1] & can_supply[1]) | (fw2_sel[2] & can_supply[2]);

assign fw1_data_o  = ({32{fw1_sel[1]}} & alu_result[1]) | ({32{fw1_sel[2]}} & alu_result[2]);
assign fw2_data_o  = ({32{fw2_sel[1]}} & alu_result[1]) | ({32{fw2_sel[2]}} & alu_result[2]);

alu u_alu(
    .alu_op     (alu_op         ),
    .alu_src1   (alu_src1       ),
    .alu_src2   (alu_src2       ),
    .alu_result (alu_result_wire)
);

// mul: 2 internal stages (EX stages 1 and 2); its combinational CPA output is
// already valid at EX stage 2 and goes straight to MEM
mul u_mul(
    .clk    (clk                ),
    .reset  (reset              ),
    .stall  (stall_pipeline[2:1]),
    .src1   (alu_src1           ),
    .src2   (alu_src2           ),
    .md_op  (md_op[0]           ),
    .result (mul_result_wire    )
);

// div: blocking, fed with the stage-0 operands, result read out at stage 2.
// done is left open on purpose: it is a pulse, ~busy is what stage 2 waits for.
div u_div(
    .clk       (clk         ),
    .reset     (reset       ),
    .start     (div_start   ),
    .is_signed (div_is_signed),
    .src1      (alu_src1    ),
    .src2      (alu_src2    ),
    .busy      (div_busy    ),
    .done      (            ),
    .quot      (div_quot    ),
    .rem       (div_rem     )
);

// one div in flight at a time: a following div waits in stage 0 until this one
// has carried its result out of EX
always @(posedge clk) begin
    if (reset)                                  div_started <= 1'b0;
    else if (div_start & stall_pipeline[1])     div_started <= 1'b1;
    else if (~ stall_pipeline[1])               div_started <= 1'b0;
end

endmodule
