module EX(
    input  wire        clk,
    input  wire        reset,
    input  wire        receive,         // receive_new_inst[EX]: ID -> EX transfer
    input  wire        allow_next,      // allow[MEM]: the next stage can accept
                                        // (using receive[MEM] here would close a
                                        //  loop through stall_o -> allow[EX])

    // ID -> EX: combinational at ID
    input  wire [11:0] alu_op_i,
    input  wire [31:0] alu_src1_i,
    input  wire [31:0] alu_src2_i,

    // should be stored through out the stages
    input  wire        src1_used_i,
    input  wire        src2_used_i,
    input  wire [31:0] pc_i,
    input  wire [ 6:0] md_op_i,    
    input  wire        res_from_mem_i,
    input  wire        mem_we_i,
    input  wire        mem_en_i,
    input  wire [31:0] mem_wdata_i,
    input  wire        gr_we_i,
    input  wire [ 4:0] dest_i,

    // EX -> MEM: registered at EX
    output wire [31:0] pc_o,
    output wire        src1_used_o,
    output wire        src2_used_o,
    output wire        res_from_mem_o,
    output wire        mem_we_o,
    output wire        mem_en_o,
    output wire [31:0] mem_wdata_o,
    output wire        gr_we_o,
    output wire [ 4:0] dest_o,

    // execute result, valid and handshake
    output wire        valid_o,
    output wire [31:0] ex_result,
    output wire        stall_o
);

// md_op bit positions: one-hot mul/div operation select
localparam MUL_W    = 0;
localparam MULH_W   = 1;
localparam MULH_WU  = 2;
localparam DIV_W    = 3;
localparam MOD_W    = 4;
localparam DIV_WU   = 5;
localparam MOD_WU   = 6;

// mul goes with four stages; thus we use 0-3(so that no need to define in localparam) to mark the four stages

reg  [3:0] valid;
reg  [3:0] alu_pos;
reg  [3:0] mul_pos;
reg  [3:0] div_pos;              // this instruction is a div/mod
reg  [3:0] div_ready;            // its div result is already latched in final_result
wire [2:0] alu_no_mul_shortcut;  // if there is no mul in progress, alu should jump to stage[i]
wire [3:0] stall_pipeline /* verilator split_var */;

reg  [11:0] alu_op;
reg  [31:0] alu_src1;
reg  [31:0] alu_src2;

reg  [31:0] alu_result  [2:0];
reg  [31:0] final_result;

// One-hot view of which EX stage drives the EX -> MEM interface this cycle.
// Normally that is stage 3; an ALU result that is already the oldest
// instruction in EX may leave from stage 2/1/0 instead (short-circuit).
wire [3:0] out_sel;

reg         src1_used   [3:0];
reg         src2_used   [3:0];
reg  [31:0] pc          [3:0];
reg  [ 6:0] md_op       [3:0];
reg         res_from_mem[3:0];
reg         mem_we      [3:0];
reg         mem_en      [3:0];
reg  [31:0] mem_wdata   [3:0];
reg         gr_we       [3:0];
reg  [ 4:0] dest        [3:0];

wire [31:0] alu_result_wire;
wire [31:0] mul_result_wire;

// div is a blocking unit: it is started from EX stage 0 (the operands are
// registered there), the instruction walks to stage 3 and waits there until the
// unit goes idle, then the result is latched into final_result (the stage-3
// register) and handed to MEM. A following div cannot start until this one has
// left EX, so the unit's result can never be overwritten while it is consumed.
wire        div_busy;
wire [31:0] div_quot, div_rem;
reg         div_inflight;        // a div has been started and not yet left EX
wire        div_wait3;           // stage-3 div whose result is not latched yet
wire        div_latch;           // ...and the unit has its result ready

// While a div waits in stage 3 it must freeze everything behind it, and it must
// not look valid to MEM yet: receive_new_inst[MEM] only checks ~stall[EX], and
// with bubbles behind the div that would be 0, so out_sel[3] is gated too.
assign div_wait3 = valid[3] & div_pos[3] & ~ div_ready[3];

assign stall_pipeline[3] = valid[3] & (~ allow_next | div_wait3);
assign stall_pipeline[2] = stall_pipeline[3] & valid[2];
assign stall_pipeline[1] = (stall_pipeline[2] & valid[1]) | (valid[0] & div_pos[0] & div_inflight);
assign stall_pipeline[0] = stall_pipeline[1] & valid[0];
assign stall_o = stall_pipeline[0];

assign div_is_signed = md_op[0][DIV_W] | md_op[0][MOD_W];
assign div_is_mod    = md_op[3][MOD_W] | md_op[3][MOD_WU];
assign div_start     = valid[0] & div_pos[0] & ~ stall_pipeline[1];
assign div_leave     = valid[3] & div_pos[3] & div_ready[3] & allow_next;
// ~busy instead of done: done is a one-cycle pulse and would be missed if the
// div instruction were stalled between stage 0 and stage 3
assign div_latch     = div_wait3 & ~ div_busy;



always @(posedge clk) begin
    if (receive) begin
        alu_op           <= alu_op_i;
        alu_src1         <= alu_src1_i;
        alu_src2         <= alu_src2_i;
    end
end

always @(*) begin
    alu_result [0] = alu_result_wire;
end

always @(posedge clk) begin
    if (! stall_pipeline[1]) alu_result [1] <= alu_result [0];
    if (! stall_pipeline[2]) alu_result [2] <= alu_result [1];

    // div_latch implies div_wait3, which freezes the stage-2 -> stage-3
    // transfer, so the two sources of final_result never overlap
    if (div_latch) begin
        final_result <= div_is_mod ? div_rem : div_quot;
    end
    else if (! stall_pipeline[3]) begin
        final_result <= ({32{alu_pos[2]}} & alu_result[2]) | ({32{mul_pos[2]}} & mul_result_wire);
    end
end

always @(posedge clk) begin
    if (reset) begin
        valid           <= 4'b0;
    end
    else begin
        if (receive)         valid [0] <= 1'b1;
        else if (! stall_pipeline[1]) valid [0] <= 1'b0;  // stage 0 moves on when stage 1 takes it

        // a short-circuited stage hands its instruction to MEM, not to stage+1
        if (! stall_pipeline[1]) valid [1] <= valid [0] & ~ alu_no_mul_shortcut[0];
        if (! stall_pipeline[2]) valid [2] <= valid [1] & ~ alu_no_mul_shortcut[1];
        if (! stall_pipeline[3]) valid [3] <= valid [2] & ~ alu_no_mul_shortcut[2];
    end
end

always @(posedge clk) begin
    if (reset) begin
        alu_pos <= 4'b0;
        mul_pos <= 4'b0;
        div_pos <= 4'b0;
        div_ready <= 4'b0;
    end
    else begin
        if (receive) begin
            if (~| md_op_i) begin
                alu_pos [0] <= 1'b1;
            end
            else begin
                alu_pos [0] <= 1'b0;
            end

            if (| md_op_i[2:0]) begin
                mul_pos [0] <= 1'b1;
            end
            else begin
                mul_pos [0] <= 1'b0;
            end

            div_pos  [0] <= | md_op_i[6:3];
            div_ready[0] <= 1'b0;
        end
        if (! stall_pipeline[1]) begin
            alu_pos [1] <= alu_pos [0];
            mul_pos [1] <= mul_pos [0];
            div_pos [1] <= div_pos [0];
            div_ready[1] <= div_ready[0];
        end
        if (! stall_pipeline[2]) begin
            alu_pos [2] <= alu_pos [1];
            mul_pos [2] <= mul_pos [1];
            div_pos [2] <= div_pos [1];
            div_ready[2] <= div_ready[1];
        end
        if (! stall_pipeline[3]) begin
            alu_pos [3] <= alu_pos [2];
            mul_pos [3] <= mul_pos [2];
            div_pos [3] <= div_pos [2];
        end
        // the unit's result is stable here (a new div can only start after this
        // one leaves), so latch it as soon as it is idle
        if (div_latch)                div_ready[3] <= 1'b1;
        else if (! stall_pipeline[3]) div_ready[3] <= div_ready[2];
    end
    
end

always @(posedge clk) begin
    if (receive) begin
        pc           [0] <= pc_i;
        src1_used    [0] <= src1_used_i;
        src2_used    [0] <= src2_used_i; 
        md_op        [0] <= md_op_i;
        res_from_mem [0] <= res_from_mem_i;
        mem_we       [0] <= mem_we_i;
        mem_en       [0] <= mem_en_i;
        mem_wdata    [0] <= mem_wdata_i;
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
        gr_we        [2] <= gr_we        [1];
        dest         [2] <= dest         [1];
    end

    if (! stall_pipeline[3]) begin
        pc           [3] <= pc           [2];
        src1_used    [3] <= src1_used    [2];
        src2_used    [3] <= src2_used    [2];
        md_op        [3] <= md_op        [2];
        res_from_mem [3] <= res_from_mem [2];
        mem_we       [3] <= mem_we       [2];
        mem_en       [3] <= mem_en       [2];
        mem_wdata    [3] <= mem_wdata    [2];
        gr_we        [3] <= gr_we        [2];
        dest         [3] <= dest         [2];
    end
end

assign alu_no_mul_shortcut[2] = (~ mul_pos[3])   & (~ div_pos[3])   & alu_pos[2] & valid[2] & (~|(alu_pos[  3] & valid[  3])) & allow_next;
assign alu_no_mul_shortcut[1] = (~|mul_pos[3:2]) & (~|div_pos[3:2]) & alu_pos[1] & valid[1] & (~|(alu_pos[3:2] & valid[3:2])) & allow_next;
assign alu_no_mul_shortcut[0] = (~|mul_pos[3:1]) & (~|div_pos[3:1]) & alu_pos[0] & valid[0] & (~|(alu_pos[3:1] & valid[3:1])) & allow_next;

// Stage 3 normally drives the outputs; when it is empty an ALU result that is
// already the oldest instruction short-circuits straight out of its stage.
// (valid[3] keeps priority so an older valid instruction is never overtaken.)
// A div waiting in stage 3 is still there, so it must not look valid to MEM.
assign out_sel[3]   = valid [3] & ~ div_wait3;
assign out_sel[2:0] = alu_no_mul_shortcut & {3{~ valid[3]}};

assign valid_o        = | out_sel;
assign pc_o           = out_sel[0] ? pc          [0] : out_sel[1] ? pc          [1] : out_sel[2] ? pc          [2] : pc          [3];
assign src1_used_o    = out_sel[0] ? src1_used   [0] : out_sel[1] ? src1_used   [1] : out_sel[2] ? src1_used   [2] : src1_used   [3];
assign src2_used_o    = out_sel[0] ? src2_used   [0] : out_sel[1] ? src2_used   [1] : out_sel[2] ? src2_used   [2] : src2_used   [3];
assign res_from_mem_o = out_sel[0] ? res_from_mem[0] : out_sel[1] ? res_from_mem[1] : out_sel[2] ? res_from_mem[2] : res_from_mem[3];
assign mem_we_o       = out_sel[0] ? mem_we      [0] : out_sel[1] ? mem_we      [1] : out_sel[2] ? mem_we      [2] : mem_we      [3];
assign mem_en_o       = out_sel[0] ? mem_en      [0] : out_sel[1] ? mem_en      [1] : out_sel[2] ? mem_en      [2] : mem_en      [3];
assign mem_wdata_o    = out_sel[0] ? mem_wdata   [0] : out_sel[1] ? mem_wdata   [1] : out_sel[2] ? mem_wdata   [2] : mem_wdata   [3];
assign gr_we_o        = out_sel[0] ? gr_we       [0] : out_sel[1] ? gr_we       [1] : out_sel[2] ? gr_we       [2] : gr_we       [3];
assign dest_o         = out_sel[0] ? dest        [0] : out_sel[1] ? dest        [1] : out_sel[2] ? dest        [2] : dest        [3];

// stage 3 already holds its value in final_result; a short-circuit takes the
// ALU result on the spot (mul never short-circuits: it needs stage 3 to
// register its combinational CPA output)
assign ex_result      = out_sel[0] ? alu_result[0] : out_sel[1] ? alu_result[1] : out_sel[2] ? alu_result[2] : final_result;

alu u_alu(
    .alu_op     (alu_op         ),
    .alu_src1   (alu_src1       ),
    .alu_src2   (alu_src2       ),
    .alu_result (alu_result_wire)
);

// mul: 2 internal stages (EX stages 1 and 2); sel is combinational and comes
// out at EX stage 2, register it into stage 3 outside this module
mul u_mul(
    .clk    (clk                ),
    .reset  (reset              ),
    .stall  (stall_pipeline[2:1]),
    .src1   (alu_src1           ),
    .src2   (alu_src2           ),
    .md_op  (md_op[0]           ),
    .result (mul_result_wire    )
);

// div: blocking, fed with the stage-0 operands, result collected in stage 3.
// done is left open on purpose: it is a pulse and may happen before the
// instruction reaches stage 3, where ~busy is used instead.
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
    if (reset)               div_inflight <= 1'b0;
    else if (div_start)      div_inflight <= 1'b1;
    else if (div_leave)      div_inflight <= 1'b0;
end

endmodule
