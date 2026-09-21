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
    input  wire [ 2:0] mem_size_i,
    input  wire        gr_we_i,
    input  wire [ 4:0] dest_i,

    // EX -> MEM: combinational, the MEM stage registers them on receive[MEM]
    output wire [31:0] pc_o,
    output wire        src1_used_o,
    output wire        src2_used_o,
    output wire        res_from_mem_o,
    output wire        mem_we_o,
    output wire        mem_en_o,
    output wire [31:0] mem_wdata_o,
    output wire [ 2:0] mem_size_o,
    output wire        gr_we_o,
    output wire [ 4:0] dest_o,

    // execute result, valid and handshake
    output wire        valid_o,
    output wire [31:0] ex_result,
    output wire        stall_o,

    // forwarding, one group per EX stage (stage 0 is the youngest):
    //   valid   : this stage can supply the operand right now (ALU only)
    //   pending : this stage writes that register but its value is not here
    //             yet (mul / div / load), so ID has to wait for MEM
    //   dest    : destination register of the instruction in that stage
    //   data    : its result (only meaningful together with valid)
    output wire        fwd0_valid,
    output wire        fwd0_pending,
    output wire [ 4:0] fwd0_dest,
    output wire [31:0] fwd0_data,
    output wire        fwd1_valid,
    output wire        fwd1_pending,
    output wire [ 4:0] fwd1_dest,
    output wire [31:0] fwd1_data,
    output wire        fwd2_valid,
    output wire        fwd2_pending,
    output wire [ 4:0] fwd2_dest,
    output wire [31:0] fwd2_data
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
// instruction is in stage 2, and MEM registers it itself (ex_result_mem) - one
// more EX register would only add latency and area, not throughput.
// mul / div results are therefore NOT forwarded inside EX: their consumers wait
// until the value is in MEM.

reg  [2:0] valid;
reg  [2:0] alu_pos;
reg  [2:0] mul_pos;
reg  [2:0] div_pos;              // this instruction is a div/mod
wire [1:0] alu_no_mul_shortcut;  // if there is no mul in progress, alu should jump to stage[i]
wire [2:0] stall_pipeline /* verilator split_var */;

reg  [11:0] alu_op;
reg  [31:0] alu_src1;
reg  [31:0] alu_src2;

reg  [31:0] alu_result  [2:0];

// One-hot view of which EX stage drives the EX -> MEM interface this cycle.
// Normally that is stage 2; an ALU result that is already the oldest
// instruction in EX may leave from stage 1/0 instead (short-circuit).
wire [2:0] out_sel;

reg         src1_used   [2:0];
reg         src2_used   [2:0];
reg  [31:0] pc          [2:0];
reg  [ 6:0] md_op       [2:0];
reg         res_from_mem[2:0];
reg         mem_we      [2:0];
reg         mem_en      [2:0];
reg  [31:0] mem_wdata   [2:0];
reg  [ 2:0] mem_size    [2:0];
reg         gr_we       [2:0];
reg  [ 4:0] dest        [2:0];

wire [31:0] alu_result_wire;
wire [31:0] mul_result_wire;

// div is a blocking unit: it is started from EX stage 0 (the operands are
// registered there), the instruction walks to stage 2 and waits there until the
// unit goes idle, then its (registered) quotient/remainder go straight to MEM.
// A following div cannot start until this one has left EX, so the unit's result
// can never be overwritten while it is consumed.
wire        div_busy;
wire [31:0] div_quot, div_rem;
reg         div_inflight;        // a div has been started and not yet left EX
wire        div_wait2;           // stage-2 div that is still computing
wire        div_is_signed, div_is_mod;
wire        div_start, div_leave;

// While a div waits in stage 2 it must freeze everything behind it, and it must
// not look valid to MEM yet: receive_new_inst[MEM] only checks ~stall[EX], and
// with bubbles behind the div that would be 0, so out_sel[2] is gated too.
assign div_wait2 = valid[2] & div_pos[2] & div_busy;

assign stall_pipeline[2] = valid[2] & (~ allow_next | div_wait2);
assign stall_pipeline[1] = (stall_pipeline[2] & valid[1]) | (valid[0] & div_pos[0] & div_inflight);
assign stall_pipeline[0] = stall_pipeline[1] & valid[0];
assign stall_o = stall_pipeline[0];

assign div_is_signed = md_op[0][DIV_W] | md_op[0][MOD_W];
assign div_is_mod    = md_op[2][MOD_W] | md_op[2][MOD_WU];
assign div_start     = valid[0] & div_pos[0] & ~ stall_pipeline[1];
assign div_leave     = valid[2] & div_pos[2] & ~ div_busy & allow_next;

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
end

always @(posedge clk) begin
    if (reset) begin
        valid           <= 3'b0;
    end
    else begin
        if (receive)         valid [0] <= 1'b1;
        else if (! stall_pipeline[1]) valid [0] <= 1'b0;  // stage 0 moves on when stage 1 takes it

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
        end
        if (! stall_pipeline[1]) begin
            alu_pos [1] <= alu_pos [0];
            mul_pos [1] <= mul_pos [0];
            div_pos [1] <= div_pos [0];
        end
        if (! stall_pipeline[2]) begin
            alu_pos [2] <= alu_pos [1];
            mul_pos [2] <= mul_pos [1];
            div_pos [2] <= div_pos [1];
        end
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
        mem_size     [0] <= mem_size_i;
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
        gr_we        [2] <= gr_we        [1];
        dest         [2] <= dest         [1];
    end
end

assign alu_no_mul_shortcut[1] = (~ mul_pos[2])   & (~ div_pos[2])   & alu_pos[1] & valid[1] & (~|(alu_pos[  2] & valid[  2])) & allow_next;
assign alu_no_mul_shortcut[0] = (~|mul_pos[2:1]) & (~|div_pos[2:1]) & alu_pos[0] & valid[0] & (~|(alu_pos[2:1] & valid[2:1])) & allow_next;

// Stage 2 normally drives the outputs; when it is empty an ALU result that is
// already the oldest instruction short-circuits straight out of its stage.
// (valid[2] keeps priority so an older valid instruction is never overtaken.)
// A div waiting in stage 2 is still there, so it must not look valid to MEM.
assign out_sel[2]   = valid [2] & ~ div_wait2;
assign out_sel[1:0] = alu_no_mul_shortcut & {2{~ valid[2]}};

// out_sel is one-hot, so the outputs are a plain wired-OR of masked stages
// (no implied priority chain: the terms are mutually exclusive by construction)
assign valid_o        = | out_sel;
assign pc_o           = ({32{out_sel[0]}} & pc          [0]) | ({32{out_sel[1]}} & pc          [1]) | ({32{out_sel[2]}} & pc          [2]);
assign src1_used_o    = (out_sel[0] & src1_used   [0]) | (out_sel[1] & src1_used   [1]) | (out_sel[2] & src1_used   [2]);
assign src2_used_o    = (out_sel[0] & src2_used   [0]) | (out_sel[1] & src2_used   [1]) | (out_sel[2] & src2_used   [2]);
assign res_from_mem_o = (out_sel[0] & res_from_mem[0]) | (out_sel[1] & res_from_mem[1]) | (out_sel[2] & res_from_mem[2]);
assign mem_we_o       = (out_sel[0] & mem_we      [0]) | (out_sel[1] & mem_we      [1]) | (out_sel[2] & mem_we      [2]);
assign mem_en_o       = (out_sel[0] & mem_en      [0]) | (out_sel[1] & mem_en      [1]) | (out_sel[2] & mem_en      [2]);
assign mem_wdata_o    = ({32{out_sel[0]}} & mem_wdata   [0]) | ({32{out_sel[1]}} & mem_wdata   [1]) | ({32{out_sel[2]}} & mem_wdata   [2]);
assign mem_size_o     = ({ 3{out_sel[0]}} & mem_size    [0]) | ({ 3{out_sel[1]}} & mem_size    [1]) | ({ 3{out_sel[2]}} & mem_size    [2]);
assign gr_we_o        = (out_sel[0] & gr_we       [0]) | (out_sel[1] & gr_we       [1]) | (out_sel[2] & gr_we       [2]);
assign dest_o         = ({ 5{out_sel[0]}} & dest        [0]) | ({ 5{out_sel[1]}} & dest        [1]) | ({ 5{out_sel[2]}} & dest        [2]);

// stage 2: ALU (registered), mul (combinational CPA on top of its stage-1
// registers) or div (registered quotient/remainder, only once ~busy).
// Each option is qualified by out_sel[2], which makes all five terms exclusive.
wire [31:0] div_result = ({32{div_is_mod}} & div_rem) | ({32{~ div_is_mod}} & div_quot);
wire        sel_alu2   = out_sel[2] & alu_pos[2];
wire        sel_mul2   = out_sel[2] & mul_pos[2];
wire        sel_div2   = out_sel[2] & div_pos[2];

assign ex_result      = ({32{out_sel[0]}} & alu_result[0]) |
                        ({32{out_sel[1]}} & alu_result[1]) |
                        ({32{sel_alu2}}   & alu_result[2]) |
                        ({32{sel_mul2}}   & mul_result_wire) |
                        ({32{sel_div2}}   & div_result);

// ---------------------------------------------------------------------------
// EX -> ID forwarding, one group per stage.
// Only the ALU can supply a value from EX (stage 0..2). mul / div / load all
// show up as "pending" so ID waits until the value reaches MEM.
wire [2:0] will_write, can_supply;

assign will_write[0] = valid[0] & gr_we[0];
assign will_write[1] = valid[1] & gr_we[1];
assign will_write[2] = valid[2] & gr_we[2];

assign can_supply[0] = will_write[0] & ~ mem_en[0] & alu_pos[0];
assign can_supply[1] = will_write[1] & ~ mem_en[1] & alu_pos[1];
assign can_supply[2] = will_write[2] & ~ mem_en[2] & alu_pos[2];

assign fwd0_valid   = can_supply[0];
assign fwd0_pending = will_write[0] & ~ can_supply[0];
assign fwd0_dest    = dest[0];
assign fwd0_data    = alu_result[0];

assign fwd1_valid   = can_supply[1];
assign fwd1_pending = will_write[1] & ~ can_supply[1];
assign fwd1_dest    = dest[1];
assign fwd1_data    = alu_result[1];

assign fwd2_valid   = can_supply[2];
assign fwd2_pending = will_write[2] & ~ can_supply[2];
assign fwd2_dest    = dest[2];
assign fwd2_data    = alu_result[2];

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
    if (reset)               div_inflight <= 1'b0;
    else if (div_start)      div_inflight <= 1'b1;
    else if (div_leave)      div_inflight <= 1'b0;
end

endmodule
