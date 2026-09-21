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

assign stall_pipeline[3] = ~ allow_next & valid[3];
assign stall_pipeline[2] = stall_pipeline[3] & valid[2];
assign stall_pipeline[1] = stall_pipeline[2] & valid[1];
assign stall_pipeline[0] = stall_pipeline[1] & valid[0];
assign stall_o = stall_pipeline[0];



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
    if (! stall_pipeline[3]) final_result <= ({32{alu_pos[2]}} & alu_result[2]) | ({32{mul_pos[2]}} & mul_result_wire);
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
        end
        if (! stall_pipeline[1]) begin
            alu_pos [1] <= alu_pos [0];
            mul_pos [1] <= mul_pos [0];
        end
        if (! stall_pipeline[2]) begin
            alu_pos [2] <= alu_pos [1];
            mul_pos [2] <= mul_pos [1];
        end
        if (! stall_pipeline[3]) begin
            alu_pos [3] <= alu_pos [2];
            mul_pos [3] <= mul_pos [2];
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

assign alu_no_mul_shortcut[2] = (~ mul_pos[  3]) & alu_pos[2] & valid[2] & (~|(alu_pos[  3] & valid[  3])) & allow_next;
assign alu_no_mul_shortcut[1] = (~|mul_pos[3:2]) & alu_pos[1] & valid[1] & (~|(alu_pos[3:2] & valid[3:2])) & allow_next;
assign alu_no_mul_shortcut[0] = (~|mul_pos[3:1]) & alu_pos[0] & valid[0] & (~|(alu_pos[3:1] & valid[3:1])) & allow_next;

// Stage 3 normally drives the outputs; when it is empty an ALU result that is
// already the oldest instruction short-circuits straight out of its stage.
// (valid[3] keeps priority so an older valid instruction is never overtaken.)
assign out_sel[3]   = valid [3];
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

endmodule
