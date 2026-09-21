module EX(
    input  wire        clk,
    input  wire        reset,
    input  wire        receive,         // receive_new_inst[EX]: ID -> EX transfer

    // ID -> EX: combinational at ID
    input  wire [31:0] pc_i,
    input  wire        src1_used_i,
    input  wire        src2_used_i,
    input  wire [11:0] alu_op_i,
    input  wire [31:0] alu_src1_i,
    input  wire [31:0] alu_src2_i,
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

    // execute result and handshake
    output wire [31:0] alu_result,
    output wire        stall_o
);

// EX-level registers: one register per field, never packed into one wide
// vector, so that adding a new pass-through field stays local
reg  [31:0] pc_ex;
reg         src1_used_ex;
reg         src2_used_ex;
reg  [11:0] alu_op_ex;
reg  [31:0] alu_src1_ex;
reg  [31:0] alu_src2_ex;
reg  [ 6:0] md_op_ex;
reg         res_from_mem_ex;
reg         mem_we_ex;
reg         mem_en_ex;
reg  [31:0] mem_wdata_ex;
reg         gr_we_ex;
reg  [ 4:0] dest_ex;

always @(posedge clk) begin
    if (reset)          pc_ex <= 32'h0;
    else if (receive)   pc_ex <= pc_i;
end

always @(posedge clk) begin
    if (receive) begin
        src1_used_ex    <= src1_used_i;
        src2_used_ex    <= src2_used_i;
        alu_op_ex       <= alu_op_i;
        alu_src1_ex     <= alu_src1_i;
        alu_src2_ex     <= alu_src2_i;
        md_op_ex        <= md_op_i;
        res_from_mem_ex <= res_from_mem_i;
        mem_we_ex       <= mem_we_i;
        mem_en_ex       <= mem_en_i;
        mem_wdata_ex    <= mem_wdata_i;
        gr_we_ex        <= gr_we_i;
        dest_ex         <= dest_i;
    end
end

alu u_alu(
    .alu_op     (alu_op_ex  ),
    .alu_src1   (alu_src1_ex),
    .alu_src2   (alu_src2_ex),
    .alu_result (alu_result )
);

assign pc_o           = pc_ex;
assign src1_used_o    = src1_used_ex;
assign src2_used_o    = src2_used_ex;
assign res_from_mem_o = res_from_mem_ex;
assign mem_we_o       = mem_we_ex;
assign mem_en_o       = mem_en_ex;
assign mem_wdata_o    = mem_wdata_ex;
assign gr_we_o        = gr_we_ex;
assign dest_o         = dest_ex;

// no multi-cycle operation yet; mul/div will drive this later
assign stall_o = 1'b0;

endmodule
