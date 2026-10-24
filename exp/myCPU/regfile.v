module regfile(
    input  wire        clk,
    // READ PORT 1 (ID)
    input  wire [ 4:0] raddr1,
    output wire [31:0] rdata1,
    // READ PORT 2 (ID)
    input  wire [ 4:0] raddr2,
    output wire [31:0] rdata2,
    // READ PORT 3/4 (EX0)
    input  wire [ 4:0] raddr1_ex0,
    output wire [31:0] rdata1_ex0,
    input  wire [ 4:0] raddr2_ex0,
    output wire [31:0] rdata2_ex0,
    // READ PORT 5/6 (EX1)
    input  wire [ 4:0] raddr1_ex1,
    output wire [31:0] rdata1_ex1,
    input  wire [ 4:0] raddr2_ex1,
    output wire [31:0] rdata2_ex1,
    // WRITE PORT (pipeline)
    input  wire        we,       //write enable, HIGH valid
    input  wire [ 4:0] waddr,
    input  wire [31:0] wdata,
    // CSR data exchange: no read bypass
    input  wire        we_csr,
    input  wire [31:0] wdata_csr
);
reg [31:0] rf[31:0];

//WRITE
always @(posedge clk) begin
    if (we)          rf[waddr] <= wdata;
    else if (we_csr) rf[waddr] <= wdata_csr;
end

// ID read ports use write-first so the branch/jump operand is never the
// pre-write value of a register committed in the same cycle.
assign rdata1 = (raddr1==5'b0) ? 32'b0 : (raddr1 != waddr || !we) ? rf[raddr1] : wdata;
assign rdata2 = (raddr2==5'b0) ? 32'b0 : (raddr2 != waddr || !we) ? rf[raddr2] : wdata;

// EX0 / EX1 read ports also use write-first: the ex0_fw / ex1_fw networks
// therefore do not need a separate WB-stage forwarding tap.
assign rdata1_ex0 = (raddr1_ex0==5'b0) ? 32'b0 : (raddr1_ex0 != waddr || !we) ? rf[raddr1_ex0] : wdata;
assign rdata2_ex0 = (raddr2_ex0==5'b0) ? 32'b0 : (raddr2_ex0 != waddr || !we) ? rf[raddr2_ex0] : wdata;
assign rdata1_ex1 = (raddr1_ex1==5'b0) ? 32'b0 : (raddr1_ex1 != waddr || !we) ? rf[raddr1_ex1] : wdata;
assign rdata2_ex1 = (raddr2_ex1==5'b0) ? 32'b0 : (raddr2_ex1 != waddr || !we) ? rf[raddr2_ex1] : wdata;

endmodule
