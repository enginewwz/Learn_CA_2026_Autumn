module regfile(
    input  wire        clk,
    // READ PORT 1 (ID)
    input  wire [ 4:0] raddr1,
    output wire [31:0] rdata1,
    // READ PORT 2 (ID)
    input  wire [ 4:0] raddr2,
    output wire [31:0] rdata2,
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

endmodule
