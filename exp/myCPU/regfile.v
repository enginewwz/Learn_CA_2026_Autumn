module regfile(
    input  wire        clk,
    // READ PORT 1 (ID slot 0, src1)
    input  wire [ 4:0] raddr1_0,
    output wire [31:0] rdata1_0,
    // READ PORT 2 (ID slot 0, src2)
    input  wire [ 4:0] raddr2_0,
    output wire [31:0] rdata2_0,
    // READ PORT 3 (ID slot 1, src1)
    input  wire [ 4:0] raddr1_1,
    output wire [31:0] rdata1_1,
    // READ PORT 4 (ID slot 1, src2)
    input  wire [ 4:0] raddr2_1,
    output wire [31:0] rdata2_1,
    // WRITE PORT (pipeline slot 0)
    input  wire        we_0,
    input  wire [ 4:0] waddr_0,
    input  wire [31:0] wdata_0,
    // WRITE PORT (pipeline slot 1): younger instruction, wins a WAW inside a pair
    input  wire        we_1,
    input  wire [ 4:0] waddr_1,
    input  wire [31:0] wdata_1,
    // CSR data exchange: same address as the slot 0 port, no read bypass
    input  wire        we_csr,
    input  wire [31:0] wdata_csr
);
reg [31:0] rf[31:0];

// WRITE: both slots may write in the same cycle.  Non-blocking assignments to
// the same word resolve in program order, so slot 1 is placed last.
always @(posedge clk) begin
    if (we_0)   rf[waddr_0] <= wdata_0;
    if (we_1)   rf[waddr_1] <= wdata_1;
    if (we_csr) rf[waddr_0] <= wdata_csr;
end

// READ: write-first bypass for both pipeline ports (slot 1 checked first: it is
// the younger of the pair).  The CSR data exchange keeps its "no bypass" rule --
// the CSR-write-RF dependency is resolved by waiting in ID (csr_dep).
assign rdata1_0 = (raddr1_0 == 5'b0) ? 32'b0
                : (we_1 & (raddr1_0 == waddr_1)) ? wdata_1
                : (we_0 & (raddr1_0 == waddr_0)) ? wdata_0
                : rf[raddr1_0];
assign rdata2_0 = (raddr2_0 == 5'b0) ? 32'b0
                : (we_1 & (raddr2_0 == waddr_1)) ? wdata_1
                : (we_0 & (raddr2_0 == waddr_0)) ? wdata_0
                : rf[raddr2_0];
assign rdata1_1 = (raddr1_1 == 5'b0) ? 32'b0
                : (we_1 & (raddr1_1 == waddr_1)) ? wdata_1
                : (we_0 & (raddr1_1 == waddr_0)) ? wdata_0
                : rf[raddr1_1];
assign rdata2_1 = (raddr2_1 == 5'b0) ? 32'b0
                : (we_1 & (raddr2_1 == waddr_1)) ? wdata_1
                : (we_0 & (raddr2_1 == waddr_0)) ? wdata_0
                : rf[raddr2_1];

endmodule
