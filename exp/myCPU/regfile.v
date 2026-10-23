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
    // WRITE PORT
    input  wire        we,       //write enable, HIGH valid
    input  wire [ 4:0] waddr,
    input  wire [31:0] wdata
);
reg [31:0] rf[31:0];

//WRITE
always @(posedge clk) begin
    if (we) rf[waddr] <= wdata;
end

// ID read ports use write-first so the branch/jump operand is never the
// pre-write value of a register committed in the same cycle.
assign rdata1 = (raddr1==5'b0) ? 32'b0 : (raddr1 != waddr || !we) ? rf[raddr1] : wdata;
assign rdata2 = (raddr2==5'b0) ? 32'b0 : (raddr2 != waddr || !we) ? rf[raddr2] : wdata;

// EX0 / EX1 read ports can read the array directly
assign rdata1_ex0 = (raddr1_ex0==5'b0) ? 32'b0 : rf[raddr1_ex0];
assign rdata2_ex0 = (raddr2_ex0==5'b0) ? 32'b0 : rf[raddr2_ex0];
assign rdata1_ex1 = (raddr1_ex1==5'b0) ? 32'b0 : rf[raddr1_ex1];
assign rdata2_ex1 = (raddr2_ex1==5'b0) ? 32'b0 : rf[raddr2_ex1];

endmodule
