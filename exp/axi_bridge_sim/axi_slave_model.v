// Local single-beat AXI slave model for exercising sram_axi_bridge.
//
//   * one beat per transaction (the bridge never sets len != 0)
//   * the AXI3-flavoured port set the bridge drives (wid present, 2 bit lock,
//     8 bit len, rid/bid echo the request id)
//   * MEM_WORDS words of RAM, indexed by addr[ADDR_W+1:2] (low bits ignored,
//     like the blk_mem_gen based axi_ram)
//   * LAT extra wait states on every handshake, plus optional pseudo random
//     jitter (JITTER = 0 disables it) so the bridge is stressed with a latency
//     that is not constant
//   * a read outside the RAM answers SLVERR
//
// Plain Verilog, no vendor primitives, so it runs under iverilog.
module axi_slave_model #(
    parameter MEM_WORDS = 16384,   // 64 KB
    parameter ADDR_W    = 14,      // log2(MEM_WORDS)
    parameter LAT       = 1,       // extra cycles before a handshake is accepted
    parameter JITTER    = 0        // 0 = fixed latency, 1 = pseudo random extra
)(
    input  wire        aclk,
    input  wire        aresetn,

    input  wire [ 3:0] arid,
    input  wire [31:0] araddr,
    input  wire [ 7:0] arlen,
    input  wire [ 2:0] arsize,
    input  wire [ 1:0] arburst,
    input  wire [ 1:0] arlock,
    input  wire [ 3:0] arcache,
    input  wire [ 2:0] arprot,
    input  wire        arvalid,
    output wire        arready,

    output wire [ 3:0] rid,
    output wire [31:0] rdata,
    output wire [ 1:0] rresp,
    output wire        rlast,
    output wire        rvalid,
    input  wire        rready,

    input  wire [ 3:0] awid,
    input  wire [31:0] awaddr,
    input  wire [ 7:0] awlen,
    input  wire [ 2:0] awsize,
    input  wire [ 1:0] awburst,
    input  wire [ 1:0] awlock,
    input  wire [ 3:0] awcache,
    input  wire [ 2:0] awprot,
    input  wire        awvalid,
    output wire        awready,

    input  wire [ 3:0] wid,
    input  wire [31:0] wdata,
    input  wire [ 3:0] wstrb,
    input  wire        wlast,
    input  wire        wvalid,
    output wire        wready,

    output wire [ 3:0] bid,
    output wire [ 1:0] bresp,
    output wire        bvalid,
    input  wire        bready
);

reg [31:0] mem [0:MEM_WORDS-1];

//------------------------------------------------------------------ wait states
reg [7:0] lfsr;
wire [7:0] jit = JITTER ? {5'b0, lfsr[2:0]} : 8'h0;

always @(posedge aclk) begin
    if (!aresetn) lfsr <= 8'h1;
    else          lfsr <= {lfsr[6:0], lfsr[7] ^ lfsr[5] ^ lfsr[4] ^ lfsr[3]};
end

//------------------------------------------------------------------ read channel
reg  [ 3:0] rd_id_q;
reg  [31:0] rd_addr_q;
reg         rd_busy;
reg  [ 7:0] rd_wait;
reg         rd_err;

wire [ADDR_W-1:0] rd_index = rd_addr_q[ADDR_W+1:2];

assign arready = ~ rd_busy;
assign rvalid  = rd_busy & (rd_wait == 8'h0);
assign rid     = rd_id_q;
assign rlast   = 1'b1;
assign rresp   = rd_err ? 2'b10 : 2'b00;                  // SLVERR for an unmapped word
assign rdata   = rd_err ? 32'hdead_beef : mem[rd_index];  // live read, mirrors a RAM

always @(posedge aclk) begin
    if (!aresetn) begin
        rd_busy <= 1'b0;
        rd_wait <= 8'h0;
    end
    else if (arvalid & arready) begin
        rd_busy   <= 1'b1;
        rd_id_q   <= arid;
        rd_addr_q <= araddr;
        rd_err    <= (araddr[31:ADDR_W+2] != 0);
        rd_wait   <= LAT + jit;
    end
    else if (rd_busy & (rd_wait != 8'h0)) begin
        rd_wait <= rd_wait - 8'h1;
    end
    else if (rd_busy & rready) begin
        rd_busy <= 1'b0;
    end
end

//------------------------------------------------------------------ write channel
reg  [ 3:0] aw_id_q;
reg  [31:0] aw_addr_q;
reg         aw_busy;      // holds the address until the data beat arrives
reg  [ 7:0] aw_wait;

reg         w_busy;       // holds the data beat until the address has arrived
reg  [31:0] w_data_q;
reg  [ 3:0] w_strb_q;
reg  [ 7:0] w_wait;

reg  [ 3:0] wr_id_q;
reg         b_busy;
reg  [ 7:0] b_wait;

wire [ADDR_W-1:0] wr_index = aw_addr_q[ADDR_W+1:2];

assign awready = ~ aw_busy & (aw_wait == 8'h0);
assign wready  = ~ w_busy  & (w_wait  == 8'h0);
assign bid     = wr_id_q;
assign bresp   = 2'b00;
assign bvalid  = b_busy & (b_wait == 8'h0);

always @(posedge aclk) begin
    if (!aresetn) begin
        aw_busy <= 1'b0; aw_wait <= 8'h0;
        w_busy  <= 1'b0; w_wait  <= 8'h0;
        b_busy  <= 1'b0; b_wait  <= 8'h0;
    end
    else begin
        // AW
        if (awvalid & awready) begin
            aw_busy   <= 1'b1;
            aw_id_q   <= awid;
            aw_addr_q <= awaddr;
            aw_wait   <= LAT + jit;
        end
        else if (aw_busy & (aw_wait != 8'h0)) aw_wait <= aw_wait - 8'h1;

        // W
        if (wvalid & wready) begin
            w_busy   <= 1'b1;
            w_data_q <= wdata;
            w_strb_q <= wstrb;
            w_wait   <= LAT + jit;
        end
        else if (w_busy & (w_wait != 8'h0)) w_wait <= w_wait - 8'h1;

        // commit once both halves are in, then answer B
        if (aw_busy & w_busy & (aw_wait == 8'h0) & (w_wait == 8'h0) & ~ b_busy) begin
            if (w_strb_q[0]) mem[wr_index][ 7: 0] <= w_data_q[ 7: 0];
            if (w_strb_q[1]) mem[wr_index][15: 8] <= w_data_q[15: 8];
            if (w_strb_q[2]) mem[wr_index][23:16] <= w_data_q[23:16];
            if (w_strb_q[3]) mem[wr_index][31:24] <= w_data_q[31:24];
            aw_busy <= 1'b0;
            w_busy  <= 1'b0;
            b_busy  <= 1'b1;
            wr_id_q <= aw_id_q;
            b_wait  <= LAT;
        end

        // B
        if (b_busy & (b_wait != 8'h0)) begin
            b_wait <= b_wait - 8'h1;
        end
        else if (b_busy & bready) begin
            b_busy <= 1'b0;
        end
    end
end

endmodule
