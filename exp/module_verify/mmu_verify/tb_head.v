module axi_slave_tiny #(parameter LAT = 1)(
    input  wire clk, input wire resetn,
    input  wire [3:0] arid, input wire [31:0] araddr, input wire arvalid, output wire arready,
    output wire [3:0] rid, output wire [31:0] rdata, output wire rvalid, input wire rready,
    output wire rlast,
    input  wire [3:0] awid, input wire [31:0] awaddr, input wire awvalid, output wire awready,
    input  wire [31:0] wdata, input wire [3:0] wstrb, input wire wvalid, output wire wready,
    output wire wlast, output wire [3:0] bid, output wire bvalid, input wire bready);
reg [31:0] mem [0:8191];
function [12:0] midx(input [31:0] a);
    midx = (a[31:28] == 4'h1) ? (13'h1000 | a[11:2]) : {1'b0, a[11:2]};
endfunction
localparam R_IDLE=2'd0, R_WAIT=2'd1, R_RSP=2'd2;
reg [1:0] r_state; reg [31:0] rd_addr_r, rd_data_r; reg [3:0] rd_id_r; reg [7:0] r_cnt;
assign arready = (r_state == R_IDLE); assign rdata = rd_data_r;
assign rvalid = (r_state == R_RSP);  assign rid = rd_id_r; assign rlast = 1'b1;
always @(posedge clk) begin
    if (!resetn) begin r_state<=R_IDLE; r_cnt<=0; rd_id_r<=0; rd_data_r<=0; rd_addr_r<=0; end
    else case (r_state)
        R_IDLE: if (arvalid) begin rd_addr_r<=araddr; rd_id_r<=arid; r_cnt<=LAT; r_state<=R_WAIT; end
        R_WAIT: if (r_cnt != 0) r_cnt <= r_cnt-1; else begin rd_data_r <= mem[midx(rd_addr_r)]; r_state <= R_RSP; end
        R_RSP:  if (rready) r_state <= R_IDLE;
    endcase
end
localparam W_IDLE=2'd0, W_WAIT=2'd1, W_B=2'd2;
reg [1:0] w_state; reg aw_got, w_got; reg [31:0] aw_addr_r, w_data_r;
reg [3:0] aw_id_r, w_strb_r; reg [7:0] b_cnt;
assign awready = (w_state==W_IDLE) & ~aw_got; assign wready = (w_state==W_IDLE) & ~w_got;
assign wlast = 1'b1; assign bid = aw_id_r; assign bvalid = (w_state==W_B);
wire [31:0] wmask = {{8{w_strb_r[3]}},{8{w_strb_r[2]}},{8{w_strb_r[1]}},{8{w_strb_r[0]}}};
wire [12:0] widx = midx(aw_addr_r);
always @(posedge clk) begin
    if (!resetn) begin w_state<=W_IDLE; aw_got<=0; w_got<=0; b_cnt<=0;
        aw_addr_r<=0; w_data_r<=0; aw_id_r<=0; w_strb_r<=0; end
    else case (w_state)
        W_IDLE: begin
            if (awvalid & awready) begin aw_got<=1; aw_addr_r<=awaddr; aw_id_r<=awid; end
            if (wvalid & wready)   begin w_got<=1;  w_data_r<=wdata;   w_strb_r<=wstrb; end
            if ((aw_got|(awvalid&awready)) & (w_got|(wvalid&wready))) begin w_state<=W_WAIT; b_cnt<=LAT; end
        end
        W_WAIT: if (b_cnt != 0) b_cnt <= b_cnt-1;
                else begin mem[widx] <= (mem[widx] & ~wmask) | (w_data_r & wmask); w_state <= W_B; end
        W_B:    if (bready) begin w_state<=W_IDLE; aw_got<=0; w_got<=0; end
    endcase
end
endmodule
module tb;
reg clk=0, resetn=0; always #5 clk = ~clk;
wire [3:0] arid; wire [31:0] araddr; wire [7:0] arlen; wire [2:0] arsize;
wire [1:0] arburst, arlock; wire [3:0] arcache; wire [2:0] arprot; wire arvalid, arready;
wire [3:0] rid; wire [31:0] rdata; wire [1:0] rresp; wire rlast, rvalid, rready;
wire [3:0] awid; wire [31:0] awaddr; wire [7:0] awlen; wire [2:0] awsize;
wire [1:0] awburst, awlock; wire [3:0] awcache; wire [2:0] awprot; wire awvalid, awready;
wire [3:0] wid; wire [31:0] wdata; wire [3:0] wstrb; wire wlast, wvalid, wready;
wire [3:0] bid; wire [1:0] bresp; wire bvalid, bready;
wire [31:0] dbg_pc, dbg_wdata; wire [3:0] dbg_we; wire [4:0] dbg_wnum;
mycpu_top dut(
    .aclk(clk), .aresetn(resetn),
    .arid(arid), .araddr(araddr), .arlen(arlen), .arsize(arsize), .arburst(arburst),
    .arlock(arlock), .arcache(arcache), .arprot(arprot), .arvalid(arvalid), .arready(arready),
    .rid(rid), .rdata(rdata), .rresp(rresp), .rlast(rlast), .rvalid(rvalid), .rready(rready),
    .awid(awid), .awaddr(awaddr), .awlen(awlen), .awsize(awsize), .awburst(awburst),
    .awlock(awlock), .awcache(awcache), .awprot(awprot), .awvalid(awvalid), .awready(awready),
    .wid(wid), .wdata(wdata), .wstrb(wstrb), .wlast(wlast), .wvalid(wvalid), .wready(wready),
    .bid(bid), .bresp(bresp), .bvalid(bvalid), .bready(bready),
    .debug_wb_pc(dbg_pc), .debug_wb_rf_we(dbg_we), .debug_wb_rf_wnum(dbg_wnum), .debug_wb_rf_wdata(dbg_wdata));
axi_slave_tiny #(.LAT(1)) slave(
    .clk(clk), .resetn(resetn),
    .arid(arid), .araddr(araddr), .arvalid(arvalid), .arready(arready),
    .rid(rid), .rdata(rdata), .rvalid(rvalid), .rready(rready), .rlast(rlast),
    .awid(awid), .awaddr(awaddr), .awvalid(awvalid), .awready(awready),
    .wdata(wdata), .wstrb(wstrb), .wvalid(wvalid), .wready(wready), .wlast(wlast),
    .bid(bid), .bvalid(bvalid), .bready(bready));
