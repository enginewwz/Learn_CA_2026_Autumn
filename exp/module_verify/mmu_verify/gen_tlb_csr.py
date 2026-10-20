import sys
def addi_w(rd,rj,s): return ((0x00A<<22)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def lu12i(rd,s):     return ((0x14000000)|((s&0xFFFFF)<<5)|rd)&0xFFFFFFFF
def ori(rd,rj,u):    return ((0x03800000)|((u&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def ld_w(rd,rj,s):   return ((0x28800000)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def st_w(rd,rj,s):   return ((0x29800000)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def csrrd(rd,csr):   return ((0x04000000)|((csr&0x3FFF)<<10)|rd)&0xFFFFFFFF
def csrwr(rd,csr):   return ((0x04000000)|((csr&0x3FFF)<<10)|(1<<5)|rd)&0xFFFFFFFF
def csrxchg(rd,rj,csr):return ((0x04000000)|((csr&0x3FFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
TLBIDX,TLBEHI,TLBELO0,TLBELO1,TLBRENTRY,EENTRY,DMW0,CRMD,ESTAT,ERA,BADV = 0x10,0x11,0x12,0x13,0x88,0x0c,0x180,0x00,0x05,0x06,0x07
TLBWR = 0x06483000
prog = [
 addi_w(1,0,0x18), addi_w(2,0,0x10),
 lu12i(3,0x1C00), ori(3,3,3),
 lu12i(4,0x1c000),
 addi_w(5,0,3), addi_w(6,0,0),
 lu12i(7,0xc00), lu12i(8,0xc00), ori(8,8,1),
 addi_w(9,0,0x20), lu12i(10,0xe0000), lu12i(11,4),
 lu12i(12,0x1c000), ori(12,12,0x300),
 lu12i(13,0x1c000), ori(13,13,0x200),
 lu12i(14,0xe0000), ori(14,14,9),
 addi_w(15,0,0x11),
 csrwr(7,TLBIDX), csrwr(4,TLBEHI), csrwr(3,TLBELO0), csrwr(3,TLBELO1), TLBWR,
 csrwr(8,TLBIDX), csrwr(6,TLBEHI), csrwr(5,TLBELO0), csrwr(5,TLBELO1), TLBWR,
 csrwr(12,TLBRENTRY), csrwr(13,EENTRY), csrwr(14,DMW0),
 st_w(15,9,0),
 csrxchg(2,1,CRMD),
 ld_w(16,9,0),
 st_w(16,9,4),
 ld_w(17,10,0x40),
 ld_w(18,11,0),
 0x50000000,
]
handler = [
 csrrd(20,ESTAT), st_w(20,0,0x60),
 csrrd(20,ERA),   st_w(20,0,0x64),
 csrrd(20,BADV),  st_w(20,0,0x68),
 csrrd(20,TLBEHI),st_w(20,0,0x6c),
 csrrd(20,CRMD),  st_w(20,0,0x70),
 0x50000000,
]
out = sys.stdout
out.write("""module axi_slave_tiny #(parameter LAT = 1)(
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
initial begin
""")
for i,w in enumerate(prog):
    out.write("    slave.mem[13'h%04x] = 32'h%08x;\n" % (0x1000+i, w))
for i,w in enumerate(handler):
    out.write("    slave.mem[13'h%04x] = 32'h%08x;\n" % (0x10c0+i, w))
out.write("    slave.mem[13'h0010] = 32'hdead_beef;\n")
out.write("""    resetn = 0; repeat (4) @(posedge clk); resetn = 1;
    repeat (900) @(posedge clk);
    $display("r16(tlb ld)=%h  mem9(0x24)=%h  mem8(0x20)=%h", dut.u_regfile.rf[16], slave.mem[9], slave.mem[8]);
    $display("handler: ESTAT=%h ERA=%h BADV=%h TLBEHI=%h CRMD=%h",
             slave.mem[24], slave.mem[25], slave.mem[26], slave.mem[27], slave.mem[28]);
    $display("tlb0: e=%b vppn=%h asid=%h g=%b ppn0=%h v0=%b d0=%b plv0=%h | tlb1: e=%b vppn=%h ppn0=%h",
             dut.u_tlb.tlb_e[0], dut.u_tlb.tlb_vppn[0], dut.u_tlb.tlb_asid[0], dut.u_tlb.tlb_g[0],
             dut.u_tlb.tlb_ppn0[0], dut.u_tlb.tlb_v0[0], dut.u_tlb.tlb_d0[0], dut.u_tlb.tlb_plv0[0],
             dut.u_tlb.tlb_e[1], dut.u_tlb.tlb_vppn[1], dut.u_tlb.tlb_ppn0[1]);
    $finish;
end
always @(posedge clk) if (resetn && dbg_we[0])
    $display("WB  pc=%h  r%0d <= %h", dbg_pc, dbg_wnum, dbg_wdata);
always @(posedge clk) if (resetn && dut.pc[3] >= 32'h1c000084 && dut.pc[3] <= 32'h1c0000a0)
    $display("t=%0t pcMEM=%h v=%b men=%b dreq=%b dabort=%b tlbcause=%h excp=%b ecode=%h nextpc=%h",
        $time, dut.pc[3], dut.valid[3], dut.mem_en[3], dut.data_req, dut.data_abort,
        dut.mem_tlb_cause, dut.excp_i, dut.excp_ecode, dut.nextpc);
always @(posedge clk) if (resetn && dut.pc[0] >= 32'h1c00008c && dut.pc[0] <= 32'h1c000094)
    $display("t=%0t pcIF=%h st=%b ifpend=%b abort=%b xlat=%b s0f=%b s0v=%b dmw=%b da=%b pg=%b arv=%b ara=%h rv=%b",
        $time, dut.pc[0], dut.u_mmu.if_state, dut.if_pending, dut.if_abort, dut.if_xlat,
        dut.tlb_s0_found, dut.tlb_s0_v, dut.if_dmw_hit, dut.crmd_da_o, dut.crmd_pg_o,
        dut.arvalid, dut.araddr, dut.rvalid);
endmodule
""")
