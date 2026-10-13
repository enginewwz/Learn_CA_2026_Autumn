// Local testbench for sram_axi_bridge: drives the cpu side the way mycpu_top
// does and checks everything against a reference copy of the memory.
//
//   pc_fe / pc_if model the top: an instruction leaves pre_IF (and the next pc
//   is fetched) on a cycle where allow_if is high, and its data is taken from
//   inst_rdata on the cycle the fetch completes.  A data access is presented
//   like the MEM stage does and held until mem_pending drops.
//
// Run:  ./run.sh            (fixed latency)
//       ./run.sh 3 1        (LAT=3, jitter on)
// Parameters can also be set with iverilog -P tb_axi_bridge.LAT=3
`timescale 1ns/1ps

module tb_axi_bridge;

parameter LAT           = 1;
parameter JITTER        = 0;
parameter MEM_WORDS     = 16384;
parameter ADDR_W        = 14;
parameter CODE_BASE     = 32'h0000_0000;
parameter DATA_BASE     = 32'h0000_1000;
parameter CODE_WORDS    = 64;
parameter WATCHDOG      = 400000;

reg clk = 1'b0;
always #5 clk = ~ clk;

reg resetn = 1'b0;
reg reset  = 1'b1;
always @(posedge clk) reset <= ~ resetn;

//------------------------------------------------------------------ cpu side
reg  [31:0] pc_fe;
reg  [31:0] pc_if;
reg         if_valid;
reg         fe_enable;

reg         data_req;
reg  [ 3:0] data_we;
reg  [31:0] data_addr;
reg  [31:0] data_wdata;
reg  [ 1:0] crmd_plv;

wire        allow_if = ~ inst_pending;

wire        aclk    = clk;
wire        aresetn = resetn;

wire        inst_req;
wire [31:0] inst_addr;
wire [31:0] inst_rdata;
wire        inst_pending;
wire [31:0] data_rdata;
wire        mem_pending;

assign inst_req  = fe_enable;
assign inst_addr = pc_fe;

//------------------------------------------------------------------ axi
wire [ 3:0] arid;
wire [31:0] araddr;
wire [ 7:0] arlen;
wire [ 2:0] arsize;
wire [ 1:0] arburst;
wire [ 1:0] arlock;
wire [ 3:0] arcache;
wire [ 2:0] arprot;
wire        arvalid;
wire        arready;
wire [ 3:0] rid;
wire [31:0] rdata;
wire [ 1:0] rresp;
wire        rlast;
wire        rvalid;
wire        rready;
wire [ 3:0] awid;
wire [31:0] awaddr;
wire [ 7:0] awlen;
wire [ 2:0] awsize;
wire [ 1:0] awburst;
wire [ 1:0] awlock;
wire [ 3:0] awcache;
wire [ 2:0] awprot;
wire        awvalid;
wire        awready;
wire [ 3:0] wid;
wire [31:0] wdata;
wire [ 3:0] wstrb;
wire        wlast;
wire        wvalid;
wire        wready;
wire [ 3:0] bid;
wire [ 1:0] bresp;
wire        bvalid;
wire        bready;

sram_axi_bridge u_bridge(
    .clk         (clk        ),
    .reset       (reset      ),
    .inst_req    (inst_req   ),
    .inst_addr   (inst_addr  ),
    .allow_if    (allow_if   ),
    .inst_rdata  (inst_rdata ),
    .inst_pending(inst_pending),
    .crmd_plv    (crmd_plv   ),
    .data_req    (data_req   ),
    .data_we     (data_we    ),
    .data_addr   (data_addr  ),
    .data_wdata  (data_wdata ),
    .data_rdata  (data_rdata ),
    .mem_pending (mem_pending),
    .arid        (arid       ),
    .araddr      (araddr     ),
    .arlen       (arlen      ),
    .arsize      (arsize     ),
    .arburst     (arburst    ),
    .arlock      (arlock     ),
    .arcache     (arcache    ),
    .arprot      (arprot     ),
    .arvalid     (arvalid    ),
    .arready     (arready    ),
    .rid         (rid        ),
    .rdata       (rdata      ),
    .rresp       (rresp      ),
    .rlast       (rlast      ),
    .rvalid      (rvalid     ),
    .rready      (rready     ),
    .awid        (awid       ),
    .awaddr      (awaddr     ),
    .awlen       (awlen      ),
    .awsize      (awsize     ),
    .awburst     (awburst    ),
    .awlock      (awlock     ),
    .awcache     (awcache    ),
    .awprot      (awprot     ),
    .awvalid     (awvalid    ),
    .awready     (awready    ),
    .wid         (wid        ),
    .wdata       (wdata      ),
    .wstrb       (wstrb      ),
    .wlast       (wlast      ),
    .wvalid      (wvalid     ),
    .wready      (wready     ),
    .bid         (bid        ),
    .bresp       (bresp      ),
    .bvalid      (bvalid     ),
    .bready      (bready     )
);

axi_slave_model #(
    .MEM_WORDS (MEM_WORDS),
    .ADDR_W    (ADDR_W   ),
    .LAT       (LAT      ),
    .JITTER    (JITTER   )
) u_slave(
    .aclk(aclk), .aresetn(aresetn),
    .arid(arid), .araddr(araddr), .arlen(arlen), .arsize(arsize),
    .arburst(arburst), .arlock(arlock), .arcache(arcache), .arprot(arprot),
    .arvalid(arvalid), .arready(arready),
    .rid(rid), .rdata(rdata), .rresp(rresp), .rlast(rlast),
    .rvalid(rvalid), .rready(rready),
    .awid(awid), .awaddr(awaddr), .awlen(awlen), .awsize(awsize),
    .awburst(awburst), .awlock(awlock), .awcache(awcache), .awprot(awprot),
    .awvalid(awvalid), .awready(awready),
    .wid(wid), .wdata(wdata), .wstrb(wstrb), .wlast(wlast),
    .wvalid(wvalid), .wready(wready),
    .bid(bid), .bresp(bresp), .bvalid(bvalid), .bready(bready)
);

//------------------------------------------------------------------ reference
reg [31:0] ref [0:MEM_WORDS-1];
integer i;
initial begin
    for (i = 0; i < MEM_WORDS; i = i + 1) begin
        ref[i]        = 32'h1000_0000 + i * 32'h0001_0003;
        u_slave.mem[i] = ref[i];
    end
end

//------------------------------------------------------------------ scoreboard
integer checks = 0;
integer errors = 0;
integer fe_count = 0;
integer c_fetch0 = 0;
integer sim_cycles = 0;

always @(posedge clk) sim_cycles = sim_cycles + 1;

// ---- monitor: the AXI write payload must not move while a store is in flight
reg [31:0] mon_addr_q, mon_wdata_q;
reg [ 3:0] mon_we_q;
reg        mon_busy_q;
always @(posedge clk) begin
    if (!reset) begin
        if (mon_busy_q &&
            ((u_bridge.awaddr_r  !== mon_addr_q) ||
             (u_bridge.wdata_r !== mon_wdata_q) ||
             (u_bridge.wstrb_r    !== mon_we_q))) begin
            errors = errors + 1;
            $display("MON: write payload changed while a store was in flight (t=%0t): addr %h->%h wdata %h->%h we %b->%b awvalid=%b wvalid=%b",
                     $time, mon_addr_q, u_bridge.awaddr_r, mon_wdata_q, u_bridge.wdata_r,
                     mon_we_q, u_bridge.wstrb_r, awvalid, wvalid);
        end
        mon_busy_q  <= u_bridge.mem_write_state[1] |
                       (u_bridge.mem_write_state[2] & ~ u_bridge.write_rsp_done);
        mon_addr_q  <= u_bridge.awaddr_r;
        mon_wdata_q <= u_bridge.wdata_r;
        mon_we_q    <= u_bridge.wstrb_r;
    end
end

// ---- white-box monitors on the bridge -------------------------------------
// 1. the two read requesters must never be in REQ together (shared AR channel)
// 2. a beat accepted by rready must belong to a requester whose FSM is in RSP
reg dbg_on = 1'b0;
always @(posedge clk) begin
    if (!reset) begin
        if (u_bridge.inst_state[1] & u_bridge.mem_read_state[1]) begin
            errors = errors + 1;
            $display("MON: both read requesters in REQ (t=%0t)", $time);
        end
        if (rvalid & rready & !((u_bridge.inst_state[2] & (rid == 4'b0)) |
                                (u_bridge.mem_read_state[2] & (rid == 4'b1)))) begin
            errors = errors + 1;
            $display("MON: beat consumed with no FSM in RSP: rid=%h t=%0t", rid, $time);
        end
        if (u_bridge.mem_read_state[2] & rvalid & (rid == 4'b1) & (rdata === 32'h0)) begin
            errors = errors + 1;
            $display("MON: data read got 0 from the slave, araddr was %h (t=%0t)",
                     u_bridge.araddr, $time);
        end
    end
end
always @(posedge clk) begin
    if (dbg_on) $display("t=%0t pend=%b mstate=%b arv=%b araddr=%h arready=%b rv=%b rid=%h rdata=%h drdata=%h dreq=%b dwe=%b daddr=%h",
        $time, mem_pending, u_bridge.mem_read_state, arvalid, araddr, arready,
        rvalid, rid, rdata, data_rdata, data_req, data_we, data_addr);
end

task chk;
    input        cond;
    input [255:0] name;
    begin
        checks = checks + 1;
        if (!cond) begin
            errors = errors + 1;
            $display("ERROR: %0s (t=%0t)", name, $time);
        end
    end
endtask

task chk32;
    input [31:0] exp;
    input [31:0] got;
    input [255:0] name;
    begin
        checks = checks + 1;
        if (exp !== got) begin
            errors = errors + 1;
            $display("ERROR: %0s exp=%h got=%h (t=%0t)", name, exp, got, $time);
        end
    end
endtask

//------------------------------------------------------------------ front end
// an instruction leaves pre_IF on a cycle where allow_if (& inst_req) is high:
// the next pc is fetched and the one that was in pre_IF lands in IF.  Its data
// is taken one cycle later, i.e. on the cycle the fetch completes.
always @(posedge clk) begin
    if (reset) begin
        pc_fe    <= CODE_BASE;
        pc_if    <= 32'b0;
        if_valid <= 1'b0;
    end
    else if (allow_if & inst_req) begin
        if (if_valid) begin
            checks = checks + 1;
            if (inst_rdata !== ref[pc_if[ADDR_W+1:2]]) begin
                errors = errors + 1;
                $display("ERROR: fetch pc=%h exp=%h got=%h (t=%0t)",
                         pc_if, ref[pc_if[ADDR_W+1:2]], inst_rdata, $time);
            end
            fe_count <= fe_count + 1;
        end
        pc_if    <= pc_fe;
        pc_fe    <= pc_fe + 32'h4;
        if_valid <= 1'b1;
    end
end

//------------------------------------------------------------------ mem access
// one access, presented like the MEM stage: held until the bridge is done.  A
// store retires on the cycle it is taken, a load on the cycle its data is on
// data_rdata.
task mem_access;
    input  [31:0] addr;
    input  [ 3:0] we;
    input  [31:0] wd;
    input         is_read;
    output [31:0] rd;
    begin
        @(posedge clk); #1;
        data_req   = 1'b1;
        data_we    = we;
        data_addr  = addr;
        data_wdata = wd;
        @(negedge clk);                       // let the combinational settle
        while (mem_pending) @(negedge clk);
        if (is_read) rd = data_rdata;
        @(posedge clk); #1;
        data_req   = 1'b0;
        data_we    = 4'b0;
        data_addr  = 32'b0;
        data_wdata = 32'b0;
    end
endtask

task do_load;
    input  [31:0] addr;
    output [31:0] rd;
    begin mem_access(addr, 4'b0, 32'b0, 1'b1, rd); end
endtask

task do_store;
    input [31:0] addr;
    input [ 3:0] we;
    input [31:0] wd;
    reg   [31:0] dummy;
    begin mem_access(addr, we, wd, 1'b0, dummy); end
endtask

// merge a partial store into the reference the same way the slave does
task ref_store;
    input [31:0] addr;
    input [ 3:0] we;
    input [31:0] wd;
    integer idx;
    begin
        idx = addr[ADDR_W+1:2];
        if (we[0]) ref[idx][ 7: 0] = wd[ 7: 0];
        if (we[1]) ref[idx][15: 8] = wd[15: 8];
        if (we[2]) ref[idx][23:16] = wd[23:16];
        if (we[3]) ref[idx][31:24] = wd[31:24];
    end
endtask

//------------------------------------------------------------------ tests
integer k;
integer seed   = 32'h1234_5678;
integer nrand  = 400;
reg [31:0] addr, wd, rd;
reg [ 3:0] we;

initial begin
    if ($test$plusargs("vcd")) begin
        $dumpfile("tb_axi_bridge.vcd");
        $dumpvars(0, tb_axi_bridge);
    end

    if (!$value$plusargs("seed=%d", seed)) seed  = 32'h1234_5678;
    if (!$value$plusargs("n=%d",    nrand)) nrand = 400;
    $display("[cfg] seed=%0d n=%0d", seed, nrand);

    pc_fe = CODE_BASE; pc_if = 32'b0; if_valid = 1'b0; fe_enable = 1'b0;
    data_req = 1'b0; data_we = 4'b0; data_addr = 32'b0; data_wdata = 32'b0;
    crmd_plv = 2'b00;

    repeat (8) @(posedge clk);
    resetn = 1'b1;
    repeat (4) @(posedge clk);

    //------------------------------- 0. the FIRST memory access is a store
    // (in the real program tests 1..13 are alu/shift only, so the very first
    // data access of the whole run is a store, followed by the load that reads
    // it back)
    fe_enable = 1'b0;
    addr = DATA_BASE + 32'h10;
    do_store(addr, 4'hf, 32'h0bad_f00d);
    ref[addr >> 2] = 32'h0bad_f00d;
    do_load(addr, rd);
    chk32(32'h0bad_f00d, rd, "first ever access is a store");

    //-------------------------------------------------- 1. instruction fetch
    fe_enable = 1'b1;
    c_fetch0  = sim_cycles;
    wait (fe_count >= CODE_WORDS);
    fe_enable = 1'b0;
    repeat (4) @(posedge clk);
    $display("[1] fetch    : %0d instructions in %0d cycles  -> %0d.%02d cycles/inst",
             CODE_WORDS, sim_cycles - c_fetch0,
             (sim_cycles - c_fetch0) / CODE_WORDS,
             ((sim_cycles - c_fetch0) * 100 / CODE_WORDS) % 100);

    //-------------------------------------------------- 2. plain load
    // keep the instruction fetch running: in the real soc the first ld.w of a
    // test happens while fetches are in flight on the shared AR/R channel
    fe_enable = 1'b1;
    dbg_on = $test$plusargs("dbg");
    do_load(DATA_BASE + 32'h40, rd);
    dbg_on = 1'b0;
    chk32(ref[(DATA_BASE + 32'h40) >> 2], rd, "load word");

    //-------------------------------------------------- 3. store / read back
    addr = DATA_BASE + 32'h80; wd = 32'hcafe_1234;
    do_store(addr, 4'hf, wd);
    ref_store(addr, 4'hf, wd);
    do_load(addr, rd);
    chk32(wd, rd, "store then load");

    //-------------------------------------------------- 4. byte / half stores
    addr = DATA_BASE + 32'h84;
    do_store(addr, 4'b0001, 32'h0000_00ab);
    ref_store(addr, 4'b0001, 32'h0000_00ab);
    do_load(addr, rd);
    chk32(ref[addr >> 2], rd, "byte store");

    do_store(addr, 4'b1100, 32'hbeef_0000);
    ref_store(addr, 4'b1100, 32'hbeef_0000);
    do_load(addr, rd);
    chk32(ref[addr >> 2], rd, "half store");

    //-------------------------------------------------- 5. back to back stores
    // the cpu presents the second store on the very cycle the first is taken;
    // the bridge must keep them in order (WAW) and not lose either of them
    addr = DATA_BASE + 32'hc0;
    @(posedge clk); #1;
    data_req = 1'b1; data_we = 4'hf; data_addr = addr; data_wdata = 32'haaaa_1111;
    @(negedge clk);                           // let the combinational settle
    while (mem_pending) @(negedge clk);
    @(posedge clk); #1;                               // first store is taken here
    data_wdata = 32'hbbbb_2222;                       // second store, same word
    while (mem_pending) @(negedge clk);               // waits while the slot is busy
    @(posedge clk); #1;
    data_req = 1'b0; data_we = 4'b0; data_addr = 32'b0; data_wdata = 32'b0;
    ref[addr >> 2] = 32'hbbbb_2222;
    repeat (20) @(posedge clk);                       // let the write drain
    do_load(addr, rd);
    chk32(32'hbbbb_2222, rd, "back-to-back stores keep order");

    //-------------------------------------------------- 6. RAW
    addr = DATA_BASE + 32'h100; wd = 32'h5a5a_0f0f;
    do_store(addr, 4'hf, wd);
    ref_store(addr, 4'hf, wd);
    do_load(addr, rd);                                // issued while the write drains
    chk32(wd, rd, "read after write");

    //------------------------------- 6b. mirror of TEST_LD_W's instruction mix
    // three stores back to back, then a load of the FIRST store's word: the load
    // must wait for the whole write chain (RAW) and read the stored data
    fe_enable = 1'b1;
    addr = DATA_BASE + 32'h200;
    @(posedge clk); #1;
    data_req = 1'b1; data_we = 4'hf; data_addr = addr; data_wdata = 32'hc822c7e8;
    @(negedge clk); while (mem_pending) @(negedge clk);
    @(posedge clk); #1;                                   // store 1 taken
    data_addr = addr + 4; data_wdata = addr + 4;           // store 2 right behind
    @(negedge clk); while (mem_pending) @(negedge clk);
    @(posedge clk); #1;
    data_addr = addr - 8; data_wdata = addr - 8;           // store 3
    @(negedge clk); while (mem_pending) @(negedge clk);
    @(posedge clk); #1;
    data_we = 4'b0; data_addr = addr; data_wdata = 32'b0;  // and now the load
    @(negedge clk); while (mem_pending) @(negedge clk);
    rd = data_rdata;
    @(posedge clk); #1;
    data_req = 1'b0; data_we = 4'b0; data_addr = 32'b0; data_wdata = 32'b0;
    fe_enable = 1'b0;
    ref[addr >> 2] = 32'hc822c7e8;
    ref[(addr + 4) >> 2] = addr + 4;
    ref[(addr - 8) >> 2] = addr - 8;
    chk32(32'hc822c7e8, rd, "three stores then load the first word");

    // and every one of the three stores must have landed with ITS OWN payload:
    // a read taken while the write is still in flight must not clobber it
    do_load(addr + 4, rd);
    chk32(addr + 4, rd, "store 2 payload intact");
    do_load(addr - 8, rd);
    chk32(addr - 8, rd, "store 3 payload intact");

    //-------------------------------------------------- 7. fetch with a store in flight
    fe_enable = 1'b1;
    @(posedge clk); #1;
    data_req = 1'b1; data_we = 4'hf; data_addr = DATA_BASE + 32'h140; data_wdata = 32'h1234_abcd;
    @(negedge clk);                           // let the combinational settle
    while (mem_pending) @(negedge clk);
    @(posedge clk); #1;
    data_req = 1'b0; data_we = 4'b0;
    repeat (40) @(posedge clk);
    fe_enable = 1'b0;
    ref_store(DATA_BASE + 32'h140, 4'hf, 32'h1234_abcd);
    do_load(DATA_BASE + 32'h140, rd);
    chk32(32'h1234_abcd, rd, "store landed while fetching");

    //-------------------------------------------------- 8. random mix
    // background fetches run the whole time, so the two read requesters fight
    // over the shared AR/R channel
    fe_enable = 1'b1;
    for (k = 0; k < nrand; k = k + 1) begin
        addr = DATA_BASE + (($unsigned($random(seed)) % 512) * 4);
        we   = 4'hf;                                    // full words in the mix
        wd   = $random(seed);
        if (($random(seed) % 2) == 0) begin
            do_store(addr, we, wd);
            ref_store(addr, we, wd);
        end
        else begin
            do_load(addr, rd);
            chk32(ref[addr >> 2], rd, "random load");
        end
    end
    fe_enable = 1'b0;
    repeat (20) @(posedge clk);
    $display("[8] random   : %0d mixed accesses done", nrand);

    //-------------------------------------------------- done
    $display("==========================================");
    if (errors == 0) $display("PASS: %0d checks, 0 errors (%0d cycles)", checks, sim_cycles);
    else             $display("FAIL: %0d checks, %0d errors", checks, errors);
    $display("==========================================");
    $finish;
end

//------------------------------------------------------------------ watchdog
initial begin
    repeat (WATCHDOG) @(posedge clk);
    $display("FAIL: watchdog timeout (%0d checks, %0d errors, t=%0t)", checks, errors, $time);
    $finish;
end

endmodule
