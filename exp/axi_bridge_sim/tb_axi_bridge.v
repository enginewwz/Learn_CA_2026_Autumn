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
integer sim_cycles = 0;

always @(posedge clk) sim_cycles = sim_cycles + 1;

reg dbg_on = 1'b0;
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
reg [31:0] addr, wd, rd;
reg [ 3:0] we;

initial begin
    if ($test$plusargs("vcd")) begin
        $dumpfile("tb_axi_bridge.vcd");
        $dumpvars(0, tb_axi_bridge);
    end

    pc_fe = CODE_BASE; pc_if = 32'b0; if_valid = 1'b0; fe_enable = 1'b0;
    data_req = 1'b0; data_we = 4'b0; data_addr = 32'b0; data_wdata = 32'b0;
    crmd_plv = 2'b00;

    repeat (8) @(posedge clk);
    resetn = 1'b1;
    repeat (4) @(posedge clk);

    //-------------------------------------------------- 1. instruction fetch
    fe_enable = 1'b1;
    wait (fe_count >= CODE_WORDS);
    fe_enable = 1'b0;
    repeat (4) @(posedge clk);
    $display("[1] fetch    : %0d instructions, fe_count=%0d", CODE_WORDS, fe_count);

    //-------------------------------------------------- 2. plain load
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
    for (k = 0; k < 200; k = k + 1) begin
        addr = DATA_BASE + (($unsigned($random) % 512) * 4);
        we   = 4'hf;                                    // full words in the mix
        wd   = $random;
        if (($random % 2) == 0) begin
            do_store(addr, we, wd);
            ref_store(addr, we, wd);
        end
        else begin
            do_load(addr, rd);
            chk32(ref[addr >> 2], rd, "random load");
        end
    end
    $display("[8] random   : 200 mixed accesses done");

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
