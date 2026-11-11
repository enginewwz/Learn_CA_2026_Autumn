// ===========================================================================
// CPU-side <-> AXI adapter.
//
// TWO CLOCKS:
//   clk     : processor clock (1x).  Everything on the CPU side lives here.
//   clk2    : memory clock (2x clk, same PLL, edges aligned).  The AXI side
//             (ar/r/aw/w/b) belongs here so a 16-beat line refill can be
//             drained at 2 beats per processor cycle.
//
// The gluing of the two domains -- which signals cross, how the CPU-side
// single-cycle pulses are turned into 2x-side events and how the collected
// beats/pending come back -- is *not* written yet: for now every register and
// every output still runs on clk, exactly as before, and clk2/reset2 are
// brought in but unused.  That keeps the whole SoC regression green while the
// 2x clock is already routed here (see docs/fetch_opt_step1_clock.md).
// ===========================================================================
module sram_axi_bridge(
    input  wire        clk,          // 1x: processor clock
    input  wire        clk2,         // 2x: memory / AXI clock
    input  wire        reset,        // 1x domain reset
    input  wire        reset2,       // 2x domain reset (synchronised to clk2)

    // fetch side: inst_req is a single-cycle pulse
    input  wire        inst_req,
    input  wire [31:0] inst_addr,
    input  wire        inst_line,    // 1: 16-beat burst (64B line), 0: single beat
    input  wire        inst_flush,   // drop a queued request
    output wire [511:0] inst_rdata,  // beat 0 at [31:0], single access -> [31:0]
    output wire        inst_pending,
    output wire [ 4:0] inst_beat,    // beats already collected in inst_rdata (the
                                     // mmu hands them out while the burst runs)

    input  wire [ 1:0] crmd_plv,

    // data side: data_req is a single-cycle pulse
    input  wire        data_req,
    input  wire [ 3:0] data_we,      // byte strobes; 4'b0 = read
    input  wire [31:0] data_addr,
    input  wire [127:0] data_wdata,  // beat 0 at [31:0]
    input  wire        data_line,    // 1: 4-beat burst, 0: single beat
    output wire [127:0] data_rdata,
    output wire        mem_pending,

    // AXI3
    output wire [ 3:0] arid,
    output wire [31:0] araddr,
    output wire [ 7:0] arlen,
    output wire [ 2:0] arsize,
    output wire [ 1:0] arburst,
    output wire [ 1:0] arlock,
    output wire [ 3:0] arcache,
    output wire [ 2:0] arprot,
    output wire        arvalid,
    input  wire        arready,

    input  wire [ 3:0] rid,
    input  wire [31:0] rdata,
    input  wire [ 1:0] rresp,
    input  wire        rlast,
    input  wire        rvalid,
    output wire        rready,

    output wire [ 3:0] awid,
    output wire [31:0] awaddr,
    output wire [ 7:0] awlen,
    output wire [ 2:0] awsize,
    output wire [ 1:0] awburst,
    output wire [ 1:0] awlock,
    output wire [ 3:0] awcache,
    output wire [ 2:0] awprot,
    output wire        awvalid,
    input  wire        awready,

    output wire [ 3:0] wid,
    output wire [31:0] wdata,
    output wire [ 3:0] wstrb,
    output wire        wlast,
    output wire        wvalid,
    input  wire        wready,

    input  wire [ 3:0] bid,
    input  wire [ 1:0] bresp,
    input  wire        bvalid,
    output wire        bready
);

localparam QUE  = 4'b0001;
localparam REQ  = 4'b0010;
localparam RSP  = 4'b0100;
localparam RET  = 4'b1000;

localparam que  = 0;
localparam req  = 1;
localparam rsp  = 2;
localparam ret  = 3;

localparam inst_rid = 4'b0;
localparam mem_rid  = 4'b1;
localparam mem_wid  = 4'b0;

reg  [3:0] inst_state,      next_inst_state;
reg  [3:0] mem_read_state,  next_mem_read_state;
reg  [3:0] mem_write_state, next_mem_write_state;

reg [31:0] inst_addr_r, rd_addr_r;
reg [31:0] awaddr_r, wdata_r;
reg [127:0] wdata128_r;   // whole line of a burst write, latched with the AW
reg [ 3:0] wstrb_r;
reg        inst_arprot0_r, mem_arprot0_r, awprot0_r;

// burst flags, latched with the address
reg inst_line_r, rd_line_r, wr_line_r;

// read beat collection, one set per read channel.  The icache line is 64B (16
// beats), the dcache line stays 16B (4 beats).
reg  [ 3:0] icnt;
reg  [ 1:0] mcnt;
reg  [511:0] ibuf;
reg  [127:0] mbuf;
// beat counter for the sequential fetch: 5 bits so it reads 16 (not 0) after the
// last beat of a full line; the mmu compares the word offset against its low 4
// bits, so the wrap at 16 is what keeps word 15 out of the streaming window
reg  [ 4:0] inst_beat_r;

// write beat issue
reg mem_write_entry_cycle;

reg  [ 1:0] w_cnt;
reg         aw_done, w_done;

reg [31:0]  awaddr_c;
reg [31:0]  wdata_c;
reg [127:0] wdata128_c;
reg [3:0]   wstrb_c;
reg         awprot0_c, wr_line_c;

reg         waw_req;

wire        mem_read_req;
wire        mem_write_req;

wire        read_rsp_fire;
wire        read_req_occupy;

wire        inst_read_rsp_fire;
wire        mem_read_rsp_fire;

wire        inst_read_rsp_done;
wire        mem_read_rsp_done;

wire        aw_fire, w_fire, w_last_fire;
wire        write_req_fire;
wire        write_rsp_fire;
wire        write_rsp_done;
wire        write_slot_free;

wire        mem_read_takes_req;
wire        mem_read_take;
wire        mem_write_take;

// use for 2x clock
wire        read_req_fire_w;
wire        inst_read_req_w;       // AR handshake belongs to the fetch channel
wire        mem_read_req_w;        // ... or to the data channel    
wire        inst_read_rsp_done_w;
wire        mem_read_rsp_done_w;
wire        write_req_fire_w;
wire        write_rsp_done_w;

reg         inst_read_req_r;       // sticky, per channel: cleared when *its* fsm leaves REQ
reg         mem_read_req_r;    
reg         inst_read_rsp_done_r;
reg         mem_read_rsp_done_r;
reg         write_req_fire_r;
reg         write_rsp_done_r;

// AXI constant attributes
assign arsize  = 3'd2;  // 4 bytes
assign arburst = 2'b01; // INCR
assign arlock  = 2'b0;
assign arcache = 4'b0011;
assign arlen   = inst_state[req] ? (inst_line_r ? 8'd15 : 8'd0)
                                 : (rd_line_r   ? 8'd3  : 8'd0);

assign awsize  = 3'd2;
assign awburst = 2'b01;
assign awlock  = 2'b0;
assign awcache = 4'b0011;
assign awlen   = wr_line_r ? 8'd3 : 8'd0;

assign arprot[0] = inst_state[req] ? inst_arprot0_r : mem_arprot0_r;
assign arprot[1] = 1'b0;
assign arprot[2] = inst_state[req];
assign awprot[0] = awprot0_r;
assign awprot[1] = 1'b0;
assign awprot[2] = 1'b0;

assign awid = mem_wid;
assign wid  = mem_wid;

// request latches
always @(posedge clk) begin
    if (reset) begin
        inst_addr_r    <= 32'b0;
        inst_arprot0_r <= 1'b0;
        inst_line_r    <= 1'b0;
    end
    else if (inst_req) begin
        inst_addr_r    <= inst_addr;
        inst_arprot0_r <= (crmd_plv == 2'b00);
        inst_line_r    <= inst_line;
    end
end

always @(posedge clk) begin
    if (reset) begin
        rd_addr_r     <= 32'b0;
        mem_arprot0_r <= 1'b0;
        rd_line_r     <= 1'b0;
    end
    else if (mem_read_take) begin
        rd_addr_r     <= data_addr;
        mem_arprot0_r <= (crmd_plv == 2'b00);
        rd_line_r     <= data_line;
    end
end

wire w_capture = mem_write_req & (~ mem_write_take | waw_req);

always @(posedge clk) begin
    if (reset)               waw_req <= 1'b0;
    else if (w_capture)      waw_req <= 1'b1;   // the capture holds a request
    else if (mem_write_take) waw_req <= 1'b0;   // held request issued
end

always @(posedge clk) begin
    if (reset) begin
        awaddr_c  <= 32'b0;
        wdata_c   <= 32'b0;
        wdata128_c<= 128'b0;
        wstrb_c   <= 4'b0;
        awprot0_c <= 1'b0;
        wr_line_c <= 1'b0;
    end
    else if (w_capture) begin
        awaddr_c  <= data_addr;
        wdata_c   <= data_wdata[31:0];
        wdata128_c<= data_wdata;
        wstrb_c   <= data_we;
        awprot0_c <= (crmd_plv == 2'b00);
        wr_line_c <= data_line;
    end
end

always @(posedge clk) begin
    if (reset) begin
        awaddr_r  <= 32'b0;
        wdata_r   <= 32'b0;
        wdata128_r<= 128'b0;
        wstrb_r   <= 4'b0;
        awprot0_r <= 1'b0;
        wr_line_r <= 1'b0;
    end
    else if (mem_write_take) begin
        awaddr_r  <= waw_req ? awaddr_c : data_addr;
        wdata_r   <= waw_req ? wdata_c : data_wdata[31:0];
        wdata128_r<= waw_req ? wdata128_c : data_wdata;
        wstrb_r   <= waw_req ? wstrb_c : data_we;
        awprot0_r <= waw_req ? awprot0_c : (crmd_plv == 2'b00);
        wr_line_r <= waw_req ? wr_line_c : data_line;
    end
end

always @(posedge clk2) begin
    if (reset2) begin
        inst_read_req_r      <= 1'b0;
        mem_read_req_r       <= 1'b0;
        inst_read_rsp_done_r <= 1'b0;
        mem_read_rsp_done_r  <= 1'b0;
        write_req_fire_r     <= 1'b0;
        write_rsp_done_r     <= 1'b0;
    end
    else begin
        if (inst_read_req_w)                                inst_read_req_r      <= 1'b1;
        else if (~ inst_state[req])                         inst_read_req_r      <= 1'b0;

        if (mem_read_req_w)                                 mem_read_req_r       <= 1'b1;
        else if (~ mem_read_state[req])                     mem_read_req_r       <= 1'b0;

        if (inst_read_rsp_done_w)                           inst_read_rsp_done_r <= 1'b1;
        else if (~ inst_state[rsp])                         inst_read_rsp_done_r <= 1'b0;

        if (mem_read_rsp_done_w)                            mem_read_rsp_done_r  <= 1'b1;
        else if (~ mem_read_state[rsp])                     mem_read_rsp_done_r  <= 1'b0;

        if (write_req_fire_w)                               write_req_fire_r     <= 1'b1;
        else if (~ mem_write_state[req])                    write_req_fire_r     <= 1'b0;

        if (write_rsp_done_w)                               write_rsp_done_r     <= 1'b1;
        else if (~ mem_write_state[rsp])                    write_rsp_done_r     <= 1'b0;
    end
end

assign inst_read_rsp_done = inst_read_rsp_done_r | inst_read_rsp_done_w;
assign mem_read_rsp_done  = mem_read_rsp_done_r  | mem_read_rsp_done_w;
assign write_req_fire     = write_req_fire_r     | write_req_fire_w;
assign write_rsp_done     = write_rsp_done_r     | write_rsp_done_w;

// read beats: one counter + buffer per channel, gated by the response id.
// the last beat is merged combinationally so the data is complete on the cycle
// the response ends (the mmu samples it when pending drops).
assign inst_read_rsp_fire   = inst_state[rsp]     & read_rsp_fire & (rid == inst_rid);
assign mem_read_rsp_fire    = mem_read_state[rsp] & read_rsp_fire & (rid == mem_rid);

always @(posedge clk2) begin
    if (reset2 | inst_req)      icnt <= 4'b0;
    else if (inst_read_rsp_fire)icnt <= icnt + 4'd1;
end

always @(posedge clk2) begin
    if (reset2 | inst_req)            inst_beat_r <= 5'd0;
    else if (inst_read_rsp_fire & ~&inst_beat_r) inst_beat_r <= inst_beat_r + 5'd1;
end
assign inst_beat = inst_beat_r;

always @(posedge clk2) begin
    if (reset2 | mem_read_take) mcnt <= 2'b0;
    else if (mem_read_rsp_fire) mcnt <= mcnt + 2'd1;
end

always @(posedge clk2) begin
    if (inst_read_rsp_fire) ibuf[icnt*32 +: 32] <= rdata;
end

always @(posedge clk2) begin
    if (mem_read_rsp_fire) mbuf[mcnt*32 +: 32] <= rdata;
end

// A line refill is always a 4-beat burst (arlen=3), so on its last beat the
// line is the three beats already collected in the buffer plus the beat on the
// bus; an uncached access is a single beat at position 0.  Building the line
// this way keeps the 128-bit variable shifter (a 4:1 mux on every bit) out of
// the AXI rdata -> cache data RAM / load-data path.
assign inst_rdata = (inst_state[rsp]     & ~ inst_read_rsp_done_r)
                  ? (inst_line_r ? {rdata, ibuf[479:0]} : {480'b0, rdata})
                  : ibuf;
assign data_rdata = (mem_read_state[rsp] & ~ mem_read_rsp_done_r)
                  ? (rd_line_r ? {rdata, mbuf[95:0]} : {96'b0, rdata})
                  : mbuf;

// write beats: AW is one beat, W is one or four
assign awvalid = mem_write_state[req] & ~ aw_done;
assign wvalid  = mem_write_state[req] & ~ w_done;
assign wlast   = wvalid & (wr_line_r ? (w_cnt == 2'd3) : 1'b1);

assign wdata   = wr_line_r ? wdata128_r[32*w_cnt +: 32] : wdata_r;
assign wstrb   = wr_line_r ? 4'b1111 : wstrb_r;

assign aw_fire = awvalid & awready;
assign w_fire  = wvalid  & wready;
assign w_last_fire = w_fire & (wr_line_r ? (w_cnt == 2'd3) : 1'b1);

always @(posedge clk2) begin
    if (reset2 | mem_write_take) begin
        w_cnt   <= 2'b0;
        aw_done <= 1'b0;
        w_done  <= 1'b0;
    end
    else begin
        if (aw_fire) aw_done <= 1'b1;
        if (w_fire) begin
            w_cnt <= w_cnt + 2'd1;
            if (w_cnt == (wr_line_r ? 2'd3 : 2'd0)) w_done <= 1'b1;
        end
    end
end

// channel arbitration / completion
assign read_req_occupy = mem_read_state[req] | inst_state[req];

assign mem_read_req  = data_req & ~| data_we;
assign mem_write_req = data_req &  | data_we;

assign mem_read_take  = mem_read_req  & mem_read_state[ret];
assign mem_write_take = (mem_write_req | waw_req) & write_slot_free;

assign inst_read_rsp_done_w = inst_read_rsp_fire  & rlast;
assign mem_read_rsp_done_w  = mem_read_rsp_fire   & rlast;

assign read_req_fire_w  = arvalid & arready;
assign inst_read_req_w  = read_req_fire_w & inst_state[req];
assign mem_read_req_w   = read_req_fire_w & mem_read_state[req];
assign read_rsp_fire    = rvalid & rready;

assign write_req_fire_w = (aw_done | aw_fire) & (w_done | w_last_fire);
assign write_rsp_fire = bvalid & bready;
assign write_rsp_done_w = mem_write_state[rsp] & write_rsp_fire & (bid == mem_wid);

assign write_slot_free = mem_write_state[ret];

assign arid    = inst_state[req] ? inst_rid : mem_rid;
assign araddr  = inst_state[req] ? inst_addr_r : rd_addr_r;
assign arvalid = (inst_state[req] & ~ inst_read_req_r) | (mem_read_state[req] & ~ mem_read_req_r);
assign rready  = (inst_state[rsp] & ~ inst_read_rsp_done_r) | (mem_read_state[rsp] & ~ mem_read_rsp_done_r);

assign awaddr  = awaddr_r;
assign bready  = mem_write_state[rsp] & ~ write_rsp_done_r;

assign inst_pending = ~ inst_state[ret] & ~ inst_read_rsp_done;
assign mem_pending  = (~ mem_read_state[ret] & ~ mem_read_rsp_done)
                    | (~ mem_write_entry_cycle & ~ mem_write_state[ret]);

always @(posedge clk) begin
    if (mem_write_take) mem_write_entry_cycle <= 1'b1;
    else mem_write_entry_cycle <= 1'b0;
end

// state machines
always @(posedge clk) begin
    if (reset) begin
        inst_state      <= RET;
        mem_read_state  <= RET;
        mem_write_state <= RET;
    end
    else begin
        inst_state      <= next_inst_state;
        mem_read_state  <= next_mem_read_state;
        mem_write_state <= next_mem_write_state;
    end
end

always @(*) begin
    case (inst_state)
        QUE: begin
            if (inst_flush) next_inst_state = RET;
            else if (~ read_req_occupy & ~ mem_read_takes_req) next_inst_state = REQ;
            else next_inst_state = QUE;
        end
        REQ: begin
            if (read_req_fire_w | inst_read_req_r) next_inst_state = RSP;
            else next_inst_state = REQ;
        end
        RSP: begin
            if (inst_read_rsp_done) begin
                if (inst_req) begin
                    if (~ read_req_occupy & ~ mem_read_takes_req) next_inst_state = REQ;
                    else next_inst_state = QUE;
                end
                else next_inst_state = RET;
            end
            else next_inst_state = RSP;
        end
        RET: begin
            if (inst_req) begin
                if (~ read_req_occupy & ~ mem_read_takes_req) next_inst_state = REQ;
                else next_inst_state = QUE;
            end
            else next_inst_state = RET;
        end
        default: next_inst_state = RET;
    endcase
end

// the fetch must not enter REQ in the same cycle
assign mem_read_takes_req = ~ read_req_occupy & mem_write_state[ret]
                             & (mem_read_state[que] | mem_read_state[ret] & mem_read_req);

always @(*) begin
    case (mem_read_state)
        QUE: begin
            if (~ read_req_occupy & mem_write_state[ret]) next_mem_read_state = REQ;
            else next_mem_read_state = QUE;
        end
        REQ: begin
            if (read_req_fire_w | mem_read_req_r) next_mem_read_state = RSP;
            else next_mem_read_state = REQ;
        end
        RSP: begin
            if (mem_read_rsp_done) next_mem_read_state = RET;
            else next_mem_read_state = RSP;
        end
        RET: begin
            if (mem_read_req) begin
                if (~ read_req_occupy & mem_write_state[ret]) next_mem_read_state = REQ;
                else next_mem_read_state = QUE;
            end
            else next_mem_read_state = RET;
        end
        default: next_mem_read_state = RET;
    endcase
end

always @(*) begin
    case (mem_write_state)
        REQ: begin
            if (write_req_fire) next_mem_write_state = RSP;
            else                next_mem_write_state = REQ;
        end
        RSP: begin
            if (write_rsp_done) next_mem_write_state = RET;
            else                next_mem_write_state = RSP;
        end
        RET: begin
            if (mem_write_req | waw_req) next_mem_write_state = REQ;
            else                         next_mem_write_state = RET;
        end
        default: next_mem_write_state = RET;
    endcase
end

endmodule
