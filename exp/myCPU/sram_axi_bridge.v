module sram_axi_bridge(
    input  wire        clk,
    input  wire        reset,

    // fetch side: inst_req is a single-cycle pulse
    input  wire        inst_req,
    input  wire [31:0] inst_addr,
    input  wire        inst_line,    // 1: 4-beat burst (16B line), 0: single beat
    input  wire        inst_flush,   // drop a queued request
    output wire [127:0] inst_rdata,  // beat 0 at [31:0], single access -> [31:0]
    output wire        inst_pending,

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

reg mem_write_entry_cycle;

reg [31:0] inst_addr_r, rd_addr_r;
reg [31:0] awaddr_r, wdata_r;
reg [ 3:0] wstrb_r;
reg        inst_arprot0_r, mem_arprot0_r, awprot0_r;

// burst flags, latched with the address
reg inst_line_r, rd_line_r, wr_line_r;

// read beat collection, one set per read channel
reg  [ 1:0] icnt, mcnt;
reg  [127:0] ibuf, mbuf;

// write beat issue
reg  [ 1:0] w_cnt;
reg         aw_done, w_done;

wire        mem_read_req;
wire        mem_write_req;

wire        read_req_fire;
wire        read_rsp_fire;
wire        read_req_occupy;

wire        inst_r_fire;
wire        mem_r_fire;

wire        inst_read_rsp_done;
wire        mem_read_rsp_done;

wire        aw_fire, w_fire;
wire        write_req_fire;
wire        write_rsp_fire;
wire        write_rsp_done;
wire        write_slot_free;

wire        mem_read_takes_req;
wire        mem_read_take;
wire        mem_write_take;

// AXI constant attributes
assign arsize  = 3'd2;  // 4 bytes
assign arburst = 2'b01; // INCR
assign arlock  = 2'b0;
assign arcache = 4'b0011;
assign arlen   = (inst_state[req] ? inst_line_r : rd_line_r) ? 8'd3 : 8'd0;

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

always @(posedge clk) begin
    if (reset) begin
        awaddr_r  <= 32'b0;
        wdata_r   <= 32'b0;
        wstrb_r   <= 4'b0;
        awprot0_r <= 1'b0;
        wr_line_r <= 1'b0;
    end
    else if (mem_write_take) begin
        awaddr_r  <= data_addr;
        wdata_r   <= data_wdata[31:0];
        wstrb_r   <= data_we;
        awprot0_r <= (crmd_plv == 2'b00);
        wr_line_r <= data_line;
    end
end

// read beats: one counter + buffer per channel, gated by the response id.
// the last beat is merged combinationally so the data is complete on the cycle
// the response ends (the mmu samples it when pending drops).
assign inst_r_fire = inst_state[rsp]     & rvalid & rready & (rid == inst_rid);
assign mem_r_fire  = mem_read_state[rsp] & rvalid & rready & (rid == mem_rid);

always @(posedge clk) begin
    if (reset | inst_req)     icnt <= 2'b0;
    else if (inst_r_fire)     icnt <= icnt + 2'd1;
end

always @(posedge clk) begin
    if (reset | mem_read_take) mcnt <= 2'b0;
    else if (mem_r_fire)       mcnt <= mcnt + 2'd1;
end

always @(posedge clk) begin
    if (inst_r_fire) ibuf[icnt*32 +: 32] <= rdata;
end

always @(posedge clk) begin
    if (mem_r_fire) mbuf[mcnt*32 +: 32] <= rdata;
end

wire [127:0] ilane = 128'hffff_ffff << {icnt, 5'b0};
wire [127:0] mlane = 128'hffff_ffff << {mcnt, 5'b0};

assign inst_rdata = inst_state[rsp]     ? ((ibuf & ~ ilane) | ({96'b0, rdata} << {icnt, 5'b0}))
                                        : ibuf;
assign data_rdata = mem_read_state[rsp] ? ((mbuf & ~ mlane) | ({96'b0, rdata} << {mcnt, 5'b0}))
                                        : mbuf;

// write beats: AW is one beat, W is one or four
assign awvalid = mem_write_state[req] & ~ aw_done;
assign wvalid  = mem_write_state[req] & ~ w_done;
assign wlast   = wvalid & (wr_line_r ? (w_cnt == 2'd3) : 1'b1);

assign wdata   = wr_line_r ? data_wdata[32*w_cnt +: 32] : wdata_r;
assign wstrb   = wr_line_r ? 4'b1111 : wstrb_r;

assign aw_fire = awvalid & awready;
assign w_fire  = wvalid  & wready;

always @(posedge clk) begin
    if (reset | mem_write_take) begin
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
assign mem_write_take = mem_write_req & write_slot_free;

assign inst_read_rsp_done = inst_r_fire     & rlast;
assign mem_read_rsp_done  = mem_r_fire      & rlast;

assign read_req_fire  = arvalid & arready;
assign read_rsp_fire  = rvalid & rready;

assign write_req_fire = aw_done & w_done;
assign write_rsp_fire = bvalid & bready;
assign write_rsp_done = mem_write_state[rsp] & write_rsp_fire & (bid == mem_wid);
assign write_slot_free = mem_write_state[ret] | write_rsp_done;

assign arid    = inst_state[req] ? inst_rid : mem_rid;
assign araddr  = inst_state[req] ? inst_addr_r : rd_addr_r;
assign arvalid = inst_state[req] | mem_read_state[req];
assign rready  = inst_state[rsp] | mem_read_state[rsp];

assign awaddr  = awaddr_r;
assign bready  = mem_write_state[rsp];

assign inst_pending = ~ inst_state[ret] & ~ inst_read_rsp_done;
assign mem_pending  = (~ mem_read_state[ret] & ~ mem_read_rsp_done)
                    | (~ mem_write_entry_cycle & ~ mem_write_state[ret]);

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
            if (read_req_fire) next_inst_state = RSP;
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
assign mem_read_takes_req = ~ read_req_occupy & mem_read_req & mem_write_state[ret]
                                                 & (mem_read_state[que] | mem_read_state[ret]);

always @(*) begin
    case (mem_read_state)
        QUE: begin
            if (~ read_req_occupy & mem_write_state[ret]) next_mem_read_state = REQ;
            else next_mem_read_state = QUE;
        end
        REQ: begin
            if (read_req_fire) next_mem_read_state = RSP;
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
            else next_mem_write_state = REQ;
        end
        RSP: begin
            if (write_rsp_done) begin
                if (mem_write_req) next_mem_write_state = REQ;
                else next_mem_write_state = RET;
            end
            else next_mem_write_state = RSP;
        end
        RET: begin
            if (mem_write_req) next_mem_write_state = REQ;
            else next_mem_write_state = RET;
        end
        default: next_mem_write_state = RET;
    endcase
end

always @(posedge clk) begin
    if (mem_write_take) mem_write_entry_cycle <= 1'b1;
    else mem_write_entry_cycle <= 1'b0;
end

endmodule
