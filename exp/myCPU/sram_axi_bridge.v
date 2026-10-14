module sram_axi_bridge(
    input  wire        clk,
    input  wire        reset,

    // fetch side: the request appears when the instruction enters IF
    input  wire        inst_req,     // a valid pc entering IF
    input  wire [31:0] inst_addr,    // address of the instruction entering IF
    input  wire        allow_if,     // allow[IF]
    input  wire        flush_if,     // the IF stage is being flushed this cycle
    output wire [31:0] inst_rdata,
    output wire        inst_pending, // the fetch should wait

    input  wire [ 1:0] crmd_plv,

    // register & access at MEM
    input  wire        data_req,     // = valid[MEM] & mem_en[MEM] & ~mem_excp
    input  wire [ 3:0] data_we,      // byte strobes; 4'b0 means read
    input  wire [31:0] data_addr,    // = result[MEM]
    input  wire [31:0] data_wdata,
    output wire [31:0] data_rdata,
    output wire        mem_pending,  // this access should wait

    // AXI
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
localparam ret = 3;

localparam inst_rid  = 4'b0;
localparam mem_rid = 4'b1;
localparam mem_wid = 4'b0;

reg  [3:0] inst_state,      next_inst_state;
reg  [3:0] mem_read_state,  next_mem_read_state;
reg  [3:0] mem_write_state, next_mem_write_state;

reg         write_addr_req_fire_reg;
reg         write_data_req_fire_reg;

reg  [31:0] inst_addr_r;
reg  [31:0] inst_rdata_r;
reg  [31:0] rd_addr_r;                 // AR address of a data read
reg  [31:0] awaddr_r, wdata_r;         // AW / W payload of a store
reg  [ 3:0] wstrb_r;
reg         inst_arprot0_r;
reg         mem_arprot0_r;
reg         awprot0_r;

wire        mem_read_req;
wire        mem_write_req;

wire        read_req_fire;
wire        read_rsp_fire;
wire        read_req_occupy;
wire        read_rsp_occupy;

wire        inst_read_rsp_done;
wire        mem_read_rsp_done;

wire        write_req_fire;
wire        write_rsp_fire;
wire        write_rsp_done;
wire        write_slot_free;

wire        write_addr_req_fire;
wire        write_data_req_fire;
wire        write_addr_req_fire_wire;
wire        write_data_req_fire_wire;

wire        mem_read_takes_req;

wire        mem_read_take;
wire        mem_write_take;
wire        mem_take;

assign arlen   = 8'b0;
assign arsize  = 3'd2;  // 4 bytes
assign arburst = 2'b01; // INCR
assign arlock  = 2'b0;
assign arcache = 4'b0011;   // Normal Non-cacheable Bufferable
assign arprot[0] = inst_state[req] ? inst_arprot0_r : mem_arprot0_r;
assign arprot[1] = 1'b0;

assign awlen   = 8'b0;
assign awsize  = 3'd2;  // 4 bytes
assign awburst = 2'b01; // INCR
assign awlock  = 2'b0;
assign awcache = 4'b0011;   // Normal Non-cacheable Bufferable
assign awprot[2] = 1'b0;
assign awprot[1] = 1'b0;

assign awid = mem_wid;
assign wid = mem_wid;

assign wlast = wvalid;

always @(posedge clk) begin
    if (reset) begin
        inst_addr_r    <= 32'b0;
        inst_arprot0_r <= 1'b0;
    end
    else if (inst_req & allow_if) begin
        inst_addr_r    <= inst_addr;
        inst_arprot0_r <= (crmd_plv == 2'b00);
    end
end

always @(posedge clk) begin
    if (inst_read_rsp_done) inst_rdata_r <= rdata;
end

always @(posedge clk) begin
    if (reset) begin
        rd_addr_r   <= 32'b0;
        mem_arprot0_r <= 1'b0;
    end
    else if (mem_read_take) begin
        rd_addr_r   <= data_addr;
        mem_arprot0_r <= (crmd_plv == 2'b00);
    end
end

always @(posedge clk) begin
    if (reset) begin
        awaddr_r  <= 32'b0;
        wdata_r   <= 32'b0;
        wstrb_r   <= 4'b0;
        awprot0_r <= 1'b0;
    end
    else if (mem_write_take) begin
        awaddr_r  <= data_addr;
        wdata_r   <= data_wdata;
        wstrb_r   <= data_we;
        awprot0_r <= (crmd_plv == 2'b00);
    end
end

assign inst_rdata = inst_state[rsp]     ? rdata : inst_rdata_r;
assign data_rdata = rdata;

assign arid   = inst_state[req] ? inst_rid : mem_rid;
assign araddr = inst_state[req] ? inst_addr_r : rd_addr_r;

assign mem_read_take  = mem_read_req  & mem_read_state[ret];
assign mem_write_take = mem_write_req & write_slot_free;

assign awaddr    = awaddr_r;
assign wdata     = wdata_r;
assign wstrb     = wstrb_r;
assign awprot[0] = awprot0_r;

assign arprot[2] = inst_state[req];
assign arvalid = inst_state[req] | mem_read_state[req];
assign rready = inst_state[rsp] | mem_read_state[rsp];

assign awvalid = mem_write_state[req] & ~ write_addr_req_fire_reg;
assign wvalid = mem_write_state[req] & ~ write_data_req_fire_reg;
assign bready = mem_write_state[rsp];

always @(posedge clk) begin
    if (reset) begin
        inst_state <= RET;
        mem_read_state <= RET;
        mem_write_state <= RET;
    end
    else begin
        inst_state <= next_inst_state;
        mem_read_state <= next_mem_read_state;
        mem_write_state <= next_mem_write_state;
    end
end

assign read_req_occupy = mem_read_state[req] | inst_state[req];

// the fetch must not enter REQ in the same cycle
assign mem_read_takes_req = ~ read_req_occupy & mem_read_req & mem_write_state[ret] & (mem_read_state[que] | mem_read_state[ret]);
assign read_rsp_occupy = mem_read_state[rsp] | inst_state[rsp];
assign mem_read_req = data_req & ~| data_we;
assign mem_write_req = data_req &  | data_we;

assign read_req_fire = arvalid & arready;
assign read_rsp_fire = rvalid & rready;

assign inst_read_rsp_done = inst_state[rsp]     & read_rsp_fire & (rid == inst_rid);
assign mem_read_rsp_done  = mem_read_state[rsp] & read_rsp_fire & (rid == mem_rid);

assign write_req_fire = write_addr_req_fire & write_data_req_fire;
assign write_rsp_fire = bvalid & bready;

assign write_rsp_done = mem_write_state[rsp] & write_rsp_fire & (bid == mem_wid);
assign write_slot_free = mem_write_state[ret] | write_rsp_done;

always @(*) begin
    case (inst_state)
        QUE: begin
            if (flush_if) next_inst_state = RET;      // pc redirected: drop the queued fetch
            else if (~ read_req_occupy & ~ mem_read_takes_req) next_inst_state = REQ;
            else next_inst_state = QUE;
        end
        REQ: begin
            if (read_req_fire) next_inst_state = RSP;
            else next_inst_state = REQ;
        end
        RSP: begin
            if (inst_read_rsp_done) begin
                if (inst_req & allow_if) begin
                    if (~ read_req_occupy & ~ mem_read_takes_req) next_inst_state = REQ;
                    else next_inst_state = QUE;
                end
                else next_inst_state = RET;
            end
            else next_inst_state = RSP;
        end
        RET: begin
            if (inst_req & allow_if) begin
                if (~ read_req_occupy & ~ mem_read_takes_req) next_inst_state = REQ;
                else next_inst_state = QUE;
            end
            else next_inst_state = RET;
        end
        default: next_inst_state = RET;
    endcase
end

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
                if (~ read_req_occupy & mem_write_state[ret]) next_mem_read_state = REQ;  // avoid RAW hazard
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

assign write_addr_req_fire = write_addr_req_fire_reg | write_addr_req_fire_wire;
assign write_data_req_fire = write_data_req_fire_reg | write_data_req_fire_wire;

assign write_addr_req_fire_wire = awvalid & awready;
assign write_data_req_fire_wire = wvalid & wready;

always @(posedge clk) begin
    if (reset) begin
        write_addr_req_fire_reg <= 1'b0;
        write_data_req_fire_reg <= 1'b0;
    end
    else begin
        if (mem_write_state[req]) begin
            write_addr_req_fire_reg <= write_addr_req_fire;
            write_data_req_fire_reg <= write_data_req_fire;
        end
        else begin
            write_addr_req_fire_reg <= 1'b0;
            write_data_req_fire_reg <= 1'b0;
        end
    end
end

assign inst_pending = inst_state[que] | inst_state[req] | (inst_state[rsp] & ~ inst_read_rsp_done);
assign mem_pending = (mem_read_req  & ~ mem_read_rsp_done) | (mem_write_req & ~ write_slot_free);

endmodule
