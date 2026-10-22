module mmu(
    input  wire        clk,
    input  wire        reset,

    // CPU side: fetch
    input  wire        if_req,          // = inst_req: a legal fetch address is on if_va
    input  wire [31:0] if_va,           // = pc[IF]: the address taken at the last edge
    input  wire        if_allow,        // = allow[IF]: pc[IF] <- nextpc at this edge
    input  wire        if_flush,        // = flush[IF]: this fetch is killed
    output wire        if_pending,      // -> stall[IF]
    output wire [31:0] if_rdata,

    // CPU side: load / store
    input  wire        mem_pre,         // a load / store in EX that enters MEM at this edge
    input  wire        mem_req,         // the MEM slot holds a real access
    input  wire [31:0] mem_va,          // = result[MEM]
    input  wire [ 3:0] mem_wstrb,       // byte strobes; 4'b0 = read
    input  wire [31:0] mem_stdata,      // store data, already spread over the byte lanes
    input  wire        mem_abort,       // this access should stop at stage tlb access
    input  wire        mem_allow,       // = allow[MEM]
    output wire        mem_pending,     // -> stall[MEM]
    output wire [31:0] mem_rdata,

    // AXI3, straight through the internal sram_axi_bridge
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
    output wire        bready,

    // translation results from the tlb (it lives in the core, next to the csr)
    input  wire        if_xlat,         // the tlb translated if_va
    input  wire [19:0] if_ppn,
    input  wire [ 5:0] if_ps,
    input  wire        if_dmw_hit,      // a dmw window covers if_va
    input  wire [31:0] if_dmw_pa,
    input  wire        if_abort,        // this fetch has an exception: do not issue
    input  wire        mem_xlat,
    input  wire [19:0] mem_ppn,
    input  wire [ 5:0] mem_ps,
    input  wire        mem_dmw_hit,
    input  wire [31:0] mem_dmw_pa,

    input  wire [ 1:0] crmd_plv        // -> bridge arprot[0]/awprot[0]
);

localparam IDLE    = 3'b001;
localparam TLB_ACC = 3'b010;
localparam BRI_ACC = 3'b100;

localparam idle = 0;
localparam tlb_acc = 1;
localparam bri_acc = 2;

reg [2:0] if_state,      mem_state;
reg [2:0] next_if_state, next_mem_state;

wire [31:0] if_pa, mem_pa;
wire [31:0] if_addr, mem_addr;
wire        if_live, if_done, if_take;
wire        mem_live, mem_take, mem_done;
wire        bridge_if_req, bridge_if_flush;
wire [31:0] bridge_if_addr;
wire        bridge_if_pending;
wire        bridge_data_req;
wire [31:0] bridge_data_addr, bridge_data_wdata;
wire [ 3:0] bridge_data_we;
wire        bridge_mem_pending;

// TLB look-up
assign if_pa  = (if_ps  == 6'd21) ? {if_ppn[19:9],  if_va[20:0]}
                                  : {if_ppn[19:0],  if_va[11:0]};
assign mem_pa = (mem_ps == 6'd21) ? {mem_ppn[19:9], mem_va[20:0]}
                                  : {mem_ppn[19:0], mem_va[11:0]};

// dmw window > tlb entry > the va as it is
assign if_addr  = if_dmw_hit  ? if_dmw_pa  : if_xlat  ? if_pa  : if_va;
assign mem_addr = mem_dmw_hit ? mem_dmw_pa : mem_xlat ? mem_pa : mem_va;

// bridge side commands
assign bridge_if_addr   = if_addr;
assign bridge_data_addr = mem_addr;

// fetch path
assign if_live = ~ if_state[idle];
assign if_done = if_state[bri_acc] & ~ bridge_if_pending;
assign if_take = if_req & if_allow;

assign if_pending = if_live & ~ if_done;

always @(*) begin
    case (if_state)
        IDLE: begin
            if (if_take) next_if_state = TLB_ACC;
            else         next_if_state = IDLE;
        end
        TLB_ACC: begin
            if (if_flush | if_abort) next_if_state = IDLE;
            else                        next_if_state = BRI_ACC;
        end
        BRI_ACC: begin
            if (~ bridge_if_pending) begin
                if (if_take) next_if_state = TLB_ACC;
                else         next_if_state = IDLE;
            end
            else next_if_state = BRI_ACC;
        end
        default: next_if_state = IDLE;
    endcase
end

assign bridge_if_req   = if_state[tlb_acc] & ~ if_abort & ~ if_flush;
assign bridge_if_flush = if_flush;

// mem path
assign mem_live  = ~ mem_state[idle];
assign mem_take  = (mem_pre & mem_allow) | mem_req;
assign mem_done  = mem_state[bri_acc] & ~ bridge_mem_pending;   // write: taken; read: data back

assign mem_pending       = mem_live & ~ mem_done & ~ mem_abort;
assign bridge_data_req   = mem_live & ~ mem_abort;
assign bridge_data_we    = mem_wstrb;
assign bridge_data_wdata = mem_stdata;

always @(*) begin
    case (mem_state)
        IDLE: begin
            if (mem_take) next_mem_state = TLB_ACC;
            else        next_mem_state = IDLE;
        end
        TLB_ACC: begin
            if (mem_abort) next_mem_state = IDLE;
            else if (mem_done) begin
                if (mem_pre & mem_allow) next_mem_state = TLB_ACC;
                else                    next_mem_state = IDLE;
            end
            else next_mem_state = BRI_ACC;
        end
        BRI_ACC: begin
            if (mem_done | (mem_abort & mem_live)) begin
                if (mem_pre & mem_allow) next_mem_state = TLB_ACC;
                else                    next_mem_state = IDLE;
            end
            else next_mem_state = BRI_ACC;
        end
        default: next_mem_state = IDLE;
    endcase
end

always @(posedge clk) begin
    if (reset) begin
        if_state <= IDLE;
        mem_state  <= IDLE;
    end
    else begin
        if_state <= next_if_state;
        mem_state  <= next_mem_state;
    end
end

// bridge
sram_axi_bridge u_bridge(
    .clk          (clk                ),
    .reset        (reset              ),

    .inst_req     (bridge_if_req      ),
    .inst_addr    (bridge_if_addr     ),
    .inst_flush   (bridge_if_flush    ),
    .inst_rdata   (if_rdata           ),
    .inst_pending (bridge_if_pending  ),

    .data_req     (bridge_data_req    ),
    .data_we      (bridge_data_we     ),
    .data_addr    (bridge_data_addr   ),
    .data_wdata   (bridge_data_wdata  ),
    .data_rdata   (mem_rdata          ),
    .mem_pending  (bridge_mem_pending ),

    .crmd_plv     (crmd_plv           ),

    .arid         (arid    ),
    .araddr       (araddr  ),
    .arlen        (arlen   ),
    .arsize       (arsize  ),
    .arburst      (arburst ),
    .arlock       (arlock  ),
    .arcache      (arcache ),
    .arprot       (arprot  ),
    .arvalid      (arvalid ),
    .arready      (arready ),

    .rid          (rid     ),
    .rdata        (rdata   ),
    .rresp        (rresp   ),
    .rlast        (rlast   ),
    .rvalid       (rvalid  ),
    .rready       (rready  ),

    .awid         (awid    ),
    .awaddr       (awaddr  ),
    .awlen        (awlen   ),
    .awsize       (awsize  ),
    .awburst      (awburst ),
    .awlock       (awlock  ),
    .awcache      (awcache ),
    .awprot       (awprot  ),
    .awvalid      (awvalid ),
    .awready      (awready ),

    .wid          (wid     ),
    .wdata        (wdata   ),
    .wstrb        (wstrb   ),
    .wlast        (wlast   ),
    .wvalid       (wvalid  ),
    .wready       (wready  ),

    .bid          (bid     ),
    .bresp        (bresp   ),
    .bvalid       (bvalid  ),
    .bready       (bready  )
);

endmodule
