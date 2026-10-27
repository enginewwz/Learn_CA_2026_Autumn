module mmu(
    input  wire        clk,
    input  wire        reset,

    // CPU side: fetch
    input  wire        if_pre,          // a legal fetch address is on if_va
    input  wire [31:0] if_va,           // = pc[IF]
    input  wire        if_allow,        // = allow[IF]
    output wire        if_pending,      // -> stall[IF]
    output wire [31:0] if_rdata,

    // CPU side: load / store
    input  wire        mem_pre,         // the access enters MEM at this edge
    input  wire [31:0] mem_va,          // = result[MEM]
    input  wire [ 3:0] mem_wstrb,       // 4'b0 = read
    input  wire [31:0] mem_stdata,
    input  wire        mem_abort,       // stop at tlb access, do not issue
    input  wire        mem_allow,       // = allow[MEM]
    output wire        mem_pending,     // -> stall[MEM]
    output wire [31:0] mem_rdata,

    // pre-addresses: low 12 bits to index the caches one cycle early
    input  wire [31:0] if_pre_addr,     // = nextpc   : enters IF  next cycle
    input  wire [31:0] mem_pre_addr,    // = ex_result: enters MEM next cycle

    // cacop
    input  wire        mem_cacop,
    input  wire [ 4:0] mem_cacop_code,

    // AXI3, straight through the bridge
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

    // tlb results
    input  wire        if_xlat,
    input  wire [19:0] if_ppn,
    input  wire [ 5:0] if_ps,
    input  wire        if_dmw_hit,
    input  wire [31:0] if_dmw_pa,
    input  wire        if_abort,        // the fetch has an exception
    input  wire        mem_xlat,
    input  wire [19:0] mem_ppn,
    input  wire [ 5:0] mem_ps,
    input  wire        mem_dmw_hit,
    input  wire [31:0] mem_dmw_pa,

    input  wire [ 1:0] crmd_plv
);

localparam idle             = 0;
localparam tlb_acc          = 1;
localparam cache_lookup     = 2;
localparam mem_writeback    = 3;
localparam mem_read         = 4;
localparam cache_refill     = 5;
localparam stagecnt         = 6;

localparam IDLE             = 6'b1 << idle;
localparam TLB_ACC          = 6'b1 << tlb_acc;          // also reads cache result
localparam CACHE_LOOKUP     = 6'b1 << cache_lookup;
localparam MEM_WRITEBACK    = 6'b1 << mem_writeback;
localparam MEM_READ         = 6'b1 << mem_read;
localparam CACHE_REFILL     = 6'b1 << cache_refill;

// state
reg  [stagecnt - 1:0] if_state,      mem_state;
reg  [stagecnt - 1:0] next_if_state, next_mem_state;

reg         cacop_en   = 1'b0;      // TODO: drive from mem_cacop
reg  [ 3:0] cacop_code = 4'b0;
reg         write_back_twice;

reg  [31:0] if_line_addr_r, mem_line_addr_r;  // 16B line address of the access in flight
reg  [11:0] dc_write_va_r;          // store address / way of the lookup cycle
reg         dc_write_sel_r;
reg         wb_way_r;               // way being written back

// addresses
wire [31:0] if_pa, mem_pa;
wire [31:0] if_addr, mem_addr;

wire [11:0] ic_va;
wire [19:0] ic_pt;
wire [11:0] dc_va;
wire [19:0] dc_pt;

wire        if_take;
wire        should_write_back;
wire        wb_pass2, wb_way_now, wb_sel;
wire [31:0]  wb_pa;
wire [127:0] wb_line;

// bridge
wire        bridge_if_req, bridge_if_flush, bridge_if_pending;
wire [31:0] bridge_if_addr;
wire        bridge_data_req, bridge_mem_pending;
wire [31:0] bridge_data_addr;
wire [127:0] bridge_data_wdata;
wire [ 3:0] bridge_data_we;
wire [127:0] bridge_inst_rdata, bridge_data_rdata;

// icache
wire         ic_read_req;
wire         ic_read_hit;
wire [ 31:0] ic_read_data;
wire         ic_cacop_en;
wire [  1:0] ic_cacop_code;
wire         ic_refill_we;
wire [127:0] ic_refill_data;

// dcache
wire         dc_read_req;
wire         dc_read_hit;
wire [ 31:0] dc_read_data;
wire         dc_lookup;
wire         dc_write_req;
wire [ 11:0] dc_write_va;
wire         dc_write_sel;
wire [ 31:0] dc_write_data;
wire         dc_rw_conflict;
wire         dc_cacop_en;
wire [  1:0] dc_cacop_code;
wire         dc_refill_we;
wire         dc_refill_dirty;
wire [127:0] dc_refill_data;

wire         dc_d_way0, dc_d_way1;
wire [ 31:0] dc_pa_way0, dc_pa_way1;
wire [127:0] dc_data_way0, dc_data_way1;
wire         dc_replace_sel;

// address translation: dmw window > tlb entry > the va as it is
assign if_pa  = (if_ps  == 6'd21) ? {if_ppn[19:9],  if_va[20:0]}
                                  : {if_ppn[19:0],  if_va[11:0]};
assign mem_pa = (mem_ps == 6'd21) ? {mem_ppn[19:9], mem_va[20:0]}
                                  : {mem_ppn[19:0], mem_va[11:0]};

assign if_addr  = if_dmw_hit  ? if_dmw_pa  : if_xlat  ? if_pa  : if_va;
assign mem_addr = mem_dmw_hit ? mem_dmw_pa : mem_xlat ? mem_pa : mem_va;

// latched: line address of the access in flight, and the store of the lookup
// cycle (the write comes one cycle later)
always @(posedge clk) begin
    if (reset) begin
        if_line_addr_r  <= 32'b0;
        mem_line_addr_r <= 32'b0;
        dc_write_va_r   <= 12'b0;
        dc_write_sel_r  <= 1'b0;
    end
    else begin
        if_line_addr_r  <= {if_addr [31:4], 4'b0};
        mem_line_addr_r <= {mem_addr[31:4], 4'b0};
        dc_write_va_r   <= mem_va[11:0];
        dc_write_sel_r  <= dc_replace_sel;
    end
end

// cache port
assign ic_va = if_va  [11:0];       // index + offset, request cycle
assign ic_pt = if_addr[31:12];      // physical tag, response cycle
assign dc_va = mem_va  [11:0];
assign dc_pt = mem_addr[31:12];

assign dc_write_va     = dc_write_va_r;
assign dc_write_sel    = dc_write_sel_r;
assign dc_refill_dirty = | mem_wstrb;      // store miss: the merged line is dirty

// store: byte-merge the new data into the word read back from the hit way
// (mem_stdata is already spread over the byte lanes)
assign dc_write_data = { mem_wstrb[3] ? mem_stdata[31:24] : dc_read_data[31:24],
                         mem_wstrb[2] ? mem_stdata[23:16] : dc_read_data[23:16],
                         mem_wstrb[1] ? mem_stdata[15: 8] : dc_read_data[15: 8],
                         mem_wstrb[0] ? mem_stdata[ 7: 0] : dc_read_data[ 7: 0] };

// write-back: one pass per dirty way.  entering the second pass clears
// write_back_twice and flips wb_way_r
assign wb_pass2   = mem_state[mem_writeback] & ~ bridge_mem_pending & write_back_twice;
assign wb_way_now = wb_pass2 ? ~ wb_way_r : wb_way_r;
assign wb_sel     = mem_state[cache_lookup]                                      // first pass
                  | (mem_state[mem_writeback] & write_back_twice & ~ bridge_mem_pending); // second
assign wb_pa      = wb_way_now ? dc_pa_way1   : dc_pa_way0;
assign wb_line    = wb_way_now ? dc_data_way1 : dc_data_way0;

always @(posedge clk) begin
    if (reset) begin
        wb_way_r         <= 1'b0;
        write_back_twice <= 1'b0;
    end
    else if (mem_state[cache_lookup]) begin
        wb_way_r         <= dc_replace_sel;         // victim way
        write_back_twice <= dc_d_way0 & dc_d_way1;  // two passes when both dirty
    end
    else if (wb_pass2) begin
        wb_way_r         <= ~ wb_way_r;
        write_back_twice <= 1'b0;
    end
end

assign should_write_back = dc_replace_sel ? dc_d_way1 : dc_d_way0;

// TODO: a cached hit should come from the cache instead
assign if_rdata  = bridge_inst_rdata[{if_va [3:2], 5'b0} +: 32];
assign mem_rdata = bridge_data_rdata[{mem_va[3:2], 5'b0} +: 32];

// bridge requests
assign bridge_if_req   = if_state[tlb_acc] & ~ if_abort & ~ ic_read_hit;
assign bridge_if_addr  = if_line_addr_r;
assign bridge_if_flush = if_abort;

assign bridge_data_req = mem_state[tlb_acc] & ~ mem_abort & ~ dc_read_hit
                                            & ~ should_write_back & ~ cacop_en  // refill read, no wb
                       | (mem_state[mem_writeback] & ~ bridge_mem_pending
                                                  & ~ write_back_twice & ~ cacop_en) // refill read after wb
                       | wb_sel;                                                      // write-back

assign bridge_data_addr  = wb_sel ? wb_pa : mem_line_addr_r;
assign bridge_data_we    = wb_sel ? 4'b1111 : 4'b0;   // refill reads; TODO: uncached bypass
assign bridge_data_wdata = wb_sel ? wb_line : {96'b0, mem_stdata};

// fetch path
assign if_take = if_pre & if_allow;

assign if_pending = ~ if_state[idle]
                  & ~(if_state[tlb_acc]      & (if_abort | ic_read_hit))
                  & ~(if_state[cache_refill] & if_take);

always @(*) begin
    case (if_state)
        IDLE: begin
            if (if_take)    next_if_state = TLB_ACC;
            else            next_if_state = IDLE;
        end
        TLB_ACC: begin
            if (if_abort)       next_if_state = IDLE;
            else if (ic_read_hit) begin
                if (if_take)    next_if_state = TLB_ACC;
                else            next_if_state = IDLE;
            end
            else                next_if_state = MEM_READ;
        end
        MEM_READ: begin
            if (~ bridge_if_pending) next_if_state = CACHE_REFILL;
            else                     next_if_state = MEM_READ;
        end
        CACHE_REFILL: begin
            if (if_take)    next_if_state = TLB_ACC;
            else            next_if_state = IDLE;
        end
        default:            next_if_state = IDLE;
    endcase
end

// mem path
assign mem_pending = ~ mem_state[idle]
                   & ~(mem_state[cache_refill] & mem_pre)
                   & ~(mem_state[tlb_acc]      & (mem_abort | dc_read_hit & ~ dc_rw_conflict));

always @(*) begin
    case (mem_state)
        IDLE: begin
            if (mem_pre)    next_mem_state = TLB_ACC;
            else            next_mem_state = IDLE;
        end
        TLB_ACC: begin
            if (mem_abort)          next_mem_state = IDLE;
            else if (dc_read_hit) begin
                if (~ dc_rw_conflict) next_mem_state = TLB_ACC;
                else                  next_mem_state = IDLE;
            end
            else if (should_write_back) next_mem_state = CACHE_LOOKUP;
            else if (cacop_en)          next_mem_state = CACHE_REFILL;
            else                        next_mem_state = MEM_READ;
        end
        CACHE_LOOKUP:       next_mem_state = MEM_WRITEBACK;
        MEM_WRITEBACK: begin
            if (~ bridge_mem_pending) begin
                if (write_back_twice)   next_mem_state = MEM_WRITEBACK;
                else if (cacop_en)      next_mem_state = CACHE_REFILL;
                else                    next_mem_state = MEM_READ;
            end
            else                            next_mem_state = MEM_WRITEBACK;
        end
        MEM_READ: begin
            if (~ bridge_mem_pending)       next_mem_state = CACHE_REFILL;
            else                            next_mem_state = MEM_READ;
        end
        CACHE_REFILL: begin
            if (mem_pre)    next_mem_state = TLB_ACC;
            else            next_mem_state = IDLE;
        end
        default:            next_mem_state = IDLE;
    endcase
end

always @(posedge clk) begin
    if (reset) begin
        if_state  <= IDLE;
        mem_state <= IDLE;
    end
    else begin
        if_state  <= next_if_state;
        mem_state <= next_mem_state;
    end
end

// bridge
sram_axi_bridge u_bridge(
    .clk          (clk                ),
    .reset        (reset              ),

    .inst_req     (bridge_if_req      ),
    .inst_addr    (bridge_if_addr     ),
    .inst_line    (1'b1               ),   // TODO: uncached fetch -> 1'b0
    .inst_flush   (bridge_if_flush    ),
    .inst_rdata   (bridge_inst_rdata  ),
    .inst_pending (bridge_if_pending  ),

    .data_req     (bridge_data_req    ),
    .data_we      (bridge_data_we     ),
    .data_addr    (bridge_data_addr   ),
    .data_wdata   (bridge_data_wdata  ),
    .data_line    (1'b1               ),   // TODO: uncached access -> 1'b0
    .data_rdata   (bridge_data_rdata  ),
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

//==============================================================================
// cache tie-off: the FSM does not drive the cache controls yet, and an undriven
// wire is z -> read_go / replace_sel go x -> the bridge requests go x.
// TODO: delete this block when the wiring below is in place.
//==============================================================================
assign ic_read_req    = 1'b0;
assign ic_cacop_en    = 1'b0;
assign ic_cacop_code  = 2'b0;
assign ic_refill_we   = 1'b0;

assign dc_read_req    = 1'b0;
assign dc_lookup      = 1'b0;
assign dc_write_req   = 1'b0;
assign dc_cacop_en    = 1'b0;
assign dc_cacop_code  = 2'b0;
assign dc_refill_we   = 1'b0;

assign ic_refill_data = bridge_inst_rdata;
assign dc_refill_data = bridge_data_rdata;   // TODO: merge the store bytes when dirty

// caches
icache u_icache(
    .clk         (clk            ),
    .reset       (reset          ),
    .va          (ic_va          ),
    .pt          (ic_pt          ),
    .read_req    (ic_read_req    ),
    .read_hit    (ic_read_hit    ),
    .read_data   (ic_read_data   ),
    .cacop_en    (ic_cacop_en    ),
    .cacop_code  (ic_cacop_code  ),
    .refill_we   (ic_refill_we   ),
    .refill_data (ic_refill_data )
);

dcache u_dcache(
    .clk                (clk             ),
    .reset              (reset           ),
    .va                 (dc_va           ),
    .pt                 (dc_pt           ),

    .read_req           (dc_read_req     ),
    .read_hit           (dc_read_hit     ),
    .read_data          (dc_read_data    ),

    .d_way0             (dc_d_way0       ),
    .d_way1             (dc_d_way1       ),
    .lookup             (dc_lookup       ),
    .pa_way0            (dc_pa_way0      ),
    .data_way0          (dc_data_way0    ),
    .pa_way1            (dc_pa_way1      ),
    .data_way1          (dc_data_way1    ),
    .replace_sel        (dc_replace_sel  ),

    .write_req          (dc_write_req    ),
    .write_va           (dc_write_va     ),
    .write_sel          (dc_write_sel    ),
    .write_data         (dc_write_data   ),
    .read_write_conflict(dc_rw_conflict  ),

    .cacop_en           (dc_cacop_en     ),
    .cacop_code         (dc_cacop_code   ),

    .refill_we          (dc_refill_we    ),
    .refill_dirty       (dc_refill_dirty ),
    .refill_data        (dc_refill_data  )
);

endmodule
