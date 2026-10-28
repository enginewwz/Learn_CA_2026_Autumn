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
    input  wire        mem_cacop,          // a cacop is in MEM
    input  wire [ 4:0] mem_cacop_code,
    input  wire [ 1:0] if_mat,             // MAT of the fetch access
    input  wire [ 1:0] mem_mat,            // MAT of the load / store

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
localparam direct_acc       = 6;
localparam stagecnt         = 7;

localparam IDLE             = 7'b1 << idle;
localparam TLB_ACC          = 7'b1 << tlb_acc;          // also reads the cache result
localparam CACHE_LOOKUP     = 7'b1 << cache_lookup;
localparam MEM_WRITEBACK    = 7'b1 << mem_writeback;
localparam MEM_READ         = 7'b1 << mem_read;
localparam CACHE_REFILL     = 7'b1 << cache_refill;
localparam DIRECT_ACC       = 7'b1 << direct_acc;

// state
reg  [stagecnt - 1:0] if_state,      mem_state;
reg  [stagecnt - 1:0] next_if_state, next_mem_state;

// cacop control generated from mem_cacop / mem_cacop_code
reg         write_back_twice;

reg  [31:0] if_line_addr_r, mem_line_addr_r;  // 16B line address of the access in flight
                                              // (its [31:12] *is* the physical tag)
reg         wb_way_r;               // way being written back

// addresses
wire [31:0] if_pa, mem_pa;
wire [31:0] if_addr, mem_addr;

wire [11:0] ic_va;
wire [19:0] ic_pt;
wire [11:0] dc_va;
wire [19:0] dc_pt;

wire        if_take;

// cacop view of the memory access in flight
wire        cacop_ic, cacop_dc;
wire [ 1:0] cacop_op;
wire        cacop_way, cacop_dirty, cacop_wb;
wire        cacop_lookup, cacop_act, cacop_ic_in_mem;
wire        cacop_port_ok;
reg         cacop_looked;

// cached or direct (uncached) access, decoded from the MAT field
wire        if_direct, mem_direct;

// refill line with the store bytes merged in
wire [ 31:0] refill_bmask, refill_old_word;
wire [ 31:0] dc_refill_word;
wire [127:0] dc_refill_merge;

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
wire         ic_read_req, ic_read_use;
wire         ic_read_hit;
wire [ 31:0] ic_read_data;
wire         ic_cacop_en, ic_cacop_hit;
wire         ic_refill_we;
wire [127:0] ic_refill_data;

// dcache
wire         dc_read_req, dc_read_use;
wire         dc_read_hit;
wire [ 31:0] dc_read_data;
wire         dc_lookup;
wire         dc_write_req;
wire [ 11:0] dc_write_va;
wire         dc_write_sel;
wire [ 31:0] dc_write_data;
wire         dc_rw_conflict;
wire         dc_cacop_en, dc_cacop_hit;
wire         dc_refill_we;
wire         dc_refill_dirty;
wire [127:0] dc_refill_data;

wire         dc_d_way0, dc_d_way1;
wire [ 31:0] dc_pa_way0, dc_pa_way1;
wire [127:0] dc_data_way0, dc_data_way1;
wire         dc_replace_sel;
wire         mem_hit_done, dc_lu_now;

wire         if_lu_now;

// fetch lookups: IDLE starts the upcoming fetch, the other cycles start the
// fetch after the current one (which enters IF next cycle)
assign if_lu_now = (if_state[idle]         & if_take)
                 | (if_state[tlb_acc]      & (if_abort | (ic_read_hit & ~ if_direct)) & if_take)
                 | (if_state[cache_refill] & if_take)
                 | (if_state[direct_acc]   & ~ bridge_if_pending & if_take);

// an icache hit-invalidate compares the tag in phase 2, so the cacop has to
// issue its own tag read in phase 1 (it owns the port in both phases).  It gets
// its own port so read_req stays the fetch-only lookup -- last_va and the
// registered LRU bit inside icache must not be touched by a cacop.
assign ic_read_req     = if_lu_now & ~ cacop_ic_in_mem;
assign ic_cacop_lookup = cacop_lookup & cacop_ic;

// address translation: dmw window > tlb entry > the va as it is
assign if_pa  = (if_ps  == 6'd21) ? {if_ppn[19:9],  if_va[20:0]}
                                  : {if_ppn[19:0],  if_va[11:0]};
assign mem_pa = (mem_ps == 6'd21) ? {mem_ppn[19:9], mem_va[20:0]}
                                  : {mem_ppn[19:0], mem_va[11:0]};

assign if_addr  = if_dmw_hit  ? if_dmw_pa  : if_xlat  ? if_pa  : if_va;
assign mem_addr = mem_dmw_hit ? mem_dmw_pa : mem_xlat ? mem_pa : mem_va;

// latched: line address of the access in flight; TLB_ACC is the cycle this
// access is translated, so its line and tag are valid exactly then (both read
// paths run one cycle ahead of MEM / IF)
always @(posedge clk) begin
    if (reset) begin
        if_line_addr_r  <= 32'b0;
        mem_line_addr_r <= 32'b0;
    end
    else begin
        if (if_state[tlb_acc]) begin
            if_line_addr_r <= {if_addr[31:4], 4'b0};
        end
        if (mem_state[tlb_acc]) begin
            mem_line_addr_r <= {mem_addr[31:4], 4'b0};
        end
    end
end

// cacop: the FSM spends two TLB_ACC cycles on it.  cycle 1 (cacop_lookup)
// issues the tag / whole-line read, cycle 2 (cacop_act) consumes it and writes
// the tagvd (and, for a dirty dcache line that is about to be invalidated,
// hands the line to the write-back).
always @(posedge clk) begin
    if (reset)                cacop_looked <= 1'b0;
    else if (~ mem_cacop)     cacop_looked <= 1'b0;
    else if (cacop_lookup)    cacop_looked <= 1'b1;
    else if (cacop_act)       cacop_looked <= 1'b0;
end

assign cacop_ic      = mem_cacop & (mem_cacop_code[2:0] == 3'd0);
assign cacop_dc      = mem_cacop & (mem_cacop_code[2:0] == 3'd1);
assign cacop_op      = mem_cacop_code[4:3];             // 0 store tag / 1 index inv / 2 hit inv

// direct index addressing: the line sits in way VA[0] (2 ways)
assign cacop_way     = (cacop_op == 2'b01) ? mem_va[0] : dc_replace_sel;
assign cacop_dirty   = cacop_way ? dc_d_way1 : dc_d_way0;
// store tag never writes back (initialisation); op1 / op2 write a dirty line back
// first, but only for the dcache, and op2 only when it actually hit
assign cacop_wb      = ~ cacop_dc          ? 1'b0
                     : (cacop_op == 2'b00) ? 1'b0
                     : (cacop_op == 2'b10) ? (dc_read_hit & cacop_dirty)
                                           : cacop_dirty;

// the icache port is shared with the fetch: a refill write would spoil the read
assign cacop_port_ok   = ~ cacop_ic | ~ ic_refill_we;
assign cacop_lookup    = mem_state[tlb_acc] & mem_cacop & ~ cacop_looked & cacop_port_ok;
assign cacop_act       = mem_state[tlb_acc] & mem_cacop &   cacop_looked & cacop_port_ok
                       & ~ mem_abort;                           // a flushed cacop must not invalidate
assign cacop_ic_in_mem = mem_state[tlb_acc] & cacop_ic;

assign ic_cacop_en  = cacop_act & cacop_ic;
assign dc_cacop_en  = cacop_act & cacop_dc;
// op2 is the hit invalidate, op0 / op1 use direct index addressing
assign ic_cacop_hit = cacop_op[1];
assign dc_cacop_hit = cacop_op[1];

// cache ports
//   lookup  : a request issued this cycle, the result is consumed next cycle
//   read_use: the result of the request issued last cycle is consumed now
assign ic_va = ic_refill_we      ? if_line_addr_r[11:0]   // refill write
             : cacop_ic_in_mem   ? mem_va    [11:0]        // cacop lookup + tagvd write
                                 : if_pre_addr[11:0];      // fetch lookup, one cycle early
assign ic_pt = ic_refill_we      ? if_line_addr_r[31:12]  // PA of the line being refilled
             : cacop_ic_in_mem   ? mem_addr[31:12]         // tag compare of the cacop
                                 : if_addr  [31:12];

// fetch: no result while an icache cacop owns the port / the fetch is uncached
assign ic_read_use = if_state[tlb_acc] & ~ if_direct & ~ cacop_ic_in_mem;

// data: lookups are issued in IDLE / CACHE_REFILL (the access entering MEM),
// in the response cycle of a cached hit (the next access), in the last
// DIRECT_ACC cycle (the access after an uncached one) and in the last
// MEM_WRITEBACK cycle of a cacop (which leaves MEM right there, so the next
// access needs its response one cycle later).  A normal miss does not: it goes
// on to MEM_READ / CACHE_REFILL, which issue the lookup themselves and would
// clobber it.
assign mem_hit_done = ~ mem_cacop & ~ mem_direct & dc_read_hit;
assign dc_lu_now    = mem_pre
                    & (mem_state[idle] | mem_state[cache_refill]
                       | mem_state[mem_writeback] & mem_cacop & ~ bridge_mem_pending & ~ write_back_twice
                       | mem_state[direct_acc] & ~ bridge_mem_pending
                       | mem_state[tlb_acc] & (mem_hit_done | mem_abort));

assign dc_read_req = dc_lu_now;
assign dc_va = dc_lu_now            ? mem_pre_addr[11:0]   // lookup for the access in EX
             : mem_state[tlb_acc]   ? mem_va       [11:0]   // translation / cacop response
                                    : mem_line_addr_r[11:0]; // write-back / refill
assign dc_pt = (mem_state[idle] | mem_state[tlb_acc]) ? mem_addr[31:12]
                                                      : mem_line_addr_r[31:12];

// the response of the lookup issued last cycle belongs to the access in MEM
assign dc_read_use = mem_state[tlb_acc]
                   & ((mem_cacop & cacop_act) | (~ mem_cacop & ~ mem_direct));

assign dc_write_va  = mem_va[11:0];
assign dc_write_sel = dc_replace_sel;      // will not change as no more read-use issued
assign dc_refill_dirty = | mem_wstrb;      // store miss: the merged line is dirty

// store: byte-merge the new data into the word read back from the hit way
// (mem_stdata is already spread over the byte lanes)
assign dc_write_data = { mem_wstrb[3] ? mem_stdata[31:24] : dc_read_data[31:24],
                         mem_wstrb[2] ? mem_stdata[23:16] : dc_read_data[23:16],
                         mem_wstrb[1] ? mem_stdata[15: 8] : dc_read_data[15: 8],
                         mem_wstrb[0] ? mem_stdata[ 7: 0] : dc_read_data[ 7: 0] };

// miss: read the whole victim line while the miss is known, so that the
// write-back issues from complete data in CACHE_LOOKUP
assign dc_lookup = mem_state[cache_lookup]
                 | (cacop_lookup & cacop_dc)
                 | (mem_state[tlb_acc] & ~ mem_cacop & ~ mem_direct
                                      & ~ dc_read_hit & should_write_back);

// write-back: one pass per dirty way.  entering the second pass clears
// write_back_twice and flips wb_way_r
assign wb_pass2   = mem_state[mem_writeback] & ~ bridge_mem_pending & write_back_twice;
assign wb_way_now = mem_state[cache_lookup] ? (mem_cacop ? cacop_way : dc_replace_sel)
                  : wb_pass2                ? ~ wb_way_r
                                            : wb_way_r;
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
        wb_way_r         <= mem_cacop ? cacop_way : dc_replace_sel;   // victim / cacop way
        write_back_twice <= ~ mem_cacop & dc_d_way0 & dc_d_way1;      // cacop touches one way
    end
    else if (wb_pass2) begin
        wb_way_r         <= ~ wb_way_r;
        write_back_twice <= 1'b0;
    end
end

assign should_write_back = mem_cacop ? cacop_wb
                                     : (dc_replace_sel ? dc_d_way1 : dc_d_way0);

// the refilled line is written to the cache at the end of the last MEM_READ
// cycle, so its data is available in CACHE_REFILL
assign if_rdata  = if_direct              ? bridge_inst_rdata[31:0]
                 : if_state[cache_refill] ? ic_refill_data[{if_va[3:2], 5'b0} +: 32]
                 : ic_read_hit            ? ic_read_data
                                          : ic_refill_data[{if_va[3:2], 5'b0} +: 32];
assign mem_rdata = mem_direct             ? bridge_data_rdata[31:0]
                 : mem_state[cache_refill] ? dc_refill_data[{mem_va[3:2], 5'b0} +: 32]
                 : dc_read_hit            ? dc_read_data
                                          : dc_refill_data[{mem_va[3:2], 5'b0} +: 32];

// bridge requests
assign bridge_if_req   = if_state[tlb_acc] & ~ if_abort
                       & (if_direct | ~ ic_read_hit);    // refill read / uncached fetch
// a refill reads the 16B line, an uncached fetch / access must use the exact
// address (the bridge issues a single beat for those)
assign bridge_if_addr  = if_state[tlb_acc] ? (if_direct ? if_addr : {if_addr[31:4], 4'b0})
                                           : if_line_addr_r;
assign bridge_if_flush = if_abort;

assign bridge_data_req = (mem_state[tlb_acc] & mem_direct & ~ mem_abort & ~ mem_cacop) // uncached access
                       | (mem_state[tlb_acc] & ~ mem_abort & ~ mem_direct & ~ mem_cacop
                                            & ~ dc_read_hit & ~ should_write_back)     // refill read, no wb
                       | (mem_state[mem_writeback] & ~ bridge_mem_pending
                                                  & ~ write_back_twice & ~ mem_cacop)  // refill read after wb
                       | wb_sel;                                                       // write-back

assign bridge_data_addr  = wb_sel ? wb_pa
                         : mem_state[tlb_acc] ? (mem_direct ? mem_addr : {mem_addr[31:4], 4'b0})
                                              : mem_line_addr_r;
assign bridge_data_we    = wb_sel                             ? 4'b1111   // write-back
                         : (mem_state[tlb_acc] & mem_direct)  ? mem_wstrb // uncached store
                                                              : 4'b0;     // refill read
assign bridge_data_wdata = wb_sel ? wb_line : {96'b0, mem_stdata};

// MAT = coherent cached only; strongly / weakly ordered go straight to memory
assign if_direct  = (if_mat  != 2'b01);
assign mem_direct = (mem_mat != 2'b01);

// fetch path
assign if_take = if_pre & if_allow;

// an icache cacop holds the fetch until it is done with the port
assign if_pending = cacop_ic_in_mem
                  | (~ if_state[idle]
                     & ~(if_state[tlb_acc]      & (if_abort | (ic_read_hit & ~ if_direct)))
                     & ~(if_state[direct_acc]   & ~ bridge_if_pending)
                     & ~(if_state[cache_refill] & if_pre));

always @(*) begin
    case (if_state)
        IDLE: begin
            if (if_take)    next_if_state = TLB_ACC;
            else            next_if_state = IDLE;
        end
        TLB_ACC: begin
            // an aborted fetch still lets the next one enter IF: keep the
            // pipeline going and look that one up
            if (if_abort)       next_if_state = if_take ? TLB_ACC : IDLE;
            else if (if_direct) next_if_state = DIRECT_ACC;   // uncached page
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
        DIRECT_ACC: begin
            if (~ bridge_if_pending) begin
                if (if_take) next_if_state = TLB_ACC;
                else         next_if_state = IDLE;
            end
            else             next_if_state = DIRECT_ACC;
        end
        default:            next_if_state = IDLE;
    endcase
end

// mem path
// a cacop has nothing left to refill once its write-back is drained: let it
// leave MEM in that cycle so MEM_WRITEBACK goes straight to TLB_ACC.  Gated by
// mem_cacop -- a normal miss must stay in MEM for MEM_READ / CACHE_REFILL,
// otherwise the next access's lookup and the refill fight over the dcache port.
assign mem_pending = ~ mem_state[idle]
                   & ~(mem_state[cache_refill]  & mem_pre)
                   & ~(mem_state[mem_writeback] & mem_cacop & ~ bridge_mem_pending
                                                & ~ write_back_twice & mem_pre)
                   & ~(mem_state[tlb_acc]       & (mem_abort | (mem_hit_done & ~ dc_rw_conflict)))
                   & ~(mem_state[direct_acc]    & ~ bridge_mem_pending & mem_pre);

always @(*) begin
    case (mem_state)
        IDLE: begin
            if (mem_pre)    next_mem_state = TLB_ACC;
            else            next_mem_state = IDLE;
        end
        TLB_ACC: begin
            if (mem_abort)                              next_mem_state = mem_pre ? TLB_ACC : IDLE;
            else if (mem_cacop) begin
                if (~ cacop_act)                        next_mem_state = TLB_ACC;    // phase 2
                else if (should_write_back)             next_mem_state = CACHE_LOOKUP;
                else                                    next_mem_state = IDLE;       // done
            end
            else if (mem_direct)                        next_mem_state = DIRECT_ACC; // uncached page
            else if (dc_read_hit) begin
                if (~ dc_rw_conflict & mem_pre)          next_mem_state = TLB_ACC;
                else                                     next_mem_state = IDLE;       // lookup was dropped
            end
            else if (should_write_back)                 next_mem_state = CACHE_LOOKUP;
            else                                        next_mem_state = MEM_READ;
        end
        CACHE_LOOKUP:       next_mem_state = MEM_WRITEBACK;
        MEM_WRITEBACK: begin
            if (~ bridge_mem_pending) begin
                if (write_back_twice)                   next_mem_state = MEM_WRITEBACK;
                else if (mem_cacop) begin
                    if (mem_pre)                        next_mem_state = TLB_ACC;   // cacop_act takes place earlier
                    else                                next_mem_state = IDLE;
                end                    
                else                                    next_mem_state = MEM_READ;
            end
            else                                        next_mem_state = MEM_WRITEBACK;
        end
        MEM_READ: begin
            if (~ bridge_mem_pending)                   next_mem_state = CACHE_REFILL;
            else                                        next_mem_state = MEM_READ;
        end
        CACHE_REFILL: begin
            if (mem_pre)    next_mem_state = TLB_ACC;   // lookup was done this cycle
            else            next_mem_state = IDLE;
        end
        DIRECT_ACC: begin
            if (~ bridge_mem_pending) begin
                if (mem_pre) next_mem_state = TLB_ACC;
                else         next_mem_state = IDLE;
            end
            else             next_mem_state = DIRECT_ACC;
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
    .inst_line    (~ (if_state[tlb_acc] & if_direct)),   // uncached fetch: single beat
    .inst_flush   (bridge_if_flush    ),
    .inst_rdata   (bridge_inst_rdata  ),
    .inst_pending (bridge_if_pending  ),

    .data_req     (bridge_data_req    ),
    .data_we      (bridge_data_we     ),
    .data_addr    (bridge_data_addr   ),
    .data_wdata   (bridge_data_wdata  ),
    .data_line    (~ (mem_state[tlb_acc] & mem_direct)), // uncached access: single beat
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

// icache refill: an in-flight refill must not put a line back after an icache
// cacop invalidated it (its data was read before the cacop), and a refill for a
// fetch that is being discarded (abort / flush) is not worth caching either.
// Both are a single-cycle event, so the decision is kept until the write.
wire ic_refill_abandon;
reg  ic_refill_kill;

assign ic_refill_abandon = if_state[mem_read]
                         & (if_abort | (cacop_ic & (mem_va[11:4] == if_line_addr_r[11:4])));

always @(posedge clk) begin
    if (reset)                     ic_refill_kill <= 1'b0;
    else if (if_state[tlb_acc])    ic_refill_kill <= 1'b0;   // a new lookup / refill request starts here
    else if (ic_refill_abandon)    ic_refill_kill <= 1'b1;
end

// refill data: a store miss comes back from memory with this store's bytes merged in, a load miss is stored clean
assign ic_refill_we = if_state[mem_read] & ~ bridge_if_pending & ~ if_direct & ~ ic_refill_kill;
assign dc_refill_we = mem_state[mem_read] & ~ bridge_mem_pending & ~ mem_direct & ~ mem_cacop;

assign ic_refill_data = bridge_inst_rdata;

// the store's word inside the line is mem_va[3:2] (mem_line_addr_r is the line base, so its low bits are always zero)
assign refill_bmask    = {{8{mem_wstrb[3]}}, {8{mem_wstrb[2]}},
                          {8{mem_wstrb[1]}}, {8{mem_wstrb[0]}}};
assign refill_old_word = bridge_data_rdata[{mem_va[3:2], 5'b0} +: 32];
assign dc_refill_word  = (refill_bmask & mem_stdata)
                       | (~ refill_bmask & refill_old_word);
assign dc_refill_merge = (mem_va[3:2] == 2'd0) ? {bridge_data_rdata[127: 32], dc_refill_word}
                       : (mem_va[3:2] == 2'd1) ? {bridge_data_rdata[127: 64], dc_refill_word, bridge_data_rdata[ 31: 0]}
                       : (mem_va[3:2] == 2'd2) ? {bridge_data_rdata[127: 96], dc_refill_word, bridge_data_rdata[ 63: 0]}
                                               : {dc_refill_word, bridge_data_rdata[ 95: 0]};
// only a store miss has bytes to merge into the line; a load miss is the memory
// line itself, so it bypasses the 128-bit word-merge mux
assign dc_refill_data = (| mem_wstrb) ? dc_refill_merge : bridge_data_rdata;

// on a read/write conflict the write still wins, only the pipelined lookup
// is dropped (and mem_pending keeps the next access away for one cycle)
// store hit.
assign dc_write_req = mem_state[tlb_acc] & ~ mem_abort & ~ mem_cacop & ~ mem_direct & dc_read_hit
                    & (| mem_wstrb);                            // store hit

// caches
icache u_icache(
    .clk         (clk            ),
    .reset       (reset          ),
    .va          (ic_va          ),
    .pt          (ic_pt          ),
    .read_req    (ic_read_req    ),
    .read_use    (ic_read_use    ),
    .read_hit    (ic_read_hit    ),
    .read_data   (ic_read_data   ),
    .cacop_lookup(ic_cacop_lookup),
    .cacop_en    (ic_cacop_en    ),
    .cacop_hit   (ic_cacop_hit   ),
    .refill_we   (ic_refill_we   ),
    .refill_data (ic_refill_data )
);

dcache u_dcache(
    .clk                (clk             ),
    .reset              (reset           ),
    .va                 (dc_va           ),
    .pt                 (dc_pt           ),

    .read_req           (dc_read_req     ),
    .read_use           (dc_read_use     ),
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
    .cacop_hit          (dc_cacop_hit    ),

    .refill_we          (dc_refill_we    ),
    .refill_dirty       (dc_refill_dirty ),
    .refill_data        (dc_refill_data  )
);

endmodule
