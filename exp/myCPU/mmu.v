module mmu(
    input  wire        clk,
    input  wire        reset,

    // CPU side: fetch
    input  wire        if_pre,          // a legal fetch address is on if_va
    input  wire [31:0] if_va,           // = pc[IF]
    input  wire        if_allow,        // = allow[IF]
    output wire        if_pending,      // -> stall[IF]
    output wire [31:0] if_rdata_0,      // instruction at if_va
    output wire [31:0] if_rdata_1,      // instruction at if_va + 4 (don't care when it crosses a line)

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

reg  [31:0] if_line_addr_r, mem_line_addr_r;  // line address of the access in flight
                                              // (the icache line is 64B, the dcache 16B;
                                              //  both stay inside one 4KB page, so the
                                              //  tag compare is still addr[31:12])

// addresses
wire [31:0] if_pa, mem_pa;
wire [31:0] if_addr, mem_addr;

wire [11:0] ic_va;
wire [19:0] ic_pt;
wire [11:0] dc_va;
wire [19:0] dc_pt;

wire        if_take;

// cacop view of the memory access in flight
wire        ic_cacop, dc_cacop;
wire [ 1:0] cacop_op;
wire        cacop_wb;
wire        cacop_read, cacop_act, ic_cacop_lookup;
reg         cacop_looked;

// cached or direct (uncached) access, decoded from the MAT field
wire        if_direct, mem_direct;

// refill line with the store bytes merged in
wire [ 31:0] refill_bmask, refill_old_word;
wire [ 31:0] dc_refill_word;
wire [127:0] dc_refill_merge;

wire        should_write_back;
wire        wb_sel;
wire [31:0]  wb_pa;
wire [127:0] wb_line;

// bridge
wire        bridge_if_req, bridge_if_flush, bridge_if_pending;
wire [31:0] bridge_if_addr;
wire        bridge_data_req, bridge_mem_pending;
wire [31:0] bridge_data_addr;
wire [127:0] bridge_data_wdata;
wire [ 3:0] bridge_data_we;
wire [511:0] bridge_inst_rdata;   // 64B icache line, 16 beats
wire [127:0] bridge_data_rdata;   // 16B dcache line, 4 beats

// icache
wire         ic_read_req, ic_read_use;
wire         ic_read_hit;
wire [ 31:0] ic_read_data_0;
wire [ 31:0] ic_read_data_1;

// second word of a fetch pair: same 16B line, one word up
wire [  3:0] if_wsel_1;
wire         ic_cacop_en, ic_cacop_hit_mode;
wire         ic_refill_we;
wire [511:0] ic_refill_data;

// dcache
wire         dc_read_req, dc_read_use;
wire         dc_read_hit;
wire [ 31:0] dc_read_data;
wire         dc_lookup;
wire         dc_write_req;
wire [ 11:0] dc_write_va;
wire [ 31:0] dc_write_data;
wire         dc_rw_conflict;
wire         dc_cacop_en, dc_cacop_hit_mode;
wire         dc_refill_we;
wire         dc_refill_dirty;
wire [127:0] dc_refill_data;

wire         dc_lookup_d;
wire [ 31:0] dc_lookup_pa;
wire [127:0] dc_lookup_line;
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
assign ic_read_req     = if_lu_now & ~ ic_cacop;

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
            if_line_addr_r <= {if_addr[31:6], 6'b0};   // 64B icache line
        end
        if (mem_state[tlb_acc]) begin
            mem_line_addr_r <= {mem_addr[31:4], 4'b0};  // 16B dcache line
        end
    end
end

// cacop: an icache cacop always takes two TLB_ACC cycles -- cycle 1 (cacop_read)
// drains the port and reads the tagvd, cycle 2 (cacop_act) writes it, so the
// icache port is clean before the invalidate.  A dcache cacop has already been
// looked up one cycle early (the lookup issued for the access entering MEM), so
// it acts in its very first TLB_ACC cycle and saves that cycle.
always @(posedge clk) begin
    if (reset)                cacop_looked <= 1'b0;
    else if (~ mem_cacop)     cacop_looked <= 1'b0;
    else if (cacop_read)      cacop_looked <= 1'b1;
    else if (cacop_act)       cacop_looked <= 1'b0;
end

assign ic_cacop      = mem_cacop & (mem_cacop_code[2:0] == 3'd0);
assign dc_cacop      = mem_cacop & (mem_cacop_code[2:0] == 3'd1);
assign cacop_op      = mem_cacop_code[4:3];             // 0 store tag / 1 index inv / 2 hit inv

// store tag never writes back (initialisation); op1 / op2 write a dirty line
// back first, but only for the dcache, and op2 only when it actually hit.  The
// way to touch is the one the cache itself picked as the lookup: VA[Way-1:0]
// (VA[0] for the two ways) for the index ops, the hit way for op2.
assign cacop_wb      = ~ dc_cacop          ? 1'b0
                     : (cacop_op == 2'b00) ? 1'b0
                     : (cacop_op == 2'b10) ? (dc_read_hit & dc_lookup_d)
                                           : dc_lookup_d;

// the icache is only driven by the fetch, so a cacop targeting it reads the
// tagvd with its own port; the refill write is off meanwhile (see ic_refill_we),
// so that read is the only access and the next cycle can write the tag.
assign cacop_read      = mem_state[tlb_acc] & mem_cacop & ~ cacop_looked & ic_cacop;
assign ic_cacop_lookup = cacop_read;

assign cacop_act       = mem_state[tlb_acc] & mem_cacop
                       & (cacop_looked | ~ ic_cacop)            // dcache: already looked up
                       & ~ mem_abort;                           // a flushed cacop must not invalidate

assign ic_cacop_en  = cacop_act & ic_cacop;
assign dc_cacop_en  = cacop_act & dc_cacop;
// op2 is the hit invalidate, op0 / op1 use direct index addressing
assign ic_cacop_hit_mode = cacop_op[1];
assign dc_cacop_hit_mode = cacop_op[1];

// cache ports
//   lookup  : a request issued this cycle, the result is consumed next cycle
//   read_use: the result of the request issued last cycle is consumed now
assign ic_va = ic_refill_we      ? if_line_addr_r[11:0]   // refill write
             : ic_cacop          ? mem_va    [11:0]        // cacop tag read + tagvd write
                                 : if_pre_addr[11:0];      // fetch lookup, one cycle early
assign ic_pt = ic_refill_we      ? if_line_addr_r[31:12]  // PA of the line being refilled
             : ic_cacop          ? mem_addr[31:12]         // tag compare of the cacop
                                 : if_addr  [31:12];

// fetch: no result while an icache cacop owns the port / the fetch is uncached
assign ic_read_use = if_state[tlb_acc] & ~ if_direct & ~ ic_cacop;

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
                       | mem_state[mem_writeback] & dc_cacop & ~ bridge_mem_pending
                       | mem_state[direct_acc] & ~ bridge_mem_pending
                       | mem_state[tlb_acc] & (mem_hit_done | mem_abort));

assign dc_read_req = dc_lu_now;
assign dc_va = dc_lu_now            ? mem_pre_addr[11:0]   // lookup for the access in ID
             : mem_state[tlb_acc]   ? mem_va       [11:0]   // translation / cacop response
                                    : mem_line_addr_r[11:0]; // write-back / refill
assign dc_pt = (mem_state[idle] | mem_state[tlb_acc]) ? mem_addr[31:12]
                                                      : mem_line_addr_r[31:12];

// the response of the lookup issued last cycle belongs to the access in MEM
assign dc_read_use = mem_state[tlb_acc] & ~ mem_abort & ((dc_cacop & cacop_act) | (~ mem_cacop & ~ mem_direct));

assign dc_write_va  = mem_va[11:0];
assign dc_refill_dirty = | mem_wstrb;      // store miss: the merged line is dirty

// store: byte-merge the new data into the word read back from the hit way
// (mem_stdata is already spread over the byte lanes)
assign dc_write_data = { mem_wstrb[3] ? mem_stdata[31:24] : dc_read_data[31:24],
                         mem_wstrb[2] ? mem_stdata[23:16] : dc_read_data[23:16],
                         mem_wstrb[1] ? mem_stdata[15: 8] : dc_read_data[15: 8],
                         mem_wstrb[0] ? mem_stdata[ 7: 0] : dc_read_data[ 7: 0] };

// the whole line has to be read out before it is handed to the write-back: a
// miss with a dirty victim, and a dcache cacop that will write its line back --
// its own lookup only read the bank of the access, so the rest of the line is
// not in the RAM outputs.  A plain hit needs nothing, an icache cacop never
// touches this cache, and CACHE_LOOKUP then uses the outputs of this read (it
// does not issue one of its own).
assign dc_lookup = mem_state[tlb_acc] & ~ mem_direct & ~ mem_abort & should_write_back
                 & (~ mem_cacop ? ~ dc_read_hit        // miss, dirty victim
                                : dc_cacop);           // cacop, dirty line

// write-back: only the evicted line leaves the cache -- the LRU way of a miss,
// or the single way a cacop selected before invalidating it.  The other way,
// even if dirty, stays resident and is written back when it is evicted itself.
// The cache itself has already picked that line (lookup_*), so the write-back
// only forwards it in the cycle that hands it to the bridge.
assign wb_sel  = mem_state[cache_lookup];
assign wb_pa   = dc_lookup_pa;
assign wb_line = dc_lookup_line;

assign should_write_back = mem_cacop ? cacop_wb : dc_lookup_d;

// the refilled line is written to the cache at the end of the last MEM_READ
// cycle, so its data is available in CACHE_REFILL
assign if_wsel_1 = if_va[5:2] + 4'd1;

assign if_rdata_0 = if_direct              ? bridge_inst_rdata[31:0]
                  : if_state[cache_refill] ? ic_refill_data[{if_va[5:2], 5'b0} +: 32]
                  : ic_read_hit            ? ic_read_data_0
                                           : ic_refill_data[{if_va[5:2], 5'b0} +: 32];

// the pair is only returned when if_va + 4 is still inside the same 64B line
// (a simple low-bit mux; the "nextpc" fetch just carries the same line) -- an
// uncached fetch has one beat only, so it can never return a pair either
assign if_rdata_1 = if_state[cache_refill] ? ic_refill_data[{if_wsel_1, 5'b0} +: 32]
                  : ic_read_hit            ? ic_read_data_1
                                           : ic_refill_data[{if_wsel_1, 5'b0} +: 32];   // use valid to avoid dirty data
assign mem_rdata = mem_direct             ? bridge_data_rdata[31:0]
                 : mem_state[cache_refill] ? dc_refill_data[{mem_va[3:2], 5'b0} +: 32]
                 : dc_read_hit            ? dc_read_data
                                          : dc_refill_data[{mem_va[3:2], 5'b0} +: 32];

// bridge requests
assign bridge_if_req   = if_state[tlb_acc] & ~ if_abort
                       & (if_direct | ~ ic_read_hit);    // refill read / uncached fetch
// a refill reads the 16B line, an uncached fetch / access must use the exact
// address (the bridge issues a single beat for those)
assign bridge_if_addr  = if_state[tlb_acc] ? (if_direct ? if_addr : {if_addr[31:6], 6'b0})
                                           : if_line_addr_r;
assign bridge_if_flush = if_abort;

assign bridge_data_req = (mem_state[tlb_acc] & mem_direct & ~ mem_abort & ~ mem_cacop) // uncached access
                       | (mem_state[tlb_acc] & ~ mem_abort & ~ mem_direct & ~ mem_cacop
                                            & ~ dc_read_hit & ~ should_write_back)     // refill read, no wb
                       | (mem_state[mem_writeback] & ~ bridge_mem_pending
                                                  & ~ mem_cacop)                       // refill read after wb
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

// an icache cacop owns the icache port for its whole stay in MEM: hold IF so no
// lookup is issued (and lost) while it is in flight
assign if_pending = ic_cacop
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
            // an icache cacop owns the port: drop this slot and wait in IDLE
            // (if_pending holds IF meanwhile, so nothing enters IF).  An aborted
            // fetch still lets the next one enter IF: keep the pipeline going
            // and consume the lookup issued for it last cycle.
            if (ic_cacop)               next_if_state = IDLE;
            else if (if_abort)          next_if_state = if_take ? TLB_ACC : IDLE;
            else if (if_direct)         next_if_state = DIRECT_ACC;   // uncached page
            else if (ic_read_hit) begin
                if (if_take)            next_if_state = TLB_ACC;
                else                    next_if_state = IDLE;
            end
            else                        next_if_state = MEM_READ;
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
                                                & mem_pre)
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
                if (mem_cacop) begin
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

assign ic_refill_abandon = if_state[mem_read] & (if_abort | ic_cacop);

always @(posedge clk) begin
    if (reset)                     ic_refill_kill <= 1'b0;
    else if (if_state[tlb_acc])    ic_refill_kill <= 1'b0;   // a new lookup / refill request starts here
    else if (ic_refill_abandon)    ic_refill_kill <= 1'b1;
end

// refill data: a store miss comes back from memory with this store's bytes merged in, a load miss is stored clean
// ... and it stays off while an icache cacop is in MEM: that cacop's two phases
// own the port, and the fetch will be re-fetched after the cacop's flush
assign ic_refill_we = if_state[mem_read] & ~ bridge_if_pending & ~ if_direct & ~ ic_refill_kill
                    & ~ ic_refill_abandon;
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
    .clk            (clk                ),
    .reset          (reset              ),
    .va             (ic_va              ),
    .pt             (ic_pt              ),
    .read_req       (ic_read_req        ),
    .read_use       (ic_read_use        ),
    .read_hit       (ic_read_hit        ),
    .read_data_0    (ic_read_data_0     ),
    .read_data_1    (ic_read_data_1     ),
    .cacop_lookup   (ic_cacop_lookup    ),
    .cacop_en       (ic_cacop_en        ),
    .cacop_hit_mode (ic_cacop_hit_mode  ),
    .refill_we      (ic_refill_we       ),
    .refill_data    (ic_refill_data     )
);

dcache u_dcache(
    .clk                (clk                ),
    .reset              (reset              ),
    .va                 (dc_va              ),
    .pt                 (dc_pt              ),
    
    .read_req           (dc_read_req        ),
    .read_use           (dc_read_use        ),
    .read_hit           (dc_read_hit        ),
    .read_data          (dc_read_data       ),
    
    .lookup             (dc_lookup          ),
    .lookup_d           (dc_lookup_d        ),
    .lookup_pa          (dc_lookup_pa       ),
    .lookup_line        (dc_lookup_line     ),
    
    .write_req          (dc_write_req       ),
    .write_va           (dc_write_va        ),
    .write_data         (dc_write_data      ),
    .read_write_conflict(dc_rw_conflict     ),
    
    .cacop_en           (dc_cacop_en        ),
    .cacop_hit_mode     (dc_cacop_hit_mode  ),

    .refill_we          (dc_refill_we       ),
    .refill_dirty       (dc_refill_dirty    ),
    .refill_data        (dc_refill_data     )
);

endmodule
