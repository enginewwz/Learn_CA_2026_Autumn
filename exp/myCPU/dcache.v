//==============================================================================
// dcache.v -- 2-way / 256-set / 16-byte-line, write-back data cache
//
// VIPT: va (low 12 bits of VA and PA) indexes the RAMs in the lookup cycle;
//       pt is the physical tag from the async TLB, valid in the response
//       cycle.  So: tag compare -> current pt, word select -> last_va.
//
// Per-bank enable (bank = va[3:2] / write_va[3:2]):
//   read    : bank va[3:2] of both ways          (rd_en)
//   store   : bank write_va[3:2] of way write_sel
//   refill  : all four banks of the replaced way
//   lookup  : all four banks of both ways -> data_wayN holds the whole line
// A read and a write may share a cycle iff same line and different bank.
//
// Lookup / response is pipelined: read_req in cycle N, read_hit / read_data /
// replace_sel / d_wayN in cycle N+1 with read_use high.  read_use qualifies
// every state update, so a lookup whose result is not consumed (e.g. a
// conflict with a store, or an access that turns out to be uncached) cannot
// corrupt the LRU or the kept victim way.
//
// Store: cycle 1 read_req (data_wayN / read_hit / replace_sel),
//        cycle 2 write_req with the byte-merged word; d is set there.
//==============================================================================
module dcache(
    input  wire         clk,
    input  wire         reset,

    input  wire [11:0]  va,             // {index, offset}
    input  wire [19:0]  pt,             // physical tag, valid in the response cycle

    // read
    input  wire         read_req,       // issue a lookup, result is valid one cycle later
    input  wire         read_use,       // this cycle consumes the previous lookup's result
    output wire         read_hit,
    output wire [31:0]  read_data,

    // miss / cacop view, valid in the response cycle
    output wire         d_way0,         // dirty
    output wire         d_way1,

    input  wire         lookup,         // 1: also read the other banks -> full line
    output wire [31:0]  pa_way0,        // {tag, index, 4'b0}
    output wire [127:0] data_way0,      // {w12, w8, w4, w0}
    output wire [31:0]  pa_way1,
    output wire [127:0] data_way1,

    output wire         replace_sel,    // hit -> hit way, miss -> LRU way

    // write
    input  wire         write_req,
    input  wire [11:0]  write_va,
    input  wire         write_sel,
    input  wire [31:0]  write_data,     // byte-merged by the mmu
    output wire         read_write_conflict,

    // cacop
    input  wire         cacop_en,
    input  wire         cacop_hit,      // 1 = hit invalidate, 0 = direct index

    // refill
    input  wire         refill_we,
    input  wire         refill_dirty,   // d bit stored with the line
    input  wire [127:0] refill_data
);

wire [ 7:0] index;
wire [ 7:0] write_index;
wire [ 1:0] write_bank;
wire hit_way [1:0];
wire tag_we_way [1:0];

wire [127:0] rdata_way_line [1:0];
wire [31:0]  rdata_way_word [1:0];
wire [21:0]  rtagvd_way [1:0];
wire [19:0]  rtagvd_tag [1:0];
wire         rtagvd_v [1:0];
wire         rtagvd_d [1:0];

wire cacop_we_way [1:0];
wire cacop_way0, cacop_way1;
wire [21:0] cacop_tagvd_data;

wire refill_we_way [1:0];
wire [21:0] refill_tagvd_data;
wire [21:0] write_tagvd_data;

wire [21:0] wtagvd_data;

wire [ 7:0] last_index;
reg [255:0] lru_table;
reg  [11:0] last_va;

// read / write conflict, write port and per-bank enables
wire        same_index, same_bank, need_d_we, read_go;
wire [ 3:0] rd_bank, rd_en;
wire        wr_way0, wr_way1, wr_tag_way0, wr_tag_way1;
wire [ 3:0] wr_bank_way0, wr_bank_way1;
wire [ 3:0] bank_en0, bank_en1, bank_we0, bank_we1;
wire [ 7:0] addr0_b0, addr0_b1, addr0_b2, addr0_b3;
wire [ 7:0] addr1_b0, addr1_b1, addr1_b2, addr1_b3;
wire [127:0] bank_din;

wire        sel_now;

assign index       = va[11:4];
assign write_index = write_va[11:4];
assign write_bank  = write_va[3:2];
assign last_index  = last_va[11:4];

// read / write in the same cycle
//   ok   : same line, different bank (different data RAMs, same tag address)
//   stall: same bank, or different line while the tag still needs its d written
// on conflict the write wins, the read is dropped (read_go = 0)
assign same_index = (index == write_index);
assign same_bank  = (va[3:2] == write_va[3:2]);
assign need_d_we  = ~ rtagvd_d[write_sel];

assign read_write_conflict = read_req & write_req & (same_bank | (~same_index & need_d_we));
assign read_go = read_req & ~ read_write_conflict;

// bank select
assign rd_bank      = 4'b0001 << va[3:2];
assign wr_way0      = write_req & ~ write_sel;
assign wr_way1      = write_req &   write_sel;
assign wr_tag_way0  = wr_way0 & need_d_we;      // d already set -> no tag write
assign wr_tag_way1  = wr_way1 & need_d_we;
assign wr_bank_way0 = {4{wr_way0}} & (4'b0001 << write_bank);
assign wr_bank_way1 = {4{wr_way1}} & (4'b0001 << write_bank);

assign rd_en = lookup ? 4'b1111 : (read_go ? rd_bank : 4'b0000);

assign bank_en0 = refill_we_way[0] ? 4'b1111 : rd_en | wr_bank_way0;
assign bank_en1 = refill_we_way[1] ? 4'b1111 : rd_en | wr_bank_way1;

assign bank_we0 = refill_we_way[0] ? 4'b1111 : wr_bank_way0;
assign bank_we1 = refill_we_way[1] ? 4'b1111 : wr_bank_way1;

assign addr0_b0 = wr_bank_way0[0] ? write_index : index;
assign addr0_b1 = wr_bank_way0[1] ? write_index : index;
assign addr0_b2 = wr_bank_way0[2] ? write_index : index;
assign addr0_b3 = wr_bank_way0[3] ? write_index : index;
assign addr1_b0 = wr_bank_way1[0] ? write_index : index;
assign addr1_b1 = wr_bank_way1[1] ? write_index : index;
assign addr1_b2 = wr_bank_way1[2] ? write_index : index;
assign addr1_b3 = wr_bank_way1[3] ? write_index : index;

assign bank_din = write_req ? {4{write_data}} : refill_data;

// word select: only the read bank is fresh, pick it with last_va[3:2]
genvar i;
generate
    wire [31:0] rdata_way0_word_offset [3:0];
    wire [31:0] rdata_way1_word_offset [3:0];
    for (i = 0; i < 4; i = i + 1)begin: offset_sel
        assign rdata_way0_word_offset [i] = (last_va[3:2] == i[1:0]) ? rdata_way_line [0][(i + 1) * 32 - 1: i * 32] : 32'b0;
        assign rdata_way1_word_offset [i] = (last_va[3:2] == i[1:0]) ? rdata_way_line [1][(i + 1) * 32 - 1: i * 32] : 32'b0;
    end
    assign rdata_way_word [0] = rdata_way0_word_offset [0]
                              | rdata_way0_word_offset [1]
                              | rdata_way0_word_offset [2]
                              | rdata_way0_word_offset [3];
    assign rdata_way_word [1] = rdata_way1_word_offset [0]
                              | rdata_way1_word_offset [1]
                              | rdata_way1_word_offset [2]
                              | rdata_way1_word_offset [3];
endgenerate

assign {rtagvd_tag[0], rtagvd_v[0], rtagvd_d[0]} = rtagvd_way[0];
assign {rtagvd_tag[1], rtagvd_v[1], rtagvd_d[1]} = rtagvd_way[1];

assign hit_way[0] = (rtagvd_tag[0] == pt) & rtagvd_v[0];
assign hit_way[1] = (rtagvd_tag[1] == pt) & rtagvd_v[1];

assign read_hit  = hit_way[0] | hit_way[1];
assign read_data = hit_way[0] ? rdata_way_word[0] : rdata_way_word [1];

// direct index addressing: way = VA[Way-1:0], i.e. VA[0] for the two ways;
// hit invalidate instead picks the way whose tag matches
assign cacop_way0 = ~ va[0];
assign cacop_way1 =   va[0];

assign cacop_we_way [0] = cacop_en & (cacop_hit ? hit_way[0] : cacop_way0);
assign cacop_we_way [1] = cacop_en & (cacop_hit ? hit_way[1] : cacop_way1);

assign cacop_tagvd_data  = 22'b0;                           // invalidate: clear tag/v/d
assign refill_tagvd_data = {pt, 1'b1, refill_dirty};         // {tag, v=1, d}
assign write_tagvd_data  = {rtagvd_tag[write_sel], 2'b11};  // {tag, v=1, d=1}

assign wtagvd_data = refill_we ? refill_tagvd_data
                   : write_req ? write_tagvd_data
                   :             cacop_tagvd_data;

assign refill_we_way[0] = refill_we & ~ lru_table[index];
assign refill_we_way[1] = refill_we &   lru_table[index];

assign tag_we_way[0] = cacop_we_way[0] | refill_we_way[0] | wr_tag_way0;
assign tag_we_way[1] = cacop_we_way[1] | refill_we_way[1] | wr_tag_way1;

// the lookup address / offset of the access whose response is consumed this cycle
always @(posedge clk) begin
    if (read_go) last_va <= va;
end

// LRU: only the access that consumes its lookup result may touch it
always @(posedge clk) begin
    if (reset)                          lru_table <= 256'b0;
    else if (read_use & read_hit)       lru_table[last_index] <= hit_way[0];
    else if (refill_we)                 lru_table[index] <= ~ lru_table[index];
end

// way select: hit -> hit way, miss -> LRU way.  Held after the response cycle
// so the write-back / refill sequence can keep using it.
assign sel_now = read_hit ? hit_way[1] : lru_table[last_index];
reg  sel_r;

always @(posedge clk) begin
    if (reset)         sel_r <= 1'b0;
    else if (read_use) sel_r <= sel_now;
end

assign replace_sel = read_use ? sel_now : sel_r;

// view for the mmu.  va is the line of interest in every cycle that reads
// these (response cycle, cache_lookup, mem_writeback), so it gives the index.
assign d_way0    = rtagvd_d[0];
assign d_way1    = rtagvd_d[1];
assign pa_way0   = {rtagvd_tag[0], va[11:4], 4'b0};
assign pa_way1   = {rtagvd_tag[1], va[11:4], 4'b0};
assign data_way0 = rdata_way_line[0];
assign data_way1 = rdata_way_line[1];

// RAM instances
tagvd u_tagvd_way0(
    .clka  (clk                                            ),
    .ena   (read_go | cacop_en | refill_we | wr_way0 | lookup),
    .wea   (tag_we_way[0]                                  ),
    .addra (wr_way0 ? write_index : index                  ),
    .dina  (wtagvd_data                                    ),
    .douta (rtagvd_way[0]                                  )
);

data_ram u_way0_w0 (
    .clka(clk),
    .ena(bank_en0[0]),
    .wea(bank_we0[0]),
    .addra(addr0_b0),
    .dina(bank_din[ 31:  0]),
    .douta(rdata_way_line[0][ 31:  0])
);
data_ram u_way0_w4 (
    .clka(clk),
    .ena(bank_en0[1]),
    .wea(bank_we0[1]),
    .addra(addr0_b1),
    .dina(bank_din[ 63: 32]),
    .douta(rdata_way_line[0][ 63: 32])
);
data_ram u_way0_w8 (
    .clka(clk),
    .ena(bank_en0[2]),
    .wea(bank_we0[2]),
    .addra(addr0_b2),
    .dina(bank_din[ 95: 64]),
    .douta(rdata_way_line[0][ 95: 64])
);
data_ram u_way0_w12(
    .clka(clk),
    .ena(bank_en0[3]),
    .wea(bank_we0[3]),
    .addra(addr0_b3),
    .dina(bank_din[127: 96]),
    .douta(rdata_way_line[0][127: 96])
);

tagvd u_tagvd_way1(
    .clka  (clk                                            ),
    .ena   (read_go | cacop_en | refill_we | wr_way1 | lookup),
    .wea   (tag_we_way[1]                                  ),
    .addra (wr_way1 ? write_index : index                  ),
    .dina  (wtagvd_data                                    ),
    .douta (rtagvd_way[1]                                  )
);

data_ram u_way1_w0 (
    .clka(clk),
    .ena(bank_en1[0]),
    .wea(bank_we1[0]),
    .addra(addr1_b0),
    .dina(bank_din[ 31:  0]),
    .douta(rdata_way_line[1][ 31:  0])
);
data_ram u_way1_w4 (
    .clka(clk),
    .ena(bank_en1[1]),
    .wea(bank_we1[1]),
    .addra(addr1_b1),
    .dina(bank_din[ 63: 32]),
    .douta(rdata_way_line[1][ 63: 32])
);
data_ram u_way1_w8 (
    .clka(clk),
    .ena(bank_en1[2]),
    .wea(bank_we1[2]),
    .addra(addr1_b2),
    .dina(bank_din[ 95: 64]),
    .douta(rdata_way_line[1][ 95: 64])
);
data_ram u_way1_w12(
    .clka(clk),
    .ena(bank_en1[3]),
    .wea(bank_we1[3]),
    .addra(addr1_b3),
    .dina(bank_din[127: 96]),
    .douta(rdata_way_line[1][127: 96])
);

endmodule
