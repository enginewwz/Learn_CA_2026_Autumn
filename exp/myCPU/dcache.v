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

    // miss / cacop view: the one line this access would evict / invalidate.
    // lookup_d is valid on the response cycle, lookup_pa / lookup_line while
    // `lookup` has read the line out; the way itself is chosen inside the cache
    input  wire         lookup,         // 1: also read the other banks -> full line
    output wire         lookup_d,       // d bit of that line
    output wire [31:0]  lookup_pa,      // {tag, index, 4'b0}
    output wire [127:0] lookup_line,    // {w12, w8, w4, w0}

    // write
    input  wire         write_req,
    input  wire [11:0]  write_va,
    input  wire [31:0]  write_data,     // byte-merged by the mmu
    output wire         read_write_conflict,

    // cacop
    input  wire         cacop_en,
    input  wire         cacop_hit_mode,      // 1 = hit invalidate, 0 = direct index

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
// FPGA: no reset on the table; the power-up value is the configuration bit
// instead.  It must still be defined for 4-state simulation.
initial lru_table = 256'b0;

reg  [11:0] last_va;
reg         lru_r;      // lru_table[index] of the lookup in flight

// read / write conflict, write port and per-bank enables
wire        same_index, same_bank, need_d_we, read_go;
wire [ 3:0] rd_bank, rd_en;
wire        wr_way0, wr_way1, wr_tag_way0, wr_tag_way1;
wire [ 3:0] wr_bank_way0, wr_bank_way1;
wire [ 3:0] bank_en0, bank_en1, bank_we0, bank_we1;
wire [ 7:0] addr0_b0, addr0_b1, addr0_b2, addr0_b3;
wire [ 7:0] addr1_b0, addr1_b1, addr1_b2, addr1_b3;
wire [127:0] bank_din;

wire [ 1:0] rword_offset;

wire        lookup_sel_wire;
reg         lookup_sel_reg;

assign index       = va[11:4];
assign write_index = write_va[11:4];
assign write_bank  = write_va[3:2];
assign last_index  = last_va[11:4];

// read / write in the same cycle
//   ok   : same set, different bank -- the data RAMs are per bank, and the
//          tagvd douta still holds this very set (only v/d change with a store)
//   stall: different set -- the tagvd is a *single port* NO_CHANGE RAM, the
//          store's tag write would swallow this cycle's lookup read and the
//          next access would compare against another set's tags
//   stall: same set, same bank -- one data RAM cannot be read and written
// on conflict the write wins, the read is dropped (read_go = 0)
assign same_index = (index == write_index);
assign same_bank  = (va[3:2] == write_va[3:2]);
assign need_d_we  = ~ rtagvd_d[hit_way[1]];

assign read_write_conflict = read_req & write_req & (~ same_index | same_bank);
assign read_go = read_req & ~ read_write_conflict;

// bank select.  A line lives in exactly one way: a lookup hits at most one way
// and a refill only writes the LRU way of a miss.  A store is written on the
// response cycle of its own lookup, so the way is simply the hit way held by
// the tag compare (hit_way[1]); read_data may key off hit_way[0].
assign rd_bank      = 4'b0001 << va[3:2];
assign wr_way0      = write_req & ~ hit_way[1];
assign wr_way1      = write_req &   hit_way[1];
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
assign rword_offset = last_va[3:2];
assign rdata_way_word[0] = rdata_way_line[0][rword_offset * 32 +: 32]; 
assign rdata_way_word[1] = rdata_way_line[1][rword_offset * 32 +: 32];

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

assign cacop_we_way [0] = cacop_en & (cacop_hit_mode ? hit_way[0] : cacop_way0);
assign cacop_we_way [1] = cacop_en & (cacop_hit_mode ? hit_way[1] : cacop_way1);

assign cacop_tagvd_data  = 22'b0;                           // invalidate: clear tag/v/d
assign refill_tagvd_data = {pt, 1'b1, refill_dirty};         // {tag, v=1, d}
assign write_tagvd_data  = {rtagvd_tag[hit_way[1]], 2'b11}; // {tag, v=1, d=1}

assign wtagvd_data = refill_we ? refill_tagvd_data
                   : write_req ? write_tagvd_data
                   :             cacop_tagvd_data;

// lru_table[i] is a 256:1 mux; the refill way uses the copy registered with the
// lookup (below), so the way/RAM enable does not wait for that mux
assign refill_we_way[0] = refill_we & ~ lru_r;
assign refill_we_way[1] = refill_we &   lru_r;

assign tag_we_way[0] = cacop_we_way[0] | refill_we_way[0] | wr_tag_way0;
assign tag_we_way[1] = cacop_we_way[1] | refill_we_way[1] | wr_tag_way1;

// the lookup address / offset of the access whose response is consumed this cycle
always @(posedge clk) begin
    if (read_go) last_va <= va;
end

// read the LRU bit together with the tag / data RAMs: hit way / lookup way, the
// store way, the refill way and every RAM address/enable then come from a flop
// instead of from the 256:1 LRU mux inside the response cycle.  If the response
// this cycle updates the LRU of the set being looked up, take the new value.
always @(posedge clk) begin
    if (reset)         lru_r <= 1'b0;
    else if (read_go)  lru_r <= (read_use & read_hit & (last_index == index))
                              ? hit_way[0] : lru_table[index];
end

// LRU: only the access that consumes its lookup result may touch it
always @(posedge clk) begin
    if (read_use & read_hit)            lru_table[last_index] <= hit_way[0];
    else if (refill_we)                 lru_table[index] <= ~ lru_table[index];
end

// way select: hit -> hit way, miss -> LRU way, and for a cacop the way it
// addresses (VA[Way-1:0] = VA[0] for store tag / index invalidate, the hit way
// for hit invalidate).  Held after the response cycle so the write-back
// sequence keeps using the way that was chosen when the line was read out.
assign lookup_sel_wire = cacop_en ? (cacop_hit_mode ? hit_way[1] : va[0])
                    : lru_r;    // read / write hit case should not need to look up

always @(posedge clk) begin
    if (reset)         lookup_sel_reg <= 1'b0;
    else if (read_use) lookup_sel_reg <= lookup_sel_wire;
end

// view for the mmu: only the lookup line.  lookup_d is read while the result is
// consumed (read_use, i.e. the cacop act / a miss), lookup_pa and lookup_line
// only one cycle later in cache_lookup, where lookup_sel_reg still holds the way.
// va is the line of interest in both, so it gives the index.
assign lookup_d    = lookup_sel_wire   ? rtagvd_d[1]    : rtagvd_d[0];
assign lookup_pa   = {lookup_sel_reg ? rtagvd_tag[1] : rtagvd_tag[0], last_index, 4'b0};
assign lookup_line = lookup_sel_reg ? rdata_way_line[1] : rdata_way_line[0];

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
