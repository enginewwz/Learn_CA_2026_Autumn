// 2-way, 64-set, 64-byte-line (16 words) instruction cache = 8 KB.
//
// Why 64 sets: the fetch lookup is a VIPT one -- the index comes from nextpc
// one cycle before the translation is available, so the index bits must stay
// inside the 4 KB page offset.  With a 64 B line the page offset splits into
//   va[11:6] index (6 bits, 64 sets) | va[5:2] word (4 bits) | va[1:0] byte
// and the tag compare stays the 4 KB page number (pt), exactly as before.
//
// The line RAM (inst_ram) is 64 x 512 (WRITE_FIRST) and the tag RAM (tagvd64) is
// 64 x 22 (tag + valid + dirty, NO_CHANGE) -- the dcache keeps its own 256 x 22
// tagvd.  local_testbench/cache_ram_sim.v has the iverilog stand-ins and
// exp/soc_verify/soc_axi/run_vivado/create_project.tcl generates the real IPs
// with the same geometry.
module icache(
    input  wire         clk,
    input  wire         reset,

    input  wire [11:0]  va,     // {index[11:6], word[5:2], byte[1:0]}
    input  wire [19:0]  pt,     // 4 KB page number; expand for larger pages

    // read
    input  wire         read_req,   // issue a lookup, data is valid one cycle later
    input  wire         read_use,   // this cycle consumes the previous lookup's result
    output wire         read_hit,
    output wire [31:0]  read_data_0,    // word at last_va[5:2]       (fetched pc)
    output wire [31:0]  read_data_1,    // word at last_va[5:2] + 1   (pc + 4 of a pair)

    // cacop
    input  wire         cacop_lookup,   // issue the tag read of the cacop (phase 1)
    input  wire         cacop_en,
    input  wire         cacop_hit_mode,      // 1 = hit invalidate, 0 = direct index

    // refill
    input  wire         refill_we,
    input  wire [511:0] refill_data
);

// geometry
localparam LINE_W = 512;        // 64 B line
localparam IDX_W  = 6;          // 64 sets
localparam WORD_W = 4;          // 16 words per line

wire [IDX_W-1:0]  index;
wire hit_way [1:0];
wire en;
wire tag_we_way [1:0];

wire [LINE_W-1:0] rdata_way_line [1:0];
wire [21:0] rtagvd_way [1:0];
wire [19:0] rtagvd_tag [1:0];
wire rtagvd_v [1:0];
wire rtagvd_d [1:0];    // not used

wire cacop_we_way [1:0];
wire cacop_way0, cacop_way1;
wire [21:0] cacop_tagvd_data;

wire refill_we_way [1:0];
wire [21:0] refill_tagvd_data;

wire [21:0] wtagvd_data;

wire [IDX_W-1:0]  last_index;
reg  [2**IDX_W-1:0] lru_table;
reg  [11:0] last_va;
reg         lru_r;      // lru_table[index] of the lookup in flight

// word select inside the line: the fetched instruction, and the second one of a
// pair (wraps to 0 at the line end; the caller gates that case out)
wire [WORD_W-1:0] last_wsel_0;
wire [WORD_W-1:0] last_wsel_1;

assign en = read_req | cacop_lookup | cacop_en | refill_we;

assign index      = va[11:6];
assign last_index = last_va[11:6];

assign last_wsel_0 = last_va[5:2];
assign last_wsel_1 = last_va[5:2] + 1'b1;

// 16:1 word mux per way (variable part select keeps this to one mux instead of a
// mask-and-or per word)
reg [31:0] rdata_way_word_0 [1:0];
reg [31:0] rdata_way_word_1 [1:0];
integer k;
always @(*) begin
    rdata_way_word_0[0] = 32'b0;
    rdata_way_word_0[1] = 32'b0;
    rdata_way_word_1[0] = 32'b0;
    rdata_way_word_1[1] = 32'b0;
    for (k = 0; k < 2**WORD_W; k = k + 1) begin
        if (last_wsel_0 == k[WORD_W-1:0]) begin
            rdata_way_word_0[0] = rdata_way_line[0][k*32 +: 32];
            rdata_way_word_0[1] = rdata_way_line[1][k*32 +: 32];
        end
        if (last_wsel_1 == k[WORD_W-1:0]) begin
            rdata_way_word_1[0] = rdata_way_line[0][k*32 +: 32];
            rdata_way_word_1[1] = rdata_way_line[1][k*32 +: 32];
        end
    end
end

assign {rtagvd_tag[0], rtagvd_v[0], rtagvd_d[0]} = rtagvd_way[0];
assign {rtagvd_tag[1], rtagvd_v[1], rtagvd_d[1]} = rtagvd_way[1];

assign hit_way[0] = (rtagvd_tag[0] == pt) & rtagvd_v[0];
assign hit_way[1] = (rtagvd_tag[1] == pt) & rtagvd_v[1];

assign read_hit = hit_way[0] | hit_way[1];
assign read_data_0 = hit_way[0] ? rdata_way_word_0[0] : rdata_way_word_0 [1];
assign read_data_1 = hit_way[0] ? rdata_way_word_1[0] : rdata_way_word_1 [1];

// direct index addressing: way = VA[Way-1:0], i.e. VA[0] for the two ways;
// hit invalidate instead picks the way whose tag matches
assign cacop_way0 = ~ va[0];
assign cacop_way1 =   va[0];

assign cacop_we_way [0] = cacop_en & (cacop_hit_mode ? hit_way[0] : cacop_way0);
assign cacop_we_way [1] = cacop_en & (cacop_hit_mode ? hit_way[1] : cacop_way1);

assign cacop_tagvd_data = 22'b0;
assign refill_tagvd_data = {pt, 2'b10};

assign wtagvd_data = refill_we ? refill_tagvd_data : cacop_tagvd_data;

// lru_table[i] is a 64:1 mux; the refill way uses the copy registered with the
// lookup (below), so the way/RAM enable does not wait for that mux
assign refill_we_way[0] = refill_we & ~ lru_r;
assign refill_we_way[1] = refill_we & lru_r;

assign tag_we_way[0] = cacop_we_way[0] | refill_we_way[0];
assign tag_we_way[1] = cacop_we_way[1] | refill_we_way[1];

// the lookup address / offset of the access whose response is consumed this cycle
always @(posedge clk) begin
    if (read_req) last_va <= va;
end

// read the LRU bit together with the tag / data RAMs, one cycle before the
// refill needs it.  If the response this cycle updates the LRU of the very set
// being looked up, take that post-update value.
always @(posedge clk) begin
    if (reset)         lru_r <= 1'b0;
    else if (read_req) lru_r <= (read_use & read_hit & (last_index == index))
                              ? hit_way[0] : lru_table[index];
end

// LRU: only the access that consumes its lookup result may touch it
always @(posedge clk) begin
    if (reset)                          lru_table <= {2**IDX_W{1'b0}};
    else if (read_use & read_hit)       lru_table[last_index] <= hit_way[0];
    else if (refill_we)                 lru_table[index] <= ~ lru_table[index];
end

tagvd64 u_tagvd_way0(
    .clka  (clk                 ),
    .ena   (en                  ),
    .wea   (tag_we_way[0]       ),
    .addra (index               ),
    .dina  (wtagvd_data         ),
    .douta (rtagvd_way[0]       )
);

inst_ram u_line_way0(
    .clka  (clk                 ),
    .ena   (en                  ),
    .wea   (refill_we_way[0]    ),
    .addra (index               ),
    .dina  (refill_data         ),
    .douta (rdata_way_line[0]   )
);

tagvd64 u_tagvd_way1(
    .clka  (clk                 ),
    .ena   (en                  ),
    .wea   (tag_we_way[1]       ),
    .addra (index               ),
    .dina  (wtagvd_data         ),
    .douta (rtagvd_way[1]       )
);

inst_ram u_line_way1(
    .clka  (clk                 ),
    .ena   (en                  ),
    .wea   (refill_we_way[1]    ),
    .addra (index               ),
    .dina  (refill_data         ),
    .douta (rdata_way_line[1]   )
);

endmodule
