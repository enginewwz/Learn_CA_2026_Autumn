module icache(
    input  wire         clk,
    input  wire         reset,

    input  wire [11:0]  va,     // {index, offset}
    input  wire [19:0]  pt,     // should expand if it is runing at 4MB mode

    // read
    input  wire         read_req,   // issue a lookup, data is valid one cycle later
    input  wire         read_use,   // this cycle consumes the previous lookup's result
    output wire         read_hit,
    output wire [31:0]  read_data,

    // cacop
    input  wire         cacop_en,
    input  wire         cacop_hit,      // 1 = hit invalidate, 0 = direct index

    // refill
    input  wire         refill_we,
    input  wire [127:0] refill_data
);

wire [ 7:0] index;
wire hit_way [1:0];
wire en;
wire tag_we_way [1:0];

wire [127:0] rdata_way_line [1:0];
wire [31:0] rdata_way_word [1:0];
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

wire [ 7:0] last_index;
reg [255:0] lru_table;
reg  [11:0] last_va;

assign en = read_req | cacop_en | refill_we;

assign index      = va[11:4];
assign last_index = last_va[11:4];

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

assign read_hit = hit_way[0] | hit_way[1];
assign read_data = hit_way[0] ? rdata_way_word[0] : rdata_way_word [1];

// direct index addressing: way = VA[Way-1:0], i.e. VA[0] for the two ways;
// hit invalidate instead picks the way whose tag matches
assign cacop_way0 = ~ va[0];
assign cacop_way1 =   va[0];

assign cacop_we_way [0] = cacop_en & (cacop_hit ? hit_way[0] : cacop_way0);
assign cacop_we_way [1] = cacop_en & (cacop_hit ? hit_way[1] : cacop_way1);

assign cacop_tagvd_data = 22'b0;
assign refill_tagvd_data = {pt, 2'b10};

assign wtagvd_data = refill_we ? refill_tagvd_data : cacop_tagvd_data;

assign refill_we_way[0] = refill_we & ~ lru_table[index];
assign refill_we_way[1] = refill_we & lru_table[index];

assign tag_we_way[0] = cacop_we_way[0] | refill_we_way[0];
assign tag_we_way[1] = cacop_we_way[1] | refill_we_way[1];

// the lookup address / offset of the access whose response is consumed this cycle
always @(posedge clk) begin
    if (read_req) last_va <= va;
end

// LRU: only the access that consumes its lookup result may touch it
always @(posedge clk) begin
    if (reset)                          lru_table <= 256'b0;
    else if (read_use & read_hit)       lru_table[last_index] <= hit_way[0];
    else if (refill_we)                 lru_table[index] <= ~ lru_table[index];
end

tagvd u_tagvd_way0(
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

tagvd u_tagvd_way1(
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
