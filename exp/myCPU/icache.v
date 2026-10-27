module icache(
    input  wire         clk,
    input  wire         reset,

    input  wire         lookup_en,
    input  wire [31:0]  lookup_va,
    output wire         lookup_valid,
    output wire [21:0]  tagv_way0,
    output wire [21:0]  tagv_way1,
    output wire [127:0] line_way0,
    output wire [127:0] line_way1,
    output wire         lru,
    output wire         hit,
    output wire         hit_way,
    output wire [31:0]  hit_data,

    input  wire         tag_we,
    input  wire         tag_way,
    input  wire [ 7:0]  tag_index,
    input  wire [19:0]  tag_data,
    input  wire         tag_v,
    input  wire         tag_d,

    input  wire         line_we,
    input  wire         line_way,
    input  wire [ 7:0]  line_index,
    input  wire [127:0] line_data,

    input  wire         lru_we,
    input  wire [ 7:0]  lru_index,
    input  wire         lru_data
);

wire [7:0] tag_addra_way0 = tag_we  & ~tag_way  ? tag_index  : lookup_va[11:4];
wire [7:0] tag_addra_way1 = tag_we  &  tag_way  ? tag_index  : lookup_va[11:4];
wire [7:0] lin_addra_way0 = line_we & ~line_way ? line_index : lookup_va[11:4];
wire [7:0] lin_addra_way1 = line_we &  line_way ? line_index : lookup_va[11:4];

tagvd u_tagvd_way0(
    .clka  (clk                    ),
    .wea   (tag_we  & ~tag_way     ),
    .addra (tag_addra_way0         ),
    .dina  ({tag_data, tag_v, tag_d}),
    .douta (tagv_way0              )
);

inst_ram u_line_way0(
    .clka  (clk                    ),
    .wea   (line_we & ~line_way    ),
    .addra (lin_addra_way0         ),
    .dina  (line_data              ),
    .douta (line_way0              )
);

tagvd u_tagvd_way1(
    .clka  (clk                    ),
    .wea   (tag_we  &  tag_way     ),
    .addra (tag_addra_way1         ),
    .dina  ({tag_data, tag_v, tag_d}),
    .douta (tagv_way1              )
);

inst_ram u_line_way1(
    .clka  (clk                    ),
    .wea   (line_we &  line_way    ),
    .addra (lin_addra_way1         ),
    .dina  (line_data              ),
    .douta (line_way1              )
);

assign lookup_valid = 1'b0;
assign lru          = 1'b0;
assign hit          = 1'b0;
assign hit_way      = 1'b0;
assign hit_data     = 32'b0;

endmodule
