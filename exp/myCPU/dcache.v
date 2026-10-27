module dcache(
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

    input  wire [ 3:0]  word_we,
    input  wire         word_way,
    input  wire [ 7:0]  word_index,
    input  wire [127:0] word_data,

    input  wire         lru_we,
    input  wire [ 7:0]  lru_index,
    input  wire         lru_data
);

wire [7:0] tag_addra_way0 = tag_we & ~tag_way ? tag_index : lookup_va[11:4];
wire [7:0] tag_addra_way1 = tag_we &  tag_way ? tag_index : lookup_va[11:4];

wire [3:0] word_we_way0 = word_we & {4{~word_way}};
wire [3:0] word_we_way1 = word_we & {4{ word_way}};

wire [7:0] dat_addra_way0 = (|word_we_way0) ? word_index : lookup_va[11:4];
wire [7:0] dat_addra_way1 = (|word_we_way1) ? word_index : lookup_va[11:4];

wire [31:0] w0_way0,  w4_way0,  w8_way0,  w12_way0;
wire [31:0] w0_way1,  w4_way1,  w8_way1,  w12_way1;

assign line_way0 = {w12_way0, w8_way0, w4_way0, w0_way0};
assign line_way1 = {w12_way1, w8_way1, w4_way1, w0_way1};

tagvd u_tagvd_way0(
    .clka  (clk                    ),
    .wea   (tag_we  & ~tag_way     ),
    .addra (tag_addra_way0         ),
    .dina  ({tag_data, tag_v, tag_d}),
    .douta (tagv_way0              )
);

data_ram u_way0_w0 (
    .clka(clk), .wea(word_we_way0[0]), .addra(dat_addra_way0),
    .dina(word_data[ 31:  0]), .douta(w0_way0 )
);
data_ram u_way0_w4 (
    .clka(clk), .wea(word_we_way0[1]), .addra(dat_addra_way0),
    .dina(word_data[ 63: 32]), .douta(w4_way0 )
);
data_ram u_way0_w8 (
    .clka(clk), .wea(word_we_way0[2]), .addra(dat_addra_way0),
    .dina(word_data[ 95: 64]), .douta(w8_way0 )
);
data_ram u_way0_w12(
    .clka(clk), .wea(word_we_way0[3]), .addra(dat_addra_way0),
    .dina(word_data[127: 96]), .douta(w12_way0)
);

tagvd u_tagvd_way1(
    .clka  (clk                    ),
    .wea   (tag_we  &  tag_way     ),
    .addra (tag_addra_way1         ),
    .dina  ({tag_data, tag_v, tag_d}),
    .douta (tagv_way1              )
);
-
data_ram u_way1_w0 (
    .clka(clk), .wea(word_we_way1[0]), .addra(dat_addra_way1),
    .dina(word_data[ 31:  0]), .douta(w0_way1 )
);
data_ram u_way1_w4 (
    .clka(clk), .wea(word_we_way1[1]), .addra(dat_addra_way1),
    .dina(word_data[ 63: 32]), .douta(w4_way1 )
);
data_ram u_way1_w8 (
    .clka(clk), .wea(word_we_way1[2]), .addra(dat_addra_way1),
    .dina(word_data[ 95: 64]), .douta(w8_way1 )
);
data_ram u_way1_w12(
    .clka(clk), .wea(word_we_way1[3]), .addra(dat_addra_way1),
    .dina(word_data[127: 96]), .douta(w12_way1)
);

assign lookup_valid = 1'b0;
assign lru          = 1'b0;
assign hit          = 1'b0;
assign hit_way      = 1'b0;
assign hit_data     = 32'b0;

endmodule
