module decoder_2_4(
    input  wire [ 1:0] in,
    output wire [ 3:0] out
);

genvar i;
generate for (i=0; i<4; i=i+1) begin : gen_for_dec_2_4
    assign out[i] = (in == i);
end endgenerate

endmodule


module decoder_4_16(
    input  wire [ 3:0] in,
    output wire [15:0] out
);

genvar i;
generate for (i=0; i<16; i=i+1) begin : gen_for_dec_4_16
    assign out[i] = (in == i);
end endgenerate

endmodule


module decoder_5_32(
    input  wire [ 4:0] in,
    output wire [31:0] out
);

genvar i;
generate for (i=0; i<32; i=i+1) begin : gen_for_dec_5_32
    assign out[i] = (in == i);
end endgenerate

endmodule


module decoder_6_64(
    input  wire [ 5:0] in,
    output wire [63:0] out
);

genvar i;
generate for (i=0; i<64; i=i+1) begin : gen_for_dec_6_64
    assign out[i] = (in == i);
end endgenerate

endmodule


// 3:2 carry-save compressor; a + b + c == s + co (co is already shifted left)
module csa64(
    input  wire [63:0] a,
    input  wire [63:0] b,
    input  wire [63:0] c,
    output wire [63:0] s,
    output wire [63:0] co
);
    assign s  = a ^ b ^ c;
    assign co = ((a & b) | (a & c) | (b & c)) << 1;
endmodule

// 3:2 carry-save compressor; a + b + c == s + co (co is already shifted left)
