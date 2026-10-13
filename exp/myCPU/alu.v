module alu(
  input  wire [11:0] alu_op,
  input  wire [31:0] src1,
  input  wire [31:0] src2,
  output wire [31:0] alu_result,
  output wire        lt_o,       // signed   src1 < src2, tapped off the borrow tree
  output wire        ltu_o       // unsigned src1 < src2, = ltu32 (borrow tree root)
);

// control code decomposition: one bit per operation
wire op_add;   //add operation
wire op_sub;   //sub operation
wire op_slt;   //signed compared and set less than
wire op_sltu;  //unsigned compared and set less than
wire op_and;   //bitwise and
wire op_nor;   //bitwise nor
wire op_or;    //bitwise or
wire op_xor;   //bitwise xor
wire op_sll;   //logic left shift
wire op_srl;   //logic right shift
wire op_sra;   //arithmetic right shift
wire op_lui;   //Load Upper Immediate

assign op_add  = alu_op[ 0];
assign op_sub  = alu_op[ 1];
assign op_slt  = alu_op[ 2];
assign op_sltu = alu_op[ 3];
assign op_and  = alu_op[ 4];
assign op_nor  = alu_op[ 5];
assign op_or   = alu_op[ 6];
assign op_xor  = alu_op[ 7];
assign op_sll  = alu_op[ 8];
assign op_srl  = alu_op[ 9];
assign op_sra  = alu_op[10];
assign op_lui  = alu_op[11];

// carry-lookahead combine of two adjacent (generate, propagate) groups:
//   {G,P} = {Ghi | (Phi & Glo), Phi & Plo}
function [1:0] cla;
    input ghi, phi, glo, plo;
    begin
        cla[1] = ghi | (phi & glo);
        cla[0] = phi & plo;
    end
endfunction

genvar g;

wire [31:0] and_result;
wire [31:0] nor_result;
wire [31:0] or_result;
wire [31:0] xor_result;
wire [31:0] lui_result;
wire [31:0] sll_result;
wire [63:0] sr64_result;   //only [31:0] is used below
wire [31:0] sr_result;

assign and_result = src1 & src2;
assign or_result  = src1 | src2;
assign nor_result = ~or_result;
assign xor_result = src1 ^ src2;
assign lui_result = src2;

// SLL result
assign sll_result = src1 << src2[4:0];                                    //rj << i5

// SRL, SRA result
assign sr64_result = {{32{op_sra & src1[31]}}, src1[31:0]} >> src2[4:0]; //rj >> i5
assign sr_result   = sr64_result[31:0];

wire [31:0] early_result =
      ({32{op_and         }} & and_result)
    | ({32{op_nor         }} & nor_result)
    | ({32{op_or          }} & or_result)
    | ({32{op_xor         }} & xor_result)
    | ({32{op_lui         }} & lui_result)
    | ({32{op_sll         }} & sll_result)
    | ({32{op_srl | op_sra}} & sr_result);

// unsigned compare tree (a <u b), and the low-half borrow of sub
//
// For bit i of a subtraction let
//     g_i = ~a_i & b_i     (this bit generates a borrow)
//     p_i =  a_i ~^ b_i    (this bit propagates a borrow)
// then brw_{i+1} = g_i | (p_i & brw_i), brw_0 = 0, and the group generate
//     G(0..15) = borrow out of the low 16 bits
//     G(0..31) = borrow out of all 32 bits = (a <u b)
// The tree starts from 2-bit groups (one LUT each) and combines
// 2 -> 4 -> 8 -> 16 -> 32, i.e. five LUT levels
wire [1:0] brw2 [0:15];    //aligned 2-bit groups
wire [1:0] brw4 [0: 7];
wire [1:0] brw8 [0: 3];
wire [1:0] brw16_1;

generate
    for (g = 0; g < 16; g = g + 1) begin : g_brw2
        assign brw2[g] = cla(~src1[2*g+1] & src2[2*g+1],
                              src1[2*g+1] ~^ src2[2*g+1],
                             ~src1[2*g  ] & src2[2*g  ],
                              src1[2*g  ] ~^ src2[2*g  ]);
    end
    for (g = 0; g < 8; g = g + 1) begin : g_brw4
        assign brw4[g] = cla(brw2[2*g+1][1], brw2[2*g+1][0],
                             brw2[2*g  ][1], brw2[2*g  ][0]);
    end
    for (g = 0; g < 4; g = g + 1) begin : g_brw8
        assign brw8[g] = cla(brw4[2*g+1][1], brw4[2*g+1][0],
                             brw4[2*g  ][1], brw4[2*g  ][0]);
    end
endgenerate

wire brw_lo16;   //G(0..15): borrow out of the low half, used by sub below
wire ltu32;      //G(0..31): a <u b

assign brw_lo16 = brw8[1][1] | (brw8[1][0] & brw8[0][1]);
assign brw16_1  = cla(brw8[3][1], brw8[3][0], brw8[2][1], brw8[2][0]);
assign ltu32    = brw16_1[1] | (brw16_1[0] & brw_lo16);

// add and sub
wire [1:0] car2 [0:7];
wire [1:0] car4 [0:3];
wire       car_lo8;
wire       car_hi8;
wire       car_hi8_p;
wire       carry16;

generate
    for (g = 0; g < 8; g = g + 1) begin : g_car2
        assign car2[g] = cla(src1[2*g+1] & src2[2*g+1],
                             src1[2*g+1] ^ src2[2*g+1],
                             src1[2*g  ] & src2[2*g  ],
                             src1[2*g  ] ^ src2[2*g  ]);
    end
    for (g = 0; g < 4; g = g + 1) begin : g_car4
        assign car4[g] = cla(car2[2*g+1][1], car2[2*g+1][0],
                             car2[2*g  ][1], car2[2*g  ][0]);
    end
endgenerate

assign car_lo8   = car4[1][1] | (car4[1][0] & car4[0][1]);   //G(0..7)
assign car_hi8   = car4[3][1] | (car4[3][0] & car4[2][1]);   //G(8..15)
assign car_hi8_p = car4[3][0] & car4[2][0];                  //P(8..15)
assign carry16   = car_hi8 | (car_hi8_p & car_lo8);          //G(0..15)

// add
wire [15:0] add_lo;
wire [15:0] add_hi;
(* keep = "true", dont_touch = "true" *) wire [15:0] add_hi0;   //high half if no carry
(* keep = "true", dont_touch = "true" *) wire [15:0] add_hi1;   //high half with carry
wire [31:0] sum_add;

assign add_lo  = src1[15:0]  + src2[15:0];
assign add_hi0 = src1[31:16] + src2[31:16];
assign add_hi1 = src1[31:16] + src2[31:16] + 16'd1;
assign add_hi  = carry16 ? add_hi1 : add_hi0;
assign sum_add = {add_hi, add_lo};

// sub
wire [15:0] sub_lo;
wire [15:0] sub_hi;
(* keep = "true", dont_touch = "true" *) wire [15:0] sub_hi0;   //high half if no borrow
(* keep = "true", dont_touch = "true" *) wire [15:0] sub_hi1;   //high half with borrow
wire [31:0] sum_sub;

assign sub_lo  = src1[15:0]  - src2[15:0];
assign sub_hi0 = src1[31:16] - src2[31:16];
assign sub_hi1 = src1[31:16] - src2[31:16] - 16'd1;
assign sub_hi  = brw_lo16 ? sub_hi1 : sub_hi0;
assign sum_sub = {sub_hi, sub_lo};

wire cmp_bit;

assign lt_o  = (src1[31] & ~src2[31])
             | ((src1[31] ~^ src2[31]) & ltu32);
assign ltu_o = ltu32;

assign cmp_bit = (op_slt & lt_o) | (op_sltu & ltu32);

assign alu_result = ({32{op_add}} & sum_add)
                  | ({32{op_sub}} & sum_sub)
                  | {31'b0, cmp_bit}
                  | early_result;

endmodule
