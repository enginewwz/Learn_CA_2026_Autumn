`timescale 1ns/1ps
// tlb.v regression: search/选半页/4MB 键/invtlb 六种 op/读写口
module tb_tlb;

reg clk = 0;
reg reset = 1;
always #5 clk = ~clk;

reg  [18:0] s0_vppn = 0;
reg         s0_va_bit12 = 0;
reg  [ 9:0] s0_asid = 0;
wire        s0_found;
wire [ 3:0] s0_index;
wire [19:0] s0_ppn;
wire [ 5:0] s0_ps;
wire [ 1:0] s0_plv, s0_mat;
wire        s0_d, s0_v;

reg  [18:0] s1_vppn = 0;
reg         s1_va_bit12 = 0;
reg  [ 9:0] s1_asid = 0;
wire        s1_found;
wire [ 3:0] s1_index;
wire [19:0] s1_ppn;
wire [ 5:0] s1_ps;
wire [ 1:0] s1_plv, s1_mat;
wire        s1_d, s1_v;

reg         invtlb_valid = 0;
reg  [ 4:0] invtlb_op = 0;

reg         we = 0;
reg  [ 3:0] w_index = 0;
reg         w_e = 0;
reg  [18:0] w_vppn = 0;
reg  [ 5:0] w_ps = 0;
reg  [ 9:0] w_asid = 0;
reg         w_g = 0;
reg  [19:0] w_ppn0 = 0;
reg  [ 1:0] w_plv0 = 0;
reg  [ 1:0] w_mat0 = 0;
reg         w_d0 = 0;
reg         w_v0 = 0;
reg  [19:0] w_ppn1 = 0;
reg  [ 1:0] w_plv1 = 0;
reg  [ 1:0] w_mat1 = 0;
reg         w_d1 = 0;
reg         w_v1 = 0;

reg  [ 3:0] r_index = 0;
wire        r_e;
wire [18:0] r_vppn;
wire [ 5:0] r_ps;
wire [ 9:0] r_asid;
wire        r_g;
wire [19:0] r_ppn0;
wire [ 1:0] r_plv0, r_mat0;
wire        r_d0, r_v0;
wire [19:0] r_ppn1;
wire [ 1:0] r_plv1, r_mat1;
wire        r_d1, r_v1;

tlb u_tlb(
    .clk            (clk            ),
    .reset          (reset          ),
    .s0_vppn        (s0_vppn        ),
    .s0_va_bit12    (s0_va_bit12    ),
    .s0_asid        (s0_asid        ),
    .s0_found       (s0_found       ),
    .s0_index       (s0_index       ),
    .s0_ppn         (s0_ppn         ),
    .s0_ps          (s0_ps          ),
    .s0_plv         (s0_plv         ),
    .s0_mat         (s0_mat         ),
    .s0_d           (s0_d           ),
    .s0_v           (s0_v           ),
    .s1_vppn        (s1_vppn        ),
    .s1_va_bit12    (s1_va_bit12    ),
    .s1_asid        (s1_asid        ),
    .s1_found       (s1_found       ),
    .s1_index       (s1_index       ),
    .s1_ppn         (s1_ppn         ),
    .s1_ps          (s1_ps          ),
    .s1_plv         (s1_plv         ),
    .s1_mat         (s1_mat         ),
    .s1_d           (s1_d           ),
    .s1_v           (s1_v           ),
    .invtlb_valid   (invtlb_valid   ),
    .invtlb_op      (invtlb_op      ),
    .we             (we             ),
    .w_index        (w_index        ),
    .w_e            (w_e            ),
    .w_vppn         (w_vppn         ),
    .w_ps           (w_ps           ),
    .w_asid         (w_asid         ),
    .w_g            (w_g            ),
    .w_ppn0         (w_ppn0         ),
    .w_plv0         (w_plv0         ),
    .w_mat0         (w_mat0         ),
    .w_d0           (w_d0           ),
    .w_v0           (w_v0           ),
    .w_ppn1         (w_ppn1         ),
    .w_plv1         (w_plv1         ),
    .w_mat1         (w_mat1         ),
    .w_d1           (w_d1           ),
    .w_v1           (w_v1           ),
    .r_index        (r_index        ),
    .r_e            (r_e            ),
    .r_vppn         (r_vppn         ),
    .r_ps           (r_ps           ),
    .r_asid         (r_asid         ),
    .r_g            (r_g            ),
    .r_ppn0         (r_ppn0         ),
    .r_plv0         (r_plv0         ),
    .r_mat0         (r_mat0         ),
    .r_d0           (r_d0           ),
    .r_v0           (r_v0           ),
    .r_ppn1         (r_ppn1         ),
    .r_plv1         (r_plv1         ),
    .r_mat1         (r_mat1         ),
    .r_d1           (r_d1           ),
    .r_v1           (r_v1           )
);

integer errors = 0;

task chk(input cond, input [255:0] msg);
    begin
        if (!cond) begin
            errors = errors + 1;
            $display("FAIL: %0s (t=%0t)", msg, $time);
        end
    end
endtask

task wr(input [3:0] idx, input [18:0] vppn, input [5:0] ps, input [9:0] asid, input g,
        input [19:0] p0, input [19:0] p1, input [1:0] plv, input [1:0] mat, input d);
    begin
        we = 1; w_index = idx; w_e = 1; w_vppn = vppn; w_ps = ps; w_asid = asid; w_g = g;
        w_ppn0 = p0; w_ppn1 = p1; w_plv0 = plv; w_plv1 = ~plv;
        w_mat0 = mat; w_mat1 = ~mat; w_d0 = d; w_d1 = ~d; w_v0 = 1; w_v1 = 0;
        @(posedge clk); #1; we = 0;
    end
endtask

task inv(input [4:0] op, input [9:0] asid, input [31:0] va);
    begin
        invtlb_valid = 1; invtlb_op = op;
        w_asid = asid; w_vppn = va[31:13];
        @(posedge clk); #1; invtlb_valid = 0;
    end
endtask

initial begin
    // reset only clears E, so found/ps/index are defined from here on; the data
    // outputs stay X until an entry has been written (they only mean something on a hit)
    #20 reset = 0; #1;
    s0_asid = 10'haa; s0_vppn = 19'h1200; #1;
    chk(s0_found === 1'b0, "empty tlb misses after reset");
    chk(s0_index === 4'd0 && s0_ps === 6'd12, "empty tlb: index/ps defined after reset");

    // 4MB entry at 3: two 2MB halves, PPN 差 0x200
    wr(3, 19'h1200, 6'd21, 10'haa, 1'b0, 20'h00100, 20'h00300, 2'd3, 2'd1, 1'b0);

    s0_asid = 10'haa; s0_va_bit12 = 1'b0;
    s0_vppn = 19'h1200; #1;
    chk(s0_found && s0_index == 4'd3 && s0_ps == 6'd21, "4MB hit, PS=21");
    chk(s0_ppn == 20'h00100 && s0_plv == 2'd3 && s0_mat == 2'd1 && s0_d == 1'b0 && s0_v == 1'b1,
        "4MB even half");
    s0_vppn = 19'h1240; #1;
    chk(s0_found && s0_ppn == 20'h00100, "4MB ignores vppn[8:0]");
    s0_vppn = 19'h1300; #1;
    chk(s0_found && s0_ppn == 20'h00300 && s0_plv == 2'd0 && s0_mat == 2'd2 && s0_d == 1'b1,
        "4MB odd half (vppn[8]=1)");
    s0_vppn = 19'h1400; #1;
    chk(!s0_found, "4MB different vppn[18:9] misses");
    s0_asid = 10'hab; s0_vppn = 19'h1200; #1;
    chk(!s0_found, "ASID mismatch misses");

    // 4KB entry at 5
    wr(5, 19'h1aab, 6'd12, 10'haa, 1'b0, 20'h0a000, 20'h0b000, 2'd0, 2'd3, 1'b1);
    s0_asid = 10'haa; s0_vppn = 19'h1aab; s0_va_bit12 = 1'b0; #1;
    chk(s0_found && s0_index == 4'd5 && s0_ps == 6'd12 && s0_ppn == 20'h0a000, "4KB even half");
    s0_va_bit12 = 1'b1; #1;
    chk(s0_ppn == 20'h0b000, "4KB odd half");
    s0_va_bit12 = 1'b0; s0_vppn = 19'h1aac; #1;
    chk(!s0_found, "4KB compares all 19 bits");

    // G=1
    wr(7, 19'h1600, 6'd12, 10'h55, 1'b1, 20'h0c000, 20'h0c200, 2'd0, 2'd0, 1'b0);
    s0_asid = 10'hff; s0_vppn = 19'h1600; #1;
    chk(s0_found && s0_index == 4'd7, "G=1 skips the ASID compare");

    // read port
    r_index = 4'd3; #1;
    chk(r_e === 1'b1 && r_vppn === 19'h1200 && r_ps === 6'd21 && r_asid === 10'haa && r_g === 1'b0,
        "read index 3");
    chk(r_ppn0 === 20'h00100 && r_ppn1 === 20'h00300 && r_v0 === 1'b1 && r_v1 === 1'b0,
        "read both halves");
    r_index = 4'd2; #1;
    chk(r_e === 1'b0, "read empty entry");

    // s1 sees the same entry
    s1_asid = 10'haa; s1_vppn = 19'h1300; #1;
    chk(s1_found && s1_index == 4'd3 && s1_ppn == 20'h00300, "s1 independent search");

    // invtlb op4: G=0 and ASID match
    inv(5'd4, 10'haa, 32'h0);
    s0_asid = 10'haa; s0_vppn = 19'h1200; #1;
    chk(!s0_found, "op4 killed the 4MB entry");
    s0_vppn = 19'h1aab; #1;
    chk(!s0_found, "op4 killed the 4KB entry");
    s0_asid = 10'hff; s0_vppn = 19'h1600; #1;
    chk(s0_found, "op4 kept the G=1 entry");

    // invtlb op6: (G or ASID) and VA
    wr(3, 19'h1200, 6'd21, 10'haa, 1'b0, 20'h00100, 20'h00300, 2'd3, 2'd1, 1'b0);
    inv(5'd6, 10'haa, 32'h0240_0000);
    s0_asid = 10'haa; s0_vppn = 19'h1200; #1;
    chk(!s0_found, "op6 killed the matching row");
    s0_asid = 10'hff; s0_vppn = 19'h1600; #1;
    chk(s0_found, "op6 kept a G=1 row with another VA");

    // reserved op clears nothing
    inv(5'd7, 10'hff, 32'h0);
    s0_asid = 10'hff; s0_vppn = 19'h1600; #1;
    chk(s0_found, "reserved op is a no-op");

    // op2 / op3
    inv(5'd2, 10'h0, 32'h0);
    s0_asid = 10'hff; s0_vppn = 19'h1600; #1;
    chk(!s0_found, "op2 killed the G=1 row");

    // E=0 write: TLBWR with NE=1
    wr(11, 19'h1b00, 6'd12, 10'h77, 1'b0, 20'h0e000, 20'h0e200, 2'd0, 2'd0, 1'b0);
    w_e = 1'b0;
    we = 1; w_index = 4'd11; w_e = 1'b0; w_vppn = 19'h1b00; w_ps = 6'd12; w_asid = 10'h77; w_g = 1'b0;
    w_ppn0 = 20'h0e000; w_ppn1 = 20'h0e200; w_v0 = 1'b1; w_v1 = 1'b1;
    @(posedge clk); #1; we = 0;
    s0_asid = 10'h77; s0_vppn = 19'h1b00; #1;
    chk(!s0_found, "w_e=0 leaves the row invalid");
    r_index = 4'd11; #1;
    chk(r_e === 1'b0, "read reports E=0");

    // last write wins over invtlb in the same cycle
    wr(12, 19'h1c00, 6'd12, 10'h77, 1'b0, 20'h0f000, 20'h0f200, 2'd0, 2'd0, 1'b0);
    invtlb_valid = 1; invtlb_op = 5'd0;
    we = 1; w_index = 4'd12; w_e = 1'b1; w_vppn = 19'h1c00; w_ps = 6'd12; w_asid = 10'h77; w_g = 1'b0;
    w_ppn0 = 20'h0f000; w_ppn1 = 20'h0f200; w_v0 = 1'b1; w_v1 = 1'b1;
    @(posedge clk); #1; we = 0; invtlb_valid = 0;
    r_index = 4'd12; #1;
    chk(r_e === 1'b1, "write wins over invtlb in the same cycle");

    if (errors == 0) $display("PASS: tb_tlb all checks");
    else             $display("FAIL: %0d error(s)", errors);
    $finish;
end

endmodule
