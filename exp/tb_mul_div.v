// Standalone self-checking testbench for mul.v and div.v.
// Not part of the CPU build (the testbench Makefile only globs myCPU/*.v).
//
//   iverilog -g2005 -o /tmp/tb_mul_div impl/myCPU/mul.v impl/myCPU/div.v impl/tb_mul_div.v
//   vvp /tmp/tb_mul_div
`timescale 1ns/1ps

module tb_mul_div;

reg clk = 1'b0;
always #5 clk = ~clk;
reg reset = 1'b1;

// ---------------- mul ----------------
reg         m_valid_in, m_stall;
reg  [31:0] m_a, m_b;
reg  [ 1:0] m_op;
wire [ 6:0] m_md_op  = 7'b1 << m_op;      // one-hot: 0=mul.w 1=mulh.w 2=mulh.wu
wire [ 2:0] m_allow  = m_stall ? 3'b000 : 3'b111;  // mul.v now takes allow
wire [31:0] m_sel;

mul u_mul(
    .clk   (clk     ),
    .reset (reset   ),
    .allow (m_allow ),
    .src1  (m_a     ),
    .src2  (m_b     ),
    .md_op (m_md_op ),
    .result(m_sel   )
);

// ---------------- div ----------------
reg         d_start, d_signed;
reg  [31:0] d_a, d_b;
wire        d_busy, d_done;
wire [31:0] d_q, d_r;

div u_div(
    .clk       (clk     ),
    .reset     (reset   ),
    .start     (d_start ),
    .is_signed (d_signed),
    .src1      (d_a     ),
    .src2      (d_b     ),
    .busy      (d_busy  ),
    .done      (d_done  ),
    .quot      (d_q     ),
    .rem       (d_r     )
);

integer errors = 0;
integer checks = 0;
integer k;

function [31:0] mul_exp;
    input [31:0] x, y;
    input [ 1:0] o;
    reg signed [63:0] sp;
    reg        [63:0] up;
    begin
        sp = $signed(x) * $signed(y);
        up = {32'b0, x} * {32'b0, y};
        mul_exp = (o == 2'd0) ? up[31:0] : (o == 2'd1) ? sp[63:32] : up[63:32];
    end
endfunction

// mirrored expectation pipeline: 3 internal stages, frozen by stall.
// mul.v has no valid; its result register is written by internal stage 2, so the
// expected result lines up 3 cycles after the inputs.
reg        ev0, ev1, ev2;
reg [31:0] eq0, eq1, eq2;
always @(posedge clk) begin
    if (reset) begin
        ev0 <= 1'b0; ev1 <= 1'b0; ev2 <= 1'b0;
    end
    else if (!m_stall) begin
        ev0 <= m_valid_in;
        eq0 <= mul_exp(m_a, m_b, m_op);
        ev1 <= ev0;
        eq1 <= eq0;
        ev2 <= ev1;
        eq2 <= eq1;
    end
end

always @(posedge clk) begin
    #1;
    if (!reset && ev2) begin
        checks = checks + 1;
        if (m_sel !== eq2) begin
            errors = errors + 1;
            $display("MUL ERROR t=%0t exp=%h got=%h (a=%h b=%h op=%0d)",
                     $time, eq2, m_sel, m_a, m_b, m_op);
        end
    end
end

reg [31:0] eq_q, eq_r;

task do_div;
    input [31:0] a, b;
    input        sgn;
    begin
        @(negedge clk);
        d_a = a; d_b = b; d_signed = sgn; d_start = 1'b1;
        @(negedge clk);
        d_start = 1'b0;
        while (!d_done) @(negedge clk);
        checks = checks + 1;
        if (b == 32'd0) begin
            eq_q = 32'hffffffff;
            eq_r = a;
        end
        else if (sgn) begin
            eq_q = $signed(a) / $signed(b);
            eq_r = $signed(a) % $signed(b);
        end
        else begin
            eq_q = a / b;
            eq_r = a % b;
        end
        if (d_q !== eq_q || d_r !== eq_r) begin
            errors = errors + 1;
            $display("DIV ERROR t=%0t a=%h b=%h sgn=%b exp q=%h r=%h got q=%h r=%h",
                     $time, a, b, sgn, eq_q, eq_r, d_q, d_r);
        end
    end
endtask

reg [31:0] ra, rb;

initial begin
    m_valid_in = 1'b0; m_stall = 1'b0; m_a = 32'd0; m_b = 32'd0; m_op = 2'd0;
    d_start = 1'b0; d_signed = 1'b0; d_a = 32'd0; d_b = 32'd0;
    repeat (4) @(posedge clk);
    reset = 1'b0;
    repeat (2) @(posedge clk);

    // ---- mul: random stream with random stalls ----
    @(negedge clk);
    for (k = 0; k < 400; k = k + 1) begin
        m_valid_in = 1'b1;
        m_a    = $random;
        m_b    = $random;
        m_op   = k % 3;
        m_stall = (($random % 7) == 0);
        @(negedge clk);
    end
    m_stall = 1'b0;
    repeat (6) @(negedge clk);
    m_valid_in = 1'b0;
    repeat (6) @(negedge clk);

    // ---- mul: directed edge cases ----
    begin : edge_mul
        integer e, o;
        reg [31:0] ea [0:13];
        reg [31:0] eb [0:13];
        ea[0]=32'h00000000; eb[0]=32'hffffffff;
        ea[1]=32'h80000000; eb[1]=32'h80000000;
        ea[2]=32'h7fffffff; eb[2]=32'h7fffffff;
        ea[3]=32'hffffffff; eb[3]=32'hffffffff;
        ea[4]=32'h0000ffff; eb[4]=32'h00010000;
        ea[5]=32'h80000000; eb[5]=32'hffffffff;
        ea[6]=32'h00000001; eb[6]=32'h00000001;
        ea[7]=32'hdeadbeef; eb[7]=32'h12345678;
        ea[8]=32'h55555555; eb[8]=32'haaaaaaaa;
        ea[9]=32'h24924924; eb[9]=32'h92492492;
        ea[10]=32'h00000003; eb[10]=32'hfffffffd;
        ea[11]=32'h40000001; eb[11]=32'hc0000003;
        ea[12]=32'h0000aaaa; eb[12]=32'h00005555;
        ea[13]=32'hffff0001; eb[13]=32'h0001ffff;
        for (e = 0; e < 14; e = e + 1) begin
            for (o = 0; o < 3; o = o + 1) begin
                m_a = ea[e]; m_b = eb[e]; m_op = o[1:0];
                m_valid_in = 1'b1; m_stall = 1'b0;
                @(negedge clk);
            end
        end
        m_valid_in = 1'b0;
        repeat (6) @(negedge clk);
    end

    // ---- div: directed ----
    do_div(32'h00000007, 32'h00000002, 1'b0);
    do_div(32'h00000007, 32'h00000002, 1'b1);
    do_div(32'hfffffff9, 32'h00000003, 1'b1); // -7 / 3
    do_div(32'h00000007, 32'hfffffffd, 1'b1); // 7 / -3
    do_div(32'hfffffff9, 32'hfffffffd, 1'b1); // -7 / -3
    do_div(32'h80000000, 32'hffffffff, 1'b1); // -2^31 / -1
    do_div(32'h80000000, 32'h00000001, 1'b1);
    do_div(32'hffffffff, 32'h00000002, 1'b0);
    do_div(32'hffffffff, 32'h00000002, 1'b1);
    do_div(32'h12345678, 32'h00000001, 1'b0);
    do_div(32'h00000000, 32'h0000000f, 1'b1);
    do_div(32'h00000007, 32'h00000000, 1'b0); // div by zero
    do_div(32'h00000007, 32'h00000000, 1'b1);

    // ---- div: random ----
    for (k = 0; k < 200; k = k + 1) begin
        ra = $random;
        rb = $random;
        if (rb == 32'd0) rb = 32'd1;
        do_div(ra, rb, k[0]);
    end

    repeat (4) @(posedge clk);
    $display("==========================================");
    if (errors == 0) $display("PASS: %0d checks, 0 errors", checks);
    else             $display("FAIL: %0d checks, %0d errors", checks, errors);
    $display("==========================================");
    $finish;
end

endmodule
