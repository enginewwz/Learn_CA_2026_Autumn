// Standalone independent random / directed scan for exp/myCPU/div.v.
// Independent of the repo test set: nothing globs it, it is compiled by hand.
// It has its own reference model and does not reuse anything from tb_mul_div.v.
//
//   iverilog -g2005 -o /tmp/tb_scan exp/myCPU/tools.v exp/myCPU/div.v exp/tb_div_scan.v
//   timeout 600 vvp /tmp/tb_scan
//
// Coverage:
//   * 200000 random (a, b) pairs, random sign mode, b forced non-zero
//   * divisor == 0 in both sign modes
//   * a in {0, 1, 2, 3, 0x7fffffff, 0x80000000, 0xffffffff, 0x12345678, ...}
//     crossed with b in [1, 256] and b in {0, 0x7fffffff, 0x80000000, 0xffffffff}
// Checks: quotient and remainder both, plus busy/done handshake shape.
`timescale 1ns/1ps

module tb_div_scan;

reg clk = 1'b0;
always #5 clk = ~clk;

reg         reset = 1'b1;
reg         start = 1'b0;
reg         is_signed = 1'b0;
reg  [31:0] a_in = 32'd0, b_in = 32'd0;
wire        busy, done;
wire [31:0] q_out, r_out;

div u_div(
    .clk       (clk      ),
    .reset     (reset    ),
    .start     (start    ),
    .is_signed (is_signed),
    .src1      (a_in     ),
    .src2      (b_in     ),
    .busy      (busy     ),
    .done      (done     ),
    .quot      (q_out    ),
    .rem       (r_out    )
);

integer checks = 0;
integer errors = 0;
integer lat_min = 1000, lat_max = 0, lat_sum = 0, lat_n = 0;
reg [31:0] eq_q, eq_r;

function [31:0] ref_q;
    input [31:0] a, b;
    input        sgn;
    reg signed [63:0] q;
    begin
        if (b == 32'd0)      ref_q = 32'hffffffff;
        else if (sgn) begin
            q = $signed(a) / $signed(b);
            ref_q = q[31:0];
        end
        else                 ref_q = a / b;
    end
endfunction

function [31:0] ref_r;
    input [31:0] a, b;
    input        sgn;
    reg signed [63:0] q, r;
    begin
        if (b == 32'd0)      ref_r = a;
        else if (sgn) begin
            q = $signed(a) / $signed(b);
            r = $signed(a) - q * $signed(b);
            ref_r = r[31:0];
        end
        else                 ref_r = a % b;
    end
endfunction

// one blocking division; asserts nothing itself, returns via eq_q / eq_r
task do_div;
    input [31:0] a, b;
    input        sgn;
    integer      lat;
    reg          seen_busy, done_next;
    begin
        @(negedge clk);
        a_in = a; b_in = b; is_signed = sgn; start = 1'b1;
        @(negedge clk);
        start = 1'b0;
        lat = 0; seen_busy = 1'b0;
        while (!done) begin
            @(negedge clk);
            lat = lat + 1;
            if (busy) seen_busy = 1'b1;
            if (lat > 60) begin
                errors = errors + 1;
                $display("TIMEOUT t=%0t a=%h b=%h sgn=%b", $time, a, b, sgn);
                disable do_div;
            end
        end
        // done is high right now (negedge window); it must be a single cycle
        @(negedge clk);
        done_next = done;
        checks = checks + 1;
        lat_sum = lat_sum + lat; lat_n = lat_n + 1;
        if (lat < lat_min) lat_min = lat;
        if (lat > lat_max) lat_max = lat;
        if (!seen_busy) begin
            errors = errors + 1;
            $display("NO BUSY t=%0t a=%h b=%h sgn=%b", $time, a, b, sgn);
        end
        if (done_next) begin
            errors = errors + 1;
            $display("DONE NOT SINGLE CYCLE t=%0t a=%h b=%h", $time, a, b);
        end
        if ((q_out !== eq_q) || (r_out !== eq_r)) begin
            errors = errors + 1;
            if (errors <= 20)
                $display("MISMATCH a=%h b=%h sgn=%b exp q=%h r=%h got q=%h r=%h",
                         a, b, sgn, eq_q, eq_r, q_out, r_out);
        end
    end
endtask

task run;                       // one checked case
    input [31:0] a, b;
    input        sgn;
    begin
        eq_q = ref_q(a, b, sgn);
        eq_r = ref_r(a, b, sgn);
        do_div(a, b, sgn);
    end
endtask

// 64-bit LCG so the stream is reproducible and independent of $random quirks
reg [63:0] lcg = 64'h0123_4567_89ab_cdef;
task rnd32;
    output [31:0] v;
    begin
        lcg = lcg * 64'd6364136223846793005 + 64'd1442695040888963407;
        v = lcg[63:32] ^ lcg[31:0];
    end
endtask

integer i;
reg [31:0] ra, rb, rsgn;
reg [31:0] edge_a [0:8];
reg [31:0] edge_b [0:9];

initial begin
    edge_a[0]=32'h00000000; edge_a[1]=32'h00000001; edge_a[2]=32'h00000002;
    edge_a[3]=32'h00000003; edge_a[4]=32'h7fffffff; edge_a[5]=32'h80000000;
    edge_a[6]=32'hffffffff; edge_a[7]=32'h12345678; edge_a[8]=32'hdeadbeef;
    edge_b[0]=32'h00000000; edge_b[1]=32'h00000001; edge_b[2]=32'h00000002;
    edge_b[3]=32'h000000ff; edge_b[4]=32'h00000100; edge_b[5]=32'h7fffffff;
    edge_b[6]=32'h80000000; edge_b[7]=32'hffffffff; edge_b[8]=32'h00010000;
    edge_b[9]=32'h6d2b1f37;

    repeat (4) @(posedge clk);
    reset = 1'b0;
    repeat (2) @(posedge clk);

    // ---- directed: edge dividends x edge divisors, both sign modes ----
    for (i = 0; i < 9; i = i + 1) begin : dir_a
        integer j;
        for (j = 0; j < 10; j = j + 1) begin : dir_b
            run(edge_a[i], edge_b[j], 1'b0);
            run(edge_a[i], edge_b[j], 1'b1);
        end
    end

    // ---- directed: full sweep of small divisors, b in [1,256] ----
    for (i = 1; i <= 256; i = i + 1) begin : small_b
        run(32'hffffffff, i[31:0], 1'b0);
        run(32'hffffffff, i[31:0], 1'b1);
        run(32'h80000000, i[31:0], 1'b0);
        run(32'h80000000, i[31:0], 1'b1);
        rnd32(ra);
        run(ra,           i[31:0], 1'b0);
        rnd32(ra);
        run(ra,           i[31:0], 1'b1);
    end

    // ---- random: 200000 pairs, random sign mode, non-zero divisor ----
    for (i = 0; i < 200000; i = i + 1) begin : rnd
        rnd32(ra);
        rnd32(rb);
        rnd32(rsgn);
        if (rb == 32'd0) rb = 32'd1;
        run(ra, rb, rsgn[0]);
    end

    // ---- a few explicit divisor==0 cases again ----
    run(32'h00000000, 32'h00000000, 1'b0);
    run(32'hffffffff, 32'h00000000, 1'b1);
    run(32'h80000000, 32'h00000000, 1'b0);

    repeat (4) @(posedge clk);
    $display("==========================================");
    $display("div scan: %0d checks, %0d errors", checks, errors);
    $display("latency (cycles seen at negedge while waiting): min=%0d max=%0d avg=%0d/100",
             lat_min, lat_max, (lat_sum * 100) / lat_n);
    if (errors == 0) $display("SCAN PASS");
    else             $display("SCAN FAIL");
    $display("==========================================");
    $finish;
end

endmodule
