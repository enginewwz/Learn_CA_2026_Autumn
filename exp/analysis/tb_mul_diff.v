// Differential test: the working-tree exp/myCPU/mul.v vs a reference revision.
// Drives identical clk/reset/allow/src1/src2/md_op into both and compares the
// registered result.  Covers every one of the 128 md_op values (including the
// all-zero and multi-hot encodings the top never produces) x directed+random
// operands.
//
// Build the reference from git (module renamed so both can be elaborated):
//   git show b2b747f:exp/myCPU/mul.v > /tmp/mul_ref.v
//   sed -i 's/^module mul(/module mul_ref(/' /tmp/mul_ref.v
//   iverilog -g2005 -o /tmp/tb_mul_diff exp/myCPU/tools.v /tmp/mul_ref.v \
//            exp/myCPU/mul.v exp/analysis/tb_mul_diff.v && vvp /tmp/tb_mul_diff
// Baseline (2026-10, b2b747f vs working tree): 2488 checks, 0 errors.
`timescale 1ns/1ps

module tb_mul_diff;

reg clk = 1'b0;
always #5 clk = ~clk;
reg reset = 1'b1;

reg  [ 2:0] allow = 3'b111;
reg  [31:0] a, b;
reg  [ 6:0] op;

wire [31:0] r_new, r_old;

mul u_new(
    .clk (clk), .reset(reset), .allow(allow),
    .src1(a), .src2(b), .md_op(op), .result(r_new)
);

mul_ref u_old(
    .clk (clk), .reset(reset), .allow(allow),
    .src1(a), .src2(b), .md_op(op), .result(r_old)
);

integer errors = 0, checks = 0, i, k;
reg [31:0] av [0:43];
integer nav;

integer seed = 32'h1234_5678;

task drive_and_check;
    input [31:0] x, y;
    input [ 6:0] o;
    begin
        a = x; b = y; op = o;
        @(posedge clk); @(posedge clk); @(posedge clk); @(posedge clk);
        #1;
        checks = checks + 1;
        if (r_new !== r_old) begin
            errors = errors + 1;
            if (errors <= 20)
                $display("MISMATCH a=%08x b=%08x op=%07b new=%08x old=%08x",
                         x, y, o, r_new, r_old);
        end
    end
endtask

initial begin
    // directed operands first
    av[0]=32'h00000000; av[1]=32'h00000001; av[2]=32'hffffffff;
    av[3]=32'h80000000; av[4]=32'h7fffffff; av[5]=32'h0000ffff;
    av[6]=32'hffff0000; av[7]=32'h00010000; av[8]=32'h00008000;
    av[9]=32'h00007fff; av[10]=32'h55555555; av[11]=32'haaaaaaaa;
    av[12]=32'hffff8000; av[13]=32'h80000001; av[14]=32'h0000fffe;
    av[15]=32'h00010001;
    nav = 16;
    for (i = 0; i < 28; i = i + 1) begin
        av[nav] = $random(seed); nav = nav + 1;
    end

    reset = 1'b1;
    @(posedge clk); @(posedge clk);
    reset = 1'b0;

    // every md_op value, over the directed cross product
    for (k = 0; k < 128; k = k + 1) begin
        for (i = 0; i < 16; i = i + 1) begin
            drive_and_check(av[i], av[(i*3+1) % 16], k[6:0]);
        end
    end

    // random operand pairs x random md_op (biased to one-hot)
    for (i = 0; i < 400; i = i + 1) begin
        drive_and_check($random(seed), $random(seed),
                        (i % 4 == 0) ? $random(seed) : (7'b1 << (i % 7)));
    end

    // allow-gated spot checks
    for (i = 0; i < 40; i = i + 1) begin
        a = $random(seed); b = $random(seed); op = 7'b1 << (i % 3);
        allow = 3'b010; @(posedge clk);
        allow = 3'b111; @(posedge clk); @(posedge clk); @(posedge clk);
        #1; checks = checks + 1;
        if (r_new !== r_old) begin
            errors = errors + 1;
            $display("MISMATCH(allow) a=%08x b=%08x op=%07b new=%08x old=%08x",
                     a, b, op, r_new, r_old);
        end
    end

    $display("==========================================");
    if (errors == 0) $display("PASS: %0d checks, 0 errors", checks);
    else             $display("FAIL: %0d checks, %0d errors", checks, errors);
    $display("==========================================");
    $finish;
end

endmodule
