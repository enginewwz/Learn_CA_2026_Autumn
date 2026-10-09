// Differential test: the working-tree exp/myCPU/alu.v vs a reference revision.
// Both are combinational, so this is a straight wire-for-wire compare of
// alu_result / lt_o / ltu_o over EVERY one of the 4096 alu_op values
// (including every multi-hot combination) x directed+random operand pairs.
//
// Build the reference from git or from docs/ex_opt_attempt (module renamed so both can
// be elaborated):
//   git show b2b747f:exp/myCPU/alu.v > /tmp/alu_ref.v
//   sed -i 's/^module alu(/module alu_ref(/' /tmp/alu_ref.v
//   iverilog -g2005 -o /tmp/tb_alu_diff /tmp/alu_ref.v exp/myCPU/alu.v exp/analysis/tb_alu_diff.v
//   vvp /tmp/tb_alu_diff
`timescale 1ns/1ps

module tb_alu_diff;

reg  [11:0] op;
reg  [31:0] a, b;
wire [31:0] r_new, r_old;
wire        lt_new, lt_old, ltu_new, ltu_old;

alu u_new(
    .alu_op(op), .alu_src1(a), .alu_src2(b),
    .alu_result(r_new), .lt_o(lt_new), .ltu_o(ltu_new)
);

alu_ref u_old(
    .alu_op(op), .alu_src1(a), .alu_src2(b),
    .alu_result(r_old), .lt_o(lt_old), .ltu_o(ltu_old)
);

integer errors = 0, checks = 0, i, k;
reg [31:0] av [0:23];
integer seed = 32'hc0ffee01;

task chk;
    begin
        #1;
        checks = checks + 1;
        if (r_new !== r_old || lt_new !== lt_old || ltu_new !== ltu_old) begin
            errors = errors + 1;
            if (errors <= 20)
                $display("MISMATCH op=%012b a=%08x b=%08x  new={%08x %b %b} old={%08x %b %b}",
                         op, a, b, r_new, lt_new, ltu_new, r_old, lt_old, ltu_old);
        end
    end
endtask

initial begin
    av[0]=32'h00000000;  av[1]=32'h00000001;  av[2]=32'hffffffff;
    av[3]=32'h80000000;  av[4]=32'h7fffffff;  av[5]=32'h0000ffff;
    av[6]=32'hffff0000;  av[7]=32'h00010000;  av[8]=32'h80000001;
    av[9]=32'h7ffffffe;  av[10]=32'h55555555; av[11]=32'haaaaaaaa;
    av[12]=32'h00000010; av[13]=32'h0000001f; av[14]=32'h00000020;
    av[15]=32'hf0000000;
    for (i = 16; i < 24; i = i + 1) av[i] = $random(seed);

    // exhaustive alu_op x directed operand pairs
    for (k = 0; k < 4096; k = k + 1) begin
        op = k[11:0];
        for (i = 0; i < 16; i = i + 1) begin
            a = av[i]; b = av[(i*5+3) % 16];
            chk;
        end
    end

    // random operand pairs, one-hot and random alu_op
    for (i = 0; i < 4000; i = i + 1) begin
        a = $random(seed); b = $random(seed);
        op = (i % 3 == 0) ? $random(seed) : (12'b1 << (i % 12));
        chk;
    end

    $display("==========================================");
    if (errors == 0) $display("PASS: %0d checks, 0 errors", checks);
    else             $display("FAIL: %0d checks, %0d errors", checks, errors);
    $display("==========================================");
    $finish;
end

endmodule
