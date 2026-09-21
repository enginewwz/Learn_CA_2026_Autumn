// Standalone self-checking testbench for EX.v (not part of the CPU build).
//
//   iverilog -g2005 -o /tmp/tb_ex impl/myCPU/EX.v impl/myCPU/alu.v impl/myCPU/mul.v impl/tb_ex.v
//   vvp /tmp/tb_ex
//
// Checks the parts the full CPU regression cannot reach yet (div tests come
// before the mul tests):
//   * an ALU result that is the oldest instruction short-circuits out of its
//     own stage (1-cycle EX -> MEM latency) and carries its control fields
//   * mul.w / mulh.w / mulh.wu take exactly 4 cycles and produce the right
//     low / high word, with ma/mb operands
//   * allow_next = 0 freezes a mul at stage 3 without corrupting it
//   * an ALU behind a mul never overtakes it
`timescale 1ns/1ps

module tb_ex;

reg clk = 1'b0;
always #5 clk = ~clk;

reg         reset    = 1'b1;
reg         receive  = 1'b0;
reg         allow_next = 1'b1;
reg  [11:0] alu_op_i;
reg  [31:0] alu_src1_i, alu_src2_i;
reg         src1_used_i, src2_used_i;
reg  [31:0] pc_i;
reg  [ 6:0] md_op_i;
reg         res_from_mem_i, mem_we_i, mem_en_i;
reg  [31:0] mem_wdata_i;
reg  [ 2:0] mem_size_i;
reg         gr_we_i;
reg  [ 4:0] dest_i;

wire [31:0] pc_o, mem_wdata_o, ex_result;
wire [ 2:0] mem_size_o;
wire        src1_used_o, src2_used_o, res_from_mem_o, mem_we_o, mem_en_o, gr_we_o;
wire [ 4:0] dest_o;
wire        valid_o, stall_o;
wire        fwd0_valid, fwd0_pending, fwd1_valid, fwd1_pending;
wire        fwd2_valid, fwd2_pending, fwd3_valid, fwd3_pending;
wire [ 4:0] fwd0_dest, fwd1_dest, fwd2_dest, fwd3_dest;
wire [31:0] fwd0_data, fwd1_data, fwd2_data, fwd3_data;

EX u_ex(
    .clk            (clk            ),
    .reset          (reset          ),
    .receive        (receive        ),
    .allow_next     (allow_next     ),
    .alu_op_i       (alu_op_i       ),
    .alu_src1_i     (alu_src1_i     ),
    .alu_src2_i     (alu_src2_i     ),
    .src1_used_i    (src1_used_i    ),
    .src2_used_i    (src2_used_i    ),
    .pc_i           (pc_i           ),
    .md_op_i        (md_op_i        ),
    .res_from_mem_i (res_from_mem_i ),
    .mem_we_i       (mem_we_i       ),
    .mem_en_i       (mem_en_i       ),
    .mem_wdata_i    (mem_wdata_i    ),
    .mem_size_i     (mem_size_i     ),
    .gr_we_i        (gr_we_i        ),
    .dest_i         (dest_i         ),
    .pc_o           (pc_o           ),
    .src1_used_o    (src1_used_o    ),
    .src2_used_o    (src2_used_o    ),
    .res_from_mem_o (res_from_mem_o ),
    .mem_we_o       (mem_we_o       ),
    .mem_en_o       (mem_en_o       ),
    .mem_wdata_o    (mem_wdata_o    ),
    .mem_size_o     (mem_size_o     ),
    .gr_we_o        (gr_we_o        ),
    .dest_o         (dest_o         ),
    .valid_o        (valid_o        ),
    .ex_result      (ex_result      ),
    .stall_o        (stall_o        ),
    .fwd0_valid     (fwd0_valid     ),
    .fwd0_pending   (fwd0_pending   ),
    .fwd0_dest      (fwd0_dest      ),
    .fwd0_data      (fwd0_data      ),
    .fwd1_valid     (fwd1_valid     ),
    .fwd1_pending   (fwd1_pending   ),
    .fwd1_dest      (fwd1_dest      ),
    .fwd1_data      (fwd1_data      ),
    .fwd2_valid     (fwd2_valid     ),
    .fwd2_pending   (fwd2_pending   ),
    .fwd2_dest      (fwd2_dest      ),
    .fwd2_data      (fwd2_data      ),
    .fwd3_valid     (fwd3_valid     ),
    .fwd3_pending   (fwd3_pending   ),
    .fwd3_dest      (fwd3_dest      ),
    .fwd3_data      (fwd3_data      )
);

localparam MUL_W   = 0;
localparam MULH_W  = 1;
localparam MULH_WU = 2;
localparam DIV_W   = 3;
localparam MOD_W   = 4;
localparam DIV_WU  = 5;
localparam MOD_WU  = 6;

integer errors = 0;
integer checks = 0;

task step; begin @(posedge clk); #1; end endtask

// present one instruction at the EX input for one cycle
task send;
    input [11:0] op;
    input [ 6:0] md;
    input [31:0] a, b;
    input [ 4:0] d;
    begin
        @(negedge clk);
        receive = 1'b1; alu_op_i = op; md_op_i = md;
        alu_src1_i = a; alu_src2_i = b; dest_i = d;
        src1_used_i = 1'b1; src2_used_i = 1'b1;
        pc_i = 32'h1c001000; res_from_mem_i = 1'b0;
        mem_we_i = 1'b0; mem_en_i = 1'b0; mem_wdata_i = 32'b0; gr_we_i = 1'b1;
        step;
        receive = 1'b0;
    end
endtask

task chk;
    input cond;
    input [511:0] name;
    begin
        checks = checks + 1;
        if (!cond) begin
            errors = errors + 1;
            $display("EX ERROR: %0s (t=%0t valid=%b ex=%h dest=%0d stall=%b)",
                     name, $time, valid_o, ex_result, dest_o, stall_o);
        end
    end
endtask

// launch one div/mod and wait (with a bound) for its result at the EX output
task do_div;
    input [ 6:0] md;
    input [31:0] a, b;
    input [ 4:0] d;
    input [31:0] exp;
    integer i;
    reg     got;
    begin
        send(12'b0, md, a, b, d);
        got = 1'b0;
        for (i = 0; i < 40 && !got; i = i + 1) begin
            step;
            if (valid_o) got = 1'b1;
        end
        chk(got && (ex_result === exp) && (dest_o === d), "div/mod result");
        repeat (2) step;   // let it hand off
    end
endtask

function [31:0] mlo;  input [31:0] a, b; begin mlo  = a * b; end endfunction
function [31:0] mhi;  input [31:0] a, b; reg signed [63:0] p; begin p = $signed(a) * $signed(b); mhi = p[63:32]; end endfunction
function [31:0] mhiu; input [31:0] a, b; reg [63:0] p; begin p = {32'b0,a} * {32'b0,b}; mhiu = p[63:32]; end endfunction

reg [31:0] A, B;

initial begin
    receive = 0; allow_next = 1; alu_op_i = 0; alu_src1_i = 0; alu_src2_i = 0;
    src1_used_i = 0; src2_used_i = 0; pc_i = 0; md_op_i = 0;
    res_from_mem_i = 0; mem_we_i = 0; mem_en_i = 0; mem_wdata_i = 0; mem_size_i = 0; gr_we_i = 0; dest_i = 0;
    repeat (4) @(posedge clk);
    reset = 1'b0;
    repeat (2) @(posedge clk);

    // ---------------------------------------------------------------- ALU
    // or: the result is ready at stage 0 and is the only instruction in EX,
    // so it must leave immediately with its control fields attached
    A = 32'hf0f0f0f0; B = 32'h0ff00ff0;
    send(12'b1 << 6, 7'b0, A, B, 5'd7);
    chk(valid_o && (ex_result === (A | B)) && (dest_o === 5'd7) && gr_we_o && !stall_o,
        "alu short-circuit from stage 0");
    repeat (2) step;

    // addi-like immediate: result must not depend on the stale ex_result
    send(12'b1 << 0, 7'b0, 32'd0, 32'hffffffff, 5'd2);
    chk(valid_o && (ex_result === 32'hffffffff) && (dest_o === 5'd2),
        "alu short-circuit addi -1");
    repeat (2) step;

    // ----------------------------------------------------------------- mul.w
    A = 32'h1234_5678; B = 32'h9abc_def0;
    send(12'b0, 7'b1 << MUL_W, A, B, 5'd9);
    chk(!valid_o, "mul.w not ready at stage 0");
    step; chk(!valid_o, "mul.w not ready at stage 1");
    step; chk(!valid_o, "mul.w not ready at stage 2");
    step; chk(valid_o && (ex_result === mlo(A, B)) && (dest_o === 5'd9),
              "mul.w low word at stage 3");
    repeat (2) step;

    // --------------------------------------------------------------- mulh.w
    A = 32'hdead_beef; B = 32'h8000_0001;
    send(12'b0, 7'b1 << MULH_W, A, B, 5'd11);
    step; step; step;
    chk(valid_o && (ex_result === mhi(A, B)), "mulh.w signed high word");
    repeat (2) step;

    // -------------------------------------------------------------- mulh.wu
    send(12'b0, 7'b1 << MULH_WU, A, B, 5'd11);
    step; step; step;
    chk(valid_o && (ex_result === mhiu(A, B)), "mulh.wu unsigned high word");
    repeat (2) step;

    // ------------------------------------------- mul frozen by allow_next=0
    A = 32'h7fff_ffff; B = 32'h7fff_ffff;
    send(12'b0, 7'b1 << MUL_W, A, B, 5'd4);
    step; step;                 // -> stage 1
    allow_next = 1'b0;
    step;                       // -> stage 2
    step;                       // -> stage 3, now frozen
    chk(valid_o && (ex_result === mlo(A, B)), "mul held at stage 3 under back-pressure");
    step;                       // still frozen, value must survive
    chk(valid_o && (ex_result === mlo(A, B)), "mul value stable while stalled");
    allow_next = 1'b1;
    step; step;
    chk(!valid_o, "mul left after allow_next");

    // ------------------------------------- an ALU behind a mul must not pass
    A = 32'h0000_ffff; B = 32'h0001_0000;
    send(12'b0, 7'b1 << MUL_W, A, B, 5'd6);          // mul, stage 0 next cycle
    send(12'b1 << 6, 7'b0, 32'hf0f0_f0f0, 32'h0f0f_0f0f, 5'd8); // ALU right behind
    // after the two sends: mul@1, alu@0 -> 2 steps put the mul at stage 3
    step; step;
    chk(valid_o && (dest_o === 5'd6) && (ex_result === mlo(A, B)),
        "mul in front drives the output, not the younger ALU");
    step;
    chk(valid_o && (dest_o === 5'd8) && (ex_result === (32'hf0f0_f0f0 | 32'h0f0f_0f0f)),
        "ALU leaves only after the mul");
    repeat (2) step;

    // ------------------------------------------------------------------ div
    do_div(7'b1 << DIV_W,  32'h0000_0007, 32'h0000_0002, 5'd12, 32'h0000_0003);
    do_div(7'b1 << DIV_W,  32'hffff_fff9, 32'h0000_0002, 5'd12, 32'hffff_fffd); // -7/2
    do_div(7'b1 << DIV_WU, 32'hffff_fff9, 32'h0000_0002, 5'd12, 32'h7fff_fffc);
    do_div(7'b1 << MOD_W,  32'hffff_fff9, 32'h0000_0002, 5'd12, 32'hffff_ffff); // -7%2
    do_div(7'b1 << MOD_WU, 32'h0000_0007, 32'h0000_0002, 5'd12, 32'h0000_0001);
    do_div(7'b1 << DIV_W,  32'h8000_0000, 32'hffff_ffff, 5'd12, 32'h8000_0000); // -2^31/-1

    // a div blocks the pipeline: the ALU right behind it must not come out first
    send(12'b0, 7'b1 << DIV_W, 32'h0000_0064, 32'h0000_0007, 5'd13); // 100/7 = 14
    send(12'b1 << 6, 7'b0, 32'hf0f0_f0f0, 32'h0f0f_0f0f, 5'd14);
    begin : div_order
        integer i;
        reg     got;
        got = 1'b0;
        for (i = 0; i < 40 && !got; i = i + 1) begin
            step;
            if (valid_o) got = 1'b1;
        end
        chk(got && (dest_o === 5'd13) && (ex_result === 32'd14), "div in front drives the output");
        step;
        chk(valid_o && (dest_o === 5'd14) && (ex_result === (32'hf0f0_f0f0 | 32'h0f0f_0f0f)),
            "ALU leaves only after the div");
        repeat (2) step;
    end

    // ------------------------------------------------- forwarding groups
    // ALU is forwardable from every stage, so hold allow_next low to make it
    // walk through the stages instead of short-circuiting out of stage 0
    A = 32'h1234_5678; B = 32'h0000_00ff;
    allow_next = 1'b0;
    send(12'b1 << 4, 7'b0, A, B, 5'd21);   // and
    chk(fwd0_valid && (fwd0_dest === 5'd21) && (fwd0_data === (A & B)), "ALU stage 0 supplies");
    step;
    chk(fwd1_valid && (fwd1_dest === 5'd21) && (fwd1_data === (A & B)), "ALU stage 1 supplies");
    step;
    chk(fwd2_valid && (fwd2_dest === 5'd21) && (fwd2_data === (A & B)), "ALU stage 2 supplies");
    step;
    chk(fwd3_valid && (fwd3_dest === 5'd21) && (fwd3_data === (A & B)), "ALU stage 3 supplies");
    allow_next = 1'b1;
    repeat (2) step;

    // mul is only forwardable from the last stage
    A = 32'h0000_0003; B = 32'h0000_0005;
    allow_next = 1'b0;
    send(12'b0, 7'b1 << MUL_W, A, B, 5'd22);
    chk(!fwd0_valid && fwd0_pending, "mul stage 0 pending");
    step; chk(!fwd1_valid && fwd1_pending, "mul stage 1 pending");
    step; chk(!fwd2_valid && fwd2_pending, "mul stage 2 pending");
    step; chk(fwd3_valid && (fwd3_dest === 5'd22) && (fwd3_data === mlo(A, B)), "mul stage 3 supplies");
    allow_next = 1'b1;
    repeat (2) step;

    // div is only forwardable once the unit is done
    send(12'b0, 7'b1 << DIV_W, 32'd100, 32'd7, 5'd23);
    begin : div_fwd
        integer i;
        reg     held;
        held = 1'b0;
        for (i = 0; i < 40 && !valid_o; i = i + 1) begin
            step;
            if (fwd3_pending) held = 1'b1;
        end
        chk(held && fwd3_valid && (fwd3_dest === 5'd23) && (fwd3_data === 32'd14),
            "div pending, then supplies");
    end
    repeat (2) step;

    $display("==========================================");
    if (errors == 0) $display("PASS: %0d checks, 0 errors", checks);
    else             $display("FAIL: %0d checks, %0d errors", checks, errors);
    $display("==========================================");
    $finish;
end

endmodule
