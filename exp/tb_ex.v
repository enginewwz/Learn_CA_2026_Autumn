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
reg         allow_cur = 1'b0;   // allow[ID]: the stage handing over may move
reg         valid_i   = 1'b1;   // and it really has an instruction
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
wire        fw_write_ex_stage0, fw_valid_ex_stage0, fw_write_ex_stage1, fw_valid_ex_stage1;
wire        fw_write_ex_stage2, fw_valid_ex_stage2;
wire [ 4:0] fw_dest_ex_stage0, fw_dest_ex_stage1, fw_dest_ex_stage2;
wire [31:0] fw_data_ex_stage1, fw_data_ex_stage2;

EX u_ex(
    .clk            (clk            ),
    .reset          (reset          ),
    .allow_cur      (allow_cur      ),
    .valid_i        (valid_i        ),
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
    .fw_valid_ex_stage0     (fw_valid_ex_stage0     ),
    .fw_write_ex_stage0   (fw_write_ex_stage0   ),
    .fw_dest_ex_stage0      (fw_dest_ex_stage0      ),
    .fw_valid_ex_stage1     (fw_valid_ex_stage1     ),
    .fw_write_ex_stage1   (fw_write_ex_stage1   ),
    .fw_dest_ex_stage1      (fw_dest_ex_stage1      ),
    .fw_data_ex_stage1      (fw_data_ex_stage1      ),
    .fw_valid_ex_stage2     (fw_valid_ex_stage2     ),
    .fw_write_ex_stage2   (fw_write_ex_stage2   ),
    .fw_dest_ex_stage2      (fw_dest_ex_stage2      ),
    .fw_data_ex_stage2      (fw_data_ex_stage2      )
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
        allow_cur = 1'b1; valid_i = 1'b1; alu_op_i = op; md_op_i = md;
        alu_src1_i = a; alu_src2_i = b; dest_i = d;
        src1_used_i = 1'b1; src2_used_i = 1'b1;
        pc_i = 32'h1c001000; res_from_mem_i = 1'b0;
        mem_we_i = 1'b0; mem_en_i = 1'b0; mem_wdata_i = 32'b0; gr_we_i = 1'b1;
        step;
        allow_cur = 1'b0;
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
    allow_cur = 0; allow_next = 1; alu_op_i = 0; alu_src1_i = 0; alu_src2_i = 0;
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

    // -------------------------------------- advancing a bubble (no instruction)
    // allow_cur (the pipeline advances) is high but ID has nothing: valid_i = 0
    // must leave EX empty instead of inventing an instruction from stale inputs
    @(negedge clk);
    allow_cur = 1'b1; valid_i = 1'b0;
    alu_op_i = 12'b1 << 0; md_op_i = 7'b1 << MUL_W;
    alu_src1_i = 32'hdead_beef; alu_src2_i = 32'h1234_5678; dest_i = 5'd31;
    src1_used_i = 1'b1; src2_used_i = 1'b1; gr_we_i = 1'b1;
    step;
    allow_cur = 1'b0; valid_i = 1'b1;
    chk(!valid_o, "a bubble (valid_i = 0) leaves EX empty");
    repeat (2) step;

    // ------------------- a blocked entry must not duplicate the instruction
    // entry blocked (allow_cur = 0, what a busy MEM does) while stage 1/2 are
    // free: the instruction must move on inside EX, not stay in stage 0 too
    A = 32'h0000_ffff; B = 32'h0001_0000;
    send(12'b0, 7'b1 << MUL_W, A, B, 5'd17);
    chk(!valid_o, "mul not ready right after entry");
    allow_cur = 1'b0; valid_i = 1'b1;
    step; chk(!valid_o, "mul at stage 1 while the entry is blocked");
    step; chk(valid_o && (ex_result === mlo(A, B)) && (dest_o === 5'd17),
              "mul reached stage 2 while the entry was blocked");
    step; chk(!valid_o, "mul left EX for MEM (allow_next = 1)");
    allow_cur = 1'b1; valid_i = 1'b0;
    step; step; step;
    chk(!valid_o, "nothing left in EX after the mul drained (no stage-0 copy)");
    valid_i = 1'b1;


    // ----------------------------------------------------------------- mul.w
    A = 32'h1234_5678; B = 32'h9abc_def0;
    send(12'b0, 7'b1 << MUL_W, A, B, 5'd9);
    chk(!valid_o, "mul.w not ready at stage 0");
    step; chk(!valid_o, "mul.w not ready at stage 1");
    step; chk(valid_o && (ex_result === mlo(A, B)) && (dest_o === 5'd9),
              "mul.w low word at stage 2 (comb to MEM)");
    repeat (2) step;

    // --------------------------------------------------------------- mulh.w
    A = 32'hdead_beef; B = 32'h8000_0001;
    send(12'b0, 7'b1 << MULH_W, A, B, 5'd11);
    step; step;
    chk(valid_o && (ex_result === mhi(A, B)), "mulh.w signed high word");
    repeat (2) step;

    // -------------------------------------------------------------- mulh.wu
    send(12'b0, 7'b1 << MULH_WU, A, B, 5'd11);
    step; step;
    chk(valid_o && (ex_result === mhiu(A, B)), "mulh.wu unsigned high word");
    repeat (2) step;

    // ------------------------------------------- mul frozen by allow_next=0
    A = 32'h7fff_ffff; B = 32'h7fff_ffff;
    send(12'b0, 7'b1 << MUL_W, A, B, 5'd4);
    step; step;                 // -> stage 1, stage 2
    allow_next = 1'b0;
    step;                       // held at stage 2
    chk(valid_o && (ex_result === mlo(A, B)), "mul held at stage 2 under back-pressure");
    step;                       // still frozen, value must survive
    chk(valid_o && (ex_result === mlo(A, B)), "mul value stable while stalled");
    allow_next = 1'b1;
    step;
    chk(!valid_o, "mul left after allow_next");

    // ------------------------------------- an ALU behind a mul must not pass
    A = 32'h0000_ffff; B = 32'h0001_0000;
    send(12'b0, 7'b1 << MUL_W, A, B, 5'd6);          // mul, stage 0 next cycle
    send(12'b1 << 6, 7'b0, 32'hf0f0_f0f0, 32'h0f0f_0f0f, 5'd8); // ALU right behind
    // after the two sends: mul@1, alu@0 -> 1 step puts the mul at stage 2
    step;
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
    // An ALU result is only forwarded from stage 1/2 (registers); at stage 0 it
    // is still the combinational ALU output, so ID must wait for it: pending.
    // Hold allow_next low so it walks through the stages instead of
    // short-circuiting out of stage 0.
    A = 32'h1234_5678; B = 32'h0000_00ff;
    allow_next = 1'b0;
    send(12'b1 << 4, 7'b0, A, B, 5'd21);   // and
    chk(!fw_valid_ex_stage0 && fw_write_ex_stage0 && (fw_dest_ex_stage0 === 5'd21), "ALU stage 0: will write but not forwardable");
    step;
    chk(fw_valid_ex_stage1 && (fw_dest_ex_stage1 === 5'd21) && (fw_data_ex_stage1 === (A & B)), "ALU stage 1 supplies");
    step;
    chk(fw_valid_ex_stage2 && (fw_dest_ex_stage2 === 5'd21) && (fw_data_ex_stage2 === (A & B)), "ALU stage 2 supplies");
    allow_next = 1'b1;
    repeat (2) step;

    // mul never forwards from EX: it stays "pending" until it reaches MEM
    A = 32'h0000_0003; B = 32'h0000_0005;
    allow_next = 1'b0;
    send(12'b0, 7'b1 << MUL_W, A, B, 5'd22);
    chk(!fw_valid_ex_stage0 && fw_write_ex_stage0, "mul stage 0 pending (will write, not ready)");
    step; chk(!fw_valid_ex_stage1 && fw_write_ex_stage1, "mul stage 1 pending");
    step; chk(!fw_valid_ex_stage2 && fw_write_ex_stage2, "mul stage 2 pending");
    step; chk(fw_write_ex_stage2 && !fw_valid_ex_stage2, "mul still pending while handing to MEM");
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
            if (fw_write_ex_stage2 && ! fw_valid_ex_stage2) held = 1'b1;
        end
        chk(held && valid_o && (ex_result === 32'd14) && (dest_o === 5'd23),
            "div pending in EX, result goes out to MEM");
    end
    repeat (2) step;

    $display("==========================================");
    if (errors == 0) $display("PASS: %0d checks, 0 errors", checks);
    else             $display("FAIL: %0d checks, %0d errors", checks, errors);
    $display("==========================================");
    $finish;
end

endmodule
