// Standalone self-checking testbench for EX.v (not part of the CPU build).
//
//   iverilog -g2005 -o /tmp/tb_ex exp/myCPU/tools.v exp/myCPU/EX.v exp/myCPU/alu.v exp/myCPU/mul.v exp/myCPU/div.v exp/myCPU/br.v exp/tb_ex.v
//   vvp /tmp/tb_ex
//
// Checks the parts the full CPU regression cannot reach yet:
//   * an ALU result that is the oldest instruction short-circuits out of its
//     own stage (1-cycle EX -> MEM latency) and carries its control fields
//   * mul.w / mulh.w / mulh.wu take exactly 3 cycles and produce the right
//     low / high word, with ma/mb operands
//   * allow_next = 0 freezes a mul at stage 1 without corrupting it
//   * an ALU behind a mul / div never overtakes it
//   * a div is started as soon as its operands are in stage 0 -- also while the
//     downstream is blocked -- and is started exactly once (u_ex.div_start)
//   * the branch unit: taken / target, bubble, and the held-branch case
//   * exp12/13: the CSR payload rides out with the instruction, and a global
//     flush (flush_i) empties EX and resets the mul/div in flight
//
// send() / send_br() hand the instruction over the way the top does: the
// payload is presented and held until EX's stage 0 can take it (stall_o low),
// then a bubble is offered again.
`timescale 1ns/1ps

module tb_ex;

reg clk = 1'b0;
always #5 clk = ~clk;

reg         reset    = 1'b1;
reg         valid_i   = 1'b0;   // this cycle really has an instruction for EX
reg         allow_next = 1'b1;
reg  [11:0] alu_op_i;
reg  [31:0] src1_i, src2_i;
reg         src1_used_i, src2_used_i;
reg  [31:0] pc_i;
reg  [ 6:0] md_op_i;
reg         res_from_mem_i, mem_we_i, mem_en_i;
reg  [31:0] mem_wdata_i;
reg  [ 2:0] mem_size_i;
reg         mem_zext_i;
reg         gr_we_i;
reg  [ 4:0] dest_i;
reg  [ 4:0] fw_raddr1_i = 5'd0, fw_raddr2_i = 5'd0;   // what the ID instruction behind reads
reg  [ 6:0] br_op_i;
reg  [31:0] br_base_i, br_offs_i;
reg  [13:0] csr_num_i   = 14'd0;
reg  [ 7:0] csr_ctl_i   = 8'd0;
reg  [ 5:0] excp_cause_i = 6'd0;
reg  [31:0] csr_wdata_i = 32'd0;
reg  [31:0] csr_mask_i  = 32'd0;
reg         flush_i     = 1'b0;

wire [31:0] pc_o, mem_wdata_o, ex_result;
wire [ 2:0] mem_size_o;
wire        mem_zext_o;
wire        src1_used_o, src2_used_o, res_from_mem_o, mem_we_o, mem_en_o, gr_we_o;
wire [ 4:0] dest_o;
wire [13:0] csr_num_o;
wire [ 7:0] csr_ctl_o;
wire [ 5:0] excp_cause_o;
wire [31:0] csr_wdata_o, csr_mask_o;
wire        valid_o, stall_o;
wire        fw1_hit_o, fw1_valid_o, fw2_hit_o, fw2_valid_o;
wire [31:0] fw1_data_o, fw2_data_o;
wire        br_taken_o;
wire [31:0] br_target_o;

EX u_ex(
    .clk            (clk            ),
    .reset          (reset          ),
    .flush_i        (flush_i        ),
    .valid_i        (valid_i        ),
    .allow_next     (allow_next     ),
    .alu_op_i       (alu_op_i       ),
    .src1_i     (src1_i     ),
    .src2_i     (src2_i     ),
    .br_op_i        (br_op_i        ),
    .br_base_i      (br_base_i      ),
    .br_offs_i      (br_offs_i      ),
    .csr_num_i      (csr_num_i      ),
    .csr_ctl_i      (csr_ctl_i      ),
    .excp_cause_i   (excp_cause_i   ),
    .csr_wdata_i    (csr_wdata_i    ),
    .csr_mask_i     (csr_mask_i     ),
    .src1_used_i    (src1_used_i    ),
    .src2_used_i    (src2_used_i    ),
    .pc_i           (pc_i           ),
    .md_op_i        (md_op_i        ),
    .res_from_mem_i (res_from_mem_i ),
    .mem_we_i       (mem_we_i       ),
    .mem_en_i       (mem_en_i       ),
    .mem_wdata_i    (mem_wdata_i    ),
    .mem_size_i     (mem_size_i     ),
    .mem_zext_i     (mem_zext_i     ),
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
    .mem_zext_o     (mem_zext_o     ),
    .gr_we_o        (gr_we_o        ),
    .dest_o         (dest_o         ),
    .csr_num_o      (csr_num_o      ),
    .csr_ctl_o      (csr_ctl_o      ),
    .excp_cause_o   (excp_cause_o   ),
    .csr_wdata_o    (csr_wdata_o    ),
    .csr_mask_o     (csr_mask_o     ),
    .valid_o        (valid_o        ),
    .ex_result      (ex_result      ),
    .stall_o        (stall_o        ),
    .br_taken_o     (br_taken_o     ),
    .br_target_o    (br_target_o    ),
    .fw_raddr1_i    (fw_raddr1_i    ),
    .fw_raddr2_i    (fw_raddr2_i    ),
    .fw1_hit_o      (fw1_hit_o      ),
    .fw1_valid_o    (fw1_valid_o    ),
    .fw1_data_o     (fw1_data_o     ),
    .fw2_hit_o      (fw2_hit_o      ),
    .fw2_valid_o    (fw2_valid_o    ),
    .fw2_data_o     (fw2_data_o     )
);

localparam MUL_W   = 0;
localparam MULH_W  = 1;
localparam MULH_WU = 2;
localparam DIV_W   = 3;
localparam MOD_W   = 4;
localparam DIV_WU  = 5;
localparam MOD_WU  = 6;

// br_op bit positions (see br.v)
localparam BEQ  = 0;
localparam BNE  = 1;
localparam BLT  = 2;
localparam BGE  = 3;
localparam BLTU = 4;
localparam BGEU = 5;
localparam JIRL = 6;

integer errors = 0;
integer checks = 0;

// watch how often the stage-0 div is started (must be exactly once per div)
integer   div_starts = 0;
reg       div_watch  = 1'b0;
always @(posedge clk) if (div_watch && u_ex.div_start) div_starts = div_starts + 1;

task step; begin @(posedge clk); #1; end endtask

// present one instruction at the EX input and hand it over when EX can take it.
// EX latches stage 0 on its own allow[0], so the model is: drive the payload,
// wait until stall_o drops, let one edge latch it, then offer a bubble.
task send;
    input [11:0] op;
    input [ 6:0] md;
    input [31:0] a, b;
    input [ 4:0] d;
    begin
        @(negedge clk);
        valid_i = 1'b1; alu_op_i = op; md_op_i = md;
        src1_i = a; src2_i = b; dest_i = d;
        src1_used_i = 1'b1; src2_used_i = 1'b1;
        fw_raddr1_i = d; fw_raddr2_i = d;   // the instruction behind it reads the same register
        br_op_i = 7'b0; br_base_i = 32'b0; br_offs_i = 32'b0;
        pc_i = 32'h1c001000; res_from_mem_i = 1'b0;
        mem_we_i = 1'b0; mem_en_i = 1'b0; mem_wdata_i = 32'b0; gr_we_i = 1'b1;
        while (stall_o) @(negedge clk);     // hold at the input while EX is busy
        step;
        valid_i = 1'b0;
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

// present a branch to the EX input (a, b are the compare operands, base/offs
// the target pieces), same handshake as send
task send_br;
    input [ 6:0] op;
    input [31:0] a, b;
    input [31:0] base, offs;
    begin
        @(negedge clk);
        valid_i = 1'b1;
        alu_op_i = 12'b0; md_op_i = 7'b0; src1_i = a; src2_i = b;
        br_op_i = op; br_base_i = base; br_offs_i = offs;
        src1_used_i = 1'b1; src2_used_i = 1'b1;
        fw_raddr1_i = 5'd0; fw_raddr2_i = 5'd0;   // the instruction behind reads nothing
        dest_i = 5'd0; gr_we_i = 1'b0; res_from_mem_i = 1'b0;
        mem_we_i = 1'b0; mem_en_i = 1'b0; mem_wdata_i = 32'b0; mem_size_i = 3'b0; mem_zext_i = 1'b0;
        pc_i = 32'h1c002000;
        while (stall_o) @(negedge clk);
        step;
        valid_i = 1'b0;
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
reg [31:0] OLD, YOUNG;      // older / younger writer of the WAW forwarding case

initial begin
    allow_next = 1; alu_op_i = 0; src1_i = 0; src2_i = 0;
    src1_used_i = 0; src2_used_i = 0; pc_i = 0; md_op_i = 0;
    br_op_i = 0; br_base_i = 0; br_offs_i = 0;
    res_from_mem_i = 0; mem_we_i = 0; mem_en_i = 0; mem_wdata_i = 0; mem_size_i = 0; mem_zext_i = 0; gr_we_i = 0; dest_i = 0;
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
    // valid_i = 0 must leave EX empty instead of inventing an instruction from
    // stale inputs (the payload below is deliberately garbage)
    @(negedge clk);
    valid_i = 1'b0;
    alu_op_i = 12'b1 << 0; md_op_i = 7'b1 << MUL_W;
    src1_i = 32'hdead_beef; src2_i = 32'h1234_5678; dest_i = 5'd31;
    src1_used_i = 1'b1; src2_used_i = 1'b1; gr_we_i = 1'b1;
    step;
    chk(!valid_o, "a bubble (valid_i = 0) leaves EX empty");
    repeat (2) step;

    // --------------------- an instruction must not be duplicated in stage 0
    // while the mul walks through EX (valid_i = 0 is what the top presents when
    // it has nothing to hand over): one copy in stage 0 only, none left behind
    A = 32'h0000_ffff; B = 32'h0001_0000;
    send(12'b0, 7'b1 << MUL_W, A, B, 5'd17);
    chk(!valid_o, "mul not ready right after entry");
    step; chk(valid_o && (ex_result === mlo(A, B)) && (dest_o === 5'd17),
              "mul at stage 1 (combinational output)");
    step; chk(!valid_o, "mul left EX for MEM (allow_next = 1)");
    step; step;
    chk(!valid_o, "nothing left in EX after the mul drained (no stage-0 copy)");


    // ----------------------------------------------------------------- mul.w
    A = 32'h1234_5678; B = 32'h9abc_def0;
    send(12'b0, 7'b1 << MUL_W, A, B, 5'd9);
    chk(!valid_o, "mul.w not ready at stage 0");
    step; chk(valid_o && (ex_result === mlo(A, B)) && (dest_o === 5'd9),
              "mul.w low word at stage 1 (combinational)");
    repeat (2) step;

    // --------------------------------------------------------------- mulh.w
    A = 32'hdead_beef; B = 32'h8000_0001;
    send(12'b0, 7'b1 << MULH_W, A, B, 5'd11);
    step;
    chk(valid_o && (ex_result === mhi(A, B)), "mulh.w signed high word");
    repeat (2) step;

    // -------------------------------------------------------------- mulh.wu
    send(12'b0, 7'b1 << MULH_WU, A, B, 5'd11);
    step;
    chk(valid_o && (ex_result === mhiu(A, B)), "mulh.wu unsigned high word");
    repeat (2) step;

    // ------------------------------------------- mul frozen by allow_next=0
    A = 32'h7fff_ffff; B = 32'h7fff_ffff;
    send(12'b0, 7'b1 << MUL_W, A, B, 5'd4);
    step;                       // -> stage 1, the last one
    allow_next = 1'b0;
    chk(valid_o && (ex_result === mlo(A, B)), "mul held at stage 1 under back-pressure");
    step;                       // still frozen, value must survive
    chk(valid_o && (ex_result === mlo(A, B)), "mul value stable while stalled");
    allow_next = 1'b1;
    step;
    chk(!valid_o, "mul left after allow_next");

    // ------------------------------------- an ALU behind a mul must not pass
    A = 32'h0000_ffff; B = 32'h0001_0000;
    send(12'b0, 7'b1 << MUL_W, A, B, 5'd6);          // mul, stage 0 next cycle
    send(12'b1 << 6, 7'b0, 32'hf0f0_f0f0, 32'h0f0f_0f0f, 5'd8); // ALU right behind
    // after the two sends: mul@1 (the last stage), alu@0
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

    // --------------------------------------- div starts during a stalled pipe
    // The divider must be started as soon as the operands are in stage 0, even
    // when stage 1 is blocked (then the division overlaps the stall), and a
    // div that sits in stage 0 must be started exactly once.
    A = 32'd100; B = 32'd7;
    allow_next = 1'b0;
    send(12'b1 << 4, 7'b0, 32'd0, 32'd0, 5'd0);   // one and fills stage 1, EX is two deep
    div_starts = 0; div_watch = 1'b1;
    send(12'b0, 7'b1 << DIV_W, A, B, 5'd15);      // div lands in stage 0, blocked
    step;
    chk(u_ex.div_busy, "div starts while the downstream is blocked");
    chk(stall_o, "blocked div still holds stage 0");
    allow_next = 1'b1;
    begin : div_early
        integer i;
        reg     got;
        got = 1'b0;
        for (i = 0; i < 40 && !got; i = i + 1) begin
            step;
            if (valid_o && (dest_o === 5'd15)) got = 1'b1;
        end
        div_watch = 1'b0;
        chk(got && (ex_result === 32'd14), "div started during the stall still yields 100/7");
        chk(div_starts == 1, "a held div is started exactly once");
    end
    repeat (2) step;

    // ----------------------------------- two divs back to back (both finish)
    // This is what the allow/next_valid split is for: the younger div waits in
    // stage 0 (div_hold) until the older one has left, while stage 1 keeps
    // moving.  With the older ==stall_pipeline[1]= hold this deadlocks: the
    // older div is frozen in stage 1 and the younger one never starts.
    send(12'b0, 7'b1 << DIV_W, 32'd100, 32'd7, 5'd20);   // 100/7 = 14
    send(12'b0, 7'b1 << DIV_W, 32'd81,  32'd9, 5'd21);   // 81/9  = 9
    begin : div_pair
        integer i;
        reg     got20, got21;
        got20 = 1'b0; got21 = 1'b0;
        for (i = 0; i < 80 && !(got20 && got21); i = i + 1) begin
            step;
            if (valid_o && (dest_o === 5'd20) && (ex_result === 32'd14)) got20 = 1'b1;
            if (valid_o && (dest_o === 5'd21) && (ex_result === 32'd9 )) got21 = 1'b1;
        end
        chk(got20, "back-to-back div: first result 100/7");
        chk(got21, "back-to-back div: second result 81/9");
    end
    repeat (2) step;

    // ------------------------------------------------- branch unit (stage 0)
    // The compare operands are the registered ALU operands; jirl is always
    // taken.  br_target_o = base + offs, and br_taken_o is only set while the
    // branch really leaves stage 0.
    send_br(7'b1 << BEQ, 32'd5, 32'd5, 32'h1c002000, 32'h8);
    chk(br_taken_o && (br_target_o === 32'h1c002008), "beq: equal is taken, target = base + offs");
    repeat (3) step;

    send_br(7'b1 << BNE, 32'd5, 32'd5, 32'h1c002000, 32'h8);
    chk(!br_taken_o, "bne: equal is not taken");
    repeat (3) step;

    send_br(7'b1 << BLT, 32'hffff_ffff, 32'd1, 32'h0, 32'h4);
    chk(br_taken_o, "blt: -1 < 1 (signed)");
    repeat (3) step;

    send_br(7'b1 << BGE, 32'hffff_ffff, 32'd1, 32'h0, 32'h4);
    chk(!br_taken_o, "bge: -1 >= 1 (signed) is false");
    repeat (3) step;

    send_br(7'b1 << BLTU, 32'hffff_ffff, 32'd1, 32'h0, 32'h4);
    chk(!br_taken_o, "bltu: 0xffffffff < 1 (unsigned) is false");
    repeat (3) step;

    send_br(7'b1 << BGEU, 32'hffff_ffff, 32'd1, 32'h0, 32'h4);
    chk(br_taken_o, "bgeu: 0xffffffff >= 1 (unsigned)");
    repeat (3) step;

    send_br(7'b1 << JIRL, 32'd0, 32'd0, 32'h1c003000, 32'h4);
    chk(br_taken_o && (br_target_o === 32'h1c003004), "jirl: always taken, target = rj + offs");
    repeat (3) step;

    send(12'b1 << 4, 7'b0, 32'd1, 32'd1, 5'd3);   // and
    chk(!br_taken_o, "no branch op: no redirect");
    repeat (3) step;

    // A branch that stage 1 holds back redirects right away (there is no reason
    // to wait for the pipeline) and only once: the instruction stays in stage 0
    // until the pipeline moves, but must not flush the target instruction again.
    A = 32'h1c004000; B = 32'h8;
    allow_next = 1'b0;
    send(12'b1 << 4, 7'b0, 32'd0, 32'd0, 5'd0);   // one and fills stage 1, EX is two deep
    send_br(7'b1 << BEQ, 32'd7, 32'd7, A, B);     // branch lands in stage 0, held
    chk(br_taken_o && (br_target_o === (A + B)), "held branch redirects right away");
    step;
    chk(!br_taken_o && stall_o, "held branch redirects only once");
    allow_next = 1'b1;
    repeat (4) step;

    // ------------------------------------------------- forwarding groups
    // An ALU result is forwarded from all three stages: stage 0 off the
    // combinational ALU output, stages 1/2 off the registered copies.  Hold
    // allow_next low so it walks through the stages instead of short-circuiting
    // out of stage 0.  send() points fw_raddr1_i at the dest, i.e. it pretends
    // the instruction in ID reads what the sent one writes.
    A = 32'h1234_5678; B = 32'h0000_00ff;
    allow_next = 1'b0;
    send(12'b1 << 4, 7'b0, A, B, 5'd21);   // and
    chk(fw1_hit_o && fw1_valid_o && (fw1_data_o === (A & B)),
        "ALU stage 0 supplies off the combinational result");
    step;
    chk(fw1_valid_o && (fw1_data_o === (A & B)) && valid_o,
        "ALU stage 1 supplies and drives the output");
    allow_next = 1'b1;
    repeat (2) step;

    // an ALU in stage 1 is the output stage, so its value is presented as soon
    // as it is registered -- it does not wait for allow_next
    A = 32'h0f0f_0f0f; B = 32'h00ff_00ff;
    allow_next = 1'b0;
    send(12'b1 << 4, 7'b0, A, B, 5'd28);
    step;                                   // -> stage 1
    chk(valid_o && (ex_result === (A & B)) && (dest_o === 5'd28) && !stall_o,
        "alu in stage 1 drives the output (under back-pressure)");
    allow_next = 1'b1;
    repeat (2) step;

    // mul is not in the EX -> ID forwarding path: it hits (ID must wait for the
    // MEM tap) but never supplies from inside EX
    A = 32'h0000_0003; B = 32'h0000_0005;
    allow_next = 1'b0;
    send(12'b0, 7'b1 << MUL_W, A, B, 5'd22);
    chk(fw1_hit_o && !fw1_valid_o, "mul stage 0: hit, never supplies");
    step; chk(fw1_hit_o && !fw1_valid_o && valid_o,
              "mul stage 1: hit, never supplies (drives the output)");
    allow_next = 1'b1;
    repeat (2) step;

    // div is not in the EX -> ID forwarding path either: it hits but never
    // supplies, so the consumer takes it from the MEM tap
    send(12'b0, 7'b1 << DIV_W, 32'd100, 32'd7, 5'd23);
    begin : div_fwd
        integer i;
        reg     held;
        held = 1'b0;
        for (i = 0; i < 40 && !valid_o; i = i + 1) begin
            step;
            if (fw1_hit_o && ! fw1_valid_o) held = 1'b1;
        end
        chk(held && valid_o && (ex_result === 32'd14) && (dest_o === 5'd23),
            "div pending in EX, result goes out to MEM");
    end
    repeat (2) step;

    // ------------------------------------------- forwarding: WAW in flight
    // Two instructions write the SAME register with DIFFERENT values and are
    // both inside EX, while the instruction in ID reads that register.  Only the
    // YOUNGER writer's value may be handed to ID; and while the youngest writer
    // is still in stage 0 (its ALU result is not registered yet) EX must report
    // "pending" (hit && !valid) so the top stalls ID.  Reporting valid with the
    // older writer's registered value would hand ID a stale operand, so the
    // "youngest hit wins" priority cannot be dropped.
    //
    // EX is two deep, so the overlap is: younger @stage 0, older @stage 1 (the
    // output stage).  allow_next = 0 keeps the older one parked there, and the
    // ALU short-circuit is blocked because stage 1 already holds an ALU.
    A = 32'hff00_ff00; B = 32'h0ff0_0ff0;
    OLD   = A & B;                     // 0x0f00_0f00
    YOUNG = A ^ B;                     // 0xf0f0_f0f0, not a superset of OLD
    allow_next = 1'b0;
    send(12'b1 << 4, 7'b0, A, B, 5'd5);       // older  write to r5: and
    send(12'b1 << 7, 7'b0, A, B, 5'd5);       // younger write to r5: xor
    // S0: younger @stage0, older @stage1
    chk(fw1_hit_o && fw2_hit_o, "WAW S0: both writers hit, ID reads r5");
    chk(fw1_valid_o && fw2_valid_o && (fw1_data_o === YOUNG) && (fw2_data_o === YOUNG),
        "WAW S0: youngest writer in stage 0 supplies the combinational result");
    chk(valid_o && (dest_o === 5'd5) && (ex_result === OLD),
        "WAW S0: EX output keeps program order (older writer first)");
    // let the older writer drain; the younger one is then alone in stage 1
    allow_next = 1'b1;
    step;
    // S1: younger @stage1, the output stage
    chk(fw1_valid_o && (fw1_data_o === YOUNG), "WAW S1: fw1 supplies the younger writer");
    chk(fw2_valid_o && (fw2_data_o === YOUNG), "WAW S1: fw2 supplies the younger writer");
    chk(valid_o && (dest_o === 5'd5) && (ex_result === YOUNG),
        "WAW S1: younger writer leaves EX second");
    step;
    chk(!valid_o, "WAW S2: nothing left in EX");
    repeat (2) step;

    // ------------------------------------------------- exp12/13: CSR payload
    // The payload must ride out of EX together with whichever stage drives the
    // output (here the ALU short-circuits straight out of stage 0).
    @(negedge clk);
    valid_i = 1'b1;
    alu_op_i = 12'b1 << 6; md_op_i = 7'b0; src1_i = 32'd0; src2_i = 32'd0;
    dest_i = 5'd5; src1_used_i = 1'b0; src2_used_i = 1'b0; gr_we_i = 1'b1;
    res_from_mem_i = 1'b0; mem_we_i = 1'b0; mem_en_i = 1'b0; mem_wdata_i = 32'b0;
    mem_size_i = 3'b0; mem_zext_i = 1'b0;
    br_op_i = 7'b0; br_base_i = 32'b0; br_offs_i = 32'b0;
    csr_num_i = 14'h41; csr_ctl_i = 8'b1;
    csr_wdata_i = 32'hdead_beef; csr_mask_i = 32'h0000_ffff;
    while (stall_o) @(negedge clk);
    step;
    valid_i = 1'b0;
    chk(valid_o && (csr_num_o === 14'h41) && csr_ctl_o[0]
        && (csr_wdata_o === 32'hdead_beef) && (csr_mask_o === 32'h0000_ffff),
        "CSR payload rides out with the instruction");
    repeat (2) step;
    csr_num_i = 14'd0; csr_ctl_i = 8'd0; csr_wdata_i = 32'd0; csr_mask_i = 32'd0;

    // A stage that is held (allow = 0) must still be cleared by the flush: the
    // clear has to bypass allow[], otherwise the killed instruction survives and
    // is handed to MEM one cycle later, when flush[EX] is already low again.
    A = 32'h0000_ffff; B = 32'h0001_0000;
    allow_next = 1'b0;
    send(12'b0, 7'b1 << MUL_W, A, B, 5'd30);
    step;                                   // parked in stage 1, downstream held
    chk(valid_o && u_ex.valid[1], "mul parked in stage 1 under back-pressure");
    flush_i = 1'b1;
    step;
    flush_i = 1'b0;
    chk(!valid_o && !u_ex.valid[1], "flush clears a held stage 1 (allow_next = 0)");
    allow_next = 1'b1;
    repeat (2) step;

    // ------------------------------------------- exp12/13: global flush
    // A flush drops whatever EX holds and resets the mul/div in flight; the
    // very next instruction must come out clean (nothing stale left behind).
    A = 32'h0000_ffff; B = 32'h0001_0000;
    allow_next = 1'b0;
    send(12'b0, 7'b1 << MUL_W, A, B, 5'd24);
    step;                                       // mul is walking through EX
    flush_i = 1'b1;
    step;
    flush_i = 1'b0;
    chk(!valid_o, "flush empties EX");
    chk(!u_ex.mul_pos[0] && !u_ex.mul_pos[1], "flush clears mul_pos");
    allow_next = 1'b1;
    repeat (2) step;
    send(12'b0, 7'b1 << MUL_W, A, B, 5'd25);
    step;
    chk(valid_o && (ex_result === mlo(A, B)) && (dest_o === 5'd25),
        "a mul after the flush is clean");
    repeat (2) step;

    send(12'b0, 7'b1 << DIV_W, 32'd100, 32'd7, 5'd26);
    step;                                       // divider now busy
    chk(u_ex.div_busy, "divider busy before the flush");
    flush_i = 1'b1;
    step;
    flush_i = 1'b0;
    chk(!u_ex.div_busy, "flush resets the divider");
    chk(!valid_o, "flush empties EX (div case)");
    repeat (2) step;

    // The flush must take effect in the same cycle, not just at the next edge:
    // a flushed instruction must not look like it is leaving EX.  Otherwise a
    // flushed branch still drives br_taken_o -> nextpc, and a flushed
    // instruction is still offered to MEM as the EX->MEM hand-off.
    send_br(7'b1 << BEQ, 32'd5, 32'd5, 32'h1c002000, 32'h8);
    chk(br_taken_o && valid_o, "live branch in stage 0 before the flush");
    flush_i = 1'b1;
    step;
    chk(!br_taken_o, "flush drops the branch at the next edge");
    flush_i = 1'b0;
    step; step;
    do_div(7'b1 << DIV_W, 32'd100, 32'd7, 5'd27, 32'd14);

    $display("==========================================");
    if (errors == 0) $display("PASS: %0d checks, 0 errors", checks);
    else             $display("FAIL: %0d checks, %0d errors", checks, errors);
    $display("==========================================");
    $finish;
end

endmodule
