#!/usr/bin/env python3
# 从课程给的 rtl/tlb_top.v 抽出 16 项写数据和 26 组搜索期望,生成 iverilog 版
# 等价性测试(写->读->搜索两路),用于确认 tlb.v 改结构后行为完全不变。
import os, re

HERE = os.path.dirname(os.path.abspath(__file__))
src = open(os.path.join(HERE, '..', 'rtl', 'tlb_top.v')).read()

def grab(name, n):
    out = []
    for k in range(n):
        m = re.search(r"assign\s+%s\[\s*%d\]\s*=\s*([^;]+);" % (name, k), src)
        if not m:
            raise SystemExit("missing %s[%d]" % (name, k))
        out.append(m.group(1).strip())
    return out

WF = [('e', 'w_e', 1), ('vppn', 'w_vppn', 19), ('ps', 'w_ps', 6), ('asid', 'w_asid', 10),
      ('g', 'w_g', 1), ('ppn0', 'w_ppn0', 20), ('plv0', 'w_plv0', 2), ('mat0', 'w_mat0', 2),
      ('d0', 'w_d0', 1), ('v0', 'w_v0', 1), ('ppn1', 'w_ppn1', 20), ('plv1', 'w_plv1', 2),
      ('mat1', 'w_mat1', 2), ('d1', 'w_d1', 1), ('v1', 'w_v1', 1)]
RF = [('e', 'r_e', 1), ('vppn', 'r_vppn', 19), ('asid', 'r_asid', 10), ('g', 'r_g', 1),
      ('ppn0', 'r_ppn0', 20), ('plv0', 'r_plv0', 2), ('mat0', 'r_mat0', 2), ('d0', 'r_d0', 1),
      ('v0', 'r_v0', 1), ('ppn1', 'r_ppn1', 20), ('plv1', 'r_plv1', 2), ('mat1', 'r_mat1', 2),
      ('d1', 'r_d1', 1), ('v1', 'r_v1', 1)]
SF = [('found', 1), ('index', 4), ('ppn', 20), ('ps', 6), ('plv', 2), ('mat', 2),
      ('d', 1), ('v', 1)]

W = {f: grab('tlb_' + f, 16) for f, _, _ in WF}
S = {f: grab('s_test_' + f, 26) for f, _ in SF}
S['va_bit12'] = grab('s_test_va_bit12', 26)
S['asid'] = grab('s_test_asid', 26)
S['vppn'] = grab('s_test_vppn', 26)

p = []
p.append("// generated from ../../rtl/tlb_top.v by gen_tlb_unit.py -- do not edit")
p.append("`timescale 1ns/1ps")
p.append("module tb_tlb_unit;")
p.append("reg clk = 0; always #5 clk = ~clk;")
p.append("reg reset = 1;")
p.append("""reg  [18:0] s0_vppn, s1_vppn; reg s0_va12, s1_va12; reg [9:0] s0_asid, s1_asid;
wire s0_found, s1_found; wire [3:0] s0_index, s1_index;
wire [19:0] s0_ppn, s1_ppn; wire [5:0] s0_ps, s1_ps;
wire [1:0] s0_plv, s1_plv, s0_mat, s1_mat; wire s0_d, s1_d, s0_v, s1_v;
reg we; reg [3:0] w_index; reg w_e, w_g, w_d0, w_v0, w_d1, w_v1;
reg [18:0] w_vppn; reg [5:0] w_ps; reg [9:0] w_asid;
reg [19:0] w_ppn0, w_ppn1; reg [1:0] w_plv0, w_mat0, w_plv1, w_mat1;
reg [3:0] r_index; wire r_e; wire [18:0] r_vppn; wire [5:0] r_ps; wire [9:0] r_asid;
wire r_g; wire [19:0] r_ppn0, r_ppn1; wire [1:0] r_plv0, r_mat0, r_plv1, r_mat1;
wire r_d0, r_v0, r_d1, r_v1;
tlb u_tlb(.clk(clk), .reset(reset),
    .s0_vppn(s0_vppn), .s0_va_bit12(s0_va12), .s0_asid(s0_asid), .s0_found(s0_found),
    .s0_index(s0_index), .s0_ppn(s0_ppn), .s0_ps(s0_ps), .s0_plv(s0_plv), .s0_mat(s0_mat),
    .s0_d(s0_d), .s0_v(s0_v),
    .s1_vppn(s1_vppn), .s1_va_bit12(s1_va12), .s1_asid(s1_asid), .s1_found(s1_found),
    .s1_index(s1_index), .s1_ppn(s1_ppn), .s1_ps(s1_ps), .s1_plv(s1_plv), .s1_mat(s1_mat),
    .s1_d(s1_d), .s1_v(s1_v),
    .invtlb_valid(1'b0), .invtlb_op(5'd0),
    .we(we), .w_index(w_index), .w_e(w_e), .w_vppn(w_vppn), .w_ps(w_ps), .w_asid(w_asid),
    .w_g(w_g), .w_ppn0(w_ppn0), .w_plv0(w_plv0), .w_mat0(w_mat0), .w_d0(w_d0), .w_v0(w_v0),
    .w_ppn1(w_ppn1), .w_plv1(w_plv1), .w_mat1(w_mat1), .w_d1(w_d1), .w_v1(w_v1),
    .r_index(r_index), .r_e(r_e), .r_vppn(r_vppn), .r_ps(r_ps), .r_asid(r_asid), .r_g(r_g),
    .r_ppn0(r_ppn0), .r_plv0(r_plv0), .r_mat0(r_mat0), .r_d0(r_d0), .r_v0(r_v0),
    .r_ppn1(r_ppn1), .r_plv1(r_plv1), .r_mat1(r_mat1), .r_d1(r_d1), .r_v1(r_v1));""")

# expected tables
for f, _, w in WF:
    p.append("wire [%d:0] exp_w_%s [0:15];" % (w - 1, f))
    for k in range(16):
        p.append("assign exp_w_%s[%d] = %s;" % (f, k, W[f][k]))
for f, _, w in RF:
    p.append("wire [%d:0] exp_r_%s [0:15];" % (w - 1, f))
    for k in range(16):
        p.append("assign exp_r_%s[%d] = %s;" % (f, k, W[f][k]))
for f, w in SF:
    p.append("wire [%d:0] exp_s_%s [0:25];" % (w - 1, f))
    for k in range(26):
        p.append("assign exp_s_%s[%d] = %s;" % (f, k, S[f][k]))
for nm in ['vppn', 'va_bit12', 'asid']:
    w = 19 if nm == 'vppn' else (1 if nm == 'va_bit12' else 10)
    p.append("wire [%d:0] exp_sv_%s [0:25];" % (w - 1, nm))
    for k in range(26):
        p.append("assign exp_sv_%s[%d] = %s;" % (nm, k, S[nm][k]))

p.append("integer i, k, errs = 0;")
p.append("initial begin")
p.append("    we = 0; w_index = 0; w_e = 0; w_vppn = 0; w_ps = 0; w_asid = 0; w_g = 0;")
p.append("    w_ppn0 = 0; w_plv0 = 0; w_mat0 = 0; w_d0 = 0; w_v0 = 0;")
p.append("    w_ppn1 = 0; w_plv1 = 0; w_mat1 = 0; w_d1 = 0; w_v1 = 0;")
p.append("    r_index = 0; s0_vppn = 0; s0_va12 = 0; s0_asid = 0;")
p.append("    s1_vppn = 0; s1_va12 = 0; s1_asid = 0;")
p.append("    repeat (2) @(posedge clk); reset = 0; @(negedge clk);")
p.append("    // write the 16 entries")
for k in range(16):
    p.append("    we = 1; w_index = 4'd%d;" % k)
    for f, sig, _ in WF:
        p.append("    %s = exp_w_%s[%d];" % (sig, f, k))
    p.append("    @(negedge clk);")
p.append("    we = 0; @(negedge clk);")
p.append("    // read back")
p.append("    for (k = 0; k < 16; k = k + 1) begin")
p.append("        @(negedge clk); r_index = k; #1;")
for f, sig, _ in RF:
    if f == 'ps':
        p.append("        if (r_ps !== ((exp_w_ps[k] == 6'd21) ? 6'd21 : 6'd12)) begin errs = errs + 1; $display(\"READ r_ps[%0d] = %h\", k, r_ps); end")
    else:
        p.append("        if (%s !== exp_r_%s[k]) begin errs = errs + 1; $display(\"READ %s[%%0d] = %%h exp %%h\", k, %s, exp_r_%s[k]); end"
                 % (sig, f, sig, sig, f))
p.append("    end")
p.append("    // 26 search vectors on both ports")
p.append("    for (k = 0; k < 26; k = k + 1) begin")
p.append("        s0_vppn = exp_sv_vppn[k]; s0_va12 = exp_sv_va_bit12[k]; s0_asid = exp_sv_asid[k];")
p.append("        s1_vppn = exp_sv_vppn[k]; s1_va12 = exp_sv_va_bit12[k]; s1_asid = exp_sv_asid[k];")
p.append("        #1;")
p.append("        if (s0_found !== exp_s_found[k]) begin errs = errs + 1; $display(\"S0 found[%0d] = %b exp %b\", k, s0_found, exp_s_found[k]); end")
p.append("        if (s1_found !== exp_s_found[k]) begin errs = errs + 1; $display(\"S1 found[%0d] = %b exp %b\", k, s1_found, exp_s_found[k]); end")
p.append("        if (s0_found) begin")
for f in ['index', 'ppn', 'plv', 'mat', 'd', 'v']:
    p.append("            if (s0_%s !== exp_s_%s[k]) begin errs = errs + 1; $display(\"S0 %s[%%0d] = %%h exp %%h\", k, s0_%s, exp_s_%s[k]); end" % (f, f, f, f, f))
p.append("            if (s0_ps !== ((exp_s_ps[k] == 6'd21) ? 6'd21 : 6'd12)) begin errs = errs + 1; $display(\"S0 ps[%0d] = %h\", k, s0_ps); end")
p.append("        end")
p.append("        if (s1_found) begin")
for f in ['index', 'ppn', 'plv', 'mat', 'd', 'v']:
    p.append("            if (s1_%s !== exp_s_%s[k]) begin errs = errs + 1; $display(\"S1 %s[%%0d] = %%h exp %%h\", k, s1_%s, exp_s_%s[k]); end" % (f, f, f, f, f))
p.append("            if (s1_ps !== ((exp_s_ps[k] == 6'd21) ? 6'd21 : 6'd12)) begin errs = errs + 1; $display(\"S1 ps[%0d] = %h\", k, s1_ps); end")
p.append("        end")
p.append("        @(negedge clk);")
p.append("    end")
p.append("    if (errs == 0) $display(\"TLB UNIT: PASS\");")
p.append("    else           $display(\"TLB UNIT: FAIL (%0d)\", errs);")
p.append("    $finish;")
p.append("end")
p.append("endmodule")
print("\n".join(p))
