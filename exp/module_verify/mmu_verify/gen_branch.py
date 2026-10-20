# far branch right after a fetch: the sequential fetch must not leak its data
def addi_w(rd,rj,s): return ((0x00A<<22)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def b(offs26):       return ((0x14<<26)|((offs26&0xFFFF)<<10)|((offs26>>16)&0x3FF))&0xFFFFFFFF
prog = { 0x1000: addi_w(1,0,0x11), 0x1001: b((0x100-4)>>2),
         0x1002: addi_w(2,0,0x22), 0x1003: addi_w(3,0,0x33),
         0x1040: addi_w(4,0,0x44), 0x1041: 0x50000000 }
print(open('tb_head.v').read())
print("initial begin")
for a, w in sorted(prog.items()):
    print("    slave.mem[13'h%04x] = 32'h%08x;" % (a, w))
print("""    resetn = 0; repeat (4) @(posedge clk); resetn = 1;
    repeat (300) @(posedge clk);
    $display("BR: r1=%h r2=%h r3=%h r4=%h (expect 11 x x 44)",
             dut.u_regfile.rf[1], dut.u_regfile.rf[2], dut.u_regfile.rf[3], dut.u_regfile.rf[4]);
    $finish;
end
endmodule""")
