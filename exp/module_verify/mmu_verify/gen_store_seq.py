# back to back stores (both are registered at the end of EX now), then loads
def addi_w(rd,rj,s): return ((0x00A<<22)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def ld_w(rd,rj,s):   return ((0x28800000)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def st_w(rd,rj,s):   return ((0x29800000)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
prog = [addi_w(1,0,0x20), addi_w(2,0,0x11), addi_w(3,0,0x22),
        st_w(2,1,0), st_w(3,1,4),          # two stores back to back
        ld_w(4,1,0), ld_w(5,1,4),
        st_w(5,1,0), ld_w(6,1,0),          # store right behind a load
        0x50000000]
print(open('tb_head.v').read())
print("initial begin")
for i, w in enumerate(prog):
    print("    slave.mem[13'h%04x] = 32'h%08x;" % (0x1000 + i, w))
print("""    resetn = 0; repeat (4) @(posedge clk); resetn = 1;
    repeat (400) @(posedge clk);
    $display("ST_SEQ: mem20=%h mem24=%h r4=%h r5=%h r6=%h (expect 22 22 11 22 22)",
             slave.mem[8], slave.mem[9], dut.u_regfile.rf[4],
             dut.u_regfile.rf[5], dut.u_regfile.rf[6]);
    $finish;
end
endmodule""")
