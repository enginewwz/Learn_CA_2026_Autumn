# exception entry fields: ERA / ESTAT / PRMD / CRMD must not be disturbed
# (a non-refill exception must leave DA/PG alone, n47 checks exactly this)
def addi_w(rd,rj,s): return ((0x00A<<22)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def lu12i(rd,s):     return ((0x14000000)|((s&0xFFFFF)<<5)|rd)&0xFFFFFFFF
def ori(rd,rj,u):    return ((0x03800000)|((u&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def csrrd(rd,csr):   return ((0x04000000)|((csr&0x3FFF)<<10)|rd)&0xFFFFFFFF
def csrwr(rd,csr):   return ((0x04000000)|((csr&0x3FFF)<<10)|(1<<5)|rd)&0xFFFFFFFF
def st_w(rd,rj,s):   return ((0x29800000)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
EENTRY, ESTAT, CRMD, PRMD, ERA = 0x0c, 0x05, 0x00, 0x01, 0x06
SYSCALL, BREAK = 0x002B0000, 0x002A0000
prog = [ lu12i(12, 0x1c000), ori(12, 12, 0x200), csrwr(12, EENTRY),
         addi_w(1, 0, 0x11),
         SYSCALL,
         csrrd(3, CRMD), st_w(3, 0, 0x70),   # back from the handler: DA=1, PG=0 (0x8)
         0x50000000 ]
handler = [ csrrd(20, ESTAT), st_w(20, 0, 0x60), csrrd(20, CRMD), st_w(20, 0, 0x64),
            csrrd(20, PRMD),  st_w(20, 0, 0x68), csrrd(20, ERA),  st_w(20, 0, 0x6c),
            addi_w(20, 20, 4), csrwr(20, ERA), 0x06483800 ]   # ERA+=4, ertn
print(open('tb_head.v').read())
print("initial begin")
for i, w in enumerate(prog):
    print("    slave.mem[13'h%04x] = 32'h%08x;" % (0x1000 + i, w))
for i, w in enumerate(handler):
    print("    slave.mem[13'h%04x] = 32'h%08x;" % (0x1080 + i, w))
print("""    resetn = 0; repeat (4) @(posedge clk); resetn = 1;
    repeat (400) @(posedge clk);
    $display("EXCP_ENTRY: ESTAT=%h CRMD=%h PRMD=%h ERA=%h r1=%h CRMD_after_ertn=%h (expect 000b0000 00000008 0 1c000010 11 00000008)",
             slave.mem[24], slave.mem[25], slave.mem[26], slave.mem[27],
             dut.u_regfile.rf[1], slave.mem[28]);
    $finish;
end
endmodule""")
