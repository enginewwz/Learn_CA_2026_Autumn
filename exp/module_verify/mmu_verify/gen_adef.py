# ADEF on a fetch: jump to a misaligned address, check ESTAT/ERA/BADV
def addi_w(rd,rj,s): return ((0x00A<<22)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def lu12i(rd,s):     return ((0x14000000)|((s&0xFFFFF)<<5)|rd)&0xFFFFFFFF
def ori(rd,rj,u):    return ((0x03800000)|((u&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def jirl(rd,rj,o):   return ((0x4C000000)|((o&0xFFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def csrrd(rd,csr):   return ((0x04000000)|((csr&0x3FFF)<<10)|rd)&0xFFFFFFFF
def csrwr(rd,csr):   return ((0x04000000)|((csr&0x3FFF)<<10)|(1<<5)|rd)&0xFFFFFFFF
def st_w(rd,rj,s):   return ((0x29800000)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
EENTRY,ESTAT,ERA,BADV,PRMD = 0x0c,0x05,0x06,0x07,0x01
prog = [ lu12i(12,0x1c000), ori(12,12,0x200),   # EENTRY = 0x1c000200
         csrwr(12,EENTRY),
         addi_w(10,0,3),                        # r10 = 3 (misaligned)
         jirl(0,10,0),                          # fetch 3 -> ADEF
         0x50000000 ]
handler = [ csrrd(20,ESTAT), st_w(20,0,0x60), csrrd(20,ERA),  st_w(20,0,0x64),
            csrrd(20,BADV),  st_w(20,0,0x68), csrrd(20,PRMD), st_w(20,0,0x6c),
            0x50000000 ]
print(open('tb_head.v').read())
print("initial begin")
for i, w in enumerate(prog):
    print("    slave.mem[13'h%04x] = 32'h%08x;" % (0x1000 + i, w))
for i, w in enumerate(handler):
    print("    slave.mem[13'h%04x] = 32'h%08x;" % (0x1080 + i, w))
print("""    resetn = 0; repeat (4) @(posedge clk); resetn = 1;
    repeat (500) @(posedge clk);
    $display("ADEF: ESTAT=%h ERA=%h BADV=%h PRMD=%h",
             slave.mem[24], slave.mem[25], slave.mem[26], slave.mem[27]);
    $finish;
end
endmodule""")
