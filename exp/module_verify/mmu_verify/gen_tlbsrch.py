# TLBSRCH goes through the dedicated s2 port now: check that it reports the
# *index of the matching entry* (entry 5 here, not 0) and that a miss only sets NE
def addi_w(rd,rj,s): return ((0x00A<<22)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def lu12i(rd,s):     return ((0x14000000)|((s&0xFFFFF)<<5)|rd)&0xFFFFFFFF
def ori(rd,rj,u):    return ((0x03800000)|((u&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def csrrd(rd,csr):   return ((0x04000000)|((csr&0x3FFF)<<10)|rd)&0xFFFFFFFF
def csrwr(rd,csr):   return ((0x04000000)|((csr&0x3FFF)<<10)|(1<<5)|rd)&0xFFFFFFFF
def st_w(rd,rj,s):   return ((0x29800000)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
TLBIDX, TLBEHI, TLBELO0, TLBELO1, ESTAT = 0x10, 0x11, 0x12, 0x13, 0x05
TLBWR, TLBSRCH, INVTLB = 0x06483000, 0x06482800, 0x06498000
prog = [
 lu12i(12, 0x1c000), ori(12, 12, 0x200), csrwr(12, 0x0c),      # EENTRY (so a stray exception shows up)
 # fill entry 5: VPPN=0x1c000000, same page as the code
 lu12i(8, 0xC000), ori(8, 8, 5), csrwr(8, TLBIDX),               # TLBIDX: NE=0, PS=12, Index=5
 lu12i(9, 0x1c00), csrwr(9, TLBEHI),                            # TLBEHI = 0x1c000000  (VPPN = 0xe000)
 lu12i(10, 0x1C00), ori(10, 10, 3), csrwr(10, TLBELO0), csrwr(10, TLBELO1),
 TLBWR,
 # poison TLBIDX, then search: index must come back as 5
 lu12i(8, 0xC000), ori(8, 8, 0x10), csrwr(8, TLBIDX),            # NE=1, Index=0
 lu12i(9, 0x1c00), csrwr(9, TLBEHI),
 TLBSRCH,
 csrrd(3, TLBIDX), st_w(3, 0, 0x60),                            # expect 0x0c000005
 # search an unmapped page: NE=1, index must stay 5
 lu12i(9, 0x20), csrwr(9, TLBEHI),                              # VPPN = 1
 TLBSRCH,
 csrrd(4, TLBIDX), st_w(4, 0, 0x64),                            # expect 0x8c000005
 0x50000000 ]
print(open('tb_head.v').read())
print("initial begin")
for i, w in enumerate(prog):
    print("    slave.mem[13'h%04x] = 32'h%08x;" % (0x1000 + i, w))
print("""    resetn = 0; repeat (4) @(posedge clk); resetn = 1;
    repeat (400) @(posedge clk);
    $display("TLBSRCH: hit TLBIDX=%h (expect 0c000005)  miss TLBIDX=%h (expect 8c000005)",
             slave.mem[24], slave.mem[25]);
    $finish;
end
endmodule""")
