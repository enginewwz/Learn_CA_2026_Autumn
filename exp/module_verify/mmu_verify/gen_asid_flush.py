# csrwr ASID changes the TLB search key, so it must flush and re-fetch: a load right
# behind it has to see the NEW ASID's mapping.  VA 0x20 is mapped to PA 0x20 for
# ASID 0 and to PA 0x101020 for ASID 1; the load after the ASID write must read the
# ASID=1 value.
def addi_w(rd,rj,s): return ((0x00A<<22)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def lu12i(rd,s):     return ((0x14000000)|((s&0xFFFFF)<<5)|rd)&0xFFFFFFFF
def ori(rd,rj,u):    return ((0x03800000)|((u&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def ld_w(rd,rj,s):   return ((0x28800000)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def st_w(rd,rj,s):   return ((0x29800000)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def csrwr(rd,csr):   return ((0x04000000)|((csr&0x3FFF)<<10)|(1<<5)|rd)&0xFFFFFFFF
def csrxchg(rd,rj,csr): return ((0x04000000)|((csr&0x3FFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
TLBIDX, TLBEHI, TLBELO0, TLBELO1, ASID, CRMD, EENTRY = 0x10,0x11,0x12,0x13,0x18,0x00,0x0c
TLBWR = 0x06483000

def li(rd, val):                       # 32 bit constant, lu12i + ori
    return [lu12i(rd, val >> 12), ori(rd, rd, val & 0xFFF)]

def fill(idx, vppn, tlbelo, asid):
    p  = li(8, 0x0C000000 | idx) + [csrwr(8, TLBIDX)]
    p += li(7, asid)            + [csrwr(7, ASID)]
    p += li(6, vppn)            + [csrwr(6, TLBEHI)]
    p += li(5, tlbelo)          + [csrwr(5, TLBELO0), csrwr(5, TLBELO1), TLBWR]
    return p

CODE_LO = (0x1C000 << 8) | 0x03        # PPN 0x1c000, V=1 D=1
DATA0   = (0x00000 << 8) | 0x03        # PPN 0,      V=1 D=1
DATA1   = (0x00101 << 8) | 0x03        # PPN 0x101,  V=1 D=1

prog  = li(1, 0x20) + li(2, 0x1111) + [st_w(2, 1, 0)]          # PA 0x20 = 0x1111
prog += li(3, 0x101020) + li(4, 0x2222) + [st_w(4, 3, 0)]      # PA 0x101020 = 0x2222
prog += li(15, 0x1C000200) + [csrwr(15, EENTRY)]
prog += fill(0, 0x1C000000, CODE_LO, 0)                        # code page, ASID 0
prog += fill(1, 0x00000000, DATA0,   0)                        # data page 0, ASID 0
prog += fill(2, 0x00000000, DATA1,   1)                        # data page 0, ASID 1
prog += fill(3, 0x1C000000, CODE_LO, 1)                        # code page, ASID 1
prog += li(7, 0) + [csrwr(7, ASID)]                            # ASID 0
prog += li(12, 0x10) + li(13, 0x1f) + [csrxchg(12, 13, CRMD)]  # mapped mode, PLV 0
prog += [ld_w(9, 1, 0)]                                        # ASID 0 -> 0x1111
prog += li(7, 1) + [csrwr(7, ASID)]                            # <-- flush under test
prog += [ld_w(10, 1, 0)]                                       # ASID 1 -> 0x2222
prog += [0x50000000]

print(open('tb_head.v').read())
print("initial begin")
for i, w in enumerate(prog):
    print("    slave.mem[13'h%04x] = 32'h%08x;" % (0x1000 + i, w))
print("""    resetn = 0; repeat (4) @(posedge clk); resetn = 1;
    repeat (800) @(posedge clk);
    $display("ASID_FLUSH: r9=%h r10=%h (expect 1111 2222)", dut.u_regfile.rf[9], dut.u_regfile.rf[10]);
    $finish;
end
endmodule""")
