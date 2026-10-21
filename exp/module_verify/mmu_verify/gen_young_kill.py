# an instruction that raises an exception is only flushed when it reaches WB; the
# younger store in MEM at that moment must not reach memory.  VA 0x20 (LO0) is
# mapped with V=0 so the first store faults, VA 0x1000 (LO1) is valid and points at
# PA 0x101000: the handler reads that PA and must see 0 (the younger store died).
def addi_w(rd,rj,s): return ((0x00A<<22)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def lu12i(rd,s):     return ((0x14000000)|((s&0xFFFFF)<<5)|rd)&0xFFFFFFFF
def ori(rd,rj,u):    return ((0x03800000)|((u&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def ld_w(rd,rj,s):   return ((0x28800000)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def st_w(rd,rj,s):   return ((0x29800000)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def csrrd(rd,csr):   return ((0x04000000)|((csr&0x3FFF)<<10)|rd)&0xFFFFFFFF
def csrwr(rd,csr):   return ((0x04000000)|((csr&0x3FFF)<<10)|(1<<5)|rd)&0xFFFFFFFF
def csrxchg(rd,rj,csr): return ((0x04000000)|((csr&0x3FFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
TLBIDX,TLBEHI,TLBELO0,TLBELO1,EENTRY,CRMD,ERA = 0x10,0x11,0x12,0x13,0x0c,0x00,0x06
TLBWR = 0x06483000
def li(rd,val): return [lu12i(rd, val>>12), ori(rd, rd, val & 0xFFF)]

HANDLER = 0x1c000200
prog  = []
prog += li(5, 0x101000) + [st_w(0, 5, 0)]                       # PA 0x101000 = 0
prog += li(12, HANDLER) + [csrwr(12, EENTRY)]
prog += li(13, 0xBB)                                            # store data
# entry 0: code page, V=1 D=1, PPN 0x1c000
prog += li(8, 0x0C000000) + [csrwr(8, TLBIDX)]
prog += li(7, (0x1C000 << 8) | 0x03) + [csrwr(7, TLBELO0), csrwr(7, TLBELO1)]
prog += li(6, 0x1C000000) + [csrwr(6, TLBEHI), TLBWR]
# entry 1: VPPN 0 -> LO0 V=0 (VA 0x20 faults), LO1 PPN 0x101 V=1 D=1 (VA 0x1000 ok)
prog += li(8, 0x0C000001) + [csrwr(8, TLBIDX)]
prog += li(7, 0x00)                     + [csrwr(7, TLBELO0)]      # LO0: V=0
prog += li(7, (0x101 << 8) | 0x03)      + [csrwr(7, TLBELO1)]      # LO1: PPN 0x101, V=1 D=1
prog += li(6, 0x0)                      + [csrwr(6, TLBEHI), TLBWR]
# mapped mode, PLV 0
prog += li(12, 0x10) + li(1, 0x1f) + [csrxchg(12, 1, CRMD)]
prog += li(3, 0x20) + li(4, 0x1000)
prog += [st_w(13, 3, 0)]            # faults (V=0) -> PIS at WB
prog += [st_w(13, 4, 0)]            # younger: must be killed
prog += [0x50000000]                # ERA+8 lands here
# handler: read PA 0x100000, make ERA point past both stores, ertn
handler = [ld_w(16, 4, 0), csrrd(17, ERA), addi_w(17, 17, 8),
           csrwr(17, ERA), 0x06483800]
assert len(prog) <= 0x80, len(prog)
prog += [0] * (0x80 - len(prog)) + handler

print(open('tb_head.v').read())
print("initial begin")
for i, w in enumerate(prog):
    print("    slave.mem[13'h%04x] = 32'h%08x;" % (0x1000 + i, w))
print("""    resetn = 0; repeat (4) @(posedge clk); resetn = 1;
    repeat (400) @(posedge clk);
    $display("YOUNG_KILL: r16=%h mem@PA101000=%h (expect 0 0)",
             dut.u_regfile.rf[16], slave.mem[15'h400]);
    $finish;
end
endmodule""")
