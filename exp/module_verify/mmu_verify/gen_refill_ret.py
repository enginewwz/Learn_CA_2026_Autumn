# n71 shape: mapped mode -> TLBR -> handler skips the instruction -> ertn must put
# the core back in mapped mode (CRMD keeps DATM, DA=0/PG=1)
# note: csrxchg keeps its mask register in inst[9:5], so rj must not be r0/r1
# (those encodings are csrrd/csrwr) -- use r13 as the mask register.
def addi_w(rd,rj,s): return ((0x00A<<22)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def lu12i(rd,s):     return ((0x14000000)|((s&0xFFFFF)<<5)|rd)&0xFFFFFFFF
def ori(rd,rj,u):    return ((0x03800000)|((u&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def ld_w(rd,rj,s):   return ((0x28800000)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def csrrd(rd,csr):   return ((0x04000000)|((csr&0x3FFF)<<10)|rd)&0xFFFFFFFF
def csrwr(rd,csr):   return ((0x04000000)|((csr&0x3FFF)<<10)|(1<<5)|rd)&0xFFFFFFFF
def csrxchg(rd,rj,csr): return ((0x04000000)|((csr&0x3FFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
TLBIDX, TLBEHI, TLBELO0, TLBELO1, TLBRENTRY, EENTRY, CRMD, ERA = 0x10,0x11,0x12,0x13,0x88,0x0c,0x00,0x06
prog = [
 lu12i(7,0xc00), lu12i(3,0x1C00), ori(3,3,3), lu12i(4,0x1c000),
 csrwr(7,TLBIDX), csrwr(4,TLBEHI), csrwr(3,TLBELO0), csrwr(3,TLBELO1), 0x06483000,
 lu12i(14,0x1c000), ori(14,14,0x200), csrwr(14,TLBRENTRY), csrwr(14,EENTRY),
 addi_w(12,0,0xa0), addi_w(13,0,0x180), csrxchg(12,13,CRMD),   # DATM[0] = 1
 lu12i(5,8),                                                   # r5 = 0x8000, unmapped
 addi_w(12,0,0x10), addi_w(13,0,0x1f), csrxchg(12,13,CRMD),    # PG=1, DA=0
 ld_w(4,5,0),                                                  # -> TLBR, skipped by the handler
 addi_w(6,0,0x66),
 csrrd(7,CRMD),                                                # expect 0x90
 0x50000000 ]
handler = [ csrrd(20,ERA), addi_w(20,20,4), csrwr(20,ERA), 0x06483800 ]   # ertn
print(open('tb_head.v').read())
print("initial begin")
for i, w in enumerate(prog):
    print("    slave.mem[13'h%04x] = 32'h%08x;" % (0x1000 + i, w))
for i, w in enumerate(handler):
    print("    slave.mem[13'h%04x] = 32'h%08x;" % (0x1080 + i, w))
print("""    resetn = 0; repeat (4) @(posedge clk); resetn = 1;
    repeat (500) @(posedge clk);
    $display("REFILL_RET: CRMD=%h r4=%h r6=%h (expect 00000090 x 66)", dut.u_regfile.rf[7],
             dut.u_regfile.rf[4], dut.u_regfile.rf[6]);
    $finish;
end
endmodule""")
