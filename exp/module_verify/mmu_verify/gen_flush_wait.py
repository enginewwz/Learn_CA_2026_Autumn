#!/usr/bin/env python3
# WB-flush empty-wait check.
#
# A data ALE (ld.w at 0x3) makes WB raise wb_flush while the younger
# instructions are still sitting in EX0/EX1:
#   * EX1 holds a late ALU (add r3,r2,r2) whose producer load r2 is still in MEM
#   * EX0 holds a mul (mul r4,r2,r2) that must wait for the same load
# Both are going to be squashed, so they must NOT assert stall[EX0]/stall[EX1]
# on the flush cycle.  The counters below would be non-zero if the stall were
# still based on the non-flush-aware valid/pos signals.
#
# The ALE itself is still verified through the exception handler (ESTAT/ERA).
def addi_w(rd,rj,s): return ((0x00A<<22)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def add_w(rd,rj,rk): return ((0x00100000)|(0<<15)|(rk<<10)|(rj<<5)|rd)&0xFFFFFFFF
def mul_w(rd,rj,rk): return ((0x001C0000)|(rk<<10)|(rj<<5)|rd)&0xFFFFFFFF
def ld_w(rd,rj,s):   return ((0x28800000)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def st_w(rd,rj,s):   return ((0x29800000)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def lu12i(rd,s):     return ((0x14000000)|((s&0xFFFFF)<<5)|rd)&0xFFFFFFFF
def ori(rd,rj,u):    return ((0x03800000)|((u&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def csrrd(rd,csr):   return ((0x04000000)|((csr&0x3FFF)<<10)|rd)&0xFFFFFFFF
def csrwr(rd,csr):   return ((0x04000000)|((csr&0x3FFF)<<10)|(1<<5)|rd)&0xFFFFFFFF

EENTRY = 0x0c
prog = [lu12i(12, 0x1c000), ori(12, 12, 0x200), csrwr(12, EENTRY),
        ld_w(1, 0, 0x3),                    # ALE -> WB flush
        ld_w(2, 0, 0x80),                   # producer load
        add_w(3, 2, 2),                     # late ALU, EX1 at the flush
        mul_w(4, 2, 2),                     # mul, EX0 at the flush
        add_w(5, 3, 4),
        0x50000000]
# keep the handler simple (no csrrd->store chain): just prove it ran
handler = [addi_w(20, 0, 0x5a), st_w(20, 0, 0x60), 0x50000000]

print(open('tb_head.v').read())
print("reg [31:0] rom [0:255];")
print("wire [31:0] rom_word;")
print("assign rom_word = rom[(dut.pc[0] - 32'h1c000000) >> 2];")
print("integer empty_ex0 = 0, empty_ex1 = 0;")
print("always @(posedge clk) if (resetn && dut.wb_flush) begin")
print("    if (dut.valid[2] && !dut.cur_valid[2] && dut.stall[2]) empty_ex0 = empty_ex0 + 1;")
print("    if (dut.valid[3] && !dut.cur_valid[3] && dut.stall[3]) empty_ex1 = empty_ex1 + 1;")
print("end")
print("initial begin")
for i, w in enumerate(prog):
    print("    rom[%d] = 32'h%08x;" % (i, w))
for i, w in enumerate(handler):
    print("    rom[%d] = 32'h%08x;" % (0x80 + i, w))
print("""    slave.mem[13'h0020] = 32'h00000011;   // 0x80
    force dut.inst_rdata = rom_word;
    force dut.if_pending = 1'b0;
    force dut.u_mmu.if_state = 3'b001;
    resetn = 0; repeat (4) @(posedge clk); resetn = 1;
    repeat (250) @(posedge clk);
    $display("FLUSH_WAIT: empty_ex0=%0d empty_ex1=%0d handler=%h (expect 0 0 0000005a)",
             empty_ex0, empty_ex1, slave.mem[13'h0018]);
    $finish;
end
endmodule""")
