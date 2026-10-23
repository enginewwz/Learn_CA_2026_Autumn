#!/usr/bin/env python3
# Forwarding regression for the ID branch / jump path: beq / bne / bl / jirl
# whose operands are produced by the immediately preceding instructions.
# Fetch is bypassed exactly like gen_fwd_alu.py (see the comment there).
def addi_w(rd,rj,s): return ((0x00A<<22)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def lu12i_w(rd,i20): return ((0x05<<26)|((i20&0xFFFFF)<<5)|rd)&0xFFFFFFFF
def b(offs26):       return ((0x14<<26)|((offs26&0xFFFF)<<10)|((offs26>>16)&0x3FF))&0xFFFFFFFF
def bl(offs26):      return ((0x15<<26)|((offs26&0xFFFF)<<10)|((offs26>>16)&0x3FF))&0xFFFFFFFF
def beq(rj,rd,o):    return ((0x16<<26)|((o&0xFFFF)<<10)|(rd<<5)|rj)&0xFFFFFFFF
def bne(rj,rd,o):    return ((0x17<<26)|((o&0xFFFF)<<10)|(rd<<5)|rj)&0xFFFFFFFF
def jirl(rd,rj,o):   return ((0x13<<26)|((o&0xFFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def ld_w(rd,rj,s):   return ((0x28800000)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def st_w(rd,rj,s):   return ((0x29800000)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF

prog=[addi_w(1,0,5), addi_w(2,0,5), beq(1,2,2), addi_w(3,0,0xdead),
      addi_w(4,0,0x44), addi_w(5,0,0x55), bne(4,4,2), addi_w(6,0,0x66),
      addi_w(7,0,0x77), bl(2), addi_w(8,0,0x88),
      addi_w(9,0,0x99), lu12i_w(10,0x1c000), addi_w(10,10,0x40),
      jirl(11,10,0), addi_w(12,0,0xcc),
      addi_w(13,0,0xdd),
      # load -> branch: the branch waits in ID until the load reaches WB
      addi_w(14,0,0x11), addi_w(15,0,0x11),
      st_w(14,0,0x100), st_w(15,0,0x104),
      ld_w(16,0,0x100), ld_w(17,0,0x104),
      beq(16,17,2), addi_w(18,0,0xbad), addi_w(19,0,0x123),
      b(0)]

print(open('tb_head.v').read())
print("reg [31:0] rom [0:255];")
print("wire [31:0] rom_word;")
print("assign rom_word = rom[(dut.pc[0] - 32'h1c000000) >> 2];")
print("initial begin")
for i, w in enumerate(prog):
    print("    rom[%d] = 32'h%08x;" % (i, w))
print("""    force dut.inst_rdata = rom_word;
    force dut.if_pending = 1'b0;
    force dut.u_mmu.if_state = 3'b001;
    resetn = 0; repeat (4) @(posedge clk); resetn = 1;
    repeat (400) @(posedge clk);
    $display("FWD_BR: r1=%h r2=%h r3=%h r4=%h r5=%h r6=%h r7=%h r8=%h r9=%h r10=%h r11=%h r12=%h r13=%h r14=%h r15=%h r16=%h r17=%h r18=%h r19=%h",
             dut.u_regfile.rf[1], dut.u_regfile.rf[2], dut.u_regfile.rf[3], dut.u_regfile.rf[4],
             dut.u_regfile.rf[5], dut.u_regfile.rf[6], dut.u_regfile.rf[7], dut.u_regfile.rf[8],
             dut.u_regfile.rf[9], dut.u_regfile.rf[10], dut.u_regfile.rf[11], dut.u_regfile.rf[12],
             dut.u_regfile.rf[13], dut.u_regfile.rf[14], dut.u_regfile.rf[15], dut.u_regfile.rf[16],
             dut.u_regfile.rf[17], dut.u_regfile.rf[18], dut.u_regfile.rf[19]);
    $finish;
end
endmodule""")
