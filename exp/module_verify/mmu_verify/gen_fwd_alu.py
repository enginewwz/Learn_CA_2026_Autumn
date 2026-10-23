#!/usr/bin/env python3
# Forwarding regression: back-to-back ALU / mul / div / load RAW chains at full
# speed.  The normal fetch path is serialised (one AXI read per instruction,
# about 5 cycles apart), so the ID / EX0 stages never overlap and no RAW
# hazard is ever exercised.  Here the fetch is bypassed with `force`: the
# instruction word is indexed from pc[IF] combinationally and if_pending is
# held low.  Everything after the fetch (decode, id_fw, ex0_fw, MEM/WB) is the
# real design.
def addi_w(rd,rj,s): return ((0x00A<<22)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def add_w(rd,rj,rk): return ((0x00100000)|(0<<15)|(rk<<10)|(rj<<5)|rd)&0xFFFFFFFF
def sub_w(rd,rj,rk): return ((0x00100000)|(2<<15)|(rk<<10)|(rj<<5)|rd)&0xFFFFFFFF
def and_w(rd,rj,rk): return ((0x00100000)|(9<<15)|(rk<<10)|(rj<<5)|rd)&0xFFFFFFFF
def or_w(rd,rj,rk):  return ((0x00100000)|(10<<15)|(rk<<10)|(rj<<5)|rd)&0xFFFFFFFF
def xor_w(rd,rj,rk): return ((0x00100000)|(11<<15)|(rk<<10)|(rj<<5)|rd)&0xFFFFFFFF
def slt(rd,rj,rk):   return ((0x00100000)|(4<<15)|(rk<<10)|(rj<<5)|rd)&0xFFFFFFFF
def sltu(rd,rj,rk):  return ((0x00100000)|(5<<15)|(rk<<10)|(rj<<5)|rd)&0xFFFFFFFF
def slli_w(rd,rj,s): return ((0x00400000)|(1<<15)|((s&0x1f)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def mul_w(rd,rj,rk): return ((0x001C0000)|(rk<<10)|(rj<<5)|rd)&0xFFFFFFFF
def mulh_w(rd,rj,rk):return ((0x00100000)|(0x19<<15)|(rk<<10)|(rj<<5)|rd)&0xFFFFFFFF
def div_w(rd,rj,rk): return ((0x00200000)|(0<<15)|(rk<<10)|(rj<<5)|rd)&0xFFFFFFFF
def mod_w(rd,rj,rk): return ((0x00200000)|(1<<15)|(rk<<10)|(rj<<5)|rd)&0xFFFFFFFF
def ld_w(rd,rj,s):   return ((0x28800000)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def st_w(rd,rj,s):   return ((0x29800000)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF

prog=[addi_w(1,0,0x11), addi_w(2,0,0x22), add_w(3,1,2), sub_w(4,3,1), and_w(5,4,3),
      or_w(6,5,4), xor_w(7,6,5), slt(8,7,6), sltu(9,7,6), slli_w(10,9,1),
      mul_w(11,10,2), mulh_w(12,11,2), div_w(13,12,1), mod_w(14,13,1),
      st_w(14,0,0x80), ld_w(15,0,0x80), add_w(16,15,15), and_w(17,16,15),
      add_w(18,17,17), sub_w(19,18,17), 0x50000000]

print(open('tb_head.v').read())
print("reg [31:0] rom [0:255];")
print("wire [31:0] rom_word;")
print("assign rom_word = rom[(dut.pc[0] - 32'h1c000000) >> 2];")
print("initial begin")
for i, w in enumerate(prog):
    print("    rom[%d] = 32'h%08x; slave.mem[13'h%04x] = 32'h%08x;" % (i, w, 0x1000 + i, w))
print("""    force dut.inst_rdata = rom_word;
    force dut.if_pending = 1'b0;
    force dut.u_mmu.if_state = 3'b001;
    resetn = 0; repeat (4) @(posedge clk); resetn = 1;
    repeat (300) @(posedge clk);
    $display("FWD_ALU: r1=%h r2=%h r3=%h r4=%h r5=%h r6=%h r7=%h r8=%h r9=%h r10=%h r11=%h r12=%h r13=%h r14=%h r15=%h r16=%h r17=%h r18=%h r19=%h mem80=%h",
             dut.u_regfile.rf[1], dut.u_regfile.rf[2], dut.u_regfile.rf[3], dut.u_regfile.rf[4],
             dut.u_regfile.rf[5], dut.u_regfile.rf[6], dut.u_regfile.rf[7], dut.u_regfile.rf[8],
             dut.u_regfile.rf[9], dut.u_regfile.rf[10], dut.u_regfile.rf[11], dut.u_regfile.rf[12],
             dut.u_regfile.rf[13], dut.u_regfile.rf[14], dut.u_regfile.rf[15], dut.u_regfile.rf[16],
             dut.u_regfile.rf[17], dut.u_regfile.rf[18], dut.u_regfile.rf[19], slave.mem[13'h0020]);
    $finish;
end
endmodule""")
