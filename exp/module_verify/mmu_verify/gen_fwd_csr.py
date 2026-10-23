#!/usr/bin/env python3
# Forwarding regression for the CSR read / write data paths.
#   rd0: csrrd -> ALU consumer (the producer's WB value is csr_rdata, not
#        result[MEM], so this must be forwarded from the WB write data)
#   rd1: addi / mul -> csrwr write data (the CSR write payload is forwarded at
#        EX0 or rebuilt in EX1 when late), followed by csrrd of the same CSR
#   rd2: mul -> invtlb operands (late TLB payload)
#   rd3: csrrd -> store data: the CSR GPR write goes through the dedicated CSR
#        RF write port and is marked pending (not forwarded) for one cycle
# Fetch is bypassed exactly like gen_fwd_alu.py (see the comment there).
def addi_w(rd,rj,s): return ((0x00A<<22)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def add_w(rd,rj,rk): return ((0x00100000)|(0<<15)|(rk<<10)|(rj<<5)|rd)&0xFFFFFFFF
def csrrd(rd,csr):   return ((0x01<<26)|((csr&0x3FFF)<<10)|rd)&0xFFFFFFFF
def csrwr(rd,csr):   return ((0x01<<26)|((csr&0x3FFF)<<10)|(1<<5)|rd)&0xFFFFFFFF
def mul_w(rd,rj,rk): return ((0x001C0000)|(rk<<10)|(rj<<5)|rd)&0xFFFFFFFF
def ld_w(rd,rj,s):   return ((0x28800000)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def st_w(rd,rj,s):   return ((0x29800000)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def invtlb(op,rj,rk):return ((0x01<<26)|(1<<25)|(0x13<<15)|((rk&0x1f)<<10)|((rj&0x1f)<<5)|(op&0x1f))&0xFFFFFFFF

prog=[csrrd(1,0x030), addi_w(2,1,0x11), csrrd(3,0x040), add_w(4,3,3),
      addi_w(5,0,0x55), csrwr(5,0x030), csrrd(6,0x030), add_w(7,6,6),
      # late CSR write: the csrwr data comes from a mul (pending in EX0)
      addi_w(8,0,3), mul_w(9,8,8), csrwr(9,0x032),
      csrrd(10,0x032), add_w(11,10,10),
      # late invtlb: rj/rk come from a mul (pending in EX0), same payload path
      mul_w(12,8,8), invtlb(0,12,12),
      addi_w(13,0,0x5a),
      # csrrd -> store data: must go pending for one cycle, not be forwarded
      csrrd(14,0x030), st_w(14,0,0x68), ld_w(15,0,0x68), add_w(16,15,15),
      0x50000000]

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
    repeat (300) @(posedge clk);
    $display("FWD_CSR: r1=%h r2=%h r3=%h r4=%h r5=%h r6=%h r7=%h r8=%h r9=%h r10=%h r11=%h r13=%h r14=%h r15=%h r16=%h mem68=%h",
             dut.u_regfile.rf[1], dut.u_regfile.rf[2], dut.u_regfile.rf[3], dut.u_regfile.rf[4],
             dut.u_regfile.rf[5], dut.u_regfile.rf[6], dut.u_regfile.rf[7],
             dut.u_regfile.rf[8], dut.u_regfile.rf[9], dut.u_regfile.rf[10],
             dut.u_regfile.rf[11], dut.u_regfile.rf[13],
             dut.u_regfile.rf[14], dut.u_regfile.rf[15], dut.u_regfile.rf[16],
             slave.mem[13'h001a]);
    $finish;
end
endmodule""")
