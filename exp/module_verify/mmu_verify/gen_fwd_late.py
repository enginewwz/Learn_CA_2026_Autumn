#!/usr/bin/env python3
# Late-path regression: a non-mul/div instruction whose EX0 operands are not
# ready yet moves to EX1 and finishes there (second ALU, ex1_fw from MEM/WB).
# mul / div must have their operands at EX0, so they stall in EX0 instead.
# Fetch is bypassed exactly like gen_fwd_alu.py (see the comment there).
def addi_w(rd,rj,s): return ((0x00A<<22)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def add_w(rd,rj,rk): return ((0x00100000)|(0<<15)|(rk<<10)|(rj<<5)|rd)&0xFFFFFFFF
def mul_w(rd,rj,rk): return ((0x001C0000)|(rk<<10)|(rj<<5)|rd)&0xFFFFFFFF
def div_w(rd,rj,rk): return ((0x00200000)|(0<<15)|(rk<<10)|(rj<<5)|rd)&0xFFFFFFFF
def ld_w(rd,rj,s):   return ((0x28800000)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def st_w(rd,rj,s):   return ((0x29800000)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def beq(rj,rd,o):    return ((0x16<<26)|((o&0xFFFF)<<10)|(rd<<5)|rj)&0xFFFFFFFF

prog=[addi_w(1,0,3), addi_w(2,0,5),
      mul_w(3,1,2),  add_w(4,3,1),        # mul -> ALU   (late)
      div_w(5,2,1),  add_w(6,5,2),        # div -> ALU   (late)
      ld_w(7,0,0x80), add_w(8,7,1),       # load -> ALU  (late, WB tap / EX1 stall)
      add_w(9,8,1),                       # late ALU -> late ALU
      mul_w(10,9,1),                      # late ALU -> mul (mul waits in EX0)
      add_w(11,10,1),                     # mul -> ALU (late)
      st_w(11,0,0x84),                    # store data from mul (store waits in EX0)
      ld_w(12,0,0x84), add_w(13,12,11),   # load -> ALU (late)
      beq(13,13,2), addi_w(14,0,0xbad),   # branch on a late ALU result
      addi_w(15,0,0x55),
      # 4-deep dependent chain: load -> alu -> alu -> mul -> late load -> mul -> alu
      ld_w(20,0,0x88), add_w(21,20,20), add_w(22,21,21),
      mul_w(23,22,1),
      ld_w(25,23,0),                       # late load: address comes from the mul
      mul_w(26,25,1), add_w(27,26,26),
      0x50000000]

print(open('tb_head.v').read())
print("reg [31:0] rom [0:255];")
print("wire [31:0] rom_word;")
print("assign rom_word = rom[(dut.pc[0] - 32'h1c000000) >> 2];")
print("integer late_cycles = 0, late_stall_cycles = 0;")
print("always @(posedge clk) if (resetn) begin")
print("    if (dut.valid[3] && dut.alu_pos[3] && !dut.alu_done_ex0_ex1) late_cycles = late_cycles + 1;")
print("    if (dut.ex1_fw_pending) late_stall_cycles = late_stall_cycles + 1;")
print("end")
print("initial begin")
for i, w in enumerate(prog):
    print("    rom[%d] = 32'h%08x; slave.mem[13'h%04x] = 32'h%08x;" % (i, w, 0x1000 + i, w))
print("""    slave.mem[13'h0020] = 32'h00000011;   // 0x80
    slave.mem[13'h0022] = 32'h00000011;   // 0x88
    slave.mem[13'h0033] = 32'h00000077;   // 0xcc
    force dut.inst_rdata = rom_word;
    force dut.if_pending = 1'b0;
    force dut.u_mmu.if_state = 3'b001;
    resetn = 0; repeat (4) @(posedge clk); resetn = 1;
    repeat (400) @(posedge clk);
    $display("FWD_LATE: r1=%h r2=%h r3=%h r4=%h r5=%h r6=%h r7=%h r8=%h r9=%h r10=%h r11=%h r12=%h r13=%h r14=%h r15=%h r20=%h r21=%h r22=%h r23=%h r25=%h r26=%h r27=%h mem84=%h late_cycles=%0d late_stall=%0d",
             dut.u_regfile.rf[1], dut.u_regfile.rf[2], dut.u_regfile.rf[3], dut.u_regfile.rf[4],
             dut.u_regfile.rf[5], dut.u_regfile.rf[6], dut.u_regfile.rf[7], dut.u_regfile.rf[8],
             dut.u_regfile.rf[9], dut.u_regfile.rf[10], dut.u_regfile.rf[11], dut.u_regfile.rf[12],
             dut.u_regfile.rf[13], dut.u_regfile.rf[14], dut.u_regfile.rf[15],
             dut.u_regfile.rf[20], dut.u_regfile.rf[21], dut.u_regfile.rf[22],
             dut.u_regfile.rf[23], dut.u_regfile.rf[25], dut.u_regfile.rf[26],
             dut.u_regfile.rf[27], slave.mem[13'h0021], late_cycles, late_stall_cycles);
    $finish;
end
endmodule""")
