import sys
def addi_w(rd,rj,s): return ((0x00A<<22)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def lu12i(rd,s):     return ((0x14000000)|((s&0xFFFFF)<<5)|rd)&0xFFFFFFFF
def ori(rd,rj,u):    return ((0x03800000)|((u&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def ld_w(rd,rj,s):   return ((0x28800000)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def st_w(rd,rj,s):   return ((0x29800000)|((s&0xFFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
def csrrd(rd,csr):   return ((0x04000000)|((csr&0x3FFF)<<10)|rd)&0xFFFFFFFF
def csrwr(rd,csr):   return ((0x04000000)|((csr&0x3FFF)<<10)|(1<<5)|rd)&0xFFFFFFFF
def csrxchg(rd,rj,csr):return ((0x04000000)|((csr&0x3FFF)<<10)|(rj<<5)|rd)&0xFFFFFFFF
TLBIDX,TLBEHI,TLBELO0,TLBELO1,EENTRY,CRMD,ESTAT,ERA,BADV = 0x10,0x11,0x12,0x13,0x0c,0x00,0x05,0x06,0x07
TLBWR = 0x06483000
cases = {'PIL': (0x4000, 0x2, 'ld', 0), 'PIS': (0x4000, 0x2, 'st', 0),
         'PME': (0x2000, 0x1, 'st', 0), 'PPI': (0x6000, 0x3, 'ld', 1),
         'FCH_TLBR': (0x8000, 0x0, 'jirl', 0), 'FCH_PIF': (0x8000, 0x2, 'jirl', 0)}
name = sys.argv[1]
va, flags, kind, plv3 = cases[name]
mask   = 0x1f if plv3 else 0x18
crmd   = 0x13 if plv3 else 0x10
code_f = 0xF  if plv3 else 0x3      # code / page0 entries follow the running PLV
prog = [
 addi_w(1,0,mask), addi_w(2,0,crmd),
 lu12i(3,0x1C00), ori(3,3,code_f),          # TLBELO code page: PPN=0x1c000 V=1 D=1
 lu12i(4,0x1c000),                     # TLBEHI code page
 addi_w(5,0,code_f), addi_w(6,0,0),         # TLBELO/TLBEHI page0
 addi_w(8,0,flags),                    # TLBELO case page
 lu12i(9,va>>12),                         # TLBEHI case page
 lu12i(12,0x1c000), ori(12,12,0x200),  # EENTRY
 lu12i(14,0x1c000), ori(14,14,0x200),  # TLBRENTRY (same handler)
 addi_w(13,0,0x11),                    # store data
 lu12i(10,va>>12),                     # r10 = case VA
 lu12i(7,0xc00), csrwr(7,TLBIDX), csrwr(4,TLBEHI), csrwr(3,TLBELO0), csrwr(3,TLBELO1), TLBWR,
 lu12i(7,0xc00), ori(7,7,1),
 csrwr(7,TLBIDX), csrwr(6,TLBEHI), csrwr(5,TLBELO0), csrwr(5,TLBELO1), TLBWR,
 ([] if (kind=='jirl' and flags==0) else
  [lu12i(7,0xc00), ori(7,7,2),
   csrwr(7,TLBIDX), csrwr(9,TLBEHI), csrwr(8,TLBELO0), csrwr(8,TLBELO1), TLBWR]),
 csrwr(12,EENTRY), csrwr(14,0x88),
 csrxchg(2,1,CRMD),
 (ld_w(16,10,0) if kind=='ld' else st_w(13,10,0) if kind=='st' else ((0x4C000000)|((10)<<5))),
 0x50000000,
]
_flat = []
for _x in prog:
    if isinstance(_x, list): _flat.extend(_x)
    else: _flat.append(_x)
prog = _flat
handler = [ csrrd(20,ESTAT), st_w(20,0,0x60), csrrd(20,ERA),   st_w(20,0,0x64),
            csrrd(20,BADV),  st_w(20,0,0x68), csrrd(20,TLBEHI),st_w(20,0,0x6c),
            0x50000000 ]
out = sys.stdout
out.write(open('tb_head.v').read())
out.write("initial begin\n")
for i,w in enumerate(prog):
    out.write("    slave.mem[13'h%04x] = 32'h%08x;\n" % (0x1000+i, w))
for i,w in enumerate(handler):
    out.write("    slave.mem[13'h%04x] = 32'h%08x;\n" % (0x1080+i, w))
out.write("    resetn = 0; repeat (4) @(posedge clk); resetn = 1;\n")
out.write("    repeat (700) @(posedge clk);\n")
out.write('    $display("' + name + ': ESTAT=%h ERA=%h BADV=%h TLBEHI=%h mem=%h",\n')
out.write("             slave.mem[24], slave.mem[25], slave.mem[26], slave.mem[27], slave.mem[" + str(va>>2) + "]);\n")
out.write("    $finish;\nend\ninteger i;\n")
out.write("always @(posedge clk) if (resetn && dbg_we[0]) $display(\"WB pc=%h r%0d <= %h\", dbg_pc, dbg_wnum, dbg_wdata);\n")
out.write("endmodule\n")
