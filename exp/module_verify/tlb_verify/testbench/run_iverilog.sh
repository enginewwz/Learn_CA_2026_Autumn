#!/bin/sh
# 本地等价性检查(不需要 Vivado):用课程给的 26 组向量测 tlb.v
set -e
cd "$(dirname "$0")"
python3 gen_tlb_unit.py > /tmp/tb_tlb_unit.v
iverilog -g2005 -o /tmp/tb_tlb_unit /tmp/tb_tlb_unit.v ../../../myCPU/tlb.v
vvp /tmp/tb_tlb_unit
