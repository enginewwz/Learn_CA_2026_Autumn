#!/bin/sh
# run the local mmu / exception tests with iverilog (no Vivado needed)
set -e
cd "$(dirname "$0")"
CPU=../../myCPU
for t in basic adef branch excp_entry refill_ret tlb_csr; do
    python3 gen_$t.py > /tmp/mmu_$t.v
    iverilog -g2005 -o /tmp/mmu_$t /tmp/mmu_$t.v $CPU/*.v
    echo "--- $t"; vvp /tmp/mmu_$t | grep -E "basic:|ADEF:|BR:|EXCP_ENTRY:|REFILL_RET:|r16|handler"
done
for c in PIL PIS PME PPI FCH_TLBR FCH_PIF; do
    python3 gen_page_exc.py $c > /tmp/mmu_$c.v
    iverilog -g2005 -o /tmp/mmu_$c /tmp/mmu_$c.v $CPU/*.v
    echo "--- $c"; vvp /tmp/mmu_$c | grep -E "ESTAT"
done
