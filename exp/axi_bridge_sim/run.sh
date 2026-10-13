#!/bin/sh
# Local bridge simulation: no vendor IP, no Vivado.
#
#   ./run.sh            fixed latency (LAT=1, no jitter)
#   ./run.sh 3          LAT=3
#   ./run.sh 3 1        LAT=3 with random jitter
#   VCD=1 ./run.sh      also write tb_axi_bridge.vcd
set -e
LAT=${1:-1}
JIT=${2:-0}
shift 2 2>/dev/null || true
ARGS="$*"
DIR=$(cd "$(dirname "$0")" && pwd)
OUT=$(mktemp -d)
EXTRA=""
[ -n "$VCD" ] && EXTRA="+vcd"
iverilog -g2005 -o "$OUT/tb" \
    -P tb_axi_bridge.LAT=$LAT -P tb_axi_bridge.JITTER=$JIT \
    "$DIR/../myCPU/sram_axi_bridge.v" "$DIR/axi_slave_model.v" "$DIR/tb_axi_bridge.v"
echo "--- LAT=$LAT JITTER=$JIT ---"
cd "$DIR"
vvp "$OUT/tb" $EXTRA $ARGS
