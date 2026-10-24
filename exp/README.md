# `exp/` 目录地图

```
exp/
  myCPU/          设计源码（11 个 .v）——唯一会被综合的东西
  module_verify/  单元验证：cache_verify / div_verify / mul_verify / tlb_verify（留在原处）
  soc_verify/     SoC 级 testbench + Vivado 工程（golden trace 回归）
  func/           测试程序（汇编 -> inst_ram.mif）
  gettrace/       golden trace 生成器（参考 CPU）

local_testbench/  （在仓库根目录，不在 exp/ 下）
  tb_*.v          回归测试（自检、有参考模型）      <- 改了 RTL 就要跑
  mmu_verify/     MMU 验证（从 exp/module_verify/mmu_verify 搬来）
  analysis/       A/B 对照台（需要"参考版本"才能跑）<- 只在改某个单元时用
  axi_bridge_sim/ AXI 桥本地仿真（当前协议的本地版，不需要 Vivado）
```

> 约定：`local_testbench/tb_*.v` = **回归测试**，任何时候都必须全绿；
> `local_testbench/analysis/` 里 = **分析工具**，用来把"当前 RTL"和"某个候选/旧版本"逐位比。
> 两者的区别是**有没有独立的参考模型**：回归测试自己有 oracle，分析台要外部喂一个参考版本。
> 命名/线束/流水结构的完整说明见 `docs/pipeline_structure.md`。

---

## 1. 回归测试（`local_testbench/tb_*.v`）

全部是自检的（自带参考模型 + PASS/FAIL），互不重叠。RTL 在 `exp/myCPU/`。

| 测试台 | 条数 | 覆盖 | 编译命令 |
| --- | --- | --- | --- |
| `tb_alu.v` | 23092 | ALU 全操作 + **多热 `alu_op`**；参考模型按 LoongArch 语义手写，与实现结构无关 | `iverilog -g2005 -o /tmp/t exp/myCPU/alu.v local_testbench/tb_alu.v && vvp /tmp/t` |
| `tb_csr.v` | 62 | 每个 CSR 的复位值、掩码写/读回、同拍读旧值、计时器、中断 | `iverilog -g2005 -o /tmp/t exp/myCPU/csr.v local_testbench/tb_csr.v && vvp /tmp/t` |
| `tb_mul_div.v` | 661 | mul + div 常规与边界（mul 的主回归） | `iverilog -g2005 -o /tmp/t exp/myCPU/tools.v exp/myCPU/mul.v exp/myCPU/div.v local_testbench/tb_mul_div.v && vvp /tmp/t` |
| `tb_div_scan.v` | **201719** | div 的**独立**参考模型：20 万随机 + `b∈[1,256]` 全扫 + 8×8 网格 + 除零 + `INT_MIN`；另查 busy/done 协议与拍数 | `iverilog -g2005 -o /tmp/t exp/myCPU/tools.v exp/myCPU/div.v local_testbench/tb_div_scan.v && vvp /tmp/t`（约 1 分钟） |

一键跑（默认含 `tb_div_scan`，`QUICK=1` 跳过）：

```bash
local_testbench/run_regression.sh
QUICK=1 local_testbench/run_regression.sh
```

**注意重叠**：`tb_div_scan.v` 完全不依赖 `tb_mul_div.v` 的参考模型，而且强度高约 300 倍，
所以 `tb_mul_div.v` 里的 div 部分已经被它覆盖；`tb_mul_div.v` 保留的价值在于**它的 mul 部分**。
`tb_div_scan.v` 是唯一跑得比较久的（~1 分钟），日常改 ALU/EX 时不必每次都跑。

## 2. SoC 级回归（真正的验收）

```bash
cd exp/soc_verify/soc_bram/testbench && make iverilog
```

拿 `func/` 编出的程序跑，和 `gettrace/golden_trace.txt` 逐条**写回事件**（pc / 寄存器号 / 数据）比对——
比的是**顺序和值**，不是逐拍波形，所以改动延迟（比如 div 从 21 拍变 18 拍）不会破坏它。
当前基线：**58/58 Functional Test Point PASS**。

> ⚠ **`soc_verify/` 下的 `mycpu_tb.v` / `Makefile` / `rtl/` 是实验配套文件，别改。**
> 其中 `mycpu_tb.v` 无条件 `$dumpfile("dump.vcd")`，所以每跑一次都会写一个 **~1 GB 的波形**。

**跑完记得删掉波形和构建产物**（它们已被 `.gitignore` 忽略，删了不影响任何东西）：

```bash
rm -rf exp/soc_verify/soc_bram/testbench/project      # 波形 + 编译出的 gettrace 一起删
# 或者只删波形、留下编译结果：
# rm -f exp/soc_verify/soc_bram/testbench/project/loongson.sim/sim_1/behav/iverilog/dump.vcd
```

如果要留着看波形，记得别提交（`*.vcd` 和 `project/` 都在 `.gitignore` 里）。

## 3. 分析 / A-B 对照

见 `local_testbench/analysis/README.md`。

## 4. 单元验证 / MMU

- `exp/module_verify/{cache,div,mul,tlb}_verify/`：cache/div/mul/tlb 的单元验证，留在 `exp/`。
- `local_testbench/mmu_verify/`：MMU 验证（16 个生成式测试，`./run.sh`）。
- `local_testbench/axi_bridge_sim/`：AXI 桥本地仿真（当前 `sram_axi_bridge.v` 协议，
  random 400 PASS，剩少量 back-to-back/fetch 末尾 edge error）。
