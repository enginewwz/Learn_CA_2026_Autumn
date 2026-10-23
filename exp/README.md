# `exp/` 目录地图

```
exp/
  myCPU/          设计源码（11 个 .v）——唯一会被综合的东西
  tb_*.v          回归测试（自检、有参考模型）      <- 改了 RTL 就要跑
  analysis/       A/B 对照台（需要"参考版本"才能跑）<- 只在改某个单元时用
  func/           测试程序（汇编 -> inst_ram.mif）
  gettrace/       golden trace 生成器（参考 CPU）
  soc_verify/     SoC 级 testbench + Vivado 工程（golden trace 回归）
```

> 约定：`tb_*.v` 在 `exp/` 根下 = **回归测试**，任何时候都必须全绿；
> `exp/analysis/` 里 = **分析工具**，用来把"当前 RTL"和"某个候选/旧版本"逐位比。
> 两者的区别是**有没有独立的参考模型**：回归测试自己有 oracle，分析台要外部喂一个参考版本。

---

## 1. 回归测试（`exp/tb_*.v`）

全部是自检的（自带参考模型 + PASS/FAIL + 退出码），互不重叠。

| 测试台 | 条数 | 覆盖 | 大致耗时 | 编译命令 |
| --- | --- | --- | --- | --- |
| `tb_alu.v` | 23092 | ALU 全操作 + **多热 `alu_op`**；参考模型按 LoongArch 语义手写，与实现结构无关 | 秒级 | `iverilog -g2005 -o /tmp/t exp/myCPU/alu.v exp/tb_alu.v && vvp /tmp/t` |
| `tb_csr.v` | 62 | 每个 CSR 的复位值、掩码写/读回、同拍读旧值、计时器、中断 | 秒级 | `iverilog -g2005 -o /tmp/t exp/myCPU/csr.v exp/tb_csr.v && vvp /tmp/t` |
| `tb_mul_div.v` | 661 | mul + div 常规与边界（**exp/ 根下唯一的 mul 回归**） | 秒级 | `iverilog -g2005 -o /tmp/t exp/myCPU/tools.v exp/myCPU/mul.v exp/myCPU/div.v exp/tb_mul_div.v && vvp /tmp/t` |
| `tb_div_scan.v` | **201719** | div 的**独立**参考模型：20 万随机 + `b∈[1,256]` 全扫 + 8×8 网格 + 除零 + `INT_MIN`；另查 busy/done 协议与拍数 | ~1 分钟 | `iverilog -g2005 -o /tmp/t exp/myCPU/tools.v exp/myCPU/div.v exp/tb_div_scan.v && vvp /tmp/t` |

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

见 `analysis/README.md`。
