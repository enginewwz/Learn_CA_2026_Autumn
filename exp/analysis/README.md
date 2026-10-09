# `exp/analysis/` —— A/B 对照台

这里的测试台**不是回归测试**：它们没有自己的参考模型，而是把**当前 RTL** 和**另一份版本**
同时例化、逐位比对，用来回答"这个改动到底改了什么、有没有改变行为"。

回归测试在上一级 `exp/tb_*.v`（见 `exp/README.md`），任何时候都必须全绿；这里的东西只在
**动某个单元之前/之后**跑。

---

## 1. 清单

| 测试台 | 对照什么 | 条数 | 参考版本从哪来 |
| --- | --- | --- | --- |
| `tb_alu_diff.v` | ALU：穷举**全部 4096 个 `alu_op`**（含所有多热组合）× 定向+随机操作数，比 `alu_result` / `lt_o` / `ltu_o` | 69536 | git 历史 或 `docs/ex_opt_attempt/alu_plain.v` |
| `tb_mul_diff.v` | mul：**全部 128 个 `md_op`**（含多热/全零）× 定向+随机操作数，比流水寄存后的 `result` | 2488 | git 历史 或 `docs/ex_opt_attempt/*.v` |

两个台都是"把参考版本的 `module` 改名成 `*_ref`，和被测版本一起 elaborate"的模式。

## 2. 怎么跑

参考版本**必须在同一个 iverilog 命令里**，不能分两条命令（`/tmp` 在不同 shell 里可能不共享）：

```bash
# ALU：和 git 上的旧版比
git show b2b747f:exp/myCPU/alu.v > /tmp/alu_ref.v
sed -i 's/^module alu(/module alu_ref(/' /tmp/alu_ref.v
iverilog -g2005 -o /tmp/t /tmp/alu_ref.v exp/myCPU/alu.v exp/analysis/tb_alu_diff.v && vvp /tmp/t
# -> PASS: 69536 checks, 0 errors

# ALU：和候选版 alu_plain.v 比（验证候选与原版逐位等价，再拿去综合）
sed 's/^module alu(/module alu_ref(/' docs/ex_opt_attempt/alu_plain.v > /tmp/alu_ref.v
iverilog -g2005 -o /tmp/t /tmp/alu_ref.v exp/myCPU/alu.v exp/analysis/tb_alu_diff.v && vvp /tmp/t

# mul：和 b2b747f 的 4×16 carry-select 版比
git show b2b747f:exp/myCPU/mul.v > /tmp/mul_ref.v
sed -i 's/^module mul(/module mul_ref(/' /tmp/mul_ref.v
iverilog -g2005 -o /tmp/t exp/myCPU/tools.v /tmp/mul_ref.v exp/myCPU/mul.v exp/analysis/tb_mul_diff.v && vvp /tmp/t
# -> PASS: 2488 checks, 0 errors
```

## 3. 已记录的基线（2026-10-10）

| 对照 | 结果 | 结论 |
| --- | --- | --- |
| `mul.v`（裸 `+`）vs `b2b747f`（4×16 carry-select） | 2488 / 0 | 纯结构替换，功能零风险；随后测出裸 `+` 更快，采纳 |
| `mul.v` vs `mul_v2_carryselect.v` / `mul_cs32.v` | 2488 / 0 | 两个候选都与工作区版本逐位等价 |
| `alu.v` vs `alu_plain.v`（裸 `+`/`-`） | 69536 / 0 | 逐位等价，但**综合后更差**（br 被拖累），否决 |

**这里只保证"行为没变"，不保证"更快"** —— 快慢只能靠 Windows 端的综合报告，
判读口径见 `docs/cpu_full_analysis(Oct10).md` §3 / §4。

## 4. 相关材料

- 被否决/备选的版本：`docs/ex_opt_attempt/`（`alu_plain.v`、`div_plain.v`、`div_flat.v`、
  `mul_v2_carryselect.v`、`mul_cs32.v`）
- 结论与实测账本：`docs/cpu_full_analysis(Oct10).md`
- EX 阶段的两个候选改动：`docs/ex_opt_attempt/ex_stage_notes.md`
