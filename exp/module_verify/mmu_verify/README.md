# mmu / TLB / 例外 本地验证

不依赖 Vivado：`iverilog` + 一个自写的单拍 AXI3 从机（`tb_head.v`），把小程序装进去跑 `mycpu_top`。

```sh
./run.sh          # 全部用例
```

| 文件 | 作用 |
| --- | --- |
| `tb_head.v` | 单拍 AXI 从机 + `tb` 骨架；地址 `0x1c00_0000` 当指令区（mif 式布局），其余当地址的低 13 位索引 |
| `gen_basic.py` | 直接模式 load/store 冒烟：r4=r5=0x11 |
| `gen_tlb_csr.py` | exp18 + 一部分 exp19：写 16 项 TLB 里的两项 → `tlbwr/tlbrd/tlbsrch/invtlb` → 切 PG 模式 → TLB 翻译的 load/store、DMW 翻译的 load、未映射 load 的 TLBR |
| `gen_fwd_alu.py` | **前递链**：旁路取指握手，跑 ALU/mul/div/load 背靠背 RAW 链（正常取指约 5 拍一条指令，测不到 ID/EX 重叠） |
| `gen_fwd_br.py` | **ID 分支前递**：beq/bne/bl/jirl 的源操作数由紧邻指令产生 |
| `gen_fwd_csr.py` | **CSR 前递**：csrrd→ALU（走 WB tap）、addi→csrwr 写数据、写后读；mul→csrwr 的迟到写数据、mul→invtlb 的迟到 TLB 操作数 |
| `gen_fwd_late.py` | **EX1 迟到路径**：mul/div/load→ALU、load→ALU→ALU→mul→迟到 load→mul→ALU 的四级相关链；mul→store 的迟到 store data；迟到 ALU→mul（mul 在 EX0 等） |
| `gen_flush_wait.py` | **WB flush 空等检查**：数据 ALE 在 WB flush 时，EX1 迟到 ALU / EX0 mul 仍带 pending；检查 `stall[EX0]/stall[EX1]` 是否被 flushed 指令误拉高（`empty_ex0/empty_ex1` 应为 0），并确认异常处理正常 |
| `gen_page_exc.py <case>` | 页例外：`PIL` `PIS` `PME` `PPI` `FCH_TLBR` `FCH_PIF`；处理器在 EENTRY/TLBRENTRY 把 ESTAT/ERA/BADV/TLBEHI 存到内存，`run.sh` 打印出来 |

期望值见 `docs/exp17-19_mmu.md` §10.5。改 RTL 后先跑这里，正式验证仍走 `exp/soc_verify/soc_axi`。
