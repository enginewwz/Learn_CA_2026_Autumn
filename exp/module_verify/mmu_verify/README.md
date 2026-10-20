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
| `gen_page_exc.py <case>` | 页例外：`PIL` `PIS` `PME` `PPI` `FCH_TLBR` `FCH_PIF`；处理器在 EENTRY/TLBRENTRY 把 ESTAT/ERA/BADV/TLBEHI 存到内存，`run.sh` 打印出来 |

期望值见 `docs/exp17-19_mmu.md` §10.5。改 RTL 后先跑这里，正式验证仍走 `exp/soc_verify/soc_axi`。
