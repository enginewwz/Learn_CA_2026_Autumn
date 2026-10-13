# sram_axi_bridge 本地仿真

不依赖 `soc_verify`、不需要 Vivado、不需要任何 Xilinx IP —— 用 `iverilog` 直接跑桥。

## 文件

| 文件 | 作用 |
| --- | --- |
| `axi_slave_model.v` | 单拍 AXI 从机 + RAM。参数 `LAT`(每次握手额外等待拍数)、`JITTER`(伪随机抖动)。地址越界回 SLVERR,其余回 OKAY;数据线宽/ID/`wid` 与桥驱动的 AXI3 口味一致 |
| `tb_axi_bridge.v` | 按 `mycpu_top` 的行为驱动桥的 CPU 侧,并与参考内存逐条比对 |
| `run.sh` | 编译 + 运行 |

## 跑法

```sh
./run.sh            # LAT=1, 无抖动
./run.sh 3          # LAT=3
./run.sh 3 1        # LAT=3 + 抖动
VCD=1 ./run.sh      # 另存 tb_axi_bridge.vcd
./run.sh 1 0 +dbg   # 打 +dbg 会逐拍打印桥内部状态
```

## TB 怎么模拟 CPU 侧

* **取指**:`pc_fe` 相当于 `pre_IF`。在 `allow_if`(=`~inst_pending`,对应顶层的 `allow[IF]`)为高的那一拍,指令进 IF、同时发起下一条取指;取回的数据在下一次 `allow_if` 为高时被取走并与参考内存比对。这正是顶层「触发沿锁存地址」所依赖的时序。
* **访存**:`data_req`/`data_we`/`data_addr`/`data_wdata` 像 MEM 级那样保持有效,直到 `mem_pending` 拉低;load 在那一拍直接从 `data_rdata` 取值(桥在 RSP 拍组合直出),store 在那一拍 retire。

## 覆盖到的场景

1. 连续取指 64 条,逐条比对
2. 普通 load
3. store 后 load(同地址回读)
4. 字节 / 半字 store 的写掩码
5. **背靠背 store(WAW)**:CPU 在上一笔被收下的下一拍就给出新一笔,检查顺序与数据都不丢
6. **RAW**:store 后紧跟 load,读必须等到写完成并看到新值
7. **store 在飞时取指**:两条路各自完成
8. 200 次随机混合 load/store

## 已验证的配置

| LAT | JITTER | 结果 |
| --- | --- | --- |
| 0 | 0 | PASS 197 checks, 0 errors (1009 cycles) |
| 1 | 0 | PASS 190 checks, 0 errors (1385 cycles) |
| 1 | 1 | PASS 183 checks, 0 errors (2379 cycles) |
| 3 | 0 | PASS 185 checks, 0 errors (2139 cycles) |
| 3 | 1 | PASS 181 checks, 0 errors (3143 cycles) |
| 7 | 1 | PASS 180 checks, 0 errors (4600+ cycles) |

## 说明与局限

* 只覆盖**单拍** AXI 事务(桥不会发 `len != 0`),burst/窄传输没测。
* 从机可以在 `LAT`/`JITTER` 之外被改成固定扣住某个 handshake 若干拍,用来复现随机掩码那种极端情况。
* TB 不检查 `rresp`/`bresp` 错误路径(从机的 SLVERR 分支已写好,但 TB 只用到合法地址)。
* 这里验的是桥;**整机**仍需在 Vivado 里跑 `soc_verify/soc_axi`。
