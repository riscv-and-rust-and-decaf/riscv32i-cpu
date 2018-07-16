# ram 分支使用
ram 分支中, `thinpad_top.v` 中加入了访存的功能.
时钟使用 50 MHz 的时钟, 控制信号如地址, 数据, 读写模式是手动控制的.

## thinpad\_top.v
和访存相关的接口信号是

* `wire [31:0] paddr` 物理地址. 最高的 9 位被舍弃, 因为我们只有 8MB 内存.

* `wire [31:0] wdata;` 如果写内存, 写的值.

* `wire [31:0] rdata;` 读内存的值.

* `wire [3:0] mode;` 访存模式. 现在有以下值

| `mode` | 含义 |
| --- | --- |
| 0 | 不访存, `rdata` 未定义 | 
| 1 | load word |
| 2 | store word |
| 3 | load byte signed |
| 4 | load byte unsigned |
| 5 | store byte |

对于 store byte, 有如下例子.
有 `paddr = 3`, 并且 `wdata = 0xAABBCCDD`. 那么执行的操作是, `*(char*) 0x3 = 0xBB`.

## 测试
建议使用在线平台.

* BTN1-4 控制 mode, 默认是 load word.

* dip-sw 控制地址, `paddr = dip_sw`.

* 当模式是写内存的时候, `wdata = {0xDEADBE, dip_sw[31:24]}`
