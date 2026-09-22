# HPM5361 片内 Flash 启动与调试说明

## 硬件启动模式

实物板上 BOOT0、BOOT1 对应的两个 0Ω 上拉电阻均未焊接，两个引脚由板上的
10k 电阻下拉，因此启动模式固定为：

```text
BOOT1 = 0
BOOT0 = 0
BOOT_MODE[1:0] = 00
```

按照 HPM5361 手册，`00` 为 XPI NOR 启动。HPM5361ICB1 封装内带 1 MiB Flash，
CPU 映射地址为 `0x80000000`。板上的 SPI1/W25Q128 只用于日志，不属于程序启动
存储器，构建和烧录流程不得访问它。

## 工程中的 Flash 配置

板级 DTS 将 Zephyr 程序区设为：

```text
起始地址：0x80000000
容量：    0x00100000（1 MiB）
擦除块：  4096 字节
```

OpenOCD 的 `hpm5361_iflash` flash bank 使用同一地址和容量。启动镜像同时包含
BootROM 需要的 NOR 配置块和 HPM 启动头：

```text
NOR 配置块：0x80000400
启动头：    0x80001000
程序入口：  0x80003000
```

NOR 配置字使用已在本板验证过的 HPM5300 参数：

```text
header = 0xfcf90002
option0 = 0x00000005
option1 = 0x00001000
```

相关文件：

- `Board/hpm5361/boards/riscv/hpm5361icb/hpm5361icb.dts`
- `Board/hpm5361/boards/riscv/hpm5361icb/hpm5361icb_defconfig`
- `Board/hpm5361/hpm5361_target.cfg`
- `Script/build&flash.cmd`

## 构建和烧录

VS Code 的 `Build` 任务执行全量构建。`Flash` 任务独立运行，调用
`Script/build&flash.cmd`，先进行增量构建，再擦除、写入并校验 HPM5361 片内
Flash。烧录成功时 OpenOCD 必须打印：

```text
** Programming Finished **
** Verify Started **
** Verified OK **
```

`build&flash.cmd` 的文件名包含 `&`。VS Code 任务必须通过 PowerShell 调用运算符
执行：

```powershell
& '${workspaceFolder}\Script\build&flash.cmd'
```

不能把该路径直接拼进 `cmd.exe /c call`，否则 `&` 会被解释为命令分隔符。

## 调试

绿色调试三角使用 `.vscode/launch.json`。调试器连接已有的片内 Flash 镜像，复位
并停在 `main()`，随后可以下源码断点、单步进入和单步跳过。调试配置不依赖
VS Code Task，也不会把程序临时下载到 RAM。

## UART3 和 CAN0

| 功能 | MCU 引脚 | 配置 |
| --- | --- | --- |
| UART3 RX | PA14 | 115200 bit/s |
| UART3 TX | PA15 | Zephyr 控制台输出 |
| CAN0 TX | PB00 | 1 Mbit/s，正常模式 |
| CAN0 RX | PB01 | 接收 C620 的 `0x201` 反馈帧 |

C620 控制程序发送标准帧 `0x200`，第一个 16 位大端有符号数是 ID `0x201` 电调的
电流指令。UART TX 和 CAN TX 在总线空闲时均为高电平；判断程序是否工作应使用
示波器下降沿触发或串口/CAN 解码，不能只看万用表的直流电压。

## 当前结论

程序的持久化目标是 HPM5361 封装内 1 MiB Flash，地址 `0x80000000`。外接
SPI1/W25Q128 未参与链接、启动或烧录。掉电后 BootROM 在 BOOT=`00` 时会重新从
片内 Flash 读取启动配置和程序镜像。
