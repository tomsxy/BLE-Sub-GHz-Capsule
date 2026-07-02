# 915RX

[English](README.md) · [仓库总览](../README_zh-CN.md)

**功能：** 接收 915 MHz 无线数据，并作为 SPI 主机将数据发送给 Jetson。

本目录是胶囊图像链路中的 TI CC1310 接收端。它从 915 MHz Sub-GHz 无线链路接收 250 字节数据包，再通过 SPI 将恢复出的应用帧转发给 Jetson。

## 当前行为

- `rfPacketErrorRate.c` 上电后自动进入接收模式，接收任务优先级为 3。
- RF 配置为高速模式、915 MHz 对应频率表项、持续接收和 250 字节负载。
- `rx.c` 负责 RF 接收、包检查、缓冲、SPI 转发和链路诊断。
- 当前开发板配置提供 RF 前端控制引脚时，启动阶段会初始化这些引脚。

## 主要文件

| 文件或目录 | 作用 |
| --- | --- |
| `rfPacketErrorRate.c` | 板级初始化和自动 RX 任务入口 |
| `rx.c` | RF 到 SPI 的转发和接收状态机 |
| `config.c`、`config.h` | RF 模式和频率配置 |
| `smartrf_settings/` | SmartRF 生成的无线参数 |
| `CC1310_LAUNCHXL*`、`Board.h` | CC1310 LaunchPad 板级配置 |
| `.project`、`.cproject`、`.ccsproject` | Code Composer Studio 工程元数据 |

## 构建

在 TI Code Composer Studio 中将本目录作为已有工程导入。工程依赖 TI-RTOS 和兼容的 SimpleLink CC13x0 SDK。`Debug/`、`Release/` 等构建输出不会提交到仓库。

SPI 接收方必须与本固件使用一致的事务长度、字节序和应用帧边界。仓库中的 Jetson 接收程序默认按 250 字节分块读取。

更完整的 TI 原始示例说明保留在英文 [README.md](README.md) 的后半部分。
