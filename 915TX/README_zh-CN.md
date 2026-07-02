# 915TX

[English](README.md) · [仓库总览](../README_zh-CN.md)

本目录是胶囊图像链路中的 TI CC1310 发送端。它接收上游 BLE/SPI 桥接端送来的图像应用帧，并通过 915 MHz Sub-GHz 无线链路发送。

## 当前行为

- `rfPacketErrorRate.c` 上电后自动进入发送模式，正常数据链路不依赖按键菜单。
- RF 配置为高速模式、915 MHz 对应频率表项、持续发送、250 字节负载，代码中配置发射功率为 8 dBm。
- `tx.c` 负责 SPI 输入、缓冲、数据包校验和 RF 发送流程。
- UART 输出用于显示精简的链路状态和诊断计数器。

## 主要文件

| 文件或目录 | 作用 |
| --- | --- |
| `rfPacketErrorRate.c` | 板级初始化和自动 TX 任务入口 |
| `tx.c` | SPI 到 RF 的转发和发送状态机 |
| `config.c`、`config.h` | RF 模式和频率配置 |
| `smartrf_settings/` | SmartRF 生成的无线参数 |
| `CC1310_LAUNCHXL*`、`Board.h` | CC1310 LaunchPad 板级配置 |
| `.project`、`.cproject`、`.ccsproject` | Code Composer Studio 工程元数据 |

## 构建

在 TI Code Composer Studio 中将本目录作为已有工程导入。工程依赖 TI-RTOS 和兼容的 SimpleLink CC13x0 SDK。`Debug/`、`Release/` 等构建输出不会提交到仓库。

修改协议时，必须同时核对 `BLE-SPI-Rx`、`915RX` 和 Jetson 接收端的 250 字节帧定义。

更完整的 TI 原始示例说明保留在英文 [README.md](README.md) 的后半部分。
