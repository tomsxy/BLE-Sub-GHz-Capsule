# BLE-SPI-Rx

[English](README.md) · [仓库总览](../README_zh-CN.md)

本目录是 Sub-GHz 链路输入端的 Nordic nRF Connect SDK/Zephyr 桥接工程。它通过低功耗蓝牙接收胶囊图像数据，再经 SPI 将数据流转发给 `915TX`。

## 主要职责

- 作为 BLE central/client 连接图像传输服务。
- 接收图像元数据、控制消息和图像 payload。
- 缓冲输入数据，并按 CC1310 发送端要求输出固定长度 SPI 事务。
- 通过 Zephyr devicetree overlay 配置不同开发板的 SPI 和 GPIO 引脚。
- 将链路诊断与正常转发路径分开，减少调试输出对传输的影响。

## 主要文件

| 文件或目录 | 作用 |
| --- | --- |
| `src/main.c` | BLE 连接、图像接收、缓冲和 SPI 转发主流程 |
| `src/PID.c`、`include/PID.h` | 基于 PID 的传输节奏控制支持 |
| `include/its.h` | Image Transport Service 定义 |
| `app.overlay` | 通用硬件 overlay |
| `boards/` | 各开发板配置和 overlay |
| `prj.conf` | Zephyr 应用配置 |
| `sysbuild/`、`sysbuild_bt_rpc.conf` | 多镜像和 Bluetooth RPC 配置 |
| `README.rst` | 保留的 Nordic Central UART 原始示例说明 |

## 构建

在 Nordic nRF Connect SDK 工作区中构建，并选择 `boards/` 中已有配置对应的开发板。典型 sysbuild 命令为：

```bash
west build -b <board-target> --sysbuild BLE-SPI-Rx
```

不同 nRF Connect SDK 版本的 board target 名称可能不同。连接 CC1310 前，应核对所选 overlay 中的 SPI 引脚和控制器角色。

下游帧长必须与 `915TX` 保持一致；当前端到端应用链路使用 250 字节帧。
