# wifi_TCP-main

[English](README.md) · [仓库总览](../README_zh-CN.md)

**功能：** 通过 BLE 发送胶囊图像。

本目录是基于 Nordic nRF7002 DK 的另一套图像传输实验工程，包含 Wi-Fi socket 传输、BLE 控制/数据链路、内置测试图像和桌面接收工具。

## 主要职责

- 以 Station 模式连接 Wi-Fi，或运行配置好的 SoftAP 模式。
- 通过面向 TCP/UDP 的 socket 路径发送图像数据。
- 在 Wi-Fi 实验之外提供 BLE 图像和控制传输。
- 使用固定图像数据执行可重复的吞吐量测试。
- 提供用于保存和显示接收图像的 Python 桌面端程序。

## 主要文件

| 文件或目录 | 作用 |
| --- | --- |
| `src/main.c` | 应用主流程、图像源选择和传输调度 |
| `src/socket_util.c` | Socket 建立、发送、接收和连接辅助逻辑 |
| `src/wifi_station_mode.c` | Wi-Fi Station 模式处理 |
| `src/wifi_softap_mode.c` | Wi-Fi SoftAP 模式处理 |
| `src/app_bluetooth.c`、`src/its.c` | BLE 应用和 Image Transport Service |
| `include/fixed_image_*.h` | 用于确定性测试的内置图像 |
| `boards/` | nRF7002 DK 配置和 overlay |
| `overlay‘s/` | Zephyr 内存和性能实验 overlay；目录名沿用原工程 |
| `receiver/` | Python TCP、UDP、BLE 和 GUI 接收实验 |
| `prj.conf` | 主要 Zephyr 和网络配置 |

## 构建

在 Nordic nRF Connect SDK 工作区中构建。对于仍使用旧 board target 命名的 SDK，典型命令为：

```bash
west build -b nrf7002dk_nrf5340_cpuapp --sysbuild wifi_TCP-main
```

实际命令应使用当前 SDK 支持的 board target 语法。仓库有意排除了生成的 `build/` 和 `7002/` 目录。

## 接收端工具

`receiver/` 中包含多个实验性客户端，并非单一标准程序。运行前应查看 `receiver/requirements.txt`，根据 TCP、UDP 或 BLE 链路选择对应脚本，并确认绑定地址和输出路径。

## Wi-Fi 凭据

公开仓库的 `prj.conf` 只保留以下占位符：

```text
CONFIG_WIFI_CREDENTIALS_STATIC_SSID="YOUR_WIFI_SSID"
CONFIG_WIFI_CREDENTIALS_STATIC_PASSWORD="YOUR_WIFI_PASSWORD"
```

只在本地替换这些值，不要提交真实凭据。仓库当前配置中静态凭据功能处于关闭状态。
