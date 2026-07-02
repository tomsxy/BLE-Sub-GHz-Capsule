# BLE-Sub-GHz Capsule

[English README](README.md)

本仓库包含胶囊图像通信原型的固件和主机端接收程序。完整链路组合了低功耗蓝牙、SPI、915 MHz Sub-GHz 无线传输，以及 Jetson 端的 JPEG 重组。

## 数据链路

```text
BLE-Sub-GHz Capsule
   -> BLE-SPI-Rx
   -> SPI
   -> CC1310 915TX
   -> 915 MHz Sub-GHz 无线链路
   -> CC1310 915RX
   -> SPI
   -> Jetson JPEG 接收程序
```

CC1310 工程使用 915 MHz 高速 RF 模式，射频包负载为 250 字节。Jetson 接收程序按以下格式解析每个 250 字节应用帧：

| 偏移 | 长度 | 字段 |
| ---: | ---: | --- |
| 0 | 1 | 同步字节 `0xAA` |
| 1 | 4 | JPEG 大小，小端序 |
| 5 | 4 | Payload offset，小端序 |
| 9 | 1 | 包头尾字节 `0xBB` |
| 10 | 240 | JPEG 数据 |

Jetson 程序会统计包边界、offset gap、短读和非法包头。只有同时具备 `FF D8` 开始标记和 `FF D9` 结束标记的数据才会被视为完整 JPEG。

## 仓库结构

| 路径 | 用途 |
| --- | --- |
| [`BLE-SPI-Rx/`](BLE-SPI-Rx/README_zh-CN.md) | Nordic/Zephyr BLE 接收端，通过 SPI 转发图像数据 |
| [`915TX/`](915TX/README_zh-CN.md) | TI CC1310 SPI 到 915 MHz 的发送端固件 |
| [`915RX/`](915RX/README_zh-CN.md) | TI CC1310 915 MHz 到 SPI 的接收端固件 |
| `jetson_spi_slave_jpeg_rx_simple.c` | Linux/Jetson SPI 接收与 JPEG 重组程序 |
| [`wifi_TCP-main/`](wifi_TCP-main/README_zh-CN.md) | Nordic nRF7002 Wi-Fi/BLE 图像传输实验及主机接收工具 |

仓库有意排除了构建产物、树莓派实验、接收结果图片和原始 ZIP 压缩包。

## 开发工具链

- `915TX` 和 `915RX`：Texas Instruments Code Composer Studio、TI-RTOS，以及适用于 CC1310 LaunchPad 的 SimpleLink CC13x0 SDK。
- `BLE-SPI-Rx`：Nordic nRF Connect SDK/Zephyr。将该目录作为应用导入，并从 `boards/` 中选择受支持的开发板配置。
- `wifi_TCP-main`：Nordic nRF Connect SDK/Zephyr，使用 nRF7002 DK 配置。
- Jetson 接收端：Linux C 编译工具链、pthreads 和内核 `spidev` 接口。

准确 SDK 版本取决于本机已有的厂商开发环境。仓库保留了工程文件和开发板 overlay，便于在对应工具链中导入。

## Jetson 接收程序

编译：

```bash
gcc -O2 -Wall -Wextra -pthread \
  -o jetson_spi_slave_jpeg_rx_simple \
  jetson_spi_slave_jpeg_rx_simple.c
```

查看全部运行参数：

```bash
./jetson_spi_slave_jpeg_rx_simple --help
```

使用默认配置运行，默认设备为 `/dev/spidev0.0`，SPI mode 为 1，速率为 4 MHz，分块长度为 250 字节：

```bash
sudo ./jetson_spi_slave_jpeg_rx_simple
```

接收程序支持 `read()`、`SPI_IOC_MESSAGE`、批量事务、队列处理、数据包十六进制打印和可配置的 gap 诊断。

## Wi-Fi 配置

公开仓库中的 `wifi_TCP-main/prj.conf` 使用以下占位符：

```text
CONFIG_WIFI_CREDENTIALS_STATIC_SSID="YOUR_WIFI_SSID"
CONFIG_WIFI_CREDENTIALS_STATIC_PASSWORD="YOUR_WIFI_PASSWORD"
```

需要启用静态凭据时，只在本地配置中替换这些值。不要把真实网络凭据提交到仓库。

## 注意事项

- 所有设备必须使用一致的 250 字节应用帧定义。
- 调试图像重组前，先确认 SPI mode、片选行为和时钟速率在两端一致。
- 仓库不保存构建输出，应使用对应厂商 SDK 重新生成。
- 第三方源码保留各自原有的版权和许可证声明；除非后续增加独立许可证文件，否则本仓库不授予统一许可证。
