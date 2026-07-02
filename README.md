# BLE-Sub-GHz Capsule

[中文说明](README_zh-CN.md)

This repository contains the firmware and host-side receiver for a capsule image communication prototype. The complete path combines Bluetooth Low Energy, SPI, a 915 MHz Sub-GHz link, and JPEG reconstruction on Jetson.

## Data path

```text
wifi_TCP-main
   -> BLE-SPI-Rx
   -> SPI
   -> CC1310 915TX
   -> 915 MHz Sub-GHz RF
   -> CC1310 915RX
   -> SPI
   -> Jetson JPEG receiver
```

The CC1310 radio projects use the high-speed RF mode at 915 MHz with a 250-byte packet payload. The Jetson receiver expects 250-byte application frames with the following layout:

| Offset | Size | Field |
| ---: | ---: | --- |
| 0 | 1 | Sync byte `0xAA` |
| 1 | 4 | JPEG size, little-endian |
| 5 | 4 | Payload offset, little-endian |
| 9 | 1 | Header tail `0xBB` |
| 10 | 240 | JPEG payload |

The Jetson program tracks packet alignment, offset gaps, short reads, and invalid headers. A file is treated as a JPEG only when it has the `FF D8` start marker and `FF D9` end marker.

## Repository layout

| Path | Purpose |
| --- | --- |
| [`wifi_TCP-main/`](wifi_TCP-main/README.md) | Nordic/Zephyr BLE image transmitter |
| [`BLE-SPI-Rx/`](BLE-SPI-Rx/README.md) | Nordic/Zephyr BLE receiver, SPI transmitter |
| [`915TX/`](915TX/README.md) | TI CC1310, SPI receiver, 915-MHz transmitter |
| [`915RX/`](915RX/README.md) | TI CC1310, 915-MHz receiver, SPI transmitter |
| `jetson_spi_slave_jpeg_rx_simple.c` | Linux/Jetson SPI receiver and JPEG reassembler |

Generated build directories, Raspberry Pi experiments, receive-output images, and the original ZIP archive are intentionally excluded.

## Toolchains

- `915TX` and `915RX`: Texas Instruments Code Composer Studio, TI-RTOS, and the SimpleLink CC13x0 SDK for CC1310 LaunchPad.
- `BLE-SPI-Rx`: Nordic nRF Connect SDK/Zephyr. Import the directory as an application and select a supported board configuration from `boards/`.
- `wifi_TCP-main`: Nordic nRF Connect SDK/Zephyr with an nRF7002 DK configuration.
- Jetson receiver: Linux C toolchain, pthreads, and the kernel `spidev` interface.

Exact SDK versions depend on the installed vendor workspace. The project files and board overlays are retained so each application can be imported into its native toolchain.

## Jetson receiver

Build:

```bash
gcc -O2 -Wall -Wextra -pthread \
  -o jetson_spi_slave_jpeg_rx_simple \
  jetson_spi_slave_jpeg_rx_simple.c
```

Show all runtime options:

```bash
./jetson_spi_slave_jpeg_rx_simple --help
```

Example using the defaults (`/dev/spidev0.0`, SPI mode 1, 4 MHz, 250-byte chunks):

```bash
sudo ./jetson_spi_slave_jpeg_rx_simple
```

The receiver supports both `read()` and `SPI_IOC_MESSAGE`, batched transfers, queue-based processing, packet dumps, and configurable gap diagnostics.

## Wi-Fi configuration

Public-repository placeholders are used in `wifi_TCP-main/prj.conf`:

```text
CONFIG_WIFI_CREDENTIALS_STATIC_SSID="YOUR_WIFI_SSID"
CONFIG_WIFI_CREDENTIALS_STATIC_PASSWORD="YOUR_WIFI_PASSWORD"
```

Replace them only in a local configuration before enabling static credentials. Do not commit real network credentials.

## Notes

- Keep all devices on the same 250-byte application-frame definition.
- Verify SPI mode, chip-select behavior, and clock rate at both ends before debugging image reconstruction.
- Build outputs are not versioned; regenerate them with the corresponding vendor SDK.
- Third-party source files retain their original copyright and license notices. No repository-wide license is granted unless a separate license file is added.
