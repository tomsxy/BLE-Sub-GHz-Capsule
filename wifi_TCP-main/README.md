# wifi_TCP-main

[中文说明](README_zh-CN.md) · [Repository overview](../README.md)

This directory is an alternative image transport experiment based on the Nordic nRF7002 DK. It combines Wi-Fi socket transport, BLE control/data support, embedded test images, and desktop receiver utilities.

## Responsibilities

- Connect to Wi-Fi in station mode or expose the configured SoftAP mode.
- Send image data over TCP/UDP-oriented socket paths.
- Support BLE image/control transport alongside Wi-Fi experiments.
- Provide fixed-image data for repeatable throughput tests.
- Provide Python desktop receivers for saving and displaying incoming images.

## Main files

| File or directory | Role |
| --- | --- |
| `src/main.c` | Application flow, image source selection, and transport scheduling |
| `src/socket_util.c` | Socket setup, send, receive, and connection helpers |
| `src/wifi_station_mode.c` | Wi-Fi station-mode handling |
| `src/wifi_softap_mode.c` | Wi-Fi SoftAP-mode handling |
| `src/app_bluetooth.c`, `src/its.c` | BLE application and Image Transport Service |
| `include/fixed_image_*.h` | Embedded images used for deterministic tests |
| `boards/` | nRF7002 DK configuration and overlays |
| `overlay‘s/` | Zephyr memory/performance experiment overlays; the directory name is retained from the source project |
| `receiver/` | Python TCP, UDP, BLE, and GUI receiver experiments |
| `prj.conf` | Main Zephyr and network configuration |

## Build

Use a Nordic nRF Connect SDK workspace. For SDK versions that use the legacy board target naming, a typical command is:

```bash
west build -b nrf7002dk_nrf5340_cpuapp --sysbuild wifi_TCP-main
```

Use the board target syntax required by your installed SDK. Generated `build/` and `7002/` directories are intentionally excluded.

## Receiver utilities

The `receiver/` directory contains several experimental clients rather than one canonical application. Check `receiver/requirements.txt`, select the client matching TCP, UDP, or BLE, and verify its bind address and output path before running it.

## Credentials

`prj.conf` contains public placeholders only:

```text
CONFIG_WIFI_CREDENTIALS_STATIC_SSID="YOUR_WIFI_SSID"
CONFIG_WIFI_CREDENTIALS_STATIC_PASSWORD="YOUR_WIFI_PASSWORD"
```

Replace these values locally and never commit real credentials. Static credentials are disabled in the committed configuration.
