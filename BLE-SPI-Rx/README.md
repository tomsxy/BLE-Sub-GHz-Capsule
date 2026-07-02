# BLE-SPI-Rx

[中文说明](README_zh-CN.md) · [Repository overview](../README.md)

**Function:** Receive capsule images over BLE and transmit them to `915TX` over SPI.

This directory is the Nordic nRF Connect SDK/Zephyr bridge at the input of the Sub-GHz path. It receives capsule image data over Bluetooth Low Energy and forwards the stream to `915TX` over SPI.

## Responsibilities

- Operate as a BLE central/client for the image transport service.
- Receive image metadata, control messages, and image payload data.
- Buffer the incoming stream and emit fixed-size SPI transfers expected by the CC1310 transmitter.
- Expose board-specific SPI and GPIO wiring through Zephyr devicetree overlays.
- Keep transport diagnostics separate from the normal forwarding path.

## Main files

| File or directory | Role |
| --- | --- |
| `src/main.c` | BLE connection, image reception, buffering, and SPI forwarding |
| `src/PID.c`, `include/PID.h` | PID-based transport pacing support |
| `include/its.h` | Image Transport Service definitions |
| `app.overlay` | Common application hardware overlay |
| `boards/` | Supported-board configuration and overlay files |
| `prj.conf` | Zephyr application configuration |
| `sysbuild/`, `sysbuild_bt_rpc.conf` | Multi-image/Bluetooth RPC configuration |
| `README.rst` | Original Nordic Central UART sample documentation retained for reference |

## Build

Use a Nordic nRF Connect SDK workspace and select one of the board targets represented in `boards/`. A typical sysbuild command is:

```bash
west build -b <board-target> --sysbuild BLE-SPI-Rx
```

Board target spelling depends on the installed nRF Connect SDK version. Verify the SPI pins and controller role in the selected overlay before connecting the CC1310 board.

The downstream frame size must remain synchronized with `915TX`; the current end-to-end application path uses 250-byte frames.
