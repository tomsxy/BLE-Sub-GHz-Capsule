#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import asyncio
import time
import os
import signal
from typing import Optional
from bleak import BleakScanner, BleakClient

# ====== GATT UUID（按你的协议）======
SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca3e"
RX_CHAR_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca3e"   # 控制：0x02 开始流, 0x03 停止流, 其他自定义
TX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca3e"   # 数据分片（图像字节）
IMG_INFO_CHAR_UUID = "6e400004-b5a3-f393-e0a9-e50e24dcca3e"  # 帧长度信息(可选)

SAVE_DIR = "frames_ultra"
RUN_SECONDS = 0          # 0 = 无限直到 Ctrl+C
PRINT_EVERY = 1.0        # 实时吞吐统计窗口(秒)
QUIET = True             # True = 最少日志

ITS_CMD_DEBUG = 0xF0     # 示例：每秒发一次调试命令，可改/可去掉


# ====== 扫描目标设备（按服务 UUID 过滤）======
async def find_addr(timeout: float = 8.0) -> Optional[str]:
    dev = await BleakScanner.find_device_by_filter(
        lambda d, ad: any((u or "").lower() == SERVICE_UUID.lower() for u in (ad.service_uuids or [])),
        timeout=timeout,
    )
    return dev.address if dev else None


def parse_img_info_length(payload: bytes) -> int:
    """
    兼容两种常见布局：
      A: data[1:5] 为小端长度
      B: data[0:4] 为小端长度
    非法返回 0（走 EOI 模式）
    """
    if len(payload) >= 5:
        n = int.from_bytes(payload[1:5], "little")
        if 64 <= n <= 2_000_000:
            return n
    if len(payload) >= 4:
        n = int.from_bytes(payload[0:4], "little")
        if 64 <= n <= 2_000_000:
            return n
    return 0


async def get_rssi_safe(client: BleakClient) -> Optional[int]:
    """
    某些平台（尤其 macOS/CoreBluetooth）BleakClient 没有 get_rssi()。
    这里做了静默兼容：没有就返回 None。
    """
    try:
        get_rssi = getattr(client, "get_rssi", None)
        if callable(get_rssi):
            return int(await get_rssi())
    except Exception:
        pass
    return None


async def main():
    os.makedirs(SAVE_DIR, exist_ok=True)

    addr = await find_addr(8.0)
    if not addr:
        print("[ERR] No target device advertising the service.")
        return

    # ========== 会被回调闭包捕获的状态 ==========
    frame_expected = 0            # 期望帧长（有长度模式）
    frame_buf = None              # 当前帧缓冲（bytearray 或 None）
    write_pos = 0                 # 已写入字节数（有长度模式）
    frame_id = 0                  # 保存文件编号
    frame_start_ts = 0.0          # 当前帧开始时间

    # 统计
    t_stat = time.monotonic()
    bytes_in_window = 0
    frames_in_window = 0
    total_bytes = 0
    total_frames = 0
    t0 = t_stat

    # ====== 完成帧：落盘 + 统计复位 ======
    def complete_frame_view(view: memoryview):
        nonlocal frame_id, frame_expected, frame_buf, write_pos, frame_start_ts
        nonlocal bytes_in_window, frames_in_window, total_bytes, total_frames

        now = time.monotonic()
        # 更新统计
        total_len = len(view)
        total_bytes += total_len
        frames_in_window += 1
        total_frames += 1

        # 保存 JPEG
        fname = os.path.join(SAVE_DIR, f"frame_{frame_id:06d}.jpg")
        with open(fname, "wb") as f:
            f.write(view)

        frame_id += 1
        # 复位帧状态
        frame_expected = 0
        frame_buf = None
        write_pos = 0
        frame_start_ts = 0.0

    # ====== 处理帧信息（长度）通知 ======
    def img_info_handler(_sender, data: bytearray):
        nonlocal frame_expected, frame_buf, write_pos, frame_start_ts
        size = parse_img_info_length(data)
        frame_expected = size
        write_pos = 0
        frame_start_ts = time.monotonic()
        if size > 0:
            # 预分配足量缓冲（避免反复扩容）
            frame_buf = bytearray(size)
        else:
            # 没有长度：走 EOI 模式（JPEG 尾标 FF D9）
            frame_buf = bytearray()

    # ====== 处理图像分片通知 ======
    def tx_handler(_sender, data: bytearray):
        nonlocal frame_expected, frame_buf, write_pos, frame_start_ts
        nonlocal bytes_in_window

        # 把这段分片计入吞吐统计窗口
        bytes_in_window += len(data)

        # 如果因为竞态 IMG_INFO 还没到，这里兜底初始化
        if frame_buf is None:
            frame_buf = bytearray()
            if not frame_start_ts:
                frame_start_ts = time.monotonic()

        if frame_expected > 0:
            # 有长度模式：按偏移写入，防越界
            n = len(data)
            end = write_pos + n
            if end > frame_expected:
                n = frame_expected - write_pos
                end = frame_expected
            if n > 0:
                frame_buf[write_pos:end] = data[:n]
                write_pos = end
            if write_pos >= frame_expected:
                # 帧完
                complete_frame_view(memoryview(frame_buf)[:write_pos])
        else:
            # 无长度模式（EOI）：累积到遇到 JPEG 尾标 FF D9
            frame_buf.extend(data)
            if len(frame_buf) >= 2 and frame_buf[-2:] == b"\xFF\xD9":
                complete_frame_view(memoryview(frame_buf))

    # ====== Ctrl+C 退出 ======
    stop = False
    def _sig(*_):
        nonlocal stop
        stop = True
    try:
        signal.signal(signal.SIGINT, _sig)
    except Exception:
        pass

    # ====== 连接并开始工作 ======
    async with BleakClient(addr) as client:
        if not client.is_connected:
            print("[ERR] Connect failed.")
            return
        if not QUIET:
            print("[OK] Connected:", addr)

        # 先订阅通知，再开流；订阅后小睡 50ms 降低竞态
        await client.start_notify(IMG_INFO_CHAR_UUID, img_info_handler)
        await client.start_notify(TX_CHAR_UUID, tx_handler)
        await asyncio.sleep(0.05)

        # 开始流（0x02）
        try:
            await client.write_gatt_char(RX_CHAR_UUID, bytearray([2]), response=True)
        except Exception as e:
            print("[ERR] start stream write failed:", e)
            # 清理订阅
            try:
                await client.stop_notify(TX_CHAR_UUID)
            except Exception:
                pass
            try:
                await client.stop_notify(IMG_INFO_CHAR_UUID)
            except Exception:
                pass
            return

        # 主循环：打印实时统计 & 可选发送调试命令
        deadline = time.monotonic() + (RUN_SECONDS if RUN_SECONDS > 0 else 10**9)
        while (time.monotonic() < deadline) and (not stop):
            await asyncio.sleep(3)
            now = time.monotonic()
            if (now - t_stat) >= PRINT_EVERY:
                kbps = (bytes_in_window * 8) / (now - t_stat) / 1000.0
                fps = frames_in_window / (now - t_stat)

                rssi = await get_rssi_safe(client)
                rssi_str = f"{rssi} dBm" if isinstance(rssi, int) else "N/A"

                print(f"[LIVE] {kbps:7.0f} kbps, {fps:4.1f} fps, RSSI: {rssi_str}  |  total: {total_frames} frames, {total_bytes / 1e6:.2f} MB")

                # 可选：每个窗口发一条调试命令（数值 9）
                try:
                    await client.write_gatt_char(RX_CHAR_UUID, bytes([0x07, 0x00]), response=True)
                except Exception:
                    pass

                # 窗口归零
                bytes_in_window = 0
                frames_in_window = 0
                t_stat = now

        # 停止流（0x03）
        try:
            await client.write_gatt_char(RX_CHAR_UUID, bytearray([3]), response=True)
        except Exception:
            pass

        # 取消订阅
        try:
            await client.stop_notify(TX_CHAR_UUID)
        except Exception:
            pass
        try:
            await client.stop_notify(IMG_INFO_CHAR_UUID)
        except Exception:
            pass

    # ====== 最终统计 ======
    t1 = time.monotonic()
    dur = max(1e-6, t1 - t0)
    avg_kbps = (total_bytes * 8) / dur / 1000.0
    print(f"[DONE] duration={dur:.2f}s, avg={avg_kbps:.0f} kbps, frames={total_frames}, bytes={total_bytes}")


if __name__ == "__main__":
    asyncio.run(main())
