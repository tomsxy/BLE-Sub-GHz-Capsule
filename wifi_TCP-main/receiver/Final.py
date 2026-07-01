#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import socket
import configparser
import time
import os
import asyncio
import struct

from io import BytesIO
from typing import Optional

from PIL import Image
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import math
from datetime import datetime

# ====== 严格模式开关 ======
STRICT_MODE = False

# ================= BLE 常量 =================
try:
    from bleak import BleakScanner, BleakClient
    BLE_AVAILABLE = True
except Exception:
    BLE_AVAILABLE = False

SERVICE_UUID       = "6e400001-b5a3-f393-e0a9-e50e24dcca3e"
RX_CHAR_UUID       = "6e400002-b5a3-f393-e0a9-e50e24dcca3e"   # 写
TX_CHAR_UUID       = "6e400003-b5a3-f393-e0a9-e50e24dcca3e"   # 图像分片通知
IMG_INFO_CHAR_UUID = "6e400004-b5a3-f393-e0a9-e50e24dcca3e"   # 帧长度信息（可选）

def _has_jpeg_markers(buf: bytes) -> bool:
    return len(buf) >= 4 and buf[:2] == b"\xFF\xD8" and buf[-2:] == b"\xFF\xD9"

# ============== 自适应等比显示的 Label ==============
class AspectRatioLabel(QLabel):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._orig_pm: Optional[QPixmap] = None
        self.setAlignment(Qt.AlignCenter)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def setPixmap(self, pm: QPixmap):  # type: ignore[override]
        self._orig_pm = QPixmap(pm)
        self._apply_scaled()

    def resizeEvent(self, e: QResizeEvent):  # type: ignore[override]
        super().resizeEvent(e)
        self._apply_scaled()

    def _apply_scaled(self):
        if self._orig_pm is None or self._orig_pm.isNull():
            super().setPixmap(QPixmap())
            return
        scaled = self._orig_pm.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        super().setPixmap(scaled)

# # ================= Wi-Fi UDP 客户端（严格：只认帧化 + SOI/EOI） =================
# class SocketClient(QObject):
#     command_receiving_signal = pyqtSignal(bytes)
#     stop_signal = pyqtSignal()

#     def __init__(self, cam_address='192.168.1.1', cam_port=60010, local_port=60000):
#         super().__init__()
#         self.cam_address = cam_address
#         self.cam_port = cam_port
#         self.local_port = local_port
#         self.buffer_size = 65535

#         self.pc_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#         self.pc_socket.bind(('0.0.0.0', self.local_port))
#         self.pc_socket.connect((cam_address, cam_port))
#         self.pc_socket.settimeout(0.3)

#         self.running = True
#         self.stop_signal.connect(self.stop)
#         self._buf = bytearray()

#     def send_command(self, hex_str: str):
#         try:
#             data = bytes.fromhex(hex_str.replace(" ", ""))
#             self.pc_socket.send(data)
#         except Exception:
#             QMessageBox.critical(None, "Error", "Fail to send command (UDP)")

#     def run(self):
#         try:
#             while self.running:
#                 try:
#                     data = self.pc_socket.recv(self.buffer_size)
#                 except socket.timeout:
#                     continue
#                 except Exception:
#                     if self.running:
#                         QMessageBox.critical(None, "Error", "Fail to receive packets (UDP)")
#                     break

#                 self._buf.extend(data)

#                 # 严格模式：仅帧化提取；禁用 MJPEG 兜底
#                 made = True
#                 while made:
#                     made = self._try_extract_framed()

#                 if len(self._buf) > 8 * 1024 * 1024:
#                     del self._buf[:4 * 1024 * 1024]
#         finally:
#             self.stop()

#     def stop(self):
#         self.running = False
#         try:
#             self.pc_socket.close()
#         except Exception:
#             pass

#     # 帧化：FF AA | type | len(4 LE) | payload | FF BB （严格校验 JPEG 头尾）
#     def _try_extract_framed(self) -> bool:
#         s = self._buf.find(b"\xFF\xAA")
#         if s == -1:
#             return False
#         if s > 0:
#             del self._buf[:s]
#         if len(self._buf) < 7:
#             return False

#         frame_type = self._buf[2]
#         payload_len = int.from_bytes(self._buf[3:7], "little", signed=False)
#         total = 7 + payload_len + 2
#         if payload_len <= 0 or payload_len > 10_000_000:
#             del self._buf[:2]
#             return True
#         if len(self._buf) < total:
#             return False
#         if self._buf[total-2:total] != b"\xFF\xBB":
#             del self._buf[:2]
#             return True

#         # 严格：仅当 type=0x01 时检查 JPEG 头尾
#         if frame_type == 0x01 and STRICT_MODE:
#             payload = self._buf[7:7+payload_len]
#             if not _has_jpeg_markers(payload):
#                 del self._buf[:total]  # 丢弃该帧
#                 return True

#         full = bytes(self._buf[:total])
#         self.command_receiving_signal.emit(full)
#         del self._buf[:total]
#         return True 严格板


class SocketClient(QObject):
    command_receiving_signal = pyqtSignal(bytes)
    stop_signal = pyqtSignal()

    def __init__(self, cam_address='192.168.1.1', cam_port=60010, local_port=60000):
        super().__init__()
        self.cam_address = cam_address
        self.cam_port = cam_port
        self.local_port = local_port
        self.buffer_size = 65535

        self.pc_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.pc_socket.bind(('0.0.0.0', self.local_port))
        self.pc_socket.connect((cam_address, cam_port))
        self.pc_socket.settimeout(0.3)

        self.running = True
        self.stop_signal.connect(self.stop)

        # 非严格：基于起止分隔符的缓冲
        self._buf = bytearray()
        self._in_frame = False
        self._max_frame = 10_000_000  # 10MB 上限，防止内存炸裂

    def run(self):
        try:
            while self.running:
                try:
                    data = self.pc_socket.recv(self.buffer_size)
                except socket.timeout:
                    continue
                except Exception:
                    if self.running:
                        QMessageBox.critical(None, "Error", "Fail to receive packets (UDP)")
                    break

                self._buf.extend(data)

                # 可能一次拼出多帧
                made = True
                while made:
                    made = self._try_extract_delimited()
                # 背压：缓存太大就丢掉前半
                if len(self._buf) > 8 * 1024 * 1024:
                    del self._buf[:4 * 1024 * 1024]
        finally:
            self.stop()

    def _try_extract_delimited(self) -> bool:
        """
        非严格成帧：FF AA ... FF BB
        - 不依赖长度字段
        - 允许跨包
        """
        # 找起始
        s = self._buf.find(b"\xFF\xAA")
        if s == -1:
            # 只保留少量尾巴以防止撕裂头
            if len(self._buf) > 4:
                del self._buf[:-4]
            return False

        # 扔掉起始前的垃圾
        if s > 0:
            del self._buf[:s]

        # 查找结束标记
        e = self._buf.find(b"\xFF\xBB", 2)  # 从2开始，避免把头当尾
        if e == -1:
            # 还没收齐
            # 保护：单帧过大就丢弃重来
            if len(self._buf) > self._max_frame:
                del self._buf[:2]  # 丢掉头两个字节，尝试重新对齐
                return True
            return False

        total = e + 2   # e 是尾标的起点，+2 含尾两个字节
        full = bytes(self._buf[:total])
        self.command_receiving_signal.emit(full)
        del self._buf[:total]
        return True

    def send_command(self, hex_str: str):
        """向相机发送十六进制命令串，例如 '55 0F AA'"""
        try:
            data = bytes.fromhex(hex_str.replace(" ", ""))
            # 已经 connect() 过，直接 send()
            self.pc_socket.send(data)
        except Exception:
            QMessageBox.critical(None, "Error", "Fail to send command (UDP)")

    def stop(self):
        self.running = False
        try:
            self.pc_socket.close()
        except Exception:
            pass

# # ================= UDP 帧解析/显示（再次做严格 JPEG 校验） =================
# class ArducamMegaCameraDataProcess:
#     def __init__(self):
#         self._bytes_accum = 0
#         self._frames_accum = 0
#         self._t0 = time.time()

#     def process_command(self, command: bytes):
#         if len(command) < 9:
#             return "Invalid packet"

#         if not command.startswith(b'\xFF\xAA') or not command.endswith(b'\xFF\xBB'):
#             return "Invalid start/stop marker"

#         ctype = command[2]
#         if ctype == 0x01:
#             n = int.from_bytes(command[3:7], "little")
#             frame = command[7:7+n]
#             if STRICT_MODE and not _has_jpeg_markers(frame):
#                 return "Dropped frame: invalid JPEG markers"
#             return self._on_jpeg(n, frame)
#         elif ctype == 0x02:
#             n = int.from_bytes(command[3:7], "little")
#             info = command[7:7+n].decode('utf-8', errors='ignore')
#             return f"Connected to WiFi Camera!\n{info}"
#         else:
#             return f"Type 0x{ctype:02X} ignored"

#     def _on_jpeg(self, size: int, buf: bytes):
#         try:
#             Image.open(BytesIO(buf)).save("img.jpg", "JPEG")
#         except Exception:
#             pass

#         self._bytes_accum += size
#         self._frames_accum += 1
#         now = time.time()
#         dt = now - self._t0
#         msg = f"FrameSize: {size} Bytes"
#         if dt >= 1.0:
#             fps = self._frames_accum / dt
#             kbps = (self._bytes_accum * 8) / 1024.0 / dt
#             msg += f" | FPS: {fps:.2f} | Throughput: {kbps:.2f} kbps"
#             self._t0 = now
#             self._bytes_accum = 0
#             self._frames_accum = 0
#         return msg

class ArducamMegaCameraDataProcess:
    def __init__(self):
        self._bytes_accum = 0
        self._frames_accum = 0
        self._t0 = time.time()

    def process_command(self, command: bytes):
        # 只要起始/结尾正确，就按 type 分支
        if not (command.startswith(b"\xFF\xAA") and command.endswith(b"\xFF\xBB")):
            return "Invalid start/stop marker"

        ctype = command[2]
        payload = command[7:-2] if len(command) >= 9 else b""
        if ctype == 0x01:
            # 宽松：不检查 JPEG SOI/EOI，直接尝试保存/显示
            return self._on_jpeg(len(payload), payload)
        elif ctype == 0x02:
            try:
                info = payload.decode("utf-8", errors="ignore")
            except Exception:
                info = ""
            return f"Connected to WiFi Camera!\n{info}"
        else:
            return f"Type 0x{ctype:02X} ignored"

    def _on_jpeg(self, size: int, buf: bytes):
        # 尝试落盘，不因异常中断
        try:
            Image.open(BytesIO(buf)).save("img.jpg", "JPEG")
        except Exception:
            pass

        self._bytes_accum += size
        self._frames_accum += 1
        now = time.time()
        dt = now - self._t0
        msg = f"FrameSize: {size} Bytes"
        if dt >= 1.0:
            fps = self._frames_accum / dt
            kbps = (self._bytes_accum * 8) / 1024.0 / dt
            msg += f" | FPS: {fps:.2f} | Throughput: {kbps:.2f} kbps"
            self._t0 = now
            self._bytes_accum = 0
            self._frames_accum = 0
        return msg

# ================= 简易 PID =================
class SimplePID:
    def __init__(self, kp=1e-6, ki=0.0, kd=0.0, i_limit=0.5, out_limit=3.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_limit = float(i_limit)
        self.out_limit = float(out_limit)
        self.integral = 0.0
        self.prev_err: Optional[float] = None

    def reset(self):
        self.integral = 0.0
        self.prev_err = None

    def step(self, err: float, dt: float) -> float:
        if dt <= 0:
            dt = 1e-3
        # I
        self.integral += err * dt
        if self.integral > self.i_limit:
            self.integral = self.i_limit
        elif self.integral < -self.i_limit:
            self.integral = -self.i_limit
        # D
        d = 0.0 if self.prev_err is None else (err - self.prev_err) / dt
        self.prev_err = err
        # PID
        out = self.kp * err + self.ki * self.integral + self.kd * d
        if out > self.out_limit:
            out = self.out_limit
        elif out < -self.out_limit:
            out = -self.out_limit
        return out

# ================= BLE Worker（严格 + 吞吐打印 + TXP 控制 + RSSI） =================
class BLEWorker(QObject):
    log = pyqtSignal(str)
    frame = pyqtSignal(bytes)        # 完整 JPEG
    connected = pyqtSignal()
    disconnected = pyqtSignal()
    rssi = pyqtSignal(int)           # RSSI 信号

    @property
    def is_connected(self):
        return bool(self._client and self._client.is_connected)

    def __init__(self):
        super().__init__()
        self._loop = None
        self._task = None
        self._stop_evt: Optional[asyncio.Event] = None
        self._poke_evt: Optional[asyncio.Event] = None
        self._client: Optional[BleakClient] = None

        # 组帧
        self._frame_expected = 0
        self._frame_buf = bytearray()
        self._write_pos = 0

        # 吞吐统计
        self._bytes_in_window = 0
        self._frames_in_window = 0
        self._win_t0 = 0.0
        self._PRINT_EVERY = 1.0
        self._last_rate_bps = 0.0  # 给 TXP 控制内部使用

        # TXP 控制
        self._ewma_rate_bps: Optional[float] = None
        self._ewma_tau_sec: float = 2.8  # 平滑时间常数
        self._txp_enabled = False
        self._txp_value = 20
        self._txp_min = -30
        self._txp_max = 20
        self._rate_target_bps: float = 800_000.0  # 固定目标：800000 bps
        self._pid = SimplePID(kp=9e-6, ki=0.0, kd=1e-7, i_limit=0.5, out_limit=3.0)

        # 记录最近 RSSI
        self._last_rssi: Optional[int] = None
        self._rssi_warned = False

    # --- 供 GUI 调用 ---
    @pyqtSlot(bool)
    def set_txp_control_enabled(self, enabled: bool):
        if self._loop and not self._loop.is_closed():
            def _do():
                self._txp_enabled = enabled
                if enabled:
                    self._pid.reset()
                    self.log.emit("[BLE][TXP] control enabled, target = 800 kbps")
                else:
                    self.log.emit("[BLE][TXP] control disabled")
            self._loop.call_soon_threadsafe(_do)

    @pyqtSlot()
    def start(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._stop_evt = asyncio.Event()
        self._poke_evt = asyncio.Event()
        self._task = self._loop.create_task(self._main())
        try:
            self._loop.run_until_complete(self._task)
        except asyncio.CancelledError:
            pass
        except Exception as e:
            self.log.emit(f"[BLE] Fatal: {e}")
        finally:
            try:
                if self._client and self._client.is_connected:
                    self._loop.run_until_complete(self._client.disconnect())
            except Exception:
                pass
            try:
                self._loop.stop()
                self._loop.close()
                self._last_rate_bps = 0.0
                self._pid.reset()
                self._ewma_rate_bps = None
            except Exception:
                pass
            self.disconnected.emit()

    @pyqtSlot()
    def stop(self):
        if self._loop and not self._loop.is_closed():
            self._loop.call_soon_threadsafe(self._stop_evt.set)

    @pyqtSlot()
    def poke(self):
        if self._loop and self._poke_evt and not self._loop.is_closed():
            self._loop.call_soon_threadsafe(self._poke_evt.set)

    @pyqtSlot(bytes)
    def send_raw(self, payload: bytes):
        if self._loop and self._client and self._client.is_connected:
            async def _send():
                try:
                    await self._client.write_gatt_char(RX_CHAR_UUID, payload, response=True)
                    self.log.emit(f"[BLE][TX] {payload.hex(' ')}")
                except Exception as e:
                    self.log.emit(f"[BLE][TX][ERR] {e}")
            self._loop.call_soon_threadsafe(lambda: asyncio.create_task(_send()))
        else:
            self.log.emit("[BLE][TX][ERR] Not connected.")
    @pyqtSlot(float, float)
    def send_bbox(self, x: float, y: float):
        """单次上报边框 (x, y) 给 Nordic：0x0B + <ff, LE>"""
        if not (self._loop and self._client and self._client.is_connected):
            self.log.emit("[BLE][ITS][BORDER][ERR] Not connected.")
            return

        async def _send():
            try:
                payload = struct.pack('<ff', float(x), float(y))
                await self._client.write_gatt_char(RX_CHAR_UUID, bytes([0x0B]) + payload, response=True)
                self.log.emit(f"[BLE][ITS] BORDER sent: ({x:.2f}, {y:.2f})")
            except Exception as e:
                self.log.emit(f"[BLE][ITS][BORDER][ERR] {e}")

        # 丢到 BLEWorker 的事件循环里执行
        self._loop.call_soon_threadsafe(lambda: asyncio.create_task(_send()))


    async def _main(self):
        backoff = 1.0
        while not self._stop_evt.is_set():
            try:
                self.log.emit("[BLE] Scanning by service UUID...")
                addr = await self._find_addr(timeout=8.0)
                if not addr:
                    self.log.emit("[BLE] 未找到目标设备。")
                    await self._sleep_or_poke(backoff)
                    backoff = min(backoff * 1.5, 5.0)
                    continue
                backoff = 1.0

                client = BleakClient(addr)
                try:
                    await client.connect()
                    if not client.is_connected:
                        self.log.emit("[BLE][ERR] Connect failed.")
                        await self._sleep_or_poke(1.0)
                        continue

                    self._client = client
                    self.connected.emit()
                    self.log.emit(f"[BLE] Connected: {addr}")

                    try:
                        await client.start_notify(IMG_INFO_CHAR_UUID, self._img_info_handler)
                    except Exception as e:
                        self.log.emit(f"[BLE] start_notify IMG_INFO failed: {e}")
                    try:
                        await client.start_notify(TX_CHAR_UUID, self._tx_handler)
                    except Exception as e:
                        self.log.emit(f"[BLE] start_notify TX failed: {e}")

                    # 重置统计窗口
                    self._win_t0 = time.monotonic()
                    self._bytes_in_window = 0
                    self._frames_in_window = 0

                    while not self._stop_evt.is_set() and client.is_connected:
                        await asyncio.sleep(0.2)

                        # === 每秒统计窗口：只有有数据时才计算/打印/调功 ===
                        now = time.monotonic()
                        if (now - self._win_t0) >= self._PRINT_EVERY:
                            seconds = (now - self._win_t0)

                            # --- 读取RSSI（连接态，优雅降级）
                            # 连接后循环里
                            try:
                                if hasattr(client, "get_rssi"):
                                    rssi = await client.get_rssi()  # macOS: 调用 CoreBluetooth readRSSI
                                    if isinstance(rssi, int):
                                        self.rssi.emit(rssi)  # 更新 UI
                                        self.log.emit(f"[BLE][RSSI] {rssi} dBm")
                            except Exception as e:
                                self.log.emit(f"[BLE][RSSI][ERR] {e}")

                            # 这一秒完全无数据：只滚动窗口起点
                            if self._bytes_in_window == 0 and self._frames_in_window == 0:
                                self._win_t0 = now
                                continue

                            # 有数据才计算/打印
                            kbps = (self._bytes_in_window * 8) / seconds / 1000.0
                            fps = self._frames_in_window / seconds
                            rate_bps = kbps * 1000.0
                            self._last_rate_bps = rate_bps

                            # —— EWMA 更新
                            alpha = 1.0 - math.exp(-seconds / self._ewma_tau_sec)
                            if self._ewma_rate_bps is None:
                                self._ewma_rate_bps = rate_bps
                            else:
                                self._ewma_rate_bps += alpha * (rate_bps - self._ewma_rate_bps)

                            # —— 日志：吞吐打印
                            self.log.emit(f"[BLE 1s] {kbps:7.2f} kbps, {fps:5.2f} fps")

                            # === 每秒上报吞吐（0x0A） ===
                            try:
                                tp_payload = struct.pack('<f', float(kbps))  # 单位：kbps
                                await client.write_gatt_char(RX_CHAR_UUID, bytes([0x0A]) + tp_payload, response=True)
                                self.log.emit(f"[BLE][ITS] THROUGHPUT sent: {kbps:.2f} kbps")
                            except Exception as e:
                                self.log.emit(f"[BLE][ITS][THROUGHPUT][ERR] {e}")

                            # —— TXP 控制（固定目标 800000 bps）：用 EWMA 做反馈
                            if self._txp_enabled:
                                target = self._rate_target_bps
                                meas_bps = self._ewma_rate_bps if self._ewma_rate_bps is not None else rate_bps
                                err = target - meas_bps
                                delta = self._pid.step(err, seconds)
                                self._txp_value = int(round(self._txp_value + delta))
                                if self._txp_value < self._txp_min:
                                    self._txp_value = self._txp_min
                                if self._txp_value > self._txp_max:
                                    self._txp_value = self._txp_max

                                try:
                                    payload = bytes([0x07, self._txp_value & 0xFF])
                                    await client.write_gatt_char(RX_CHAR_UUID, payload, response=True)
                                    self.log.emit(f"target={target / 1000:.0f} kbps -> txp={self._txp_value}")
                                except Exception as e:
                                    self.log.emit(f"[BLE][TXP][ERR] {e}")

                            # 清窗口
                            self._bytes_in_window = 0
                            self._frames_in_window = 0
                            self._win_t0 = now

                except Exception as e:
                    self.log.emit(f"[BLE][ERR] {e}")
                finally:
                    try:
                        if client.is_connected:
                            try:
                                await client.stop_notify(TX_CHAR_UUID)
                            except Exception:
                                pass
                            try:
                                await client.stop_notify(IMG_INFO_CHAR_UUID)
                            except Exception:
                                pass
                            await client.disconnect()
                    except Exception:
                        pass

                    self._client = None
                    self._frame_expected = 0
                    self._frame_buf = bytearray()
                    self._write_pos = 0

                    # 清空统计和 PID
                    self._bytes_in_window = 0
                    self._frames_in_window = 0
                    self._win_t0 = 0.0
                    self._last_rate_bps = 0.0
                    self._pid.reset()

                    self.disconnected.emit()
                    self.log.emit("[BLE] Disconnected. Will retry...")

                    await self._sleep_or_poke(backoff)
                    backoff = min(backoff * 1.5, 5.0)

            except Exception as e:
                self.log.emit(f"[BLE][Loop][ERR] {e}")
                await self._sleep_or_poke(1.0)

    async def _sleep_or_poke(self, seconds: float):
        self._poke_evt.clear()
        try:
            await asyncio.wait_for(self._poke_evt.wait(), timeout=seconds)
        except asyncio.TimeoutError:
            pass

    async def _find_addr(self, timeout: float = 8.0) -> Optional[str]:
        def _filter(d, ad):
            uuids = (ad.service_uuids or [])
            return any((u or "").lower() == SERVICE_UUID.lower() for u in uuids)
        dev = await BleakScanner.find_device_by_filter(_filter, timeout=timeout)
        if dev:
            try:
                self.log.emit(f"[BLE][SCAN] found {dev.address} RSSI={getattr(dev, 'rssi', 'NA')} dBm")
            except Exception:
                pass
            return dev.address
        return None

    # ---- 组帧：严格只用 IMG_INFO 长度；到点后校验 JPEG 头尾 ----
    def _parse_img_info_length(self, payload: bytes) -> int:
        if len(payload) >= 5:
            n = int.from_bytes(payload[1:5], "little")
            if 64 <= n <= 2_000_000:
                return n
        if len(payload) >= 4:
            n = int.from_bytes(payload[0:4], "little")
            if 64 <= n <= 2_000_000:
                return n
        return 0

    def _img_info_handler(self, _sender, data: bytearray):
        size = self._parse_img_info_length(bytes(data))
        self._frame_expected = size
        self._write_pos = 0
        if size > 0:
            self._frame_buf = bytearray(size)
        else:
            self._frame_buf = bytearray()

    def _tx_handler(self, _sender, data: bytearray):
        chunk = bytes(data)
        self._bytes_in_window += len(chunk)  # 统计字节数

        if self._frame_expected > 0:
            view = memoryview(chunk)
            while len(view):
                remain = self._frame_expected - self._write_pos
                n = min(remain, len(view))
                if n > 0:
                    self._frame_buf[self._write_pos:self._write_pos+n] = view[:n]
                    self._write_pos += n
                    view = view[n:]

                if self._write_pos >= self._frame_expected:
                    frame_bytes = bytes(self._frame_buf[:self._write_pos])
                    if not (STRICT_MODE and not _has_jpeg_markers(frame_bytes)):
                        self._frames_in_window += 1
                        self.frame.emit(frame_bytes)
                    # 复位
                    self._frame_expected = 0
                    self._frame_buf = bytearray()
                    self._write_pos = 0
                    break
        else:
            return  # 严格：没有 IMG_INFO 就忽略

# ================= GUI（紧凑自适应） =================
class WifiCamHostGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("WiFi/BLE Camera Host")
        self.resize(1180, 780)
        self.setWindowIcon(QIcon("icon.png"))

        # 读取配置
        self.config = configparser.ConfigParser()
        self.config.read('config.ini')
        target_ip = self.config.get('DEFAULT', 'TargetIP', fallback='192.168.1.1')
        target_port = int(self.config.get('DEFAULT', 'TargetPort', fallback='60010'))
        self.target_server = (target_ip, target_port)

        # 线程
        self.client: Optional[SocketClient] = None
        self.socket_thread: Optional[QThread] = None
        self.ble_thread: Optional[QThread] = None
        self.ble_worker: Optional[BLEWorker] = None

        # 解析器
        self.camera = ArducamMegaCameraDataProcess()

        # 日志记录
        self._is_logging = False
        self._log_fp: Optional[open] = None
        self._log_path: Optional[str] = None

        self._build_ui()
        self._start_ble_auto()

        # --- 底部右侧 Logo（最右下角） ---
        self._setup_logo_in_statusbar()

    # ---- Logo ----
    def _setup_logo_in_statusbar(self):
        self._logo_label = QLabel()
        self._logo_label.setScaledContents(True)
        self._logo_label.setFixedHeight(28)  # 状态栏高度
        pm = self._load_logo_pixmap()
        if not pm.isNull():
            # 维持比例：按设定高度计算宽度
            w = int(pm.width() * (28 / max(1, pm.height())))
            self._logo_label.setFixedWidth(max(28, w))
            self._logo_label.setPixmap(pm)
        self.statusBar().addPermanentWidget(self._logo_label, 0)

    def _load_logo_pixmap(self) -> QPixmap:
        # 优先从桌面找
        desktop = os.path.expanduser("~/Desktop")
        candidates = [
            os.path.join(desktop, "DARE LAB Logo.jpg"),
            os.path.join(desktop, "DARE_LAB_Logo.jpg"),
            os.path.join(desktop, "DARE.jpg"),
            os.path.join(desktop, "logo.jpg"),
            # 会话环境里的备选（你上传的）
            "/mnt/data/DARE LAB Logo.jpg",
        ]
        for p in candidates:
            if os.path.exists(p):
                pm = QPixmap(p)
                if not pm.isNull():
                    return pm
        return QPixmap()  # 空

    def _build_ui(self):
        main = QWidget()
        root = QVBoxLayout(main)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(8)

        splitter = QSplitter(Qt.Horizontal)
        splitter.setHandleWidth(6)
        root.addWidget(splitter)

        # 左：视频（自适应等比）
        left = QWidget()
        lyt = QVBoxLayout(left)
        lyt.setContentsMargins(0, 0, 0, 0)
        lyt.setSpacing(6)

        self.video_label = AspectRatioLabel("Video/Image will show here!")
        self.video_label.setStyleSheet("background:#222; color:#ddd; border-radius:6px;")
        self.video_label.setMinimumSize(420, 320)
        lyt.addWidget(self.video_label)

        splitter.addWidget(left)

        # 右：侧栏（更紧凑）
        right = QWidget()
        rlyt = QVBoxLayout(right)
        rlyt.setContentsMargins(0, 0, 0, 0)
        rlyt.setSpacing(8)

        # Wi-Fi 区
        wifi_box = QGroupBox("Wi-Fi Receiver")
        wifi_box.setStyleSheet("QGroupBox{font-weight:600;}")
        wlyt = QGridLayout(wifi_box)
        wlyt.setHorizontalSpacing(6)
        wlyt.setVerticalSpacing(6)
        wlyt.setContentsMargins(8, 10, 8, 8)

        lbl_addr = QLabel("Target WiFi Camera Address")
        self.target_server_entry = QLineEdit(f"{self.target_server[0]}:{self.target_server[1]}")
        self.target_server_entry.setPlaceholderText("ip:port")
        self.target_server_entry.textChanged.connect(self._update_target_server)
        self.target_server_entry.setClearButtonEnabled(True)

        btn_wifi_connect = QPushButton("Wi-Fi Connect")
        btn_wifi_connect.clicked.connect(self._on_wifi_connect_clicked)
        btn_wifi_disconnect = QPushButton("Wi-Fi Disconnect")
        btn_wifi_disconnect.clicked.connect(self._on_wifi_disconnect_clicked)

        wlyt.addWidget(lbl_addr,               0, 0, 1, 2)
        wlyt.addWidget(self.target_server_entry,1, 0, 1, 2)
        wlyt.addWidget(btn_wifi_connect,       2, 0, 1, 1)
        wlyt.addWidget(btn_wifi_disconnect,    2, 1, 1, 1)

        rlyt.addWidget(wifi_box)

        # BLE 区
        ble_box = QGroupBox("BLE Control")
        ble_box.setStyleSheet("QGroupBox{font-weight:600;}")
        blyt = QGridLayout(ble_box)
        blyt.setHorizontalSpacing(6)
        blyt.setVerticalSpacing(6)
        blyt.setContentsMargins(8, 10, 8, 8)

        self.btn_ble_connect = QPushButton("BLE Connect")
        self.btn_ble_connect.clicked.connect(self._on_ble_connect_clicked)

        # TXP 控制开关（checkable）
        self.btn_txp = QPushButton("TXP control OFF")
        self.btn_txp.setCheckable(True)
        self.btn_txp.setEnabled(False)
        self.btn_txp.toggled.connect(self._on_txp_toggled)

        self.ble_cmd_edit = QLineEdit()
        self.ble_cmd_edit.setPlaceholderText("Hex bytes, e.g. 0x09 0x01  （启用 Wi-Fi 推流）")
        self.ble_cmd_edit.returnPressed.connect(self._on_ble_send_cmd)
        btn_send = QPushButton("Send")
        btn_send.clicked.connect(self._on_ble_send_cmd)

        # RSSI 标签
        # self.lbl_rssi = QLabel("RSSI: — dBm")
        # self.lbl_rssi.setAlignment(Qt.AlignLeft)

        blyt.addWidget(self.btn_ble_connect, 0, 0, 1, 2)
        blyt.addWidget(self.btn_txp,         1, 0, 1, 2)
        blyt.addWidget(self.ble_cmd_edit,    2, 0, 1, 1)
        blyt.addWidget(btn_send,             2, 1, 1, 1)
        # blyt.addWidget(self.lbl_rssi,        3, 0, 1, 2)

        # ---- BBox 输入与发送 ----
        self.edit_bbox_x = QLineEdit("66.0")
        self.edit_bbox_x.setPlaceholderText("BBox X (float)")
        self.edit_bbox_y = QLineEdit("66.0")
        self.edit_bbox_y.setPlaceholderText("BBox Y (float)")
        # 可选：只允许浮点
        from PyQt5.QtGui import QDoubleValidator
        self.edit_bbox_x.setValidator(QDoubleValidator())
        self.edit_bbox_y.setValidator(QDoubleValidator())

        btn_send_bbox = QPushButton("Send BBox")
        btn_send_bbox.clicked.connect(self._on_send_bbox_clicked)

        # 放到第 4、5 行
        blyt.addWidget(self.edit_bbox_x, 4, 0, 1, 1)
        blyt.addWidget(self.edit_bbox_y, 4, 1, 1, 1)
        blyt.addWidget(btn_send_bbox,    5, 0, 1, 2)


        rlyt.addWidget(ble_box)

        # 日志
        log_box = QGroupBox("Log")
        log_box.setStyleSheet("QGroupBox{font-weight:600;}")
        lbox = QVBoxLayout(log_box)
        lbox.setContentsMargins(8, 10, 8, 8)
        lbox.setSpacing(6)

        self.log_text = QPlainTextEdit(
            # "严格模式：收到长度且 JPEG 含 SOI/EOI 才显示。\n"
            # "通过 BLE 命令控制推流：\n  0x09 0x01 -> Wi-Fi Active\n  0x02 0x01 -> BLE Active\n"
            # "TXP control：固定目标 800000 bps，每秒发送 0x07 <desired_txp>\n"
            # "RSSI：连接后每秒读取一次并显示在右侧。\n"
            # "Logging：点击 Start/Stop 将日志保存到桌面。\n"
            "Hello World!\n"

        )
        self.log_text.setReadOnly(True)
        self.log_text.setMinimumHeight(180)

        # 3 个按钮：Start / Stop / Clear（在 Log 框下面）
        btn_row = QHBoxLayout()
        self.btn_log_start = QPushButton("Start Logging")
        self.btn_log_stop  = QPushButton("Stop Logging")
        self.btn_log_stop.setEnabled(False)  # 初始不可用
        btn_clear = QPushButton("Clear Log")

        self.btn_log_start.clicked.connect(self._on_log_start)
        self.btn_log_stop.clicked.connect(self._on_log_stop)
        btn_clear.clicked.connect(lambda: self.log_text.clear())

        btn_row.addStretch(1)
        btn_row.addWidget(self.btn_log_start)
        btn_row.addWidget(self.btn_log_stop)
        btn_row.addWidget(btn_clear)

        lbox.addWidget(self.log_text)
        lbox.addLayout(btn_row)

        rlyt.addWidget(log_box)
        rlyt.addStretch(1)

        splitter.addWidget(right)
        splitter.setStretchFactor(0, 3)  # 左侧更宽
        splitter.setStretchFactor(1, 2)  # 右侧次之

        self.setCentralWidget(main)

    # ---------- 日志到桌面 ----------
    def _desktop_path(self) -> str:
        return os.path.expanduser("~/Desktop")

    def _on_log_start(self):
        try:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            name = f"WiFiBLE_Log_{ts}.txt"
            path = os.path.join(self._desktop_path(), name)
            self._log_fp = open(path, "w", encoding="utf-8")
            self._log_path = path
            self._is_logging = True
            self.btn_log_start.setEnabled(False)
            self.btn_log_stop.setEnabled(True)
            self._log(f"[LOG] Start recording -> {path}")
        except Exception as e:
            QMessageBox.critical(self, "Log Error", f"Failed to start logging: {e}")

    def _on_log_stop(self):
        if self._is_logging:
            try:
                self._log("[LOG] Stop recording.")
                if self._log_fp:
                    self._log_fp.flush()
                    self._log_fp.close()
            except Exception:
                pass
            finally:
                self._is_logging = False
                self._log_fp = None
                self.btn_log_start.setEnabled(True)
                self.btn_log_stop.setEnabled(False)
                if self._log_path:
                    QMessageBox.information(self, "Log saved", f"Saved to:\n{self._log_path}")

    # ---------- BLE 启动 ----------
    def _start_ble_auto(self):
        if not BLE_AVAILABLE:
            QMessageBox.critical(self, "BLE Missing", "Bleak not available. Please install bleak.")
            return
        self.ble_thread = QThread()
        self.ble_worker = BLEWorker()
        self.ble_worker.moveToThread(self.ble_thread)
        self.ble_thread.started.connect(self.ble_worker.start)
        self.ble_worker.log.connect(self._log)
        self.ble_worker.frame.connect(self._on_ble_frame)
        self.ble_worker.connected.connect(self._on_ble_connected)
        self.ble_worker.disconnected.connect(self._on_ble_disconnected)
        self.ble_worker.rssi.connect(lambda v: self.lbl_rssi.setText(f"RSSI: {v} dBm"))
        self.ble_thread.start()

    # ================= 事件 =================
    def _log(self, s: str):
        # 先写 GUI
        self.log_text.appendPlainText(s)
        # 若开启文件日志，写入带时间戳行
        if self._is_logging and self._log_fp:
            try:
                t = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                self._log_fp.write(f"[{t}] {s}\n")
                self._log_fp.flush()
            except Exception:
                pass

    def _on_ble_connect_clicked(self):
        if self.ble_worker and not self.ble_worker.is_connected:
            self._log("[BLE] Manual scan requested.")
            self.ble_worker.poke()

    def _on_txp_toggled(self, checked: bool):
        self.btn_txp.setText("TXP control ON" if checked else "TXP control OFF")
        if self.ble_worker:
            self.ble_worker.set_txp_control_enabled(checked)

    def _on_ble_send_cmd(self):
        txt = self.ble_cmd_edit.text().strip()
        if not txt:
            return
        try:
            payload = self._parse_hex_bytes(txt)
        except ValueError as e:
            QMessageBox.critical(self, "Invalid hex", str(e))
            return
        if self.ble_worker:
            self.ble_worker.send_raw(payload)
            self._log(f"[GUI] BLE TX: {payload.hex(' ')}")

    def _on_ble_connected(self):
        self._log("[BLE] Connected.")
        self.btn_ble_connect.setText("Connected")
        self.btn_ble_connect.setEnabled(False)
        self.btn_txp.setEnabled(True)
        self.btn_txp.setChecked(True)

    def _on_ble_disconnected(self):
        self._log("[BLE] Disconnected.")
        self.btn_ble_connect.setText("BLE Connect")
        self.btn_ble_connect.setEnabled(True)
        self.btn_txp.blockSignals(True)
        self.btn_txp.setChecked(False)
        self.btn_txp.setText("TXP control OFF")
        self.btn_txp.setEnabled(False)
        self.btn_txp.blockSignals(False)
        self.lbl_rssi.setText("RSSI: — dBm")

    def _on_ble_frame(self, jpeg_bytes: bytes):
        try:
            pm = QPixmap()
            if pm.loadFromData(jpeg_bytes):
                self.video_label.setPixmap(pm)
            else:
                with open("img.jpg", "wb") as f:
                    f.write(jpeg_bytes)
                self.video_label.setPixmap(QPixmap("img.jpg"))
        except Exception as e:
            self._log(f"[BLE][Frame][ERR] {e}")

    def _on_wifi_connect_clicked(self):
        ip, port = self.target_server
        try:
            if self.client is not None:
                self.client.stop_signal.emit()
                self.client.stop()
                self.socket_thread.quit()
                self.socket_thread.wait()
        except Exception:
            pass

        self.socket_thread = QThread()
        self.client = SocketClient(ip, port, local_port=60000)
        self.client.moveToThread(self.socket_thread)
        self.socket_thread.started.connect(self.client.run)
        self.client.command_receiving_signal.connect(self._on_udp_frame)
        self.socket_thread.start()
        self._log(f"[Wi-Fi] Listening UDP at 0.0.0.0:60000; target={ip}:{port}")
        try:
            self.client.send_command("55 0F AA")
        except Exception:
            pass

    def _on_wifi_disconnect_clicked(self):
        try:
            if self.client:
                self.client.command_receiving_signal.disconnect(self._on_udp_frame)
        except Exception:
            pass
        try:
            if self.client:
                self.client.stop_signal.emit()
                self.client.stop()
            if self.socket_thread:
                self.socket_thread.quit()
                self.socket_thread.wait()
        except Exception:
            pass
        self.client = None
        self.socket_thread = None
        self._log("[Wi-Fi] Disconnected.")

    def _on_send_bbox_clicked(self):
        try:
            x = float(self.edit_bbox_x.text().strip())
            y = float(self.edit_bbox_y.text().strip())
        except Exception:
            QMessageBox.critical(self, "BBox Error", "请输入有效的浮点数（X 与 Y）。")
            return

        if not self.ble_worker or not self.ble_worker.is_connected:
            QMessageBox.critical(self, "BLE not connected", "请先连接 BLE 再发送 BBox。")
            return

        self.ble_worker.send_bbox(x, y)
        self._log(f"[GUI] Request send BBox: ({x:.2f}, {y:.2f})")

    def _on_udp_frame(self, full_frame: bytes):
        msg = self.camera.process_command(full_frame)
        self._log(msg)
        if msg.startswith("FrameSize"):
            if os.path.exists("img.jpg"):
                self.video_label.setPixmap(QPixmap("img.jpg"))

    def _update_target_server(self, text):
        if ':' in text:
            ip, port = text.split(':', 1)
            try:
                self.target_server = (ip.strip(), int(port.strip()))
            except Exception:
                pass

    @staticmethod
    def _parse_hex_bytes(s: str) -> bytes:
        tokens = (
            s.replace(",", " ")
             .replace("\n", " ")
             .replace("\t", " ")
             .split()
        )
        if not tokens:
            raise ValueError("Empty command.")
        out = bytearray()
        for tok in tokens:
            t = tok.lower()
            if t.startswith("0x"):
                t = t[2:]
            if not t:
                raise ValueError(f"Bad token: '{tok}'")
            try:
                v = int(t, 16)
            except Exception:
                raise ValueError(f"Bad hex token: '{tok}'")
            if not (0 <= v <= 255):
                raise ValueError(f"Byte out of range (0-255): '{tok}'")
            out.append(v)
        return bytes(out)

    def closeEvent(self, event):
        # 停止文件日志
        try:
            if self._is_logging:
                self._on_log_stop()
        except Exception:
            pass
        # UDP
        try:
            if self.client:
                self.client.stop_signal.emit()
                self.client.stop()
            if self.socket_thread:
                self.socket_thread.quit()
                self.socket_thread.wait()
        except Exception:
            pass
        # BLE
        try:
            if self.ble_worker:
                self.ble_worker.stop()
            if self.ble_thread:
                self.ble_thread.quit()
                self.ble_thread.wait()
        except Exception:
            pass
        event.accept()

# ================= main =================
if __name__ == "__main__":
    app = QApplication(sys.argv)
    # 更紧凑全局字体/间距（可按需调整）
    font = app.font()
    font.setPointSize(font.pointSize() - 1)
    app.setFont(font)

    w = WifiCamHostGUI()
    w.show()
    sys.exit(app.exec())
 