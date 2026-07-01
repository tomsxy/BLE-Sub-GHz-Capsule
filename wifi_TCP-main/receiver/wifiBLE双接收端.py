#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import socket
import configparser
import time
import os
import asyncio
import signal
from io import BytesIO
from typing import Optional

from PIL import Image
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

# ================= BLE 协议常量（与你之前脚本一致） =================
try:
    from bleak import BleakScanner, BleakClient
    BLE_AVAILABLE = True
except Exception:
    BLE_AVAILABLE = False

SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca3e"
RX_CHAR_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca3e"      # 写入控制（0x02 开流，0x03 停流等）
TX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca3e"      # 图像分片通知
IMG_INFO_CHAR_UUID = "6e400004-b5a3-f393-e0a9-e50e24dcca3e" # 帧长度信息（可选）

ITS_CMD_DEBUG = 0xF0  # 可选调试命令（心跳）

# ================= 你原有的命令封装 =================
class CommandsToCamera:
    START_CHARACTER = 0x55
    STOP_CHARACTER = 0xAA

    COMMANDS = {
        0xFF: {"name": "RESET_CAMERA", "description": "Reset camera", "parameters": {}},
        0x01: {"name": "SET_PICTURE_RESOLUTION", "description": "Set picture resolution", "parameters": {"format": None, "resolution": None}},
        0x02: {"name": "SET_VIDEO_RESOLUTION", "description": "Set video resolution", "parameters": {"resolution": None}},
        0x03: {"name": "SET_BRIGHTNESS", "description": "Set brightness", "parameters": {"brightness": None}},
        0x04: {"name": "SET_CONTRAST", "description": "Set contrast", "parameters": {"contrast": None}},
        0x05: {"name": "SET_SATURATION", "description": "Set saturation", "parameters": {"saturation": None}},
        0x06: {"name": "SET_EV", "description": "Set EV", "parameters": {"ev": None}},
        0x07: {"name": "SET_WHITEBALANCE", "description": "Set white balance", "parameters": {"white_balance": None}},
        0x08: {"name": "SET_SPECIAL_EFFECTS", "description": "Set special effects", "parameters": {"effect": None}},
        0x09: {"name": "SET_FOCUS_ENABLE", "description": "Set focus enable", "parameters": {"focus_control": None}},
        0x0A: {"name": "SET_EXPOSURE_GAIN_ENABLE", "description": "Set exposure gain enable", "parameters": {"exposure_control": None}},
        0x0C: {"name": "SET_WHITE_BALANCE_ENABLE", "description": "Set white balance enable", "parameters": {"white_balance_control": None}},
        0x0D: {"name": "SET_MANUAL_GAIN", "description": "Set manual gain", "parameters": {"manual_gain": None}},
        0x0E: {"name": "SET_MANUAL_EXPOSURE", "description": "Set manual exposure", "parameters": {"manual_exposure": None}},
        0x0F: {"name": "GET_CAMERA_INFO", "description": "Get camera info", "parameters": {}},
        0x10: {"name": "TAKE_PICTURE", "description": "Take picture", "parameters": {}},
        0x11: {"name": "SET_SHARPNESS", "description": "Set sharpness", "parameters": {"sharpness": None}},
        0x12: {"name": "DEBUG_WRITE_REGISTER", "description": "Debug write register", "parameters": {"register": None, "value": None}},
        0x21: {"name": "STOP_STREAM", "description": "Stop stream", "parameters": {}},
        0x30: {"name": "GET_FRM_VER_INFO", "description": "Get frame version info", "parameters": {}},
        0x40: {"name": "GET_SDK_VER_INFO", "description": "Get SDK version info", "parameters": {}},
        0x50: {"name": "SET_IMAGE_QUALITY", "description": "Set image quality", "parameters": {"quality": None}},
        0x60: {"name": "SET_LOWPOWER_MODE", "description": "Set low power mode", "parameters": {"mode": None}},
    }

    @staticmethod
    def command_get_camera_info():
        command_id = 0x0F
        command_bytes = bytes([CommandsToCamera.START_CHARACTER, command_id, CommandsToCamera.STOP_CHARACTER])
        return " ".join(format(b, "02X") for b in command_bytes)

    @staticmethod
    def command_take_picture():
        command_id = 0x10
        command_bytes = bytes([CommandsToCamera.START_CHARACTER, command_id, CommandsToCamera.STOP_CHARACTER])
        return " ".join(format(b, "02X") for b in command_bytes)

    @staticmethod
    def command_stop_stream():
        command_id = 0x21
        command_bytes = bytes([CommandsToCamera.START_CHARACTER, command_id, CommandsToCamera.STOP_CHARACTER])
        return " ".join(format(b, "02X") for b in command_bytes)

    @staticmethod
    def command_set_picture_resolution(format_code, resolution):
        command_id = 0x01
        if format_code not in (1, 2, 3):
            raise ValueError("Invalid format code. It must be 1, 2, or 3.")
        if resolution < 0 or resolution > 13:
            raise ValueError("Invalid resolution code. It must be between 0 and 13.")
        parameter_byte = (format_code << 4) | resolution
        command_bytes = bytes([CommandsToCamera.START_CHARACTER, command_id, parameter_byte, CommandsToCamera.STOP_CHARACTER])
        return " ".join(format(b, "02X") for b in command_bytes)

    @staticmethod
    def command_start_streaming_mode(resolution):
        command_id = 0x02
        command_bytes = bytes([CommandsToCamera.START_CHARACTER, command_id, resolution, CommandsToCamera.STOP_CHARACTER])
        return " ".join(format(b, "02X") for b in command_bytes)

# ================= 你原有的 UDP 客户端（基本不变） =================
class SocketClient(QObject):
    command_receiving_signal = pyqtSignal(bytes)
    num_signal = pyqtSignal(int)
    stop_signal = pyqtSignal()

    def __init__(self, cam_address='192.168.1.1', cam_port=60010):
        super().__init__()
        self.cam_address = cam_address
        self.cam_port = cam_port
        self.buffer_size = 4096
        self.pc_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.pc_socket.bind(('0.0.0.0', 60000))
        self.pc_socket.connect((cam_address, cam_port))
        print("Connected to the Camera (UDP).")
        self.command_buffer = b''
        self.in_command = False
        self.running = True
        self.stop_signal.connect(self.stop)

    def send_command(self, command_str):
        try:
            command_bytes = bytes.fromhex(command_str)
            self.pc_socket.send(command_bytes)
        except Exception:
            QMessageBox.critical(None, "Error", "Fail to send command (UDP)")

    def run(self):
        try:
            print("SocketClient run")
            bytesframe = 0

            while self.running:
                data, _ = self.pc_socket.recvfrom(self.buffer_size)

                # --------- 新帧开始 ---------
                if data.startswith(b'\xFF\xAA'):
                    self.command_buffer = b''
                    self.in_command = True
                    self.command_buffer += data
                    bytesframe = 0

                # --------- 正在接收帧 ---------
                elif self.in_command:
                    self.command_buffer += data

                    # 如果帧长度还没取出，但头部已收齐，就解析 payload_len
                    if bytesframe == 0 and len(self.command_buffer) >= 7:
                        try:
                            bytesframe = int.from_bytes(
                                self.command_buffer[3:7],
                                byteorder='little',
                                signed=False
                            )
                            print(f"[UDP] Expect frame size: {bytesframe} bytes")
                        except Exception:
                            bytesframe = 0

                    # 如果有尾标记 FF BB 且长度满足，才认为是完整帧
                    if bytesframe > 0:
                        need_len = 7 + bytesframe + 2
                        if len(self.command_buffer) >= need_len and self.command_buffer.endswith(b'\xFF\xBB'):
                            frame = self.command_buffer[:need_len]
                            self.command_receiving_signal.emit(frame)

                            # 重置，准备下一帧
                            self.command_buffer = b''
                            self.in_command = False
                            bytesframe = 0

                # --------- 不在收帧状态 ---------
                else:
                    bytesframe = 0
                    self.command_buffer = b''
                    self.in_command = False

        except Exception:
            if self.running:
                QMessageBox.critical(None, "Error", "Fail to receive packets (UDP)")

    def stop(self):
        self.running = False
        try:
            self.pc_socket.close()
        except Exception:
            pass

# ================= 图像/信息解析（沿用你原来的类） =================
class ArducamMegaCameraDataProcess:
    is_res_updated = False


    def __init__(self):
        self.last_frame_time = None
        # === 新增：统计用 ===
        self._bytes_accum = 0  # 一秒内累计字节
        self._frames_accum = 0  # 一秒内累计帧数
        self._t_start = time.time()  # 窗口起点

    def process_command(self, command):
        start_marker = command[0:2]
        command_type = command[2:3]

        if start_marker == b'\xFF\xAA':
            if command_type == b'\x01':
                bytesframe = int.from_bytes(command[3:7], byteorder='little')
                frame = command[7:7+bytesframe]
                return self.process_capture_command(bytesframe, frame)
            elif command_type == b'\x02':
                payload_length = int.from_bytes(command[3:7], byteorder='little')
                payload = command[7:7+payload_length]
                return self.process_info_command(payload_length, payload)
            elif command_type == b'\x03':
                return self.process_version_command(command[3:])  # 这里按你接口占位
            elif command_type == b'\x05':
                return self.process_version2_command(command[3:])
            elif command_type == b'\x06':
                return self.process_streamoff_command(command[3:])
            elif command_type == b'\x07':
                new_resolution = command[7]
                return self.process_resolution_change_notification(new_resolution)
            elif command_type == b'\x08':
                return self.process_ble_connection_established_command()
            elif command_type == b'\x09':
                return self.process_ble_disconnect_command()
            else:
                return "Unknown command type"
        elif start_marker == b'\xFF\xBB':
            return self.process_stop_command(command_type, 0, b'')
        else:
            return "Invalid start marker"

    def process_capture_command(self, bytesframe, frame):
        # 先保存图片（保持你原来的行为）
        try:
            img = Image.open(BytesIO(frame))
            img.save("img.jpg", "JPEG")
        except Exception:
            pass

        # 累加到1秒窗口
        self._bytes_accum += bytesframe
        self._frames_accum += 1

        now = time.time()
        elapsed = now - self._t_start

        # 基础输出：每帧都报 FrameSize
        msg = f"FrameSize: {bytesframe} Bytes"

        # 满1秒时，再追加一次统计并重置窗口
        if elapsed >= 1.0:
            fps = self._frames_accum / elapsed
            kbps = (self._bytes_accum * 8) / 1024.0 / elapsed
            msg += f" | Frames-per-second: {fps:.2f} | Throughput: {kbps:.2f} kbps"
            # 重置窗口到当前时刻与计数
            self._t_start = now
            self._bytes_accum = 0
            self._frames_accum = 0

        # 不要额外换行，避免空行堆积；GUI每次 append 一行
        return msg

    def process_info_command(self, payload_length, payload):
        info_payload = payload.decode('utf-8', errors='ignore')
        if not self.is_res_updated:
            if "Camera Type:3MP" in info_payload:
                self.update_resolution_options(remove_option="2592x1944")
            elif "Camera Type:5MP" in info_payload:
                self.update_resolution_options(remove_option="2048x1536")
        return f"Connected to WiFi Camera!\n{info_payload}"

    def update_resolution_options(self, remove_option):
        mw = QApplication.instance().topLevelWidgets()[0]
        for k, v in list(mw.IMAGE_RESOLUTION_OPTIONS.items()):
            if v == remove_option:
                del mw.IMAGE_RESOLUTION_OPTIONS[k]
                break
        image_resolution_options = list(mw.IMAGE_RESOLUTION_OPTIONS.items())
        last_item = image_resolution_options[-1][1]
        mw.image_resolution_combobox.addItem(last_item)
        self.is_res_updated = True

    def process_version_command(self, payload):
        if len(payload) >= 4:
            year = payload[0]
            month = payload[1]
            day = payload[2]
            version = f"{(payload[3] >> 4)}.{payload[3] & 0x0F}"
            return f"Version: {year}/{month}/{day}, {version}"
        return "Version command received."

    def process_version2_command(self, payload):
        return self.process_version_command(payload)

    def process_streamoff_command(self, payload):
        return "Streamoff command received"

    def process_resolution_change_notification(self, new_resolution):
        return f"New Resolution from BLE Client:{new_resolution}"

    def process_stop_command(self, command_type, payload_length, payload):
        return "Stop command received."

    def process_ble_connection_established_command(self):
        return "Camera establish connection with BLE client!"

    def process_ble_disconnect_command(self):
        return "Camera disconnect with BLE client!"

# ================= BLE Worker（新） =================
class BLEWorker(QObject):
    log = pyqtSignal(str)
    frame = pyqtSignal(bytes, int, float, float)  # 旧信号先保留，fps/kbps 不再使用
    stats = pyqtSignal(float, float)              # <-- 新增：fps, kbps（按 1 秒窗口）
    connected = pyqtSignal()
    disconnected = pyqtSignal()


    def __init__(self, parent=None):
        super().__init__(parent)
        self._loop = None
        self._task = None
        self._stop_evt = asyncio.Event() if asyncio.get_event_loop_policy()._local.__dict__.get('loop') else None
        self._client = None

        # 组帧状态
        self._frame_expected = 0
        self._frame_buf = bytearray()
        self._write_pos = 0
        self._last_frame_ts = 0.0

        # 统计
        self._t_window = 0.0
        self._bytes_in_window = 0
        self._frames_in_window = 0
        self._PRINT_EVERY = 1.0

    # ============ 公共 API：供 GUI 调用 ============
    @pyqtSlot()
    def start(self):
        # 在线程里创建事件循环
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._stop_evt = asyncio.Event()
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
            except Exception:
                pass
            self.disconnected.emit()

    @pyqtSlot()
    def stop(self):
        if self._loop and not self._loop.is_closed():
            self._loop.call_soon_threadsafe(self._stop_evt.set)

    @pyqtSlot()
    def start_stream(self):
        if self._loop and self._client and self._client.is_connected:
            self._loop.call_soon_threadsafe(lambda: asyncio.create_task(self._client.write_gatt_char(RX_CHAR_UUID, bytes([0x02]), response=True)))

    @pyqtSlot()
    def stop_stream(self):
        if self._loop and self._client and self._client.is_connected:
            self._loop.call_soon_threadsafe(lambda: asyncio.create_task(self._client.write_gatt_char(RX_CHAR_UUID, bytes([0x03]), response=True)))

    @pyqtSlot(int)
    def set_resolution(self, res_code: int):
        # 如果你的固件支持把分辨率作为开流第二字节，可改为 bytes([0x02, res_code])
        # 这里仅记录日志；实际可根据设备协议扩展
        self.log.emit(f"[BLE] Set resolution request: {res_code} (请在固件支持时映射到 RX 命令)")

    @pyqtSlot()
    def take_picture(self):
        # 若固件支持拍照命令，发对应字节；此处示例用 debug 命令占位
        if self._loop and self._client and self._client.is_connected:
            self._loop.call_soon_threadsafe(lambda: asyncio.create_task(self._client.write_gatt_char(RX_CHAR_UUID, bytes([ITS_CMD_DEBUG, 0x10]), response=True)))

    # ============ 内部：BLE 主流程 ============
    async def _main(self):
        self.log.emit("[BLE] Scanning...")
        addr = await self._find_addr(8.0)
        if not addr:
            self.log.emit("[BLE][ERR] No target device advertising the service.")
            return

        async with BleakClient(addr) as client:
            self._client = client
            if not client.is_connected:
                self.log.emit("[BLE][ERR] Connect failed.")
                return
            self.connected.emit()
            self.log.emit(f"[BLE] Connected: {addr}")

            await client.start_notify(IMG_INFO_CHAR_UUID, self._img_info_handler)
            await client.start_notify(TX_CHAR_UUID, self._tx_handler)
            await asyncio.sleep(0.05)

            # 默认上来不开流，等 GUI 按钮触发；也可以在此处自动开流
            # await client.write_gatt_char(RX_CHAR_UUID, bytes([0x02]), response=True)

            self._t_window = time.monotonic()
            while not self._stop_evt.is_set():
                await asyncio.sleep(0.05)
                now = time.monotonic()
                if (now - self._t_window) >= self._PRINT_EVERY:
                    seconds = (now - self._t_window)
                    kbps = (self._bytes_in_window * 8) / seconds / 1000.0
                    fps = self._frames_in_window / seconds

                    # 发“每秒统计”信号
                    self.stats.emit(fps, kbps)

                    # 可选：也写一行简单日志（不会很频繁）
                    self.log.emit(f"[BLE 1s] {kbps:7.2f} kbps, {fps:5.2f} fps")

                    # 重置窗口
                    self._bytes_in_window = 0
                    self._frames_in_window = 0
                    self._t_window = now

            # 收到 stop 后尝试停流
            try:
                await client.write_gatt_char(RX_CHAR_UUID, bytes([0x03]), response=True)
            except Exception:
                pass

            try:
                await client.stop_notify(TX_CHAR_UUID)
            except Exception:
                pass
            try:
                await client.stop_notify(IMG_INFO_CHAR_UUID)
            except Exception:
                pass

    async def _find_addr(self, timeout: float = 8.0) -> Optional[str]:
        dev = await BleakScanner.find_device_by_filter(
            lambda d, ad: any((u or "").lower() == SERVICE_UUID.lower() for u in (ad.service_uuids or [])),
            timeout=timeout,
        )
        return dev.address if dev else None

    # ============ 组帧相关 ============
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
        self._last_frame_ts = time.monotonic()
        if size > 0:
            self._frame_buf = bytearray(size)
        else:
            self._frame_buf = bytearray()

    def _complete_frame(self, buf: bytes):
        # 仅计数；fps/kbps 的 1 秒统计在 _main() 里做
        self._frames_in_window += 1

        # 仍然立即把帧发给 GUI 显示（fps/kbps 这里传 0，不在 GUI 使用）
        size = len(buf)
        self.frame.emit(buf, size, 0.0, 0.0)

        # 复位帧状态
        self._frame_expected = 0
        self._frame_buf = bytearray()
        self._write_pos = 0
        # 注：不要更新 _last_frame_ts，这里不再做 per-frame fps

    def _tx_handler(self, _sender, data: bytearray):
        chunk = bytes(data)
        self._bytes_in_window += len(chunk)

        # 如果 IMG_INFO 尚未到达
        if self._frame_buf is None:
            self._frame_buf = bytearray()
            if self._last_frame_ts == 0:
                self._last_frame_ts = time.monotonic()

        if self._frame_expected > 0:
            # 长度模式：循环消费（处理“同一通知跨多帧”）
            view = memoryview(chunk)
            while len(view):
                remain = self._frame_expected - self._write_pos
                n = min(remain, len(view))
                if n > 0:
                    self._frame_buf[self._write_pos:self._write_pos+n] = view[:n]
                    self._write_pos += n
                    view = view[n:]
                if self._write_pos >= self._frame_expected:
                    self._complete_frame(bytes(self._frame_buf[:self._write_pos]))
                    # 若 view 里还剩下数据，可能是下一帧的开始（等待下一次 IMG_INFO 或 EOI）
                    if len(view):
                        # 没有新长度信息时，切换到 EOI 模式缓存残片
                        self._frame_buf = bytearray()
                        self._frame_buf.extend(view)
                        # 在 EOI 模式里继续找 FF D9（一次可能完成多帧）
                        self._scan_and_split_eoi()
                        break
        else:
            # EOI 模式：可能一次通知里包含多帧尾/头，循环查找 FF D9
            self._frame_buf.extend(chunk)
            self._scan_and_split_eoi()

    def _scan_and_split_eoi(self):
        # 为了不漏检跨边界，从 max(0, len-最近块长度-1) 开始，但这里已统一放在累积后扫描
        i = 0
        while True:
            j = self._frame_buf.find(b"\xFF\xD9", i)
            if j == -1:
                break
            frame_bytes = bytes(self._frame_buf[:j+2])
            self._complete_frame(frame_bytes)
            # 残留给下一帧
            self._frame_buf = self._frame_buf[j+2:]
            i = 0

# ================= GUI =================
class WifiCamHostGUI(QMainWindow):
    IMAGE_RESOLUTION_OPTIONS = {
        10: "96x96",
        1: "320x240",
        2: "640x480",
        4: "1280x720",
        8: "2048x1536",
        9: "2592x1944"
    }

    IMAGE_FORMAT_OPTIONS = {
        1: "JPEG",
        2: "RGB",
        3: "YUV",
    }

    def __init__(self):
        super().__init__()
        self.resize(1200, 900)
        self.setWindowTitle("WiFi/BLE Camera Host")
        self.setWindowIcon(QIcon("icon.png"))
        self.commands = CommandsToCamera()

        self.config = configparser.ConfigParser()
        self.config.read('config.ini')
        target_ip = self.config.get('DEFAULT', 'TargetIP', fallback='192.168.1.100')
        target_port = int(self.config.get('DEFAULT', 'TargetPort', fallback='60010'))
        self.target_server = (target_ip, target_port)

        # 运行通道：'UDP' or 'BLE'
        self.transport_mode = 'UDP'

        # 线程对象
        self.client = None              # UDP 客户端
        self.socket_thread = None
        self.ble_thread = None          # BLE 线程
        self.ble_worker = None

        self.camera = ArducamMegaCameraDataProcess()
        self.log_content_text = QTextEdit()
        self.create_layout()

    def create_layout(self):
        main_widget = QWidget()
        layout = QVBoxLayout()
        splitter = QSplitter(Qt.Horizontal)

        # 左：视频
        self.video_frame_widget = QWidget()
        self.video_frame_widget.setFixedSize(1600, 1200)
        self.video_frame_widget.setStyleSheet("background-color: gray;")
        video_frame_layout = QVBoxLayout()
        self.video_frame_label = QLabel("Video/Image will show here!")
        self.video_frame_label.setScaledContents(True)
        video_frame_layout.addWidget(self.video_frame_label)
        self.video_frame_widget.setLayout(video_frame_layout)
        splitter.addWidget(self.video_frame_widget)

        # 右：连接/控制/日志
        right_side_widget = QWidget()
        right_side_layout = QVBoxLayout()
        self.connect_window(right_side_layout)
        self.capture_window(right_side_layout)
        self.log_window(right_side_layout)
        right_side_widget.setLayout(right_side_layout)
        splitter.addWidget(right_side_widget)

        layout.addWidget(splitter)
        main_widget.setLayout(layout)
        self.setCentralWidget(main_widget)

    def connect_window(self, layout):
        add_widget = QWidget()
        layout.addWidget(add_widget)

        layout_add = QVBoxLayout()
        add_widget.setLayout(layout_add)

        label_add = QLabel("Target WiFi Camera Address")
        layout_add.addWidget(label_add)

        self.target_server_entry = QLineEdit(f"{self.target_server[0]}:{self.target_server[1]}")
        self.target_server_entry.textChanged.connect(self.update_target_server)
        layout_add.addWidget(self.target_server_entry)

        # 仅此处新增：传输方式选择（最小改动）
        self.transport_combo = QComboBox()
        self.transport_combo.addItems(["UDP (Wi-Fi)", "BLE"])
        self.transport_combo.currentIndexChanged.connect(self._on_transport_changed)
        layout_add.addWidget(self.transport_combo)

        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.handle_connect_button)
        layout_add.addWidget(self.connect_button)

    def _on_transport_changed(self, idx):
        self.transport_mode = "UDP" if idx == 0 else "BLE"
        self.log_content_text.append(f"Transport set to: {self.transport_mode}")
        # BLE 模式下 IP:port 可忽略（保持 UI 一致，不改布局）

    def capture_window(self, layout):
        capture_tab = QWidget()
        capture_layout = QVBoxLayout()

        image_resolution_layout = QHBoxLayout()
        image_resolution_label = QLabel("Image Res:")
        image_resolution_layout.addWidget(image_resolution_label)
        self.image_resolution_combobox = QComboBox()
        image_resolution_options = list(self.IMAGE_RESOLUTION_OPTIONS.items())
        # 为了可视，先不加入最后两项的做法去掉，直接全部加入（也可按你习惯保留原逻辑）
        self.image_resolution_combobox.addItems([size for _, size in image_resolution_options])
        self.image_resolution_combobox.setEnabled(False)
        self.image_resolution_combobox.currentIndexChanged.connect(self.handle_resolution_change)
        image_resolution_layout.addWidget(self.image_resolution_combobox)
        capture_layout.addLayout(image_resolution_layout)

        self.stream_button = QPushButton("Start Stream")
        self.stream_button.setEnabled(False)
        self.stream_button.clicked.connect(self.toggle_stream_button)
        capture_layout.addWidget(self.stream_button)

        self.capture_image_button = QPushButton("Take Picture")
        self.capture_image_button.setEnabled(False)
        self.capture_image_button.clicked.connect(self.capture_image)
        capture_layout.addWidget(self.capture_image_button)

        capture_tab.setLayout(capture_layout)
        layout.addWidget(capture_tab)

    def toggle_stream_button(self):
        if self.stream_button.text() == "Start Stream":
            self.stream_button.setText("Stop Stream")
            self.capture_image_button.setEnabled(False)
            self.log_content_text.clear()

            # 分发到不同通道
            if self.transport_mode == "UDP":
                res_num = self._current_res_code()
                self.send_command(self.commands.command_start_streaming_mode(res_num))
            else:
                # BLE：发送 0x02 开流
                if self.ble_worker:
                    self.ble_worker.start_stream()
        else:
            self.stream_button.setText("Start Stream")
            self.capture_image_button.setEnabled(True)
            if self.transport_mode == "UDP":
                self.send_command(self.commands.command_stop_stream())
            else:
                if self.ble_worker:
                    self.ble_worker.stop_stream()

    def handle_connect_button(self):
        if self.connect_button.text() == "Connect":
            self.clear_log()
            self.stream_button.setText("Start Stream")
            if self.transport_mode == "UDP":
                self.log_content_text.append(f"Connecting (UDP) to {self.target_server[0]}:{self.target_server[1]} ...")
                # 关闭旧的
                if self.client is not None:
                    self.client.stop_signal.emit()
                    self.client.stop()
                    self.socket_thread.quit()
                    self.socket_thread.wait()
                # 新开
                self.socket_thread = QThread()
                self.client = SocketClient(*self.target_server)
                self.client.moveToThread(self.socket_thread)
                self.socket_thread.started.connect(self.client.run)
                self.client.command_receiving_signal.connect(self.process_received_packets_udp)
                self.socket_thread.start()
                # 主动获取相机信息（按你原来的）
                self.send_command(self.commands.command_get_camera_info())
                # UI enable
                self.capture_image_button.setEnabled(True)
                self.stream_button.setEnabled(True)
                self.image_resolution_combobox.setEnabled(True)
                self.connect_button.setText("Disconnect")
            else:
                if not BLE_AVAILABLE:
                    QMessageBox.critical(self, "Error", "Bleak not available. Please install bleak.")
                    return
                self.log_content_text.append(f"Connecting (BLE) scanning by service UUID ...")
                # 关闭旧的
                if self.ble_thread is not None:
                    try:
                        self.ble_worker.stop()
                    except Exception:
                        pass
                    self.ble_thread.quit()
                    self.ble_thread.wait()

                self.ble_thread = QThread()
                self.ble_worker = BLEWorker()
                self.ble_worker.moveToThread(self.ble_thread)
                # 信号连接
                self.ble_thread.started.connect(self.ble_worker.start)
                self.ble_worker.log.connect(self._append_log)
                self.ble_worker.frame.connect(self._on_ble_frame)
                self.ble_worker.connected.connect(self._on_ble_connected)
                self.ble_worker.disconnected.connect(self._on_ble_disconnected)
                self.ble_thread.start()
        else:
            # Disconnect
            if self.transport_mode == "UDP":
                self.send_command(self.commands.command_stop_stream())
                try:
                    self.client.command_receiving_signal.disconnect(self.process_received_packets_udp)
                except Exception:
                    pass
                if self.client:
                    self.client.stop_signal.emit()
                    self.client.stop()
                if self.socket_thread:
                    self.socket_thread.quit()
                    self.socket_thread.wait()
                self.client = None
                self.socket_thread = None
            else:
                if self.ble_worker:
                    self.ble_worker.stop()
                if self.ble_thread:
                    self.ble_thread.quit()
                    self.ble_thread.wait()
                self.ble_worker = None
                self.ble_thread = None

            self.connect_button.setText("Connect")
            self.stream_button.setEnabled(False)
            self.image_resolution_combobox.setEnabled(False)
            self.capture_image_button.setEnabled(False)
            self.log_content_text.append("Disconnected.")

    def send_command(self, command_str):
        hex_array = command_str.split()
        if len(hex_array) > 6:
            QMessageBox.critical(None, "Error", "Maximum length of command is 6 digits.")
            return
        try:
            bytes.fromhex("".join(hex_array))
        except ValueError:
            QMessageBox.critical(None, "Error", "Invalid hex format.")
            return

        if self.log_content_text:
            self.log_content_text.append(f"SENT: {command_str}")

        # 分发
        if self.transport_mode == "UDP":
            if self.client:
                self.client.send_command(command_str)
        else:
            # 如需把 FF AA 命令“适配”为 BLE，请在此做映射
            # 目前我们只在按钮层面对“开/停流/拍照/分辨率”发 BLE 命令
            if self.ble_worker:
                self._append_log("[BLE] Raw command via BLE is not mapped; use buttons to control.")

    def handle_resolution_change(self, index):
        res_num = self._current_res_code()
        if self.transport_mode == "UDP":
            self.send_command(self.commands.command_set_picture_resolution(1, res_num))
        else:
            if self.ble_worker:
                self.ble_worker.set_resolution(res_num)

    def log_window(self, layout):
        log_window = QGroupBox()
        log_window.setTitle("Log Content")
        log_window_layout = QVBoxLayout()
        log_window.setLayout(log_window_layout)

        log_content_layout = QVBoxLayout()
        log_window_layout.addLayout(log_content_layout)

        self.log_content_text = QTextEdit("Check WiFi Camera Device printout for the target address.\n Example: 192.168.1.100:60010")
        self.log_content_text.setReadOnly(True)
        log_content_layout.addWidget(self.log_content_text)

        layout.addWidget(log_window)
        clear_button = QPushButton("Clear Log")
        clear_button.clicked.connect(self.clear_log)
        layout.addWidget(clear_button)

    def clear_log(self):
        if self.log_content_text:
            self.log_content_text.clear()

    # ===== UDP 回调 =====
    def process_received_packets_udp(self, command_buffer):
        try:
            self.client.command_receiving_signal.disconnect(self.process_received_packets_udp)
        except Exception:
            pass

        result = self.camera.process_command(command_buffer)
        if self.log_content_text:
            self.log_content_text.append(result)

        if result.startswith("Connected to WiFi Camera!"):
            self.capture_image_button.setEnabled(True)
            self.stream_button.setEnabled(True)
            self.image_resolution_combobox.setEnabled(True)
            self.connect_button.setText("Disconnect")

        if result.startswith("FrameSize"):
            img_path = "img.jpg"
            if os.path.exists(img_path):
                pixmap = QPixmap(img_path)
                self.video_frame_label.setPixmap(pixmap)

        if result.startswith("New Resolution from BLE Client"):
            new_resolution = int(result.split(":")[-1].strip())
            if new_resolution in self.IMAGE_RESOLUTION_OPTIONS:
                self.image_resolution_combobox.setCurrentIndex(
                    list(self.IMAGE_RESOLUTION_OPTIONS.keys()).index(new_resolution)
                )

        try:
            self.client.command_receiving_signal.connect(self.process_received_packets_udp)
        except Exception:
            pass

    # ===== BLE 回调 =====
    def _append_log(self, text: str):
        if self.log_content_text:
            self.log_content_text.append(text)

    def _on_ble_connected(self):
        self._append_log("[BLE] Connected.")
        self.capture_image_button.setEnabled(True)
        self.stream_button.setEnabled(True)
        self.image_resolution_combobox.setEnabled(True)
        self.connect_button.setText("Disconnect")

    def _on_ble_disconnected(self):
        self._append_log("[BLE] Disconnected.")

    def _on_ble_frame(self, jpeg_bytes: bytes, size: int, fps: float, kbps: float):
        # 保存并显示（与 UDP 路径保持一致）
        try:
            with open("img.jpg", "wb") as f:
                f.write(jpeg_bytes)
        except Exception:
            pass
        img_path = "img.jpg"
        if os.path.exists(img_path):
            pixmap = QPixmap(img_path)
            self.video_frame_label.setPixmap(pixmap)

    def update_target_server(self, text):
        if ':' in text:
            ip, port = text.split(':')
            self.target_server = (ip.strip(), int(port.strip()))

    def _current_res_code(self) -> int:
        # 根据当前 combo 的显示文本反查 key
        text = self.image_resolution_combobox.currentText()
        for k, v in self.IMAGE_RESOLUTION_OPTIONS.items():
            if v == text:
                return k
        # 兜底
        return 1

    def capture_image(self):
        self.log_content_text.append(f"Sending Take Picture command...")
        if self.transport_mode == "UDP":
            self.send_command(self.commands.command_take_picture())
        else:
            if self.ble_worker:
                self.ble_worker.take_picture()
        self.log_content_text.append(f"Take Picture command sent!")

    def closeEvent(self, event):
        # 优先停流
        try:
            if self.transport_mode == "UDP":
                self.send_command(self.commands.command_stop_stream())
            else:
                if self.ble_worker:
                    self.ble_worker.stop_stream()
        except Exception:
            pass

        # UDP 清理
        try:
            if self.client:
                self.client.stop_signal.emit()
                self.client.stop()
            if self.socket_thread:
                self.socket_thread.quit()
                self.socket_thread.wait()
        except Exception:
            pass

        # BLE 清理
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
    window = WifiCamHostGUI()
    window.show()
    sys.exit(app.exec())
