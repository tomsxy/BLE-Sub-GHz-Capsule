import socket
import time
import os
from io import BytesIO
from PIL import Image

# ================== 配置 ==================
CAMERA_IP   = "172.20.10.14"
CAMERA_PORT = 60010
RECV_SIZE   = 4096

START_MARK = b"\xFF\xAA"
END_MARK   = b"\xFF\xBB"

TYPE_JPEG_LEN = 0x02    # 长度通知帧
TYPE_VIDEO    = 0x01    # 兼容保留

JPEG_SOI = b"\xFF\xD8"
JPEG_EOI = b"\xFF\xD9"

MAX_JPEG_SIZE = 2_000_000   # 安全上限，防止炸内存

SAVE_DIR = os.path.expanduser("~/Desktop/zzy")
os.makedirs(SAVE_DIR, exist_ok=True)


def send_start_stream(sock):
    """
    发送启动视频流命令（与你 MCU 端一致）
    """
    cmd = bytes([0x55, 0x02, 0x0A, 0xAA])
    sock.sendall(cmd)
    print("▶ Start stream sent")


class TCPJPEGReceiver:
    """
    工程级 TCP JPEG Receiver
    协议：
        FF AA | 0x02 | plen=4 | jpeg_len(4B LE) | FF BB
        <jpeg payload bytes ... 共 jpeg_len 字节>
    """

    WAIT_START     = 0
    WAIT_LEN_FRAME = 1
    WAIT_JPEG      = 2

    def __init__(self):
        self.buf = bytearray()
        self.state = self.WAIT_START

        self.expect_len = 0
        self.jpeg_buf = bytearray()

        self.frame_id = 0
        self.last_t = None

    def feed(self, data: bytes):
        self.buf.extend(data)

        while True:
            # ==================================================
            # 等待 FF AA（唯一可信同步点）
            # ==================================================
            if self.state == self.WAIT_START:
                idx = self.buf.find(START_MARK)
                if idx < 0:
                    # 防止 buffer 无限制增长
                    if len(self.buf) > 4096:
                        del self.buf[:-2]
                    return

                # 丢弃 FF AA 之前的垃圾数据
                del self.buf[:idx]
                self.state = self.WAIT_LEN_FRAME

            # ==================================================
            # 解析长度通知帧
            # ==================================================
            elif self.state == self.WAIT_LEN_FRAME:
                # FF AA + type + plen + jpeg_len + FF BB = 13B
                if len(self.buf) < 13:
                    return

                if self.buf[0:2] != START_MARK:
                    # 理论不该发生，但一旦发生立刻重同步
                    del self.buf[0]
                    self.state = self.WAIT_START
                    continue

                ftype = self.buf[2]
                plen  = int.from_bytes(self.buf[3:7], "little")

                # 严格校验控制帧
                if ftype != TYPE_JPEG_LEN or plen != 4:
                    print("⚠️ Invalid control frame, resync")
                    del self.buf[2:]
                    self.state = self.WAIT_START
                    continue

                jpeg_len = int.from_bytes(self.buf[7:11], "little")

                if jpeg_len <= 0 or jpeg_len > MAX_JPEG_SIZE:
                    print(f"❌ Invalid JPEG length: {jpeg_len}, resync")
                    del self.buf[2:]
                    self.state = self.WAIT_START
                    continue

                # （可选）校验帧尾
                if self.buf[11:13] != END_MARK:
                    print("⚠️ Missing END_MARK in length frame, resync")
                    del self.buf[2:]
                    self.state = self.WAIT_START
                    continue

                # 吃掉整个长度通知帧
                del self.buf[:13]

                self.expect_len = jpeg_len
                self.jpeg_buf = bytearray()
                self.state = self.WAIT_JPEG

                print(f"📩 JPEG length notified: {jpeg_len} bytes")

            # ==================================================
            # 接收 JPEG payload
            # ==================================================
            elif self.state == self.WAIT_JPEG:
                need = self.expect_len - len(self.jpeg_buf)
                if need <= 0:
                    self._handle_jpeg(bytes(self.jpeg_buf))
                    self._reset()
                    continue

                if not self.buf:
                    return

                take = min(need, len(self.buf))
                self.jpeg_buf.extend(self.buf[:take])
                del self.buf[:take]

                if len(self.jpeg_buf) == self.expect_len:
                    self._handle_jpeg(bytes(self.jpeg_buf))
                    self._reset()

    def _reset(self):
        """
        强制回到同步起点
        """
        self.expect_len = 0
        self.jpeg_buf.clear()
        self.state = self.WAIT_START

    def _handle_jpeg(self, jpeg: bytes):
        """
        JPEG 帧处理
        """
        if not (jpeg.startswith(JPEG_SOI) and jpeg.endswith(JPEG_EOI)):
            print("❌ Bad JPEG markers, drop frame")
            return

        self.frame_id += 1
        now = time.time()
        fps = 0.0 if self.last_t is None else 1.0 / (now - self.last_t)
        self.last_t = now

        path = os.path.join(SAVE_DIR, f"frame_{self.frame_id:05d}.jpg")

        try:
            Image.open(BytesIO(jpeg)).save(path, "JPEG")
            print(f"✅ Frame {self.frame_id:05d} | {len(jpeg)} B | FPS={fps:.2f}")
        except Exception as e:
            print("❌ JPEG decode failed:", e)


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((CAMERA_IP, CAMERA_PORT))
    print("✅ Connected to camera")

    send_start_stream(sock)

    rx = TCPJPEGReceiver()

    try:
        while True:
            data = sock.recv(RECV_SIZE)
            if not data:
                print("⚠️ Socket closed")
                break
            rx.feed(data)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
