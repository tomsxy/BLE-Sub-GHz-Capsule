import socket
import time
import os

# 摄像头/发送端 IP 和端口（你要主动连过去）
CAMERA_IP   = "172.20.10.14"
CAMERA_PORT = 60010

RECV_SIZE = 64 * 1024          # 一次最多读 64KB
RCVBUF    = 2 * 1024 * 1024    # TCP 接收缓冲区 2MB

SAVE_DIR = os.path.expanduser("~/Desktop/zzy")
os.makedirs(SAVE_DIR, exist_ok=True)

BIN_PATH = os.path.join(
    SAVE_DIR,
    f"rx_stream_{time.strftime('%Y%m%d_%H%M%S')}.bin"
)

def send_start_stream(sock):
    """
    下行命令告诉推流端：开始发送数据
    """
    cmd = bytes([0x55, 0x02, 0x0A, 0xAA])
    sock.sendall(cmd)
    print("▶ Start stream sent")

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # 调大 OS TCP 接收缓冲区
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, RCVBUF)

    # 阻塞模式
    sock.settimeout(None)

    print(f"📡 Connecting to {CAMERA_IP}:{CAMERA_PORT} ...")
    sock.connect((CAMERA_IP, CAMERA_PORT))
    print("✅ Connected to sender")

    real_buf = sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
    print(f"📶 SO_RCVBUF = {real_buf} bytes")

    # 一定要放在 connect() 后执行
    send_start_stream(sock)

    print(f"📦 Saving TCP stream to:\n{BIN_PATH}")

    total = 0
    with open(BIN_PATH, "wb") as f:
        try:
            while True:
                data = sock.recv(RECV_SIZE)
                if not data:
                    print("⚠️ Sender closed connection")
                    break

                f.write(data)
                total += len(data)

                print(f"⬇️ RX {len(data):6d} bytes | total = {total}")

        except KeyboardInterrupt:
            print("\n⏹ Interrupted by user")
        except Exception as e:
            print(f"\n❌ Socket error: {e}")

    sock.close()
    print("🔚 Socket closed")

if __name__ == "__main__":
    main()
