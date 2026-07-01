import socket
import time
import os

CAMERA_IP   = "172.20.10.14"
CAMERA_PORT = 60010      # 摄像头 UDP 端口

RECV_SIZE = 64 * 1024    # 一次最多收 64KB；UDP 单包最大 65535
RCVBUF    = 2 * 1024 * 1024

SAVE_DIR = os.path.expanduser("~/Desktop/zzy")
os.makedirs(SAVE_DIR, exist_ok=True)

BIN_PATH = os.path.join(
    SAVE_DIR,
    f"udp_rx_stream_{time.strftime('%Y%m%d_%H%M%S')}.bin"
)

def send_start_stream_udp(sock):
    """
    UDP 推流命令
    """
    cmd = bytes([0x55, 0x02, 0x0A, 0xAA])
    sock.sendto(cmd, (CAMERA_IP, CAMERA_PORT))
    print("▶ UDP Start stream sent")


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # 设置更大的接收缓冲区
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, RCVBUF)

    # 绑定本机的任意端口用于接收摄像头流
    sock.bind(("0.0.0.0", CAMERA_PORT))
    print(f"📡 Listening on UDP port {CAMERA_PORT}")

    # 发送启动视频命令
    send_start_stream_udp(sock)

    print(f"📦 Saving UDP raw stream to:\n{BIN_PATH}")

    total = 0
    with open(BIN_PATH, "wb") as f:
        try:
            while True:
                data, addr = sock.recvfrom(RECV_SIZE)
                if not data:
                    print("⚠️ Received empty UDP packet")
                    continue

                f.write(data)
                total += len(data)

                print(f"⬇️ RX {len(data):6d} bytes | total = {total} | from {addr}")

        except KeyboardInterrupt:
            print("\n⏹ Interrupted by user (Ctrl+C)")

        except Exception as e:
            print(f"\n❌ UDP error: {e}")

    sock.close()
    print("✅ Socket closed")


if __name__ == "__main__":
    main()
