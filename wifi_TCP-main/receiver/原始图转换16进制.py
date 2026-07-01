import os
import re

DESKTOP = os.path.join(os.path.expanduser("~"), "Desktop")

INPUT_H  = os.path.join(DESKTOP, "oldfixed_image.h")
OUTPUT_BIN = os.path.join(DESKTOP, "tx_image.bin")


def extract_image_h_to_bin(h_path, out_bin):
    if not os.path.exists(h_path):
        print(f"❌ 文件不存在: {h_path}")
        return

    with open(h_path, "r", encoding="utf-8", errors="ignore") as f:
        text = f.read()

    # 1️⃣ 去注释
    text = re.sub(r'//.*', '', text)
    text = re.sub(r'/\*.*?\*/', '', text, flags=re.S)

    data = []

    # 2️⃣ 0x?? 形式
    for h in re.findall(r'0x([0-9a-fA-F]{1,2})', text):
        data.append(int(h, 16))

    # 3️⃣ 移除 0x??
    text = re.sub(r'0x[0-9a-fA-F]{1,2}', '', text)

    # 4️⃣ 十进制 0–255
    for d in re.findall(r'\b(\d{1,3})\b', text):
        v = int(d)
        if 0 <= v <= 255:
            data.append(v)

    with open(out_bin, "wb") as f:
        f.write(bytes(data))

    print(f"✅ 提取 {len(data)} 字节 → {out_bin}")


if __name__ == "__main__":
    extract_image_h_to_bin(INPUT_H, OUTPUT_BIN)
