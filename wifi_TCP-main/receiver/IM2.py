#!/usr/bin/env python3
import os
import re

IN_NAME = "1.jpg"  # 桌面上的源文件名
OUT_HEADER = "big_jpeg.h"                      # 固定输出头文件名

def to_identifier(name: str) -> str:
    """把任意文件名转成合法的 C 标识符。"""
    base = re.sub(r'[^0-9a-zA-Z_]', '_', name)
    if re.match(r'^\d', base):
        base = '_' + base
    return base

def main():
    desktop = os.path.expanduser("~/Desktop")
    in_path = os.path.join(desktop, IN_NAME)
    out_path = os.path.join(desktop, OUT_HEADER)

    if not os.path.isfile(in_path):
        raise FileNotFoundError(f"找不到图片文件：{in_path}")

    with open(in_path, "rb") as f:
        data = f.read()

    size = len(data)

    # 根据文件名生成数组名（不带扩展名）
    base_no_ext = os.path.splitext(IN_NAME)[0]
    array_name = to_identifier(base_no_ext)  # 例如 fDeeg3U_windows_98_wallpaper

    # 生成头文件文本
    lines = []
    lines.append("#ifndef BIG_JPEG_H_")
    lines.append("#define BIG_JPEG_H_")
    lines.append("")
    lines.append("#include <stdint.h>")
    lines.append("")
    lines.append(f"#define FIXED_IMAGE_SIZE {size}")
    lines.append("")
    lines.append(f"static const uint8_t {array_name}[FIXED_IMAGE_SIZE] = {{")

    # 按每行 12 个字节排版
    hex_per_line = 12
    hex_bytes = [f"0x{b:02X}" for b in data]
    for i in range(0, len(hex_bytes), hex_per_line):
        chunk = ", ".join(hex_bytes[i:i+hex_per_line])
        # 最后一行后面不要多余逗号
        if i + hex_per_line >= len(hex_bytes):
            lines.append(f"    {chunk}")
        else:
            lines.append(f"    {chunk},")
    lines.append("};")
    lines.append("")
    lines.append("#endif /* BIG_JPEG_H_ */")
    header_text = "\n".join(lines)

    with open(out_path, "w", encoding="utf-8") as f:
        f.write(header_text)

    print(f"已生成：{out_path}")
    print(f"数组名：{array_name}")
    print(f"FIXED_IMAGE_SIZE = {size} 字节")

if __name__ == "__main__":
    main()
