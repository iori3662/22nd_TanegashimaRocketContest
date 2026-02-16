from PIL import Image
import sys
import os

def rgb888_to_rgb565(r, g, b):
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)

def main():
    print("=== img_to_rgb565.py START ===")
    print("argv:", sys.argv)

    if len(sys.argv) < 3:
        print("Usage: python img_to_rgb565.py input.png output.h")
        return

    in_path = sys.argv[1]
    out_path = sys.argv[2]

    print("Input :", in_path)
    print("Output:", out_path)

    img = Image.open(in_path).convert("RGB")
    w, h = img.size
    print(f"Image size: {w}x{h}")

    pixels = list(img.getdata())

    var_name = os.path.splitext(os.path.basename(out_path))[0]

    with open(out_path, "w", encoding="utf-8") as f:
        f.write("#pragma once\n\n")
        f.write("#include <Arduino.h>\n\n")
        f.write(f"const uint16_t {var_name}_width  = {w};\n")
        f.write(f"const uint16_t {var_name}_height = {h};\n\n")
        f.write(f"const uint16_t {var_name}_data[{w*h}] PROGMEM = {{\n")

        for i, (r, g, b) in enumerate(pixels):
            color565 = rgb888_to_rgb565(r, g, b)
            if i % 12 == 0:
                f.write("    ")
            f.write(f"0x{color565:04X}, ")
            if i % 12 == 11:
                f.write("\n")

        f.write("\n};\n")

    print(f"=== Written file: {out_path} ===")

if __name__ == "__main__":
    main()
