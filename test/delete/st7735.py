#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from luma.core.interface.serial import spi
from luma.lcd.device import st7735
from PIL import Image, ImageDraw, ImageFont
import time

# ====== ここを自分の配線に合わせて変更 ======
SPI_PORT = 0
SPI_DEVICE = 0   # CE0なら0, CE1なら1
DC_PIN = 25      # DCに接続したGPIO番号（例: GPIO25）
RST_PIN = 24     # RSTに接続したGPIO番号（例: GPIO24）
# =========================================

# 解像度はモジュールに合わせる（よくあるのは 128x160 や 80x160）
WIDTH = 128
HEIGHT = 160

def main():
    # SPIインターフェースの初期化
    serial = spi(
        port=SPI_PORT,
        device=SPI_DEVICE,
        gpio_DC=DC_PIN,
        gpio_RST=RST_PIN,
        bus_speed_hz=16000000  # 16MHzくらい（下げてもOK）
    )

    # ST7735 デバイスの作成
    device = st7735(
        serial_interface=serial,
        width=WIDTH,
        height=HEIGHT,
        rotate=0  # 必要なら 90, 180, 270 に変更
    )

    # 画面を黒で塗りつぶし
    device.clear()

    # Pillowで描画用の画像を作る
    image = Image.new("RGB", (device.width, device.height), "black")
    draw = ImageDraw.Draw(image)

    # フォント（RASPIに日本語フォントがあればそれを指定してもOK）
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 16)
    except:
        font = ImageFont.load_default()

    draw.text((5, 5), "Hello ST7735!", font=font, fill="white")
    draw.text((5, 30), "Raspberry Pi", font=font, fill="yellow")

    # 画面に表示
    device.display(image)

    print("表示しました。Ctrl+Cで終了してください。")

    # 終了までずっと表示
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        device.clear()

if __name__ == "__main__":
    main()
