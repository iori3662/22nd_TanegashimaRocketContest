#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import math
import json
import pigpio
import board
import busio
import adafruit_bno055

# =========================================
# 設定
# =========================================
SERVO_RIGHT_PIN = 18   # GPIO18 右サーボ
SERVO_LEFT_PIN  = 12   # GPIO12 左サーボ

STOP_RIGHT = 1490
STOP_LEFT  = 1490

# その場旋回（右回り）
SPIN_R_RIGHT = 2000
SPIN_R_LEFT  = 2000

# その場旋回（左回り）
SPIN_L_RIGHT = 1000
SPIN_L_LEFT  = 1000

CALIBRATION_TIME_SEC = 12.0   # 旋回しながらデータ取得する時間
SAMPLE_INTERVAL_SEC  = 0.05   # 20 Hz程度
SETTLE_TIME_SEC      = 1.0    # 開始前安定待ち
SAVE_FILE = "mag_offset_bno055.json"

# =========================================
# ヘルパ
# =========================================
def safe_stop_servos(pi):
    pi.set_servo_pulsewidth(SERVO_RIGHT_PIN, STOP_RIGHT)
    pi.set_servo_pulsewidth(SERVO_LEFT_PIN,  STOP_LEFT)

def free_servos(pi):
    pi.set_servo_pulsewidth(SERVO_RIGHT_PIN, 0)
    pi.set_servo_pulsewidth(SERVO_LEFT_PIN,  0)

def get_mag_xy(sensor):
    """
    BNO055の磁気センサ値を読む。
    戻り値: (mx, my)
    読めないときは (None, None)
    """
    mag = sensor.magnetic
    if mag is None:
        return None, None

    mx, my, mz = mag
    if mx is None or my is None:
        return None, None

    return float(mx), float(my)

def heading_deg_from_xy(mx, my):
    """
    atan2で 0〜360 deg に正規化
    """
    deg = math.degrees(math.atan2(my, mx))
    if deg < 0:
        deg += 360.0
    return deg

def save_offsets(offset_x, offset_y, radius_x=None, radius_y=None):
    data = {
        "offset_x": offset_x,
        "offset_y": offset_y,
        "radius_x": radius_x,
        "radius_y": radius_y,
        "saved_at_unix": time.time(),
    }
    with open(SAVE_FILE, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

def load_offsets():
    with open(SAVE_FILE, "r", encoding="utf-8") as f:
        return json.load(f)

# =========================================
# キャリブレーション本体
# =========================================
def calibrate_mag_by_spin(sensor, pi, spin_direction="right"):
    """
    Qiita記事の簡易補正法をBNO055向けに移植したもの。
    x, y の最大値・最小値からオフセットを取る。
    """
    print("=== Magnetometer manual calibration start ===")
    print("機体を床に置き、周囲の鉄・磁石・工具・PC・電源からできるだけ離してください。")
    print(f"{SETTLE_TIME_SEC:.1f}秒待機後、その場旋回しながら磁気データを収集します。")
    time.sleep(SETTLE_TIME_SEC)

    if spin_direction == "right":
        pi.set_servo_pulsewidth(SERVO_RIGHT_PIN, SPIN_R_RIGHT)
        pi.set_servo_pulsewidth(SERVO_LEFT_PIN,  SPIN_R_LEFT)
        print("右旋回でキャリブレーション中...")
    else:
        pi.set_servo_pulsewidth(SERVO_RIGHT_PIN, SPIN_L_RIGHT)
        pi.set_servo_pulsewidth(SERVO_LEFT_PIN,  SPIN_L_LEFT)
        print("左旋回でキャリブレーション中...")

    start = time.monotonic()
    max_x = None
    min_x = None
    max_y = None
    min_y = None
    samples = 0

    try:
        while (time.monotonic() - start) < CALIBRATION_TIME_SEC:
            mx, my = get_mag_xy(sensor)
            if mx is not None and my is not None:
                if max_x is None:
                    max_x = min_x = mx
                    max_y = min_y = my
                else:
                    max_x = max(max_x, mx)
                    min_x = min(min_x, mx)
                    max_y = max(max_y, my)
                    min_y = min(min_y, my)
                samples += 1

                print(
                    f"\rraw=({mx:8.3f}, {my:8.3f})  "
                    f"x[min,max]=({min_x:8.3f}, {max_x:8.3f})  "
                    f"y[min,max]=({min_y:8.3f}, {max_y:8.3f})  "
                    f"samples={samples}",
                    end=""
                )

            time.sleep(SAMPLE_INTERVAL_SEC)

    finally:
        print()
        safe_stop_servos(pi)
        time.sleep(1.0)

    if samples < 20:
        raise RuntimeError("有効サンプルが少なすぎます。BNO055の磁気値が安定して取得できていません。")

    offset_x = (max_x + min_x) / 2.0
    offset_y = (max_y + min_y) / 2.0
    radius_x = (max_x - min_x) / 2.0
    radius_y = (max_y - min_y) / 2.0

    print("=== Calibration result ===")
    print(f"offset_x = {offset_x:.6f}")
    print(f"offset_y = {offset_y:.6f}")
    print(f"radius_x = {radius_x:.6f}")
    print(f"radius_y = {radius_y:.6f}")

    # 楕円の強さを簡易チェック
    if radius_x > 0 and radius_y > 0:
        ratio = max(radius_x, radius_y) / min(radius_x, radius_y)
        print(f"ellipse_ratio = {ratio:.3f}")
        if ratio > 1.3:
            print("警告: x/yの振れ幅差が大きいです。")
            print("      オフセット補正だけでは不十分で、周囲金属やモータ磁場の影響が強い可能性があります。")

    save_offsets(offset_x, offset_y, radius_x, radius_y)
    print(f"補正値を {SAVE_FILE} に保存しました。")

    return offset_x, offset_y

# =========================================
# 確認表示
# =========================================
def monitor_heading(sensor, offset_x, offset_y, duration_sec=20.0):
    print("=== Corrected heading monitor ===")
    print("Ctrl+C で終了できます。")
    start = time.monotonic()

    while (time.monotonic() - start) < duration_sec:
        mx, my = get_mag_xy(sensor)
        if mx is None or my is None:
            print("mag read failed")
            time.sleep(0.1)
            continue

        cx = mx - offset_x
        cy = my - offset_y
        hdg = heading_deg_from_xy(cx, cy)

        print(
            f"raw=({mx:8.3f}, {my:8.3f})  "
            f"corr=({cx:8.3f}, {cy:8.3f})  "
            f"heading={hdg:7.2f} deg"
        )
        time.sleep(0.1)

# =========================================
# main
# =========================================
def main():
    # pigpio
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio daemon に接続できません。sudo pigpiod を確認してください。")

    # BNO055
    i2c = busio.I2C(board.SCL, board.SDA)
    sensor = adafruit_bno055.BNO055_I2C(i2c, address=0x28)

    try:
        # 念のため停止
        safe_stop_servos(pi)
        print("Stopping servos...")
        time.sleep(2)

        # キャリブレーション実行
        offset_x, offset_y = calibrate_mag_by_spin(sensor, pi, spin_direction="right")

        # 確認表示
        monitor_heading(sensor, offset_x, offset_y, duration_sec=20.0)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    finally:
        safe_stop_servos(pi)
        time.sleep(0.5)
        free_servos(pi)
        pi.stop()
        print("Finish")

if __name__ == "__main__":
    main()