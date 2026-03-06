#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
BNO055 磁気センサ 簡易キャリブレーション
- Qiita記事の max/min 中心補正をベース
- さらに簡易的な楕円補正(2Dスケール補正)も追加
- pigpioでCanSatをその場旋回させながらデータ取得可能
- 補正前/補正後グラフを保存
- CSV保存

前提:
  sudo pigpiod
  pip install adafruit-circuitpython-bno055 matplotlib numpy

注意:
  1) サーボ通電中は磁気外乱が増える可能性あり
  2) 本当に方位精度を上げたいなら、まずは手回し校正で確認すること
  3) このコードは「水平面上の x-y 磁気」前提の2D補正
"""

import time
import csv
import math
from pathlib import Path

import pigpio
import board
import busio
import adafruit_bno055

import numpy as np
import matplotlib
matplotlib.use("Agg")   # headless用
import matplotlib.pyplot as plt


# =========================================
# 設定
# =========================================
SERVO1_PIN = 18   # 右サーボ
SERVO2_PIN = 12   # 左サーボ

STOP_PW = 1490
FWD_R_PW = 1000
FWD_L_PW = 2000

LEFT_SPIN_R_PW = 1000
LEFT_SPIN_L_PW = 1000

RIGHT_SPIN_R_PW = 2000
RIGHT_SPIN_L_PW = 2000

OUTDIR = Path("./bno055_mag_calib")
OUTDIR.mkdir(exist_ok=True)

# 取得設定
SAMPLE_HZ = 10.0
CALIB_SECONDS = 20.0

# 自動旋回するか
AUTO_SPIN = True

# 旋回方向
SPIN_DIRECTION = "left"   # "left" or "right"

# 開始前停止時間
PRE_STOP_SEC = 2.0

# 旋回後停止時間
POST_STOP_SEC = 1.0

# 外れ値対策
REJECT_NONE_OR_ZERO = True


# =========================================
# サーボ制御
# =========================================
def servo_stop(pi: pigpio.pi):
    pi.set_servo_pulsewidth(SERVO1_PIN, STOP_PW)
    pi.set_servo_pulsewidth(SERVO2_PIN, STOP_PW)


def servo_free(pi: pigpio.pi):
    pi.set_servo_pulsewidth(SERVO1_PIN, 0)
    pi.set_servo_pulsewidth(SERVO2_PIN, 0)


def servo_spin(pi: pigpio.pi, direction: str):
    if direction == "left":
        pi.set_servo_pulsewidth(SERVO1_PIN, LEFT_SPIN_R_PW)
        pi.set_servo_pulsewidth(SERVO2_PIN, LEFT_SPIN_L_PW)
    elif direction == "right":
        pi.set_servo_pulsewidth(SERVO1_PIN, RIGHT_SPIN_R_PW)
        pi.set_servo_pulsewidth(SERVO2_PIN, RIGHT_SPIN_L_PW)
    else:
        raise ValueError("direction must be 'left' or 'right'")


# =========================================
# BNO055
# =========================================
def init_bno055():
    i2c = busio.I2C(board.SCL, board.SDA)
    sensor = adafruit_bno055.BNO055_I2C(i2c, address=0x28)
    time.sleep(1.0)
    return sensor


def read_mag_xy(sensor):
    mag = sensor.magnetic  # (x, y, z) [uT] or None
    if mag is None:
        return None
    x, y, z = mag
    if x is None or y is None:
        return None
    if REJECT_NONE_OR_ZERO and (abs(x) < 1e-9 and abs(y) < 1e-9):
        return None
    return float(x), float(y), float(z if z is not None else 0.0)


# =========================================
# 補正計算
# =========================================
def compute_hardiron_offsets(x, y):
    """
    Qiita記事ベース:
      offset_x = (max_x + min_x)/2
      offset_y = (max_y + min_y)/2
    """
    x = np.asarray(x)
    y = np.asarray(y)

    x_min, x_max = np.min(x), np.max(x)
    y_min, y_max = np.min(y), np.max(y)

    off_x = (x_max + x_min) / 2.0
    off_y = (y_max + y_min) / 2.0

    radius_x = (x_max - x_min) / 2.0
    radius_y = (y_max - y_min) / 2.0

    return off_x, off_y, radius_x, radius_y


def apply_hardiron(x, y, off_x, off_y):
    x2 = np.asarray(x) - off_x
    y2 = np.asarray(y) - off_y
    return x2, y2


def apply_softiron_simple_scale(x, y):
    """
    簡易2Dスケール補正:
      x, y の半径差を平均半径に合わせる
    回転行列を伴う厳密楕円フィットではない。
    """
    x = np.asarray(x)
    y = np.asarray(y)

    rx = (np.max(x) - np.min(x)) / 2.0
    ry = (np.max(y) - np.min(y)) / 2.0

    if rx <= 1e-9 or ry <= 1e-9:
        return x.copy(), y.copy(), 1.0, 1.0

    r_avg = (rx + ry) / 2.0
    sx = r_avg / rx
    sy = r_avg / ry

    return x * sx, y * sy, sx, sy


def heading_deg_from_xy(x, y):
    """
    atan(y/x) ではなく atan2(y, x) を使う。
    0～360 deg に正規化。
    """
    th = math.degrees(math.atan2(y, x))
    if th < 0:
        th += 360.0
    return th


# =========================================
# 記録
# =========================================
def collect_data(sensor, pi=None, auto_spin=True, spin_direction="left",
                 calib_seconds=20.0, sample_hz=10.0):
    data = []

    dt = 1.0 / sample_hz

    print(f"[INFO] stop {PRE_STOP_SEC:.1f} sec")
    if pi is not None:
        servo_stop(pi)
    time.sleep(PRE_STOP_SEC)

    if auto_spin and pi is not None:
        print(f"[INFO] start auto spin: {spin_direction}")
        servo_spin(pi, spin_direction)
    else:
        print("[INFO] manual mode: rotate CanSat by hand slowly and cover full 360 deg")

    t0 = time.time()
    n_ok = 0
    n_all = 0

    while True:
        now = time.time()
        elapsed = now - t0
        if elapsed >= calib_seconds:
            break

        mag = read_mag_xy(sensor)
        n_all += 1

        if mag is not None:
            x, y, z = mag
            data.append([elapsed, x, y, z])
            n_ok += 1
            print(f"\r[INFO] t={elapsed:5.2f}s  x={x:8.3f}  y={y:8.3f}  z={z:8.3f}  samples={n_ok}", end="")

        time.sleep(dt)

    print()

    if pi is not None:
        servo_stop(pi)
        time.sleep(POST_STOP_SEC)

    print(f"[INFO] collected valid samples = {n_ok} / {n_all}")
    return data


def save_csv(path, rows, header):
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)


# =========================================
# プロット
# =========================================
def plot_before_after(x_raw, y_raw, x_corr, y_corr, png_path):
    plt.figure(figsize=(8, 8))
    plt.scatter(x_raw, y_raw, label="measured values", s=35)
    plt.scatter(x_corr, y_corr, label="fixed values", s=35, marker="^")
    plt.title("The Values of Geomagnetism")
    plt.xlabel("x_mag")
    plt.ylabel("y_mag")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(png_path, dpi=150)
    plt.close()


# =========================================
# メイン
# =========================================
def main():
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio に接続できません。先に 'sudo pigpiod' を実行してください。")

    try:
        sensor = init_bno055()

        print("[INFO] BNO055 calibration status:", sensor.calibration_status)
        print("[INFO] collecting magnetometer data...")

        raw = collect_data(
            sensor=sensor,
            pi=pi,
            auto_spin=AUTO_SPIN,
            spin_direction=SPIN_DIRECTION,
            calib_seconds=CALIB_SECONDS,
            sample_hz=SAMPLE_HZ,
        )

        if len(raw) < 20:
            raise RuntimeError("有効サンプルが少なすぎます。回転時間を延ばすか、取得周波数を見直してください。")

        raw = np.asarray(raw)
        t = raw[:, 0]
        x_raw = raw[:, 1]
        y_raw = raw[:, 2]
        z_raw = raw[:, 3]

        # 1) hard-iron補正
        off_x, off_y, radius_x, radius_y = compute_hardiron_offsets(x_raw, y_raw)
        x_hard, y_hard = apply_hardiron(x_raw, y_raw, off_x, off_y)

        # 2) 簡易soft-ironスケール補正
        x_corr, y_corr, sx, sy = apply_softiron_simple_scale(x_hard, y_hard)

        # heading例
        heading_rows = []
        for i in range(len(t)):
            hdg_raw = heading_deg_from_xy(x_raw[i], y_raw[i])
            hdg_corr = heading_deg_from_xy(x_corr[i], y_corr[i])
            heading_rows.append([t[i], x_raw[i], y_raw[i], x_corr[i], y_corr[i], hdg_raw, hdg_corr])

        # 保存
        csv_raw = OUTDIR / "mag_raw.csv"
        csv_corr = OUTDIR / "mag_corrected.csv"
        png_path = OUTDIR / "mag_calibration_plot.png"
        txt_path = OUTDIR / "calibration_result.txt"

        save_csv(
            csv_raw,
            raw.tolist(),
            ["time_sec", "x_raw_uT", "y_raw_uT", "z_raw_uT"]
        )

        corr_rows = []
        for i in range(len(t)):
            corr_rows.append([
                t[i],
                x_raw[i], y_raw[i], z_raw[i],
                x_hard[i], y_hard[i],
                x_corr[i], y_corr[i],
                heading_deg_from_xy(x_raw[i], y_raw[i]),
                heading_deg_from_xy(x_corr[i], y_corr[i]),
            ])

        save_csv(
            csv_corr,
            corr_rows,
            [
                "time_sec",
                "x_raw_uT", "y_raw_uT", "z_raw_uT",
                "x_hard_uT", "y_hard_uT",
                "x_corr_uT", "y_corr_uT",
                "heading_raw_deg", "heading_corr_deg"
            ]
        )

        plot_before_after(x_raw, y_raw, x_corr, y_corr, png_path)

        with open(txt_path, "w", encoding="utf-8") as f:
            f.write("=== Calibration result ===\n")
            f.write(f"offset_x = {off_x:.6f}\n")
            f.write(f"offset_y = {off_y:.6f}\n")
            f.write(f"radius_x = {radius_x:.6f}\n")
            f.write(f"radius_y = {radius_y:.6f}\n")
            f.write(f"scale_x = {sx:.6f}\n")
            f.write(f"scale_y = {sy:.6f}\n")
            f.write(f"ellipse_ratio_before = {radius_x / radius_y if abs(radius_y) > 1e-9 else float('inf'):.6f}\n")

        print("\n=== Calibration result ===")
        print(f"offset_x = {off_x:.6f}")
        print(f"offset_y = {off_y:.6f}")
        print(f"radius_x = {radius_x:.6f}")
        print(f"radius_y = {radius_y:.6f}")
        print(f"scale_x  = {sx:.6f}")
        print(f"scale_y  = {sy:.6f}")
        print(f"ellipse_ratio_before = {radius_x / radius_y if abs(radius_y) > 1e-9 else float('inf'):.6f}")

        print("\n[INFO] saved:")
        print(f"  {csv_raw}")
        print(f"  {csv_corr}")
        print(f"  {png_path}")
        print(f"  {txt_path}")

    finally:
        try:
            servo_stop(pi)
            time.sleep(0.5)
            servo_free(pi)
        except Exception:
            pass
        pi.stop()


if __name__ == "__main__":
    main()