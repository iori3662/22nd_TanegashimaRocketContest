#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
import json
import time
import math
import statistics
from datetime import datetime

import pigpio
import board
import busio
import adafruit_bno055


# =========================================
# ユーザ設定
# =========================================
SERVO_RIGHT_PIN = 18   # GPIO18 右サーボ
SERVO_LEFT_PIN  = 12   # GPIO12 左サーボ

STOP_RIGHT = 1490
STOP_LEFT  = 1490

# その場旋回（必要に応じて調整）
SPIN_RIGHT_RIGHT = 2000
SPIN_RIGHT_LEFT  = 2000

SPIN_LEFT_RIGHT = 1000
SPIN_LEFT_LEFT  = 1000

BNO055_ADDR = 0x28

CALIBRATION_TIME_SEC = 20.0   # 平面上で旋回してデータ収集
STATIC_BEFORE_SEC    = 5.0    # キャリブレーション前の静止ログ
STATIC_AFTER_SEC     = 5.0    # キャリブレーション後の静止ログ
SAMPLE_INTERVAL_SEC  = 0.05   # 20Hz

OUTPUT_DIR = "mag_cal_logs"


# =========================================
# サーボ制御
# =========================================
def safe_stop_servos(pi):
    pi.set_servo_pulsewidth(SERVO_RIGHT_PIN, STOP_RIGHT)
    pi.set_servo_pulsewidth(SERVO_LEFT_PIN,  STOP_LEFT)

def free_servos(pi):
    pi.set_servo_pulsewidth(SERVO_RIGHT_PIN, 0)
    pi.set_servo_pulsewidth(SERVO_LEFT_PIN,  0)

def spin_right(pi):
    pi.set_servo_pulsewidth(SERVO_RIGHT_PIN, SPIN_RIGHT_RIGHT)
    pi.set_servo_pulsewidth(SERVO_LEFT_PIN,  SPIN_RIGHT_LEFT)

def spin_left(pi):
    pi.set_servo_pulsewidth(SERVO_RIGHT_PIN, SPIN_LEFT_RIGHT)
    pi.set_servo_pulsewidth(SERVO_LEFT_PIN,  SPIN_LEFT_LEFT)


# =========================================
# センサ取得
# =========================================
def get_magnetic(sensor):
    """
    BNO055の磁気センサ値を取得
    戻り値: (mx, my, mz) / 読めないときは (None, None, None)
    """
    try:
        mag = sensor.magnetic
        if mag is None:
            return None, None, None
        mx, my, mz = mag
        if mx is None or my is None or mz is None:
            return None, None, None
        return float(mx), float(my), float(mz)
    except Exception:
        return None, None, None

def get_euler(sensor):
    """
    参考用。内部融合値は信用しない前提でも、
    ログとして取っておくと切り分けに役立つ。
    戻り値: (heading, roll, pitch) / 取得失敗時は (None, None, None)
    """
    try:
        e = sensor.euler
        if e is None:
            return None, None, None
        h, r, p = e
        return h, r, p
    except Exception:
        return None, None, None

def get_accel(sensor):
    try:
        a = sensor.acceleration
        if a is None:
            return None, None, None
        ax, ay, az = a
        return ax, ay, az
    except Exception:
        return None, None, None


# =========================================
# 補正とheading
# =========================================
def heading_deg(x, y):
    """
    atan2ベースで0~360 degに正規化
    """
    deg = math.degrees(math.atan2(y, x))
    if deg < 0:
        deg += 360.0
    return deg

def corrected_xy_offset(mx, my, offset_x, offset_y):
    return mx - offset_x, my - offset_y

def corrected_xy_offset_scale(mx, my, offset_x, offset_y, radius_x, radius_y):
    cx = mx - offset_x
    cy = my - offset_y

    sx = None
    sy = None
    if radius_x is not None and abs(radius_x) > 1e-9:
        sx = cx / radius_x
    if radius_y is not None and abs(radius_y) > 1e-9:
        sy = cy / radius_y
    return sx, sy

def circular_std_deg(angle_list_deg):
    """
    角度の円統計的ばらつきの簡易指標
    値が小さいほど安定
    """
    if not angle_list_deg:
        return None
    rad = [math.radians(a) for a in angle_list_deg]
    c = sum(math.cos(r) for r in rad) / len(rad)
    s = sum(math.sin(r) for r in rad) / len(rad)
    R = math.sqrt(c*c + s*s)
    if R <= 0:
        return None
    return math.degrees(math.sqrt(-2.0 * math.log(R)))


# =========================================
# ログ保存
# =========================================
def ensure_dir(path):
    os.makedirs(path, exist_ok=True)

def save_csv(path, rows, fieldnames):
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

def save_json(path, data):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


# =========================================
# 計測
# =========================================
def capture_phase(sensor, duration_sec, phase_name, correction=None):
    """
    correction:
      None
      or dict with keys:
        offset_x, offset_y, radius_x, radius_y
    """
    rows = []
    t0 = time.monotonic()

    while (time.monotonic() - t0) < duration_sec:
        now = time.monotonic()
        mx, my, mz = get_magnetic(sensor)
        eh, er, ep = get_euler(sensor)
        ax, ay, az = get_accel(sensor)

        row = {
            "phase": phase_name,
            "t_rel_sec": now - t0,
            "mx_uT": mx,
            "my_uT": my,
            "mz_uT": mz,
            "accel_x": ax,
            "accel_y": ay,
            "accel_z": az,
            "euler_heading": eh,
            "euler_roll": er,
            "euler_pitch": ep,
            "raw_heading_xy_deg": None,
            "corr_heading_offset_deg": None,
            "corr_heading_offset_scale_deg": None,
            "corr_x_offset": None,
            "corr_y_offset": None,
            "corr_x_scale": None,
            "corr_y_scale": None,
        }

        if mx is not None and my is not None:
            row["raw_heading_xy_deg"] = heading_deg(mx, my)

            if correction is not None:
                ox = correction["offset_x"]
                oy = correction["offset_y"]
                rx = correction["radius_x"]
                ry = correction["radius_y"]

                cx, cy = corrected_xy_offset(mx, my, ox, oy)
                row["corr_x_offset"] = cx
                row["corr_y_offset"] = cy
                row["corr_heading_offset_deg"] = heading_deg(cx, cy)

                sx, sy = corrected_xy_offset_scale(mx, my, ox, oy, rx, ry)
                row["corr_x_scale"] = sx
                row["corr_y_scale"] = sy
                if sx is not None and sy is not None:
                    row["corr_heading_offset_scale_deg"] = heading_deg(sx, sy)

        rows.append(row)
        time.sleep(SAMPLE_INTERVAL_SEC)

    return rows


def compute_calibration_from_rows(rows):
    xs = [r["mx_uT"] for r in rows if r["mx_uT"] is not None]
    ys = [r["my_uT"] for r in rows if r["my_uT"] is not None]

    if len(xs) < 20 or len(ys) < 20:
        raise RuntimeError("有効な磁気データが少なすぎます。")

    x_min = min(xs)
    x_max = max(xs)
    y_min = min(ys)
    y_max = max(ys)

    offset_x = (x_max + x_min) / 2.0
    offset_y = (y_max + y_min) / 2.0
    radius_x = (x_max - x_min) / 2.0
    radius_y = (y_max - y_min) / 2.0

    ellipse_ratio = None
    if radius_x > 1e-9 and radius_y > 1e-9:
        ellipse_ratio = max(radius_x, radius_y) / min(radius_x, radius_y)

    return {
        "x_min": x_min,
        "x_max": x_max,
        "y_min": y_min,
        "y_max": y_max,
        "offset_x": offset_x,
        "offset_y": offset_y,
        "radius_x": radius_x,
        "radius_y": radius_y,
        "ellipse_ratio": ellipse_ratio,
        "sample_count": len(xs),
    }


def summarize_static_rows(rows):
    def valid(key):
        return [r[key] for r in rows if r[key] is not None]

    summary = {}

    for key in [
        "mx_uT", "my_uT", "mz_uT",
        "raw_heading_xy_deg",
        "corr_heading_offset_deg",
        "corr_heading_offset_scale_deg",
        "euler_heading",
    ]:
        vals = valid(key)
        if vals:
            summary[key] = {
                "mean": statistics.mean(vals),
                "median": statistics.median(vals),
                "min": min(vals),
                "max": max(vals),
                "stddev": statistics.pstdev(vals) if len(vals) >= 2 else 0.0,
            }
            if "heading" in key:
                csd = circular_std_deg(vals)
                summary[key]["circular_std_deg"] = csd

    return summary


# =========================================
# main
# =========================================
def main():
    ensure_dir(OUTPUT_DIR)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    base = os.path.join(OUTPUT_DIR, f"mag_cal_{stamp}")
    csv_path = base + ".csv"
    json_path = base + ".json"

    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio daemon に接続できません。sudo pigpiod を確認してください。")

    i2c = busio.I2C(board.SCL, board.SDA)
    sensor = adafruit_bno055.BNO055_I2C(i2c, address=BNO055_ADDR)

    all_rows = []
    result = {
        "timestamp": stamp,
        "settings": {
            "SERVO_RIGHT_PIN": SERVO_RIGHT_PIN,
            "SERVO_LEFT_PIN": SERVO_LEFT_PIN,
            "STOP_RIGHT": STOP_RIGHT,
            "STOP_LEFT": STOP_LEFT,
            "SPIN_RIGHT_RIGHT": SPIN_RIGHT_RIGHT,
            "SPIN_RIGHT_LEFT": SPIN_RIGHT_LEFT,
            "SPIN_LEFT_RIGHT": SPIN_LEFT_RIGHT,
            "SPIN_LEFT_LEFT": SPIN_LEFT_LEFT,
            "CALIBRATION_TIME_SEC": CALIBRATION_TIME_SEC,
            "STATIC_BEFORE_SEC": STATIC_BEFORE_SEC,
            "STATIC_AFTER_SEC": STATIC_AFTER_SEC,
            "SAMPLE_INTERVAL_SEC": SAMPLE_INTERVAL_SEC,
            "BNO055_ADDR": hex(BNO055_ADDR),
        }
    }

    try:
        print("サーボ停止")
        safe_stop_servos(pi)
        time.sleep(2.0)

        print(f"静止ログ(補正前)を {STATIC_BEFORE_SEC} 秒取得します")
        rows_static_before = capture_phase(
            sensor=sensor,
            duration_sec=STATIC_BEFORE_SEC,
            phase_name="static_before",
            correction=None
        )
        all_rows.extend(rows_static_before)

        print(f"平面上で右旋回しながらキャリブレーションデータを {CALIBRATION_TIME_SEC} 秒取得します")
        spin_right(pi)
        rows_cal = []
        t0 = time.monotonic()
        while (time.monotonic() - t0) < CALIBRATION_TIME_SEC:
            rows = capture_phase(
                sensor=sensor,
                duration_sec=SAMPLE_INTERVAL_SEC,
                phase_name="calibration_spin",
                correction=None
            )
            if rows:
                rows_cal.extend(rows)
                all_rows.extend(rows)
                last = rows[-1]
                mx = last["mx_uT"]
                my = last["my_uT"]
                mz = last["mz_uT"]
                print(f"\rmx={mx!s:>8} my={my!s:>8} mz={mz!s:>8} count={len(rows_cal)}", end="")
        print()

        safe_stop_servos(pi)
        time.sleep(1.0)

        cal = compute_calibration_from_rows(rows_cal)
        result["calibration"] = cal

        correction = {
            "offset_x": cal["offset_x"],
            "offset_y": cal["offset_y"],
            "radius_x": cal["radius_x"],
            "radius_y": cal["radius_y"],
        }

        print("=== Calibration result ===")
        print(f"offset_x     = {cal['offset_x']:.6f}")
        print(f"offset_y     = {cal['offset_y']:.6f}")
        print(f"radius_x     = {cal['radius_x']:.6f}")
        print(f"radius_y     = {cal['radius_y']:.6f}")
        print(f"ellipse_ratio= {cal['ellipse_ratio']:.6f}" if cal["ellipse_ratio"] is not None else "ellipse_ratio= None")
        print(f"sample_count = {cal['sample_count']}")

        print(f"静止ログ(補正後評価用)を {STATIC_AFTER_SEC} 秒取得します")
        rows_static_after = capture_phase(
            sensor=sensor,
            duration_sec=STATIC_AFTER_SEC,
            phase_name="static_after",
            correction=correction
        )
        all_rows.extend(rows_static_after)

        # calibration_spin に対しても補正列をあと付け
        for r in rows_cal:
            mx = r["mx_uT"]
            my = r["my_uT"]
            if mx is not None and my is not None:
                r["raw_heading_xy_deg"] = heading_deg(mx, my)

                cx, cy = corrected_xy_offset(mx, my, correction["offset_x"], correction["offset_y"])
                r["corr_x_offset"] = cx
                r["corr_y_offset"] = cy
                r["corr_heading_offset_deg"] = heading_deg(cx, cy)

                sx, sy = corrected_xy_offset_scale(
                    mx, my,
                    correction["offset_x"], correction["offset_y"],
                    correction["radius_x"], correction["radius_y"]
                )
                r["corr_x_scale"] = sx
                r["corr_y_scale"] = sy
                if sx is not None and sy is not None:
                    r["corr_heading_offset_scale_deg"] = heading_deg(sx, sy)

        result["summary"] = {
            "static_before": summarize_static_rows(rows_static_before),
            "static_after": summarize_static_rows(rows_static_after),
        }

        # 行の整形
        fieldnames = [
            "phase", "t_rel_sec",
            "mx_uT", "my_uT", "mz_uT",
            "accel_x", "accel_y", "accel_z",
            "euler_heading", "euler_roll", "euler_pitch",
            "raw_heading_xy_deg",
            "corr_heading_offset_deg",
            "corr_heading_offset_scale_deg",
            "corr_x_offset", "corr_y_offset",
            "corr_x_scale", "corr_y_scale",
        ]
        save_csv(csv_path, all_rows, fieldnames)
        save_json(json_path, result)

        print()
        print("保存完了")
        print(f"CSV : {csv_path}")
        print(f"JSON: {json_path}")

    except KeyboardInterrupt:
        print("\n中断されました。")
    finally:
        safe_stop_servos(pi)
        time.sleep(0.5)
        free_servos(pi)
        pi.stop()
        print("Finish")


if __name__ == "__main__":
    main()