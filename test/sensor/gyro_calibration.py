#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import json
import time
import pigpio
import board
import adafruit_bno055

# =========================
# 設定
# =========================
SERVO_R = 18   # GPIO18 右サーボ
SERVO_L = 12   # GPIO12 左サーボ

STOP = 1490

# 必要に応じて微調整してください
FWD_R = 1000
FWD_L = 2000
SPIN_LEFT_R = 1000
SPIN_LEFT_L = 1000
SPIN_RIGHT_R = 2000
SPIN_RIGHT_L = 2000

CAL_FILE = "bno055_mag_cal.json"
DECLINATION_DEG = 0.0   # 磁北→真北補正。必要なら後で設定
USE_EXTERNAL_CRYSTAL = True

# BNO055 内部キャリブレーションの目標
TARGET_SYS = 3
TARGET_GYRO = 3
TARGET_ACCEL = 3
TARGET_MAG = 3

# 自動キャリブレーション全体の最大時間[s]
AUTO_CAL_TIMEOUT = 40.0

# =========================
# 初期化
# =========================
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("pigpio に接続できません。pigpiod を確認してください。")

i2c = board.I2C()
bno = adafruit_bno055.BNO055_I2C(i2c)

time.sleep(1.0)

try:
    bno.use_external_crystal = USE_EXTERNAL_CRYSTAL
    time.sleep(0.5)
except Exception:
    pass

# =========================
# サーボ関数
# =========================
def servo_stop():
    pi.set_servo_pulsewidth(SERVO_R, STOP)
    pi.set_servo_pulsewidth(SERVO_L, STOP)

def servo_free():
    pi.set_servo_pulsewidth(SERVO_R, 0)
    pi.set_servo_pulsewidth(SERVO_L, 0)

def forward():
    pi.set_servo_pulsewidth(SERVO_R, FWD_R)
    pi.set_servo_pulsewidth(SERVO_L, FWD_L)

def spin_left():
    pi.set_servo_pulsewidth(SERVO_R, SPIN_LEFT_R)
    pi.set_servo_pulsewidth(SERVO_L, SPIN_LEFT_L)

def spin_right():
    pi.set_servo_pulsewidth(SERVO_R, SPIN_RIGHT_R)
    pi.set_servo_pulsewidth(SERVO_L, SPIN_RIGHT_L)

# =========================
# 磁気 min/max 収集
# =========================
mag_min = [float("inf"), float("inf"), float("inf")]
mag_max = [float("-inf"), float("-inf"), float("-inf")]

def update_mag_minmax():
    mag = bno.magnetic
    if mag is None:
        return False

    if any(v is None for v in mag):
        return False

    for i in range(3):
        mag_min[i] = min(mag_min[i], mag[i])
        mag_max[i] = max(mag_max[i], mag[i])
    return True

def calc_mag_offset_scale():
    offset = [(mag_max[i] + mag_min[i]) / 2.0 for i in range(3)]
    scale = [(mag_max[i] - mag_min[i]) / 2.0 for i in range(3)]
    return offset, scale

def save_mag_calibration(path=CAL_FILE):
    offset, scale = calc_mag_offset_scale()
    data = {
        "mag_min": mag_min,
        "mag_max": mag_max,
        "offset": offset,
        "scale": scale,
        "declination_deg": DECLINATION_DEG,
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    return data

def load_mag_calibration(path=CAL_FILE):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)

# =========================
# BNO055 内部キャリブレーション状態
# =========================
def get_cal_status():
    # (sys, gyro, accel, mag)
    return bno.calibration_status

def is_calibrated_enough():
    sys_c, gyro_c, accel_c, mag_c = get_cal_status()
    return (
        sys_c >= TARGET_SYS and
        gyro_c >= TARGET_GYRO and
        accel_c >= TARGET_ACCEL and
        mag_c >= TARGET_MAG
    )

# =========================
# heading 計算
# =========================
def normalize_deg(angle_deg):
    angle_deg = angle_deg % 360.0
    if angle_deg < 0:
        angle_deg += 360.0
    return angle_deg

def heading_from_bno055_fusion():
    """
    BNO055 のセンサフュージョン結果から方位を取得
    通常は euler[0] が heading
    """
    euler = bno.euler
    if euler is None or euler[0] is None:
        return None
    return normalize_deg(float(euler[0]))

def heading_from_mag_with_offset(offset_xyz, declination_deg=0.0):
    """
    自前補正版:
    磁気 x,y から atan2 で heading を計算
    """
    mag = bno.magnetic
    if mag is None or mag[0] is None or mag[1] is None:
        return None

    mx = mag[0] - offset_xyz[0]
    my = mag[1] - offset_xyz[1]

    # ここはセンサ搭載向きに依存
    # 一般的な平面上の簡易式として atan2(my, mx) を採用
    heading_deg = math.degrees(math.atan2(my, mx))
    heading_deg += declination_deg
    return normalize_deg(heading_deg)

# =========================
# 自動キャリブレーション動作
# =========================
def run_motion_with_sampling(motion_func, duration_s, label):
    print(f"[MOTION] {label} for {duration_s:.1f}s")
    motion_func()
    start = time.monotonic()

    while (time.monotonic() - start) < duration_s:
        update_mag_minmax()
        sys_c, gyro_c, accel_c, mag_c = get_cal_status()
        print(
            f"  CAL sys={sys_c} gyro={gyro_c} accel={accel_c} mag={mag_c}",
            end="\r",
            flush=True,
        )
        time.sleep(0.05)

    servo_stop()
    print()
    time.sleep(0.8)

def auto_calibrate():
    """
    CanSat を動かして
    - BNO055 内部キャリブレーションを進める
    - 磁気 min/max 収集
    """
    print("=== AUTO CALIBRATION START ===")
    print("内部キャリブレーションと磁気 min/max 収集を同時に行います。")

    t0 = time.monotonic()

    while True:
        if is_calibrated_enough():
            print("[OK] BNO055 内部キャリブレーション目標に到達しました。")
            break

        elapsed = time.monotonic() - t0
        if elapsed > AUTO_CAL_TIMEOUT:
            print("[WARN] タイムアウト。現時点の結果で終了します。")
            break

        # その場旋回を中心に実施
        run_motion_with_sampling(spin_left, 4.0, "spin_left")
        if is_calibrated_enough():
            break

        run_motion_with_sampling(spin_right, 4.0, "spin_right")
        if is_calibrated_enough():
            break

        # 少し前進して姿勢・周囲磁場の向き変化を増やす
        run_motion_with_sampling(forward, 2.0, "forward")
        if is_calibrated_enough():
            break

    servo_stop()
    time.sleep(1.0)

    saved = save_mag_calibration()
    print("[SAVE] 磁気補正値を保存しました:", CAL_FILE)
    print("  mag_min =", saved["mag_min"])
    print("  mag_max =", saved["mag_max"])
    print("  offset  =", saved["offset"])
    print("  scale   =", saved["scale"])
    print("=== AUTO CALIBRATION END ===")

# =========================
# 方位監視
# =========================
def monitor_heading(duration_s=None, print_hz=5.0):
    """
    2種類の方位を表示する:
    1) BNO055 フュージョン方位
    2) 自前磁気補正方位
    """
    cal = load_mag_calibration(CAL_FILE)
    offset = cal["offset"]
    decl = float(cal.get("declination_deg", 0.0))

    dt = 1.0 / print_hz
    t0 = time.monotonic()

    print("=== HEADING MONITOR START ===")
    while True:
        now = time.monotonic()
        if duration_s is not None and (now - t0) > duration_s:
            break

        sys_c, gyro_c, accel_c, mag_c = get_cal_status()
        fusion_heading = heading_from_bno055_fusion()
        mag_heading = heading_from_mag_with_offset(offset, decl)

        print(
            f"CAL(sys,gyro,accel,mag)=({sys_c},{gyro_c},{accel_c},{mag_c}) | "
            f"fusion_heading={fusion_heading!s:>8} deg | "
            f"mag_heading={mag_heading!s:>8} deg"
        )
        time.sleep(dt)

    print("=== HEADING MONITOR END ===")

# =========================
# メイン
# =========================
def main():
    try:
        servo_stop()
        time.sleep(2.0)

        # 1. 自動キャリブレーション
        auto_calibrate()

        # 2. 停止状態で方位確認
        #    必要なら duration_s=None で無限監視にしてもOK
        monitor_heading(duration_s=30.0, print_hz=5.0)

    except KeyboardInterrupt:
        print("\n[EXIT] KeyboardInterrupt")
    finally:
        servo_stop()
        time.sleep(0.5)
        servo_free()
        pi.stop()
        print("Finish")

if __name__ == "__main__":
    main()