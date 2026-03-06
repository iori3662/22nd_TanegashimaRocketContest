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

# 停止
STOP_RIGHT = 1490
STOP_LEFT  = 1490

# 旋回（強）
RIGHT_SPIN_FAST_RIGHT = 2000
RIGHT_SPIN_FAST_LEFT  = 2000
LEFT_SPIN_FAST_RIGHT  = 1000
LEFT_SPIN_FAST_LEFT   = 1000

# 旋回（弱）: 必要に応じて調整
RIGHT_SPIN_SLOW_RIGHT = 1700
RIGHT_SPIN_SLOW_LEFT  = 1700
LEFT_SPIN_SLOW_RIGHT  = 1300
LEFT_SPIN_SLOW_LEFT   = 1300

# 方位制御
HEADING_TOL_DEG = 5.0           # この範囲に入ればOK候補
HEADING_SLOW_DEG = 20.0         # この範囲に入ったら低速旋回
HOLD_TIME_SEC = 0.7             # 許容範囲内をこの時間維持したら到達
CONTROL_DT_SEC = 0.05           # 20 Hz
TIMEOUT_PER_TARGET_SEC = 20.0   # 各目標角の最大試行時間

# JSON補正値ファイル
CAL_JSON = "mag_cal_20260307_002709.json"

# Trueならオフセット+スケール補正、Falseならオフセットのみ
USE_SCALE_CORRECTION = True

# BNO055
BNO055_ADDR = 0x28


# =========================================
# サーボ制御
# =========================================
def stop_servos(pi):
    pi.set_servo_pulsewidth(SERVO_RIGHT_PIN, STOP_RIGHT)
    pi.set_servo_pulsewidth(SERVO_LEFT_PIN, STOP_LEFT)

def free_servos(pi):
    pi.set_servo_pulsewidth(SERVO_RIGHT_PIN, 0)
    pi.set_servo_pulsewidth(SERVO_LEFT_PIN, 0)

def spin_right_fast(pi):
    pi.set_servo_pulsewidth(SERVO_RIGHT_PIN, RIGHT_SPIN_FAST_RIGHT)
    pi.set_servo_pulsewidth(SERVO_LEFT_PIN,  RIGHT_SPIN_FAST_LEFT)

def spin_left_fast(pi):
    pi.set_servo_pulsewidth(SERVO_RIGHT_PIN, LEFT_SPIN_FAST_RIGHT)
    pi.set_servo_pulsewidth(SERVO_LEFT_PIN,  LEFT_SPIN_FAST_LEFT)

def spin_right_slow(pi):
    pi.set_servo_pulsewidth(SERVO_RIGHT_PIN, RIGHT_SPIN_SLOW_RIGHT)
    pi.set_servo_pulsewidth(SERVO_LEFT_PIN,  RIGHT_SPIN_SLOW_LEFT)

def spin_left_slow(pi):
    pi.set_servo_pulsewidth(SERVO_RIGHT_PIN, LEFT_SPIN_SLOW_RIGHT)
    pi.set_servo_pulsewidth(SERVO_LEFT_PIN,  LEFT_SPIN_SLOW_LEFT)


# =========================================
# 補正値読み込み
# =========================================
def load_calibration(path):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    cal = data["calibration"]
    return {
        "offset_x": float(cal["offset_x"]),
        "offset_y": float(cal["offset_y"]),
        "radius_x": float(cal["radius_x"]),
        "radius_y": float(cal["radius_y"]),
    }


# =========================================
# センサ読み取り
# =========================================
def get_magnetic(sensor):
    mag = sensor.magnetic
    if mag is None:
        return None, None, None

    mx, my, mz = mag
    if mx is None or my is None or mz is None:
        return None, None, None

    return float(mx), float(my), float(mz)


# =========================================
# 角度計算
# =========================================
def normalize_0_360(deg):
    deg = deg % 360.0
    if deg < 0:
        deg += 360.0
    return deg

def shortest_angle_error_deg(target_deg, current_deg):
    """
    target - current を -180～180 に正規化
    正なら右回り、負なら左回り、という約束で使う
    """
    err = (target_deg - current_deg + 180.0) % 360.0 - 180.0
    return err

def heading_from_corrected_xy(mx, my, cal, use_scale=True):
    """
    補正済み方位角を計算
    """
    cx = mx - cal["offset_x"]
    cy = my - cal["offset_y"]

    if use_scale:
        rx = cal["radius_x"]
        ry = cal["radius_y"]
        if abs(rx) > 1e-9 and abs(ry) > 1e-9:
            cx = cx / rx
            cy = cy / ry

    hdg = math.degrees(math.atan2(cy, cx))
    return normalize_0_360(hdg)

def get_corrected_heading(sensor, cal, use_scale=True):
    mx, my, mz = get_magnetic(sensor)
    if mx is None or my is None:
        return None, None

    heading = heading_from_corrected_xy(mx, my, cal, use_scale=use_scale)
    return heading, (mx, my, mz)


# =========================================
# 目標角へ旋回
# =========================================
def rotate_to_heading(pi, sensor, cal, target_deg, use_scale=True):
    target_deg = normalize_0_360(target_deg)

    start_time = time.monotonic()
    entered_tol_time = None

    while True:
        now = time.monotonic()
        if now - start_time > TIMEOUT_PER_TARGET_SEC:
            stop_servos(pi)
            return False

        heading, mag = get_corrected_heading(sensor, cal, use_scale=use_scale)
        if heading is None:
            stop_servos(pi)
            time.sleep(CONTROL_DT_SEC)
            continue

        err = shortest_angle_error_deg(target_deg, heading)
        abs_err = abs(err)

        # デバッグ表示
        mx, my, mz = mag
        print(
            f"\rtarget={target_deg:6.1f}  heading={heading:7.2f}  "
            f"err={err:7.2f}  mag=({mx:8.3f},{my:8.3f},{mz:8.3f})",
            end=""
        )

        # 到達判定
        if abs_err <= HEADING_TOL_DEG:
            if entered_tol_time is None:
                entered_tol_time = now
            stop_servos(pi)

            if now - entered_tol_time >= HOLD_TIME_SEC:
                print()
                print(f"Reached target {target_deg:.1f} deg")
                return True
        else:
            entered_tol_time = None

            # 右回りを正、左回りを負とする
            if err > 0:
                if abs_err > HEADING_SLOW_DEG:
                    spin_right_fast(pi)
                else:
                    spin_right_slow(pi)
            else:
                if abs_err > HEADING_SLOW_DEG:
                    spin_left_fast(pi)
                else:
                    spin_left_slow(pi)

        time.sleep(CONTROL_DT_SEC)


# =========================================
# main
# =========================================
def main():
    cal = load_calibration(CAL_JSON)

    print("Loaded calibration:")
    print(cal)
    print(f"USE_SCALE_CORRECTION = {USE_SCALE_CORRECTION}")

    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio daemon に接続できません。sudo pigpiod を確認してください。")

    i2c = busio.I2C(board.SCL, board.SDA)
    sensor = adafruit_bno055.BNO055_I2C(i2c, address=BNO055_ADDR)

    targets = [0, 90, 180, 270, 360]

    try:
        stop_servos(pi)
        print("Stopping servos...")
        time.sleep(2.0)

        for tgt in targets:
            print(f"\n=== Rotate to {tgt} deg ===")
            ok = rotate_to_heading(
                pi=pi,
                sensor=sensor,
                cal=cal,
                target_deg=tgt,
                use_scale=USE_SCALE_CORRECTION
            )

            stop_servos(pi)
            time.sleep(1.5)

            if not ok:
                print(f"Timeout while rotating to {tgt} deg")
                break

        print("\nSequence finished")

    except KeyboardInterrupt:
        print("\nInterrupted by user")

    finally:
        stop_servos(pi)
        time.sleep(0.5)
        free_servos(pi)
        pi.stop()
        print("Finish")


if __name__ == "__main__":
    main()
