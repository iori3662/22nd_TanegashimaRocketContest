#!/usr/bin/env python3
import time
import math
import sys
import csv
import os
from datetime import datetime
import statistics
from collections import deque

import pigpio
import cv2
import numpy as np
import board
import busio
import adafruit_bno055
from adafruit_bme280 import basic as adafruit_bme280
from smbus2 import SMBus
import pynmea2
from picamera2 import Picamera2

# ============================================================
# 1) 設定・パラメータ
# ============================================================
GOAL_LAT = 35.00000000   # 目的地 緯度
GOAL_LON = 139.00000000  # 目的地 経度

# 判定閾値
TIME_LIMIT_SEC = 1200    # 20分強制終了
GPS_GOAL_RADIUS = 3.5    # カメラ誘導への切替距離 [m]
RED_RATIO_GOAL = 0.75    # ゴール判定とする赤色占有率 (画面の75%)
RED_RATIO_SLOW = 0.35    # 接近・減速を開始する赤色占有率
MEDIAN_WIN = 11          # 中央値フィルタの窓幅

# モーター・サーボ設定 (実機仕様準拠)
SERVO_L_PIN = 18
SERVO_R_PIN = 12
STOP_US = 1490           # 停止パルス幅
SERVO_SCALE = 500
SPEED_STRAIGHT = 0.6
SPEED_TURN = 0.4
SPEED_BACK = -0.5

# GPIO / 通信
NICROME_PIN = 17
TWELITE_TX = 15
TWELITE_RX = 14
TWELITE_BAUD = 38400
SEA_LEVEL_HPA = 1013.25

# ============================================================
# 2) ユーティリティクラス
# ============================================================

class RobustFilter:
    def __init__(self, window_size=11):
        self.buffers = {}
        self.window_size = window_size

    def update(self, key, val):
        if val is None: return None
        if key not in self.buffers:
            self.buffers[key] = deque(maxlen=self.window_size)
        self.buffers[key].append(val)
        if len(self.buffers[key]) < self.window_size:
            return val
        return statistics.median(self.buffers[key])

class TweliteManager:
    def __init__(self, pi, tx):
        self.pi = pi
        self.tx = tx
        self.pi.set_mode(tx, pigpio.OUTPUT)
        self.pi.write(tx, 1)

    def send(self, message):
        payload = (message + "\r\n").encode('ascii', 'ignore')
        self.pi.wave_clear()
        self.pi.wave_add_serial(self.tx, TWELITE_BAUD, payload, 0, 8, 2)
        wid = self.pi.wave_create()
        if wid >= 0:
            self.pi.wave_send_once(wid)
            while self.pi.wave_tx_busy(): time.sleep(0.001)
            self.pi.wave_delete(wid)

# ============================================================
# 3) メインシステム
# ============================================================

class CanSatMaster:
    def __init__(self):
        self.pi = pigpio.pi()
        if not self.pi.connected: sys.exit(1)

        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.bno = adafruit_bno055.BNO055_I2C(self.i2c, address=0x28)
        self.bme = adafruit_bme280.Adafruit_BME280_I2C(self.i2c, address=0x76)
        self.bme.sea_level_pressure = SEA_LEVEL_HPA
        self.gps_bus = SMBus(1)
        
        self.filter = RobustFilter(window_size=MEDIAN_WIN)
        self.twelite = TweliteManager(self.pi, TWELITE_TX)
        
        os.makedirs("logs", exist_ok=True)
        self.log_path = f"logs/mission_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.header = ["time", "elapsed", "mode", "status", "lat", "lon", "alt", "head", "temp", "ax", "ay", "az", "red_p"]
        with open(self.log_path, "w", newline="") as f:
            csv.writer(f).writerow(self.header)

        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(main={"format": "BGR888", "size": (640, 480)})
        self.picam2.configure(config)
        self.picam2.start()

        self.start_time = time.monotonic()
        self.mode = "STANDBY"
        self.status = "IDLE"
        self.lat, self.lon = 0.0, 0.0
        self.gps_buf = bytearray()

    def set_drive(self, fwd, turn):
        left_p = STOP_US - int((fwd + turn) * SERVO_SCALE)
        right_p = STOP_US + int((fwd - turn) * SERVO_SCALE)
        self.pi.set_servo_pulsewidth(SERVO_L_PIN, max(500, min(2500, left_p)))
        self.pi.set_servo_pulsewidth(SERVO_R_PIN, max(500, min(2500, right_p)))

    def record(self, red_ratio=0.0):
        elapsed = time.monotonic() - self.start_time
        acc = self.bno.acceleration
        ax = self.filter.update("ax", acc[0])
        ay = self.filter.update("ay", acc[1])
        az = self.filter.update("az", acc[2])
        head = self.filter.update("head", self.bno.euler[0])
        temp = self.filter.update("temp", self.bme.temperature)
        alt = self.filter.update("alt", self.bme.altitude)

        data = [
            datetime.now().strftime('%H:%M:%S.%f')[:-3], f"{elapsed:.1f}",
            self.mode, self.status, f"{self.lat:.8f}", f"{self.lon:.8f}",
            f"{alt:.1f}", f"{head if head else 0:.1f}", f"{temp:.1f}",
            f"{ax if ax else 0:.2f}", f"{ay if ay else 0:.2f}", f"{az if az else 0:.2f}",
            f"{red_ratio:.3f}"
        ]
        line = ",".join(map(str, data))
        with open(self.log_path, "a", newline="") as f:
            csv.writer(f).writerow(data)
        self.twelite.send(line)

        # 20分経過チェック
        if elapsed > TIME_LIMIT_SEC:
            print("TIME LIMIT EXCEEDED. FORCED GOAL.")
            self.mode = "TIME_UP_GOAL"
            self.set_drive(0, 0)
            self.record(red_ratio)
            self.picam2.stop()
            self.pi.stop()
            sys.exit(0)

    def update_gps(self):
        try:
            d = self.gps_bus.read_i2c_block_data(0x42, 0xFF, 32)
            self.gps_buf.extend(d)
            while b"\n" in self.gps_buf:
                line, _, rest = self.gps_buf.partition(b"\n")
                self.gps_buf = bytearray(rest)
                s = line.decode("ascii", errors="ignore").strip()
                if "$GNGGA" in s or "$GPGGA" in s:
                    msg = pynmea2.parse(s)
                    if msg.gps_qual > 0:
                        self.lat = self.filter.update("lat", msg.latitude)
                        self.lon = self.filter.update("lon", msg.longitude)
        except: pass

    def get_red_info(self):
        frame = self.picam2.capture_array("main")
        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE) # 反時計回り90度補正
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        m1 = cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255]))
        m2 = cv2.inRange(hsv, np.array([170, 120, 70]), np.array([179, 255, 255]))
        mask = cv2.bitwise_or(m1, m2)
        ratio = cv2.countNonZero(mask) / (frame.shape[0] * frame.shape[1])
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        offset = None
        if cnts:
            c = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(c) > 400:
                M = cv2.moments(c)
                if M["m00"] > 0:
                    offset = (int(M["m10"] / M["m00"]) / frame.shape[1]) - 0.5
        return ratio, offset

    def run(self):
        try:
            # 1. 落下検知フェーズ
            self.mode = "WAIT_DROP"
            print("Phase: Wait for Drop (Crane ascent...)")
            p_alt = self.filter.update("alt", self.bme.altitude)
            drop_count = 0
            while True:
                c_alt = self.filter.update("alt", self.bme.altitude)
                acc = self.bno.acceleration
                mag = math.sqrt(sum(x**2 for x in acc)) if acc[0] is not None else 10.0
                
                if c_alt is not None and p_alt is not None:
                    d_alt = c_alt - p_alt
                    # 自由落下(加速度が小さい) または 降下(高度が急激に低下)
                    if mag < 4.0 or d_alt < -0.1:
                        drop_count += 1
                    else:
                        drop_count = 0
                
                # 3回連続で条件を満たしたら落下と判定
                if drop_count >= 3:
                    print("Drop Detected!")
                    break
                
                p_alt = c_alt if c_alt else p_alt
                self.record(); time.sleep(0.1)

            # 2. 着陸検知フェーズ
            self.mode = "FLIGHT"
            print("Phase: Wait for Landing...")
            p_alt = self.filter.update("alt", self.bme.altitude)
            land_count = 0
            while True:
                c_alt = self.filter.update("alt", self.bme.altitude)
                acc = self.bno.acceleration
                mag = math.sqrt(sum(x**2 for x in acc)) if acc[0] is not None else 10.0
                
                if c_alt is not None and p_alt is not None:
                    d_alt = c_alt - p_alt
                    # 加速度が1G付近(8〜12) かつ 高度変化がほぼない(|d_alt|<0.05)
                    if 8.0 < mag < 12.0 and abs(d_alt) < 0.05:
                        land_count += 1
                    else:
                        land_count = 0
                
                # 5回連続で条件を満たしたら着陸と判定
                if land_count >= 5:
                    print("Landing Detected!")
                    break
                
                p_alt = c_alt if c_alt else p_alt
                self.record(); time.sleep(0.1)

            # 3. 溶断 & 直進
            self.mode = "MELTING"; self.status = "HEAT_ON"
            print("Melting (5s)...")
            self.pi.write(NICROME_PIN, 1)
            for _ in range(10): self.record(); time.sleep(0.5)
            self.pi.write(NICROME_PIN, 0); self.status = "HEAT_OFF"
            
            self.mode = "ESCAPE"; self.status = "RUN_3S"
            print("Escape: 3s Straight")
            self.set_drive(SPEED_STRAIGHT, 0)
            for _ in range(30): self.record(); time.sleep(0.1)
            self.set_drive(0, 0)

            # 4. GPS誘導
            print("Starting GPS Guidance...")
            self.mode = "GPS_NAV"
            l_dist, s_cnt = 999, 0
            while True:
                self.update_gps()
                head = self.filter.update("head", self.bno.euler[0])
                acc_z = self.filter.update("az", self.bno.acceleration[2])
                if head is None: self.record(); continue

                if acc_z is not None and acc_z < -7.0: # 反転リカバリ
                    self.status = "FLIP_RECV"; self.set_drive(1.0, 0); self.record(); time.sleep(1.0); continue

                d_lat, d_lon = math.radians(GOAL_LAT - self.lat), math.radians(GOAL_LON - self.lon)
                dist = math.sqrt((d_lat*6371000)**2 + (d_lon*6371000*math.cos(math.radians(self.lat)))**2)
                t_brg = (math.degrees(math.atan2(d_lon, d_lat)) + 360) % 360
                
                if abs(l_dist - dist) < 0.1: s_cnt += 1
                else: s_cnt = 0
                if s_cnt > 25: # スタックリカバリ
                    self.status = "STUCK_RECV"; self.set_drive(SPEED_BACK, 0); time.sleep(2.0)
                    self.set_drive(0, SPEED_TURN); time.sleep(1.0); s_cnt = 0
                l_dist = dist
                if dist < GPS_GOAL_RADIUS: break

                err = (t_brg - head + 180) % 360 - 180
                if abs(err) > 20:
                    self.status = "TURN"; self.set_drive(0, SPEED_TURN if err > 0 else -SPEED_TURN)
                else:
                    self.status = "STRAIGHT"; self.set_drive(SPEED_STRAIGHT, 0)
                self.record(); time.sleep(0.1)

            # 5. カメラ誘導
            print("Switched to Camera Guidance")
            self.mode = "CAM_NAV"
            while True:
                ratio, offset = self.get_red_info()
                if ratio > RED_RATIO_GOAL: 
                    self.status = "GOAL_DET"
                    print("Goal Detected by Camera!")
                    break
                if offset is None:
                    self.status = "SEARCH"; self.set_drive(0, SPEED_TURN)
                else:
                    f = SPEED_STRAIGHT * 0.4 if ratio > RED_RATIO_SLOW else SPEED_STRAIGHT
                    if abs(offset) < 0.15: self.status = "CAM_FWD"; self.set_drive(f, 0)
                    else: self.status = "CAM_ADJ"; self.set_drive(0, SPEED_TURN * 0.7 if offset > 0 else -SPEED_TURN * 0.7)
                self.record(red_ratio=ratio); time.sleep(0.1)

            self.mode = "FINISHED"; self.status = "STOP"; self.set_drive(0, 0); self.record()

        finally:
            self.set_drive(0, 0); self.picam2.stop(); self.pi.stop()

if __name__ == "__main__":
    CanSatMaster().run()