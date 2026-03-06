#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations
import time
import math
import statistics
import threading
from dataclasses import dataclass
from collections import deque
import pigpio
import cv2
import numpy as np
from picamera2 import Picamera2
import board
import busio
import adafruit_bno055
from adafruit_bme280 import basic as adafruit_bme280
from smbus2 import SMBus
import pynmea2
import os
import asyncio
from typing import Optional
from bleak import BleakClient, BleakScanner
from bleak.exc import BleakError

# ============================================================
# 0) 共通ユーティリティ
# ============================================================
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def clamp_int(x, lo, hi):
    return int(max(lo, min(hi, int(x))))

def wrap_to_180(a_deg):
    while a_deg > 180: a_deg -= 360
    while a_deg < -180: a_deg += 360
    return a_deg

def norm3(v):
    if v is None or any(x is None for x in v): return None
    return math.sqrt(sum(x*x for x in v))

def median_or_none(buf: deque):
    if len(buf) < buf.maxlen: return None
    return statistics.median(buf)

def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
    a = math.sin(dp/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1-a))

def bearing_deg(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1)*math.sin(p2) - math.sin(p1)*math.cos(p2)*math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360) % 360

def dm_to_deg(dm, direction):
    if dm is None or direction not in ("N", "S", "E", "W"): return None
    try:
        val = float(dm)
        deg = int(val // 100)
        minutes = val - deg * 100
        res = deg + minutes / 60.0
        return -res if direction in ("S", "W") else res
    except ValueError: return None

def safe_vec3(v, default=0.0):
    if v is None: return (default, default, default)
    return tuple(float(x) if x is not None else default for x in v[:3])

def safe_float(v, default=0.0):
    try: return float(v) if v is not None else default
    except: return default

def nz(v, default=-9999.0):
    return default if v is None else v

# ============================================================
# 1) サーボ駆動
# ============================================================
@dataclass
class ServoConfig:
    stop_us: int = 1490
    min_us: int = 500
    max_us: int = 2500
    pin18: int = 18   # 右
    pin12: int = 12   # 左
    max_delta: int = 500

class ServoDrive:
    def __init__(self, pi: pigpio.pi, cfg: ServoConfig):
        self.pi, self.cfg = pi, cfg
        self.last_u18 = self.last_u12 = cfg.stop_us
        self.last_fwd = self.last_turn = 0
        self.stop()

    def _write(self, us18, us12):
        self.last_u18 = clamp_int(us18, self.cfg.min_us, self.cfg.max_us)
        self.last_u12 = clamp_int(us12, self.cfg.min_us, self.cfg.max_us)
        self.pi.set_servo_pulsewidth(self.cfg.pin18, self.last_u18)
        self.pi.set_servo_pulsewidth(self.cfg.pin12, self.last_u12)

    def stop(self):
        self.last_fwd = self.last_turn = 0
        self._write(self.cfg.stop_us, self.cfg.stop_us)

    def free(self):
        self.pi.set_servo_pulsewidth(self.cfg.pin18, 0)
        self.pi.set_servo_pulsewidth(self.cfg.pin12, 0)

    def set_diff(self, fwd, turn):
        self.last_fwd, self.last_turn = int(fwd), int(turn)
        # turn > 0 で右旋回: 右車輪減速, 左車輪増速
        l_p = clamp_int(fwd + turn, -self.cfg.max_delta, self.cfg.max_delta)
        r_p = clamp_int(fwd - turn, -self.cfg.max_delta, self.cfg.max_delta)
        self._write(self.cfg.stop_us - l_p, self.cfg.stop_us + r_p)

# ============================================================
# 2) 落下・着地検知
# ============================================================
@dataclass
class FallConfig:
    dt: float = 0.10
    win: int = 11
    req: int = 5
    sea_level_hpa: float = 1013.25
    freefall_acc_th: float = 8.5
    freefall_dalt_th: float = -0.10
    acc_land_min: float = 6.0
    acc_land_max: float = 13.0
    land_dalt_abs_th: float = 0.05

@dataclass
class FallStatus:
    phase: str
    alt: float | None = None
    acc_med: float | None = None
    dalt_med: float | None = None
    cond: bool | None = None
    consec: int = 0
    confirmed: bool = False

class FallLandingDetector:
    def __init__(self, cfg: FallConfig, logger=print):
        self.cfg, self.log = cfg, logger
        self.buf_acc = deque(maxlen=cfg.win)
        self.buf_dalt = deque(maxlen=cfg.win)
        self.prev_alt = None
        self.freefall_confirmed = False
        self.ff_consec = self.land_consec = 0

    def update(self, alt_m: float, acc_vec):
        if self.prev_alt is None:
            self.prev_alt = alt_m
            return FallStatus("WARMUP", alt=alt_m)
        dalt = alt_m - self.prev_alt
        self.prev_alt = alt_m
        acc_n = norm3(acc_vec)
        if acc_n is not None: self.buf_acc.append(acc_n)
        self.buf_dalt.append(dalt)
        
        acc_med = median_or_none(self.buf_acc)
        dalt_med = median_or_none(self.buf_dalt)
        if acc_med is None or dalt_med is None: return FallStatus("WARMUP", alt_m)

        if not self.freefall_confirmed:
            cond = (acc_med < self.cfg.freefall_acc_th) or (dalt_med < self.cfg.freefall_dalt_th)
            self.ff_consec = (self.ff_consec + 1) if cond else 0
            return FallStatus("FREEFALL", alt_m, acc_med, dalt_med, cond, self.ff_consec, self.ff_consec >= self.cfg.req)
        else:
            cond = (self.cfg.acc_land_min <= acc_med <= self.cfg.acc_land_max) or (abs(dalt_med) < self.cfg.land_dalt_abs_th)
            self.land_consec = (self.land_consec + 1) if cond else 0
            return FallStatus("LANDING", alt_m, acc_med, dalt_med, cond, self.land_consec, self.land_consec >= self.cfg.req)

# ============================================================
# 3) GPS Reader (I2C)
# ============================================================
@dataclass
class GpsFix:
    lat: float | None = None
    lon: float | None = None
    alt: float | None = None
    fixq: int = 0
    nsat: int = 0
    hdop: float | None = None
    t_mono: float | None = None

class NmeaGpsReader:
    def __init__(self, bus_num=1, addr=0x42, logger=print):
        self.bus_num, self.addr, self.log = bus_num, addr, logger
        self._fix = GpsFix()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._th = None

    def start(self):
        self._stop.clear()
        self._th = threading.Thread(target=self._run, daemon=True)
        self._th.start()

    def stop(self):
        self._stop.set()
        if self._th: self._th.join(timeout=1.0)

    def get(self) -> GpsFix:
        with self._lock: return GpsFix(**self._fix.__dict__)

    def _run(self):
        buf = bytearray()
        with SMBus(self.bus_num) as bus:
            while not self._stop.is_set():
                try:
                    data = bus.read_i2c_block_data(self.addr, 0xFF, 32)
                    buf.extend(data)
                    while b"\n" in buf:
                        line, _, rest = buf.partition(b"\n")
                        buf = bytearray(rest)
                        try:
                            s = line.decode("ascii", errors="ignore").strip()
                            if not s.startswith("$"): continue
                            msg = pynmea2.parse(s)
                            with self._lock:
                                if isinstance(msg, pynmea2.types.talker.RMC) and msg.status == "A":
                                    self._fix.lat = dm_to_deg(msg.lat, msg.lat_dir)
                                    self._fix.lon = dm_to_deg(msg.lon, msg.lon_dir)
                                    self._fix.t_mono = time.monotonic()
                                elif isinstance(msg, pynmea2.types.talker.GGA):
                                    self._fix.fixq = int(msg.gps_qual or 0)
                                    self._fix.nsat = int(msg.num_sats or 0)
                                    self._fix.hdop = float(msg.horizontal_dil or 0)
                                    self._fix.alt = float(msg.altitude or 0)
                        except Exception: continue
                except OSError: pass
                time.sleep(0.02)

# ============================================================
# 4) Compass Calibration
# ============================================================
@dataclass
class CompassCalib:
    valid: bool = False
    off_x: float = 0.0
    off_y: float = 0.0
    heading_extra_offset: float = 0.0

    def heading_from_mag(self, mag_xyz) -> float | None:
        if not self.valid or mag_xyz is None: return None
        mx, my, _ = mag_xyz
        if mx is None or my is None: return None
        hdg = math.degrees(math.atan2(my - self.off_y, mx - self.off_x))
        return (hdg + self.heading_extra_offset + 360.0) % 360.0

def run_compass_calibration(drive: ServoDrive, bno, calib: CompassCalib, logger=print):
    logger("[CAL] Starting 15s calibration rotation...")
    xs, ys = [], []
    t0 = time.monotonic()
    while (time.monotonic() - t0) < 15.0:
        # 2.5秒おきに回転方向を変えてコードの絡まり防止
        sign = 1 if int((time.monotonic()-t0)/2.5) % 2 == 0 else -1
        drive.set_diff(0, sign * 220)
        try:
            m = bno.magnetic
            if m and m[0] is not None:
                xs.append(m[0]); ys.append(m[1])
        except: pass
        time.sleep(0.05)
    drive.stop()
    if len(xs) > 10:
        calib.off_x, calib.off_y = sum(xs)/len(xs), sum(ys)/len(ys)
        calib.valid = True
        logger(f"[CAL] Done. Offset: X={calib.off_x:.2f}, Y={calib.off_y:.2f}")
    else:
        logger("[CAL] Failed. No data.")

# ============================================================
# 5) Recovery (Flip/Stuck)
# ============================================================
@dataclass
class RecoveryConfig:
    flip_th: float = 120.0
    back_ms: int = 1000
    turn_ms: int = 1200
    back_p: int = -300
    turn_p: int = 350

class RecoveryManager:
    def __init__(self, cfg: RecoveryConfig, drive: ServoDrive, bno, logger=print):
        self.cfg, self.drive, self.bno, self.log = cfg, drive, bno, logger
        self._busy_until = 0.0
        self._state = 0
        self._is_flipped_at_start = False

    def is_busy(self): return time.monotonic() < self._busy_until

    def detect_flip(self) -> bool:
        try:
            e = self.bno.euler
            if e and e[1] is not None:
                return abs(e[1]) > self.cfg.flip_th or abs(e[2]) > self.cfg.flip_th
        except: pass
        return False

    def start_recover(self, reason: str):
        self.log(f"[REC] {reason}")
        self._is_flipped_at_start = self.detect_flip()
        self._state = 0
        self._busy_until = time.monotonic() + 0.01

    def tick(self):
        if not self.is_busy(): return False
        now = time.monotonic()
        if self._state == 0:
            self.drive.set_diff(self.cfg.back_p, 0)
            self._state = 1
            self._busy_until = now + self.cfg.back_ms/1000.0
        elif self._state == 1:
            # 反転している場合は回転方向を逆にする
            tp = self.cfg.turn_p if not self._is_flipped_at_start else -self.cfg.turn_p
            self.drive.set_diff(0, tp)
            self._state = 2
            self._busy_until = now + self.cfg.turn_ms/1000.0
        elif self._state == 2:
            self.drive.stop()
            self._state = 3
            self.log("[REC] Complete")
        return True

# ============================================================
# 6) GPS Guidance
# ============================================================
class GpsGuidance:
    def __init__(self, goal, drive, bno, gps, calib, logger=print):
        self.goal = goal # (lat, lon)
        self.drive, self.bno, self.gps, self.calib, self.log = drive, bno, gps, calib, logger
        self.last_dist = self.last_err = self.last_heading = 0.0

    def step(self):
        fix = self.gps.get()
        if not fix.lat: return False, False
        
        self.last_dist = haversine_m(fix.lat, fix.lon, self.goal[0], self.goal[1])
        if self.last_dist < 1.5: return True, False # 到着

        # 方位取得（磁気優先）
        hdg = self.calib.heading_from_mag(self.bno.magnetic)
        if hdg is None:
            e = self.bno.euler
            hdg = e[0] if e and e[0] is not None else None
        
        if hdg is None: return False, False
        self.last_heading = hdg
        
        target_brg = bearing_deg(fix.lat, fix.lon, self.goal[0], self.goal[1])
        self.last_err = wrap_to_180(target_brg - hdg)
        
        turn = clamp_int(self.last_err * 0.6, -400, 400)
        fwd = 350 if abs(self.last_err) < 30 else 200
        self.drive.set_diff(fwd, turn)
        return False, False

# ============================================================
# 7) Camera Guidance
# ============================================================
class CameraGuidance:
    def __init__(self, drive, logger=print):
        self.drive, self.log = drive, logger
        self.kp = 0.45 # 旋回ゲイン（下げて安定化）
        self.last_red_ratio = 0.0
        self._scan_dir = 1
        self._scan_t0 = time.monotonic()

    def step(self, frame):
        # 90度回転して縦長に
        img = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        h, w = img.shape[:2]
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(cv2.inRange(hsv, (0,120,70), (10,255,255)),
                              cv2.inRange(hsv, (170,120,70), (180,255,255)))
        
        self.last_red_ratio = cv2.countNonZero(mask) / (w*h)
        if self.last_red_ratio > 0.6: return True # ゴール判定

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            # 探索動作
            if (time.monotonic() - self._scan_t0) % 2.0 < 1.2:
                self.drive.set_diff(0, self._scan_dir * 180)
            else:
                self.drive.stop() # 静止して確認
            return False

        best = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(best) < 500: return False

        M = cv2.moments(best)
        cx = int(M["m10"]/M["m00"])
        err = cx - (w//2)
        
        if abs(err) < w*0.1:
            self.drive.set_diff(250, 0)
        else:
            turn = clamp_int(err * self.kp, -350, 350)
            self.drive.set_diff(0, turn)
        return False

# ============================================================
# 8) Main Logic
# ============================================================
def main():
    # --- Config ---
    GOAL_POS = (35.66059, 139.36688)
    pi = pigpio.pi()
    i2c = busio.I2C(board.SCL, board.SDA)
    bno = adafruit_bno055.BNO055_I2C(i2c)
    bme = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=0x76)
    
    drive = ServoDrive(pi, ServoConfig())
    gps = NmeaGpsReader()
    gps.start()
    calib = CompassCalib()
    guidance_gps = GpsGuidance(GOAL_POS, drive, bno, gps, calib)
    guidance_cam = CameraGuidance(drive)
    recovery = RecoveryManager(RecoveryConfig(), drive, bno)
    
    phase = "DROP"
    detector = FallLandingDetector(FallConfig())
    picam2 = None

    try:
        while phase != "DONE":
            # 共通処理: 反転検知
            if not recovery.is_busy() and recovery.detect_flip():
                recovery.start_recover("FLIP detected")
            
            if recovery.is_busy():
                recovery.tick()
                time.sleep(0.05)
                continue

            # --- Phase Logic ---
            if phase == "DROP":
                st = detector.update(safe_float(bme.altitude), bno.acceleration)
                if st.phase == "FREEFALL" and st.confirmed:
                    detector.freefall_confirmed = True
                if st.phase == "LANDING" and st.confirmed:
                    print("[MAIN] Landed!")
                    drive.set_diff(300, 0); time.sleep(2.0); drive.stop() # 離脱
                    phase = "CALIB"
            
            elif phase == "CALIB":
                run_compass_calibration(drive, bno, calib)
                phase = "GPS"
            
            elif phase == "GPS":
                arrived, _ = guidance_gps.step()
                if arrived:
                    drive.stop()
                    phase = "CAMERA"
            
            elif phase == "CAMERA":
                if picam2 is None:
                    picam2 = Picamera2()
                    picam2.configure(picam2.create_preview_configuration(main={"format":"BGR888", "size":(640,480)}))
                    picam2.start()
                
                frame = picam2.capture_array("main")
                if guidance_cam.step(frame):
                    drive.stop()
                    phase = "DONE"

            time.sleep(0.05)

    except KeyboardInterrupt: pass
    finally:
        drive.stop(); gps.stop()
        if picam2: picam2.stop()
        pi.stop()

if __name__ == "__main__":
    main()