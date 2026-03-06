#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import os
import csv
import time
import math
import statistics
import threading
from dataclasses import dataclass
from collections import deque
from datetime import datetime

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


# ============================================================
# 0) 共通ユーティリティ
# ============================================================
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def clamp_int(x, lo, hi):
    return int(max(lo, min(hi, int(x))))

def wrap_to_180(a_deg):
    while a_deg > 180:
        a_deg -= 360
    while a_deg < -180:
        a_deg += 360
    return a_deg

def norm3(v):
    if v is None:
        return None
    x, y, z = v
    if x is None or y is None or z is None:
        return None
    return math.sqrt(x * x + y * y + z * z)

def median_or_none(buf: deque):
    if len(buf) < buf.maxlen:
        return None
    return statistics.median(buf)

def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def bearing_deg(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    b = math.degrees(math.atan2(y, x))
    return (b + 360) % 360

def dm_to_deg(dm, direction):
    if dm is None or direction not in ("N", "S", "E", "W"):
        return None
    try:
        dm = float(dm)
    except ValueError:
        return None
    deg = int(dm // 100)
    minutes = dm - deg * 100
    val = deg + minutes / 60.0
    return -val if direction in ("S", "W") else val


# ============================================================
# 1) pigpio サーボ駆動
# ============================================================
@dataclass
class ServoConfig:
    stop_us: int = 1490
    min_us: int = 500
    max_us: int = 2500
    pin18: int = 18
    pin12: int = 12
    max_delta: int = 600

class ServoDrive:
    def __init__(self, pi: pigpio.pi, cfg: ServoConfig):
        self.pi = pi
        self.cfg = cfg
        self.stop()

    def _write(self, us18, us12):
        u18 = clamp_int(us18, self.cfg.min_us, self.cfg.max_us)
        u12 = clamp_int(us12, self.cfg.min_us, self.cfg.max_us)
        self.pi.set_servo_pulsewidth(self.cfg.pin18, u18)
        self.pi.set_servo_pulsewidth(self.cfg.pin12, u12)

    def stop(self):
        self._write(self.cfg.stop_us, self.cfg.stop_us)

    def free(self):
        self.pi.set_servo_pulsewidth(self.cfg.pin18, 0)
        self.pi.set_servo_pulsewidth(self.cfg.pin12, 0)

    def set_diff(self, fwd, turn):
        # 実機挙動に合わせて旋回符号を反転（あなたの確定済み設定）
        turn = -turn

        left_power = fwd + turn
        right_power = fwd - turn

        left_power = clamp_int(left_power, -self.cfg.max_delta, self.cfg.max_delta)
        right_power = clamp_int(right_power, -self.cfg.max_delta, self.cfg.max_delta)

        pulse18 = self.cfg.stop_us - left_power
        pulse12 = self.cfg.stop_us + right_power
        self._write(pulse18, pulse12)


# ============================================================
# 2) 落下検知＋着地判定
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
        self.cfg = cfg
        self.log = logger
        self.buf_acc = deque(maxlen=cfg.win)
        self.buf_dalt = deque(maxlen=cfg.win)
        self.prev_alt = None
        self.freefall_confirmed = False
        self.ff_consec = 0
        self.land_consec = 0

    def update(self, alt_m: float, acc_vec):
        c = self.cfg
        if self.prev_alt is None:
            self.prev_alt = alt_m
            return FallStatus(phase="WARMUP", alt=alt_m)

        dalt = alt_m - self.prev_alt
        self.prev_alt = alt_m

        acc_n = norm3(acc_vec)
        if acc_n is not None:
            self.buf_acc.append(acc_n)
        self.buf_dalt.append(dalt)

        acc_med = median_or_none(self.buf_acc)
        dalt_med = median_or_none(self.buf_dalt)
        if acc_med is None or dalt_med is None:
            return FallStatus(phase="WARMUP", alt=alt_m)

        if not self.freefall_confirmed:
            ff_cond = (acc_med < c.freefall_acc_th) or (dalt_med < c.freefall_dalt_th)
            self.ff_consec = (self.ff_consec + 1) if ff_cond else 0
            confirmed = (self.ff_consec >= c.req)
            return FallStatus("FREEFALL", alt_m, acc_med, dalt_med, ff_cond, self.ff_consec, confirmed)

        land_cond = ((c.acc_land_min <= acc_med <= c.acc_land_max) or (abs(dalt_med) < c.land_dalt_abs_th))
        self.land_consec = (self.land_consec + 1) if land_cond else 0
        confirmed = (self.land_consec >= c.req)
        return FallStatus("LANDING", alt_m, acc_med, dalt_med, land_cond, self.land_consec, confirmed)


# ============================================================
# 3) GPS (I2C NMEA) 受信スレッド
# ============================================================
@dataclass
class GpsFix:
    lat: float | None = None
    lon: float | None = None
    fixq: int = 0
    nsat: int = 0
    hdop: float | None = None
    t_mono: float | None = None

@dataclass
class GpsConfig:
    goal_lat: float = 35.66059
    goal_lon: float = 139.36688

    control_hz: float = 10.0
    gps_bus: int = 1
    gps_addr: int = 0x42
    read_len: int = 32

    gps_min_fixq: int = 1
    gps_min_sats: int = 5
    gps_max_hdop: float = 8.0
    gps_stale_sec: float = 2.5

    arrival_radius_m: float = 1.0
    angle_ok_deg: float = 3.0

    base_v: float = 0.55
    min_v: float = 0.25
    slowdown_dist_m: float = 8.0

    kp: float = 0.020
    turn_max: float = 0.85
    turn_bias: float = 0.00

    stuck_window_sec: float = 4.0
    stuck_min_progress_m: float = 0.3
    stuck_v_gate: float = 0.35

class NmeaGpsReader:
    def __init__(self, cfg: GpsConfig, logger=print):
        self.cfg = cfg
        self.log = logger
        self._fix = GpsFix()
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._th = None

    def start(self):
        if self._th is not None and self._th.is_alive():
            return
        self._stop_event.clear()
        self._th = threading.Thread(target=self._run, daemon=True)
        self._th.start()

    def stop(self):
        self._stop_event.set()

    def get(self) -> GpsFix:
        with self._lock:
            return GpsFix(**self._fix.__dict__)

    def _run(self):
        buf = bytearray()
        with SMBus(self.cfg.gps_bus) as bus:
            while not self._stop_event.is_set():
                try:
                    data = bus.read_i2c_block_data(self.cfg.gps_addr, 0xFF, self.cfg.read_len)
                except OSError:
                    time.sleep(0.05)
                    continue

                buf.extend(data)

                while b"\n" in buf:
                    line, _, rest = buf.partition(b"\n")
                    buf = bytearray(rest)

                    s = line.decode("ascii", errors="ignore").strip()
                    if not s.startswith("$"):
                        continue

                    try:
                        msg = pynmea2.parse(s)
                    except pynmea2.ParseError:
                        continue

                    if msg.sentence_type == "RMC" and getattr(msg, "status", "") == "A":
                        lat = dm_to_deg(getattr(msg, "lat", None), getattr(msg, "lat_dir", None))
                        lon = dm_to_deg(getattr(msg, "lon", None), getattr(msg, "lon_dir", None))
                        if lat is not None and lon is not None:
                            with self._lock:
                                self._fix.lat = lat
                                self._fix.lon = lon
                                self._fix.t_mono = time.monotonic()

                    elif msg.sentence_type == "GGA":
                        try:
                            fixq = int(getattr(msg, "gps_qual", 0) or 0)
                        except ValueError:
                            fixq = 0
                        try:
                            nsat = int(getattr(msg, "num_sats", 0) or 0)
                        except ValueError:
                            nsat = 0
                        try:
                            hdop = float(getattr(msg, "horizontal_dil", 0) or 0)
                        except ValueError:
                            hdop = None

                        with self._lock:
                            self._fix.fixq = fixq
                            self._fix.nsat = nsat
                            self._fix.hdop = hdop

                time.sleep(0.02)


# ============================================================
# 4) 磁気キャリブレーション（max/min中心）
# ============================================================
@dataclass
class CompassCalib:
    valid: bool = False
    off_x: float = 0.0
    off_y: float = 0.0
    heading_extra_offset_deg: float = 0.0

    def heading_from_mag(self, mag_xyz) -> float | None:
        if mag_xyz is None:
            return None
        mx, my, mz = mag_xyz
        if mx is None or my is None:
            return None
        x = mx - self.off_x
        y = my - self.off_y
        hdg = math.degrees(math.atan2(y, x))
        hdg = (hdg + 360.0) % 360.0
        hdg = (hdg + self.heading_extra_offset_deg) % 360.0
        return hdg

def run_compass_calibration(drive: ServoDrive, bno, calib: CompassCalib,
                            duration_sec: float = 15.0, turn_power: int = 220, logger=print):
    logger(f"[CAL] Compass calibration start: rotate {duration_sec:.1f}s")
    max_x = max_y = None
    min_x = min_y = None

    t0 = time.monotonic()
    while (time.monotonic() - t0) < duration_sec:
        t = time.monotonic() - t0
        sign = 1 if int(t / 2.5) % 2 == 0 else -1
        drive.set_diff(0, sign * turn_power)

        try:
            mag = bno.magnetic
        except Exception:
            mag = None

        if mag is not None:
            mx, my, mz = mag
            if mx is not None and my is not None:
                max_x = mx if (max_x is None or mx > max_x) else max_x
                min_x = mx if (min_x is None or mx < min_x) else min_x
                max_y = my if (max_y is None or my > max_y) else max_y
                min_y = my if (min_y is None or my < min_y) else min_y

        time.sleep(0.05)

    drive.stop()
    time.sleep(0.2)

    if None in (max_x, min_x, max_y, min_y):
        calib.valid = False
        logger("[CAL] FAILED: magnetic data missing")
        return

    calib.off_x = 0.5 * (max_x + min_x)
    calib.off_y = 0.5 * (max_y + min_y)
    calib.valid = True
    logger(f"[CAL] DONE: off_x={calib.off_x:.3f} off_y={calib.off_y:.3f} valid={calib.valid}")


# ============================================================
# 5) リカバリ（Flip/スタック）
# ============================================================
@dataclass
class RecoveryConfig:
    flip_roll_deg: float = 85.0
    flip_pitch_deg: float = 85.0

    back_ms: int = 1200
    turn_ms: int = 1200
    fwd_ms: int = 600
    back_power: int = -360
    turn_power: int = 420
    fwd_power: int = 260

class RecoveryManager:
    def __init__(self, cfg: RecoveryConfig, drive: ServoDrive, bno, logger=print):
        self.cfg = cfg
        self.drive = drive
        self.bno = bno
        self.log = logger
        self._busy_until = 0.0
        self._state = 0

    def _now(self):
        return time.monotonic()

    def is_busy(self):
        return self._now() < self._busy_until

    def _set_busy(self, ms):
        self._busy_until = self._now() + ms / 1000.0

    def detect_flip(self) -> bool:
        try:
            e = self.bno.euler  # (heading, roll, pitch)
        except Exception:
            return False
        if e is None:
            return False
        roll = e[1]
        pitch = e[2]
        if roll is None or pitch is None:
            return False
        return (abs(roll) >= self.cfg.flip_roll_deg) or (abs(pitch) >= self.cfg.flip_pitch_deg)

    def start_recover(self, reason: str):
        self.log(f"[REC] start recovery: {reason}")
        self._state = 0
        self._set_busy(1)

    def tick(self):
        if not self.is_busy():
            return False

        if self._state == 0:
            self.drive.set_diff(self.cfg.back_power, 0)
            self._state = 1
            self._set_busy(self.cfg.back_ms)
            return True

        if self._state == 1:
            self.drive.set_diff(0, self.cfg.turn_power)
            self._state = 2
            self._set_busy(self.cfg.turn_ms)
            return True

        if self._state == 2:
            self.drive.set_diff(self.cfg.fwd_power, 0)
            self._state = 3
            self._set_busy(self.cfg.fwd_ms)
            return True

        self.drive.stop()
        self._busy_until = 0.0
        self.log("[REC] done")
        return False


# ============================================================
# 6) GPS誘導（左右差制御）
# ============================================================
class GpsGuidance:
    def __init__(self, cfg: GpsConfig, drive: ServoDrive, bno, gps_reader: NmeaGpsReader, calib: CompassCalib, logger=print):
        self.cfg = cfg
        self.drive = drive
        self.bno = bno
        self.gps = gps_reader
        self.calib = calib
        self.log = logger
        self._last_log = time.monotonic()

        self._stuck_t0 = None
        self._stuck_d0 = None

        # 状態公開用（ダウンリンク/CSV）
        self.last_dist = None
        self.last_goal_deg = None
        self.last_head_deg = None
        self.last_err_deg = None
        self.last_cmd_v = 0.0
        self.last_cmd_turn = 0.0
        self.last_nsat = 0
        self.last_hdop = None
        self.last_lat = None
        self.last_lon = None

    def _gps_ok(self, fix: GpsFix) -> bool:
        c = self.cfg
        if fix.lat is None or fix.lon is None:
            return False
        if fix.fixq < c.gps_min_fixq:
            return False
        if fix.nsat < c.gps_min_sats:
            return False
        if fix.hdop is None or fix.hdop > c.gps_max_hdop:
            return False
        if fix.t_mono is None or (time.monotonic() - fix.t_mono) > c.gps_stale_sec:
            return False
        return True

    def _get_heading_deg(self) -> float | None:
        if self.calib.valid:
            try:
                mag = self.bno.magnetic
            except Exception:
                mag = None
            hdg = self.calib.heading_from_mag(mag)
            if hdg is not None:
                return hdg

        try:
            e = self.bno.euler
            if e is not None and e[0] is not None:
                return float(e[0]) % 360.0
        except Exception:
            pass
        return None

    def step(self) -> tuple[bool, bool]:
        c = self.cfg
        fix = self.gps.get()
        heading = self._get_heading_deg()

        # 状態保持（ダウンリンク用）
        self.last_lat = fix.lat
        self.last_lon = fix.lon
        self.last_nsat = fix.nsat
        self.last_hdop = fix.hdop

        if heading is None or not self._gps_ok(fix):
            self.drive.stop()
            self.last_head_deg = heading
            self.last_cmd_v = 0.0
            self.last_cmd_turn = 0.0
            return (False, False)

        dist = haversine_m(fix.lat, fix.lon, c.goal_lat, c.goal_lon)
        theta_goal = bearing_deg(fix.lat, fix.lon, c.goal_lat, c.goal_lon)
        err = wrap_to_180(theta_goal - heading)

        self.last_dist = dist
        self.last_goal_deg = theta_goal
        self.last_head_deg = heading
        self.last_err_deg = err

        if dist <= c.arrival_radius_m:
            self.drive.stop()
            self.last_cmd_v = 0.0
            self.last_cmd_turn = 0.0
            self.log(f"[GPS] ARRIVED dist={dist:.2f}m <= {c.arrival_radius_m}m")
            return (True, False)

        if dist < c.slowdown_dist_m:
            a = clamp(dist / c.slowdown_dist_m, 0.0, 1.0)
            v = c.min_v + (c.base_v - c.min_v) * a
        else:
            v = c.base_v

        if abs(err) < c.angle_ok_deg:
            turn = c.turn_bias
        else:
            turn = clamp(c.kp * err + c.turn_bias, -c.turn_max, c.turn_max)

        fwd_power = int(v * self.drive.cfg.max_delta)
        turn_power = int(turn * self.drive.cfg.max_delta)
        self.drive.set_diff(fwd_power, turn_power)

        self.last_cmd_v = float(v)
        self.last_cmd_turn = float(turn)

        # スタック疑い
        stuck = False
        if v >= c.stuck_v_gate:
            if self._stuck_t0 is None:
                self._stuck_t0 = time.monotonic()
                self._stuck_d0 = dist
            else:
                if (time.monotonic() - self._stuck_t0) >= c.stuck_window_sec:
                    progress = (self._stuck_d0 - dist) if (self._stuck_d0 is not None) else 0.0
                    if progress < c.stuck_min_progress_m:
                        stuck = True
                    self._stuck_t0 = time.monotonic()
                    self._stuck_d0 = dist
        else:
            self._stuck_t0 = None
            self._stuck_d0 = None

        now = time.monotonic()
        if now - self._last_log > 1.0:
            self.log(
                f"[GPS] dist={dist:6.1f}m goal={theta_goal:6.1f} head={heading:6.1f} err={err:6.1f} "
                f"v={v:.2f} turn={turn:.2f} nsat={fix.nsat} hdop={fix.hdop} calib={self.calib.valid}"
            )
            self._last_log = now

        return (False, stuck)


# ============================================================
# 7) カメラ誘導（赤コーン）
# ============================================================
@dataclass
class CameraConfig:
    w: int = 480
    h: int = 640
    show_debug: bool = False

    kp_turn: float = 0.8
    base_fwd: int = 240

    search_turn: int = 120
    scan_step_sec: float = 0.55
    scan_pause_sec: float = 0.12

    area_track_th: int = 900
    area_fwd_th: int = 2200
    center_tol_ratio: float = 0.08

    slow_red_ratio: float = 0.35
    goal_red_ratio: float = 0.60
    goal_hold_sec: float = 0.25

class CameraGuidance:
    LOWER_RED1 = np.array([0, 120, 70], dtype=np.uint8)
    UPPER_RED1 = np.array([10, 255, 255], dtype=np.uint8)
    LOWER_RED2 = np.array([170, 120, 70], dtype=np.uint8)
    UPPER_RED2 = np.array([179, 255, 255], dtype=np.uint8)
    KERNEL = np.ones((5, 5), np.uint8)

    def __init__(self, cfg: CameraConfig, drive: ServoDrive, logger=print):
        self.cfg = cfg
        self.drive = drive
        self.log = logger
        self._goal_start_t = None

        self._scan_dir = 1
        self._scan_t0 = time.monotonic()
        self._scan_phase = "TURN"

        # 状態公開用
        self.last_red_ratio = None

    def _make_red_mask(self, frame_bgr):
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        m1 = cv2.inRange(hsv, self.LOWER_RED1, self.UPPER_RED1)
        m2 = cv2.inRange(hsv, self.LOWER_RED2, self.UPPER_RED2)
        mask = cv2.bitwise_or(m1, m2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.KERNEL)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.KERNEL)
        return mask

    def _is_triangle_like(self, contour):
        area = cv2.contourArea(contour)
        if area < self.cfg.area_track_th:
            return False, None, area
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
        if len(approx) != 3:
            return False, approx, area
        if not cv2.isContourConvex(approx):
            return False, approx, area
        return True, approx, area

    def _search_scan(self):
        c = self.cfg
        now = time.monotonic()
        if self._scan_phase == "TURN":
            self.drive.set_diff(0, self._scan_dir * c.search_turn)
            if (now - self._scan_t0) >= c.scan_step_sec:
                self._scan_phase = "PAUSE"
                self._scan_t0 = now
                self.drive.stop()
        else:
            self.drive.stop()
            if (now - self._scan_t0) >= c.scan_pause_sec:
                self._scan_phase = "TURN"
                self._scan_t0 = now
                self._scan_dir *= -1

    def step(self, frame_bgr) -> bool:
        c = self.cfg
        frame = cv2.rotate(frame_bgr, cv2.ROTATE_90_COUNTERCLOCKWISE)

        h, w = frame.shape[:2]
        cx_frame = w // 2
        center_tol_px = int(w * c.center_tol_ratio)

        mask = self._make_red_mask(frame)
        red_ratio = cv2.countNonZero(mask) / float(h * w)
        self.last_red_ratio = float(red_ratio)

        if red_ratio >= c.goal_red_ratio:
            if self._goal_start_t is None:
                self._goal_start_t = time.time()
            elif (time.time() - self._goal_start_t) >= c.goal_hold_sec:
                self.drive.stop()
                self.log(f"[CAM] GOAL red_ratio={red_ratio:.3f}")
                return True
        else:
            self._goal_start_t = None

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_area = 0.0
        best_approx = None

        for ct in contours:
            ok, approx, area = self._is_triangle_like(ct)
            if ok and area > best_area:
                best = ct
                best_area = float(area)
                best_approx = approx

        if best is None:
            if red_ratio >= c.slow_red_ratio:
                self.drive.set_diff(int(c.base_fwd * 0.35), 0)
            else:
                self._search_scan()
            return False

        M = cv2.moments(best)
        if M["m00"] == 0:
            if red_ratio >= c.slow_red_ratio:
                self.drive.set_diff(int(c.base_fwd * 0.35), 0)
            else:
                self._search_scan()
            return False

        cx = int(M["m10"] / M["m00"])
        err = cx - cx_frame

        if abs(err) < center_tol_px:
            if red_ratio >= c.slow_red_ratio:
                fwd = int(c.base_fwd * 0.35)
            else:
                fwd = c.base_fwd if best_area < c.area_fwd_th else int(c.base_fwd * 0.8)
            self.drive.set_diff(fwd, 0)
        else:
            turn_gain = 0.55 if red_ratio >= c.slow_red_ratio else 1.0
            turn = int(turn_gain * c.kp_turn * err)
            turn = clamp_int(turn, -420, 420)
            self.drive.set_diff(0, turn)

        return False


# ============================================================
# 8) TWELITE Soft UART（送信専用）
# ============================================================
BAUD = 38400
TWE_RX_GPIO = 14
TWE_TX_GPIO = 15
DATA_BITS = 8
STOP_BITS = 1

class SoftUartTwelite:
    def __init__(self, pi: pigpio.pi, tx_gpio: int, rx_gpio: int, baud: int):
        self.pi = pi
        self.tx = tx_gpio
        self.rx = rx_gpio
        self.baud = baud

        self.pi.set_mode(self.tx, pigpio.OUTPUT)
        self.pi.write(self.tx, 1)

        self.pi.set_mode(self.rx, pigpio.INPUT)
        self.pi.bb_serial_read_open(self.rx, self.baud, DATA_BITS)
        time.sleep(0.05)

    def close(self):
        try:
            self.pi.bb_serial_read_close(self.rx)
        except pigpio.error:
            pass

    def send_line(self, s):
        """
        何が来ても安全に bytes(CRLF付き) にして送る
        - int/float/None → 文字列化
        - bytes/bytearray → そのまま
        """
        if s is None:
            s = ""
        if isinstance(s, (bytes, bytearray)):
            payload = bytes(s)
        else:
            payload = str(s).strip().encode("ascii", errors="ignore")

        payload += b"\r\n"

        self.pi.wave_clear()

        # pigpio の版によって offset 引数の有無があるので両対応
        try:
            # 新しめ: (gpio, baud, data_bits, stop_bits, offset, data)
            self.pi.wave_add_serial(self.tx, self.baud, DATA_BITS, STOP_BITS, 0, payload)
        except TypeError:
            # 古め: (gpio, baud, data_bits, stop_bits, data)
            self.pi.wave_add_serial(self.tx, self.baud, DATA_BITS, STOP_BITS, payload)

        wid = self.pi.wave_create()
        if wid < 0:
            raise RuntimeError("wave_create failed")

        self.pi.wave_send_once(wid)
        while self.pi.wave_tx_busy():
            time.sleep(0.001)

        self.pi.wave_delete(wid)


# ============================================================
# 9) ダウンリンク + CSV（同一内容）
# ============================================================
DOWNLINK_HZ = 1.0

DL_FIELDS = [
    "t_iso", "phase", "calib", "flip", "stuck",
    "lat", "lon", "dist_m", "goal_deg", "head_deg", "err_deg",
    "cmd_v", "cmd_turn", "nsat", "hdop", "red_ratio"
]

class DownlinkCsvLogger:
    def __init__(self, tw: SoftUartTwelite | None, csv_path: str, logger=print):
        self.tw = tw
        self.csv_path = csv_path
        self.log = logger

        os.makedirs(os.path.dirname(csv_path), exist_ok=True)
        self._fp = open(csv_path, "a", newline="", encoding="utf-8")
        self._w = csv.DictWriter(self._fp, fieldnames=DL_FIELDS)

        if self._fp.tell() == 0:
            self._w.writeheader()
            self._fp.flush()

    def close(self):
        try:
            self._fp.flush()
            self._fp.close()
        except Exception:
            pass

    @staticmethod
    def _fmt(x):
        if x is None:
            return ""
        if isinstance(x, float):
            return f"{x:.6f}"
        return str(x)

    def emit(self, row: dict):
        # 欠けているキーがあっても落ちないように埋める
        out = {k: row.get(k, "") for k in DL_FIELDS}

        # CSV
        self._w.writerow({k: self._fmt(v) for k, v in out.items()})
        self._fp.flush()

        # TWELITE
        if self.tw is not None:
            line = ",".join(self._fmt(out[k]) for k in DL_FIELDS)
            try:
                self.tw.send_line(line)
            except Exception as e:
                self.log(f"[DL] send error: {e}")


# ============================================================
# 10) GPIOパルス
# ============================================================
def gpio_pulse_high(pi: pigpio.pi, gpio_pin: int, sec: float, logger=print):
    pi.write(gpio_pin, 1)
    logger(f"[GPIO] PIN{gpio_pin} HIGH for {sec:.2f}s")
    time.sleep(sec)
    pi.write(gpio_pin, 0)
    logger(f"[GPIO] PIN{gpio_pin} LOW")


# ============================================================
# 11) フェーズ
# ============================================================
class Phase:
    DROP = "DROP"
    CALIB = "CALIB"
    GPS = "GPS"
    CAMERA = "CAMERA"
    DONE = "DONE"


# ============================================================
# 12) メイン
# ============================================================
def main():
    fall_cfg = FallConfig(dt=0.10, win=11, req=3, sea_level_hpa=1013.25)

    gps_cfg = GpsConfig(
        goal_lat=35.66059,
        goal_lon=139.36688,
        arrival_radius_m=1.0,
        angle_ok_deg=3.0,
    )

    cam_cfg = CameraConfig(w=480, h=640, show_debug=False)

    GPIO_LANDING_PIN = 17
    HEADING_EXTRA_OFFSET_DEG = 0.0

    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio daemonに接続できません。sudo pigpiod を確認してください。")

    pi.set_mode(GPIO_LANDING_PIN, pigpio.OUTPUT)
    pi.write(GPIO_LANDING_PIN, 0)

    drive = ServoDrive(pi, ServoConfig(pin18=18, pin12=12, max_delta=600))

    i2c = busio.I2C(board.SCL, board.SDA)
    bno = adafruit_bno055.BNO055_I2C(i2c, address=0x28)
    bme = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=0x76)
    bme.sea_level_pressure = fall_cfg.sea_level_hpa

    detector = FallLandingDetector(fall_cfg, logger=print)

    gps_reader = NmeaGpsReader(gps_cfg, logger=print)
    gps_reader.start()

    calib = CompassCalib(valid=False, heading_extra_offset_deg=HEADING_EXTRA_OFFSET_DEG)
    gps_guidance = GpsGuidance(gps_cfg, drive, bno, gps_reader, calib, logger=print)

    recovery = RecoveryManager(RecoveryConfig(), drive, bno, logger=print)

    cam_guidance = CameraGuidance(cam_cfg, drive, logger=print)
    picam2 = None

    # ---- TWELITE & CSV Logger ----
    tw = SoftUartTwelite(pi, tx_gpio=TWE_TX_GPIO, rx_gpio=TWE_RX_GPIO, baud=BAUD)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = f"./logs/cansat_{ts}.csv"
    dl = DownlinkCsvLogger(tw, csv_path, logger=print)
    print(f"[DL] CSV={csv_path} / TWELITE {BAUD}bps TX={TWE_TX_GPIO} RX={TWE_RX_GPIO}")

    phase = Phase.DROP
    print("=== CanSat Integrated Program Start ===")
    print(f"Phase: {phase}")

    last_dl = 0.0
    last_flip = 0
    last_stuck = 0

    try:
        while phase != Phase.DONE:
            # ---- Flip最優先 ----
            if not recovery.is_busy() and recovery.detect_flip():
                last_flip = 1
                recovery.start_recover("FLIP detected")
            if recovery.is_busy():
                recovery.tick()
                time.sleep(0.05)

            # ---- 各フェーズ処理 ----
            if phase == Phase.DROP:
                alt = bme.altitude
                acc = bno.acceleration
                st = detector.update(alt, acc)

                if st.phase == "WARMUP":
                    time.sleep(fall_cfg.dt)
                elif st.phase == "FREEFALL":
                    if st.confirmed:
                        detector.freefall_confirmed = True
                        detector.land_consec = 0
                elif st.phase == "LANDING":
                    if st.confirmed:
                        gpio_pulse_high(pi, GPIO_LANDING_PIN, 1.0, logger=print)
                        phase = Phase.CALIB
                        print(f"Phase: {phase}")
                # DROP中はGPSログは空でもOK
                time.sleep(fall_cfg.dt)

            elif phase == Phase.CALIB:
                drive.stop()
                time.sleep(0.2)
                run_compass_calibration(drive, bno, calib, duration_sec=15.0, turn_power=220, logger=print)
                phase = Phase.GPS
                print(f"Phase: {phase}")

            elif phase == Phase.GPS:
                dt = 1.0 / gps_cfg.control_hz
                t0 = time.monotonic()

                arrived, stuck = gps_guidance.step()
                last_stuck = 1 if stuck else 0
                if stuck and not recovery.is_busy():
                    recovery.start_recover("STUCK suspected (GPS progress low)")

                if arrived:
                    phase = Phase.CAMERA
                    print(f"Phase: {phase}")

                time.sleep(max(0.0, dt - (time.monotonic() - t0)))

            elif phase == Phase.CAMERA:
                if picam2 is None:
                    picam2 = Picamera2()
                    config = picam2.create_preview_configuration(
                        main={"format": "BGR888", "size": (cam_cfg.w, cam_cfg.h)}
                    )
                    picam2.configure(config)
                    picam2.start()
                    time.sleep(0.3)
                    drive.stop()
                    print("[CAM] Camera started")

                frame = picam2.capture_array("main")
                if frame is not None:
                    goal = cam_guidance.step(frame)
                    if goal:
                        phase = Phase.DONE
                        print(f"Phase: {phase}")

            # ---- 1Hz ダウンリンク + CSV ----
            now = time.monotonic()
            if (now - last_dl) >= (1.0 / DOWNLINK_HZ):
                last_dl = now

                row = {
                    "t_iso": datetime.now().isoformat(timespec="seconds"),
                    "phase": phase,
                    "calib": 1 if calib.valid else 0,
                    "flip": last_flip,
                    "stuck": last_stuck,

                    "lat": gps_guidance.last_lat,
                    "lon": gps_guidance.last_lon,
                    "dist_m": gps_guidance.last_dist,
                    "goal_deg": gps_guidance.last_goal_deg,
                    "head_deg": gps_guidance.last_head_deg,
                    "err_deg": gps_guidance.last_err_deg,
                    "cmd_v": gps_guidance.last_cmd_v,
                    "cmd_turn": gps_guidance.last_cmd_turn,
                    "nsat": gps_guidance.last_nsat,
                    "hdop": gps_guidance.last_hdop,

                    "red_ratio": cam_guidance.last_red_ratio if phase == Phase.CAMERA else None,
                }
                print("[DLDBG]", type(line), line if isinstance(line, str) else repr(line))
                dl.emit(row)

                # 1Hzフラグは送信後にクリア（瞬間イベントとして扱う）
                last_flip = 0
                last_stuck = 0

    except KeyboardInterrupt:
        print("Ctrl-C")
    finally:
        try:
            drive.stop()
            time.sleep(0.2)
            drive.free()
        except Exception:
            pass
        try:
            gps_reader.stop()
        except Exception:
            pass
        try:
            if picam2 is not None:
                picam2.stop()
        except Exception:
            pass
        try:
            dl.close()
        except Exception:
            pass
        try:
            tw.close()
        except Exception:
            pass
        try:
            pi.write(GPIO_LANDING_PIN, 0)
            pi.stop()
        except Exception:
            pass
        print("=== Finish ===")


if __name__ == "__main__":
    main()