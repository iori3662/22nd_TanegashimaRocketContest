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
import signal
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

def safe_vec3(v, default=0.0):
    if v is None:
        return (default, default, default)
    out = []
    for x in v:
        out.append(default if x is None else float(x))
    while len(out) < 3:
        out.append(default)
    return tuple(out[:3])

def safe_float(v, default=0.0):
    try:
        if v is None:
            return default
        return float(v)
    except Exception:
        return default

def nz(v, default=-9999.0):
    return default if v is None else v


# ============================================================
# 1) pigpio サーボ駆動
# ============================================================
@dataclass
class ServoConfig:
    stop_us: int = 1490
    min_us: int = 500
    max_us: int = 2500
    pin18: int = 18   # 右
    pin12: int = 12   # 左
    max_delta: int = 600

class ServoDrive:
    def __init__(self, pi: pigpio.pi, cfg: ServoConfig):
        self.pi = pi
        self.cfg = cfg
        self.last_u18 = cfg.stop_us
        self.last_u12 = cfg.stop_us
        self.last_fwd = 0
        self.last_turn = 0
        self.stop()

    def _write(self, us18, us12):
        u18 = clamp_int(us18, self.cfg.min_us, self.cfg.max_us)
        u12 = clamp_int(us12, self.cfg.min_us, self.cfg.max_us)
        self.last_u18 = u18
        self.last_u12 = u12
        self.pi.set_servo_pulsewidth(self.cfg.pin18, u18)
        self.pi.set_servo_pulsewidth(self.cfg.pin12, u12)

    def stop(self):
        self.last_fwd = 0
        self.last_turn = 0
        self._write(self.cfg.stop_us, self.cfg.stop_us)

    def free(self):
        self.pi.set_servo_pulsewidth(self.cfg.pin18, 0)
        self.pi.set_servo_pulsewidth(self.cfg.pin12, 0)

    def set_diff(self, fwd, turn):
        """
        fwd  > 0 で前進
        turn > 0 で右旋回（実機に合わせ内部で符号反転）
        """
        self.last_fwd = int(fwd)
        self.last_turn = int(turn)

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
    alt: float | None = None
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

    base_v: float = 0.70
    min_v: float = 0.35
    slowdown_dist_m: float = 8.0

    kp: float = 0.020
    turn_max: float = 0.85
    turn_bias: float = 0.00

    stuck_window_sec: float = 6.0
    stuck_min_progress_m: float = 0.6

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
        if self._th is not None:
            self._th.join(timeout=1.0)

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
                        try:
                            alt = float(getattr(msg, "altitude", 0) or 0)
                        except ValueError:
                            alt = None

                        with self._lock:
                            self._fix.fixq = fixq
                            self._fix.nsat = nsat
                            self._fix.hdop = hdop
                            self._fix.alt = alt

                time.sleep(0.02)


# ============================================================
# 4) 磁気キャリブレーション
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

def run_compass_calibration(
    drive: ServoDrive,
    bno,
    calib: CompassCalib,
    duration_sec: float = 15.0,
    turn_power: int = 220,
    logger=print,
):
    logger(f"[CAL] Compass calibration start: rotate {duration_sec:.1f}s")

    max_x = max_y = None
    min_x = min_y = None

    t0 = time.monotonic()
    while (time.monotonic() - t0) < duration_sec:
        t = time.monotonic() - t0
        sign = 1 if int(t / 2.5) % 2 == 0 else -1
        drive.set_diff(0, sign * turn_power)

        mag = None
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
    logger(f"[CAL] DONE: off_x={calib.off_x:.3f} off_y={calib.off_y:.3f} (valid={calib.valid})")


# ============================================================
# 5) リカバリ
# ============================================================
@dataclass
class RecoveryConfig:
    flip_roll_deg: float = 120.0
    flip_pitch_deg: float = 120.0

    back_ms: int = 900
    turn_ms: int = 900
    fwd_ms: int = 600
    back_power: int = -260
    turn_power: int = 320
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
            e = self.bno.euler
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
# 6) GPS誘導
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

        self.last_dist = None
        self.last_theta_goal = None
        self.last_heading = None
        self.last_err = None
        self.last_turn_cmd = 0.0
        self.last_cmd_fwd = 0.0

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

        self.last_heading = heading

        if heading is None or not self._gps_ok(fix):
            self.drive.stop()
            self.last_turn_cmd = 0.0
            self.last_cmd_fwd = 0.0
            return (False, False)

        dist = haversine_m(fix.lat, fix.lon, c.goal_lat, c.goal_lon)
        self.last_dist = dist

        if dist <= c.arrival_radius_m:
            self.drive.stop()
            self.last_turn_cmd = 0.0
            self.last_cmd_fwd = 0.0
            self.log(f"[GPS] ARRIVED dist={dist:.2f}m <= {c.arrival_radius_m}m")
            return (True, False)

        theta_goal = bearing_deg(fix.lat, fix.lon, c.goal_lat, c.goal_lon)
        err = wrap_to_180(theta_goal - heading)

        self.last_theta_goal = theta_goal
        self.last_err = err

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

        self.last_cmd_fwd = v
        self.last_turn_cmd = turn

        stuck = False
        if v >= 0.55:
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
# 7) カメラ誘導
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

        self.last_status = "INIT"
        self.last_red_ratio = 0.0
        self.last_best_area = 0.0
        self.last_err_px = 0

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
        self.last_red_ratio = red_ratio

        if red_ratio >= c.goal_red_ratio:
            if self._goal_start_t is None:
                self._goal_start_t = time.time()
            elif (time.time() - self._goal_start_t) >= c.goal_hold_sec:
                self.drive.stop()
                self.last_status = "GOAL"
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

        self.last_best_area = best_area

        if best is None:
            self.last_err_px = 0
            if red_ratio >= c.slow_red_ratio:
                self.drive.set_diff(int(c.base_fwd * 0.35), 0)
                status = "NEAR_FWD"
            else:
                self._search_scan()
                status = "SEARCH_SCAN"
            self.last_status = status
            self._debug_show(c, frame, mask, status, red_ratio, cx_frame, best_approx)
            return False

        M = cv2.moments(best)
        if M["m00"] == 0:
            self.last_err_px = 0
            if red_ratio >= c.slow_red_ratio:
                self.drive.set_diff(int(c.base_fwd * 0.35), 0)
                status = "NEAR_FWD"
            else:
                self._search_scan()
                status = "M00=0"
            self.last_status = status
            self._debug_show(c, frame, mask, status, red_ratio, cx_frame, best_approx)
            return False

        cx = int(M["m10"] / M["m00"])
        err = cx - cx_frame
        self.last_err_px = err

        if abs(err) < center_tol_px:
            if red_ratio >= c.slow_red_ratio:
                fwd = int(c.base_fwd * 0.35)
            else:
                fwd = c.base_fwd if best_area < c.area_fwd_th else int(c.base_fwd * 0.8)
            self.drive.set_diff(fwd, 0)
            status = "FORWARD"
        else:
            turn_gain = 0.55 if red_ratio >= c.slow_red_ratio else 1.0
            turn = int(turn_gain * c.kp_turn * err)
            turn = clamp_int(turn, -420, 420)
            self.drive.set_diff(0, turn)
            status = "TURN"

        self.last_status = status
        self._debug_show(c, frame, mask, status, red_ratio, cx_frame, best_approx)
        return False

    def _debug_show(self, c, frame, mask, status, red_ratio, cx_frame, best_approx):
        if not c.show_debug:
            return
        h, w = frame.shape[:2]
        disp = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        cv2.line(disp, (cx_frame, 0), (cx_frame, h), (255, 255, 255), 1)
        if best_approx is not None:
            cv2.drawContours(disp, [best_approx], -1, (0, 255, 0), 2)
        cv2.putText(disp, f"{status} red_ratio={red_ratio:.2f}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("Camera", disp)
        cv2.imshow("Mask", mask)
        cv2.waitKey(1)


# ============================================================
# 8) TWE-Lite soft UART
# ============================================================
@dataclass
class TweliteConfig:
    rx_gpio: int = 14   # Pi input  <- TWELITE TX
    tx_gpio: int = 15   # Pi output -> TWELITE RX
    baud: int = 38400
    data_bits: int = 8
    stop_half_bits: int = 2
    send_hz: float = 1.0
    tx_line_ending: bytes = b"\r\n"
    rx_buf_max: int = 4096

class TweliteSoftUART:
    def __init__(self, pi: pigpio.pi, cfg: TweliteConfig):
        self.pi = pi
        self.cfg = cfg
        self.rx_gpio = cfg.rx_gpio
        self.tx_gpio = cfg.tx_gpio
        self._rx_buf = bytearray()

        self.pi.set_mode(self.tx_gpio, pigpio.OUTPUT)
        self.pi.write(self.tx_gpio, 1)

        self.pi.set_mode(self.rx_gpio, pigpio.INPUT)
        self.pi.bb_serial_read_open(self.rx_gpio, self.cfg.baud, self.cfg.data_bits)
        time.sleep(0.05)

    def close(self):
        try:
            self.pi.bb_serial_read_close(self.rx_gpio)
        except pigpio.error:
            pass

    def write_line(self, line_str: str):
        payload = line_str.encode("ascii", errors="ignore") + self.cfg.tx_line_ending
        self.pi.wave_clear()
        self.pi.wave_add_serial(
            self.tx_gpio,
            self.cfg.baud,
            payload,
            0,
            self.cfg.data_bits,
            self.cfg.stop_half_bits
        )
        wid = self.pi.wave_create()
        if wid < 0:
            raise RuntimeError("wave_create failed")

        self.pi.wave_send_once(wid)
        while self.pi.wave_tx_busy():
            time.sleep(0.001)
        self.pi.wave_delete(wid)

    def _read_bytes(self) -> bytes:
        cnt, data = self.pi.bb_serial_read(self.rx_gpio)
        if cnt > 0 and data:
            return bytes(data)
        return b""

    def read_lines(self):
        chunk = self._read_bytes()
        if chunk:
            self._rx_buf.extend(chunk)
            if len(self._rx_buf) > self.cfg.rx_buf_max:
                self._rx_buf = self._rx_buf[-self.cfg.rx_buf_max:]

        lines = []
        while b"\n" in self._rx_buf:
            raw, _, rest = self._rx_buf.partition(b"\n")
            self._rx_buf = bytearray(rest)
            raw = raw.rstrip(b"\r")
            lines.append(raw.decode("ascii", errors="replace").strip())
        return lines


# ============================================================
# 9) テレメトリ共有
# ============================================================
class TelemetryStore:
    def __init__(self):
        self._lock = threading.Lock()
        self._data = {
            "phase": "INIT",
            "timestamp": time.monotonic(),

            "bme_alt": None,
            "temp": None,
            "press": None,
            "humid": None,

            "heading": None,
            "roll": None,
            "pitch": None,
            "gx": None,
            "gy": None,
            "gz": None,
            "ax": None,
            "ay": None,
            "az": None,
            "mx": None,
            "my": None,
            "mz": None,

            "gps_lat": None,
            "gps_lon": None,
            "gps_alt": None,
            "fixq": 0,
            "nsat": 0,
            "hdop": None,

            "dist_m": None,
            "theta_goal": None,
            "yaw_err": None,

            "cmd_fwd": 0,
            "cmd_turn": 0,
            "u18": None,
            "u12": None,

            "fall_phase": "",
            "freefall_confirmed": 0,
            "landing_confirmed": 0,

            "recovery_busy": 0,
            "cam_status": "",
            "cam_red_ratio": 0.0,
            "cam_area": 0.0,
            "cam_err_px": 0,
        }

    def update(self, **kwargs):
        with self._lock:
            self._data.update(kwargs)
            self._data["timestamp"] = time.monotonic()

    def snapshot(self):
        with self._lock:
            return dict(self._data)


# ============================================================
# 10) ダウンリンク管理
# ============================================================
class DownlinkManager:
    def __init__(self, tw: TweliteSoftUART, telem: TelemetryStore, cfg: TweliteConfig, logger=print):
        self.tw = tw
        self.telem = telem
        self.cfg = cfg
        self.log = logger
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
        if self._th is not None:
            self._th.join(timeout=1.5)

    def _build_line(self, d: dict) -> str:
        line = (
            "STAT,"
            f"{d['phase']},"
            f"{nz(d['gps_lat']):.8f},{nz(d['gps_lon']):.8f},{nz(d['gps_alt']):.1f},"
            f"{int(d['fixq'])},{int(d['nsat'])},{nz(d['hdop']):.2f},"
            f"{nz(d['bme_alt']):.2f},{nz(d['temp']):.2f},{nz(d['press']):.2f},{nz(d['humid']):.2f},"
            f"{nz(d['heading']):.2f},{nz(d['roll']):.2f},{nz(d['pitch']):.2f},"
            f"{nz(d['gx']):.4f},{nz(d['gy']):.4f},{nz(d['gz']):.4f},"
            f"{nz(d['ax']):.4f},{nz(d['ay']):.4f},{nz(d['az']):.4f},"
            f"{nz(d['mx']):.4f},{nz(d['my']):.4f},{nz(d['mz']):.4f},"
            f"{nz(d['dist_m']):.2f},{nz(d['theta_goal']):.2f},{nz(d['yaw_err']):.2f},"
            f"{int(d['cmd_fwd'])},{int(d['cmd_turn'])},{int(nz(d['u18'], 0))},{int(nz(d['u12'], 0))},"
            f"{d['fall_phase']},{int(d['freefall_confirmed'])},{int(d['landing_confirmed'])},"
            f"{int(d['recovery_busy'])},"
            f"{d['cam_status']},{nz(d['cam_red_ratio'], 0.0):.3f},{nz(d['cam_area'], 0.0):.1f},{int(nz(d['cam_err_px'], 0))}"
        )
        return line

    def _run(self):
        next_send = time.monotonic()
        while not self._stop_event.is_set():
            now = time.monotonic()

            try:
                for text in self.tw.read_lines():
                    if text:
                        self.log(f"[TWE RX] {text}")
            except Exception as e:
                self.log(f"[TWE RX ERR] {e}")

            if now >= next_send:
                next_send += 1.0 / self.cfg.send_hz
                d = self.telem.snapshot()
                line = self._build_line(d)
                try:
                    self.tw.write_line(line)
                    self.log(line)
                except Exception as e:
                    self.log(f"[TWE TX ERR] {e}")

            time.sleep(0.01)
   
# ============================================================
# BLE (Nordic UART for XIAO remote)
# ============================================================
UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
UART_RX_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  # Pi -> XIAO (Write)
UART_TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # XIAO -> Pi (Notify)

XIAO_NAME = "CanSat-Remote"

MODE_IDLE = "idle"
MODE_DEBUG = "debug"
MODE_RUN = "run"

BLE_TELEM_HZ = 10.0
BLE_LOG_DIR = "./logs_ble"
os.makedirs(BLE_LOG_DIR, exist_ok=True)

BLE_CSV_HEADER = (
    "seq,ph,yaw_deg,pitch_deg,roll_deg,"
    "lat_deg,lon_deg,alt_gps_m,"
    "temp_C,press_hPa,hum_pct,alt_bme_m,"
    "ax,ay,az,a"
)

@dataclass
class BleControlState:
    mode: str = MODE_RUN
    pi_logging: bool = True
    ble_connected: bool = False
    seq: int = 0
    requested_phase_idx: int | None = None
    last_cmd_ts: float = 0.0
    _lock: threading.Lock = threading.Lock()

    def set_mode(self, mode: str):
        with self._lock:
            self.mode = mode
            self.last_cmd_ts = time.time()

    def get_mode(self) -> str:
        with self._lock:
            return self.mode

    def toggle_logging(self) -> bool:
        with self._lock:
            self.pi_logging = not self.pi_logging
            self.last_cmd_ts = time.time()
            return self.pi_logging

    def get_logging(self) -> bool:
        with self._lock:
            return self.pi_logging

    def request_phase(self, idx: int):
        with self._lock:
            self.requested_phase_idx = max(0, min(4, int(idx)))
            self.last_cmd_ts = time.time()

    def consume_phase_request(self) -> int | None:
        with self._lock:
            v = self.requested_phase_idx
            self.requested_phase_idx = None
            return v

    def next_seq(self) -> int:
        with self._lock:
            self.seq += 1
            return self.seq

    def set_ble_connected(self, connected: bool):
        with self._lock:
            self.ble_connected = connected
            
def handle_ble_cmd(ctrl: BleControlState, current_phase: str, line: str, logger=print):
    line = line.strip()
    if not line:
        return
    if not line.startswith("cmd,"):
        logger(f"[BLE CMD] ignored: {line}")
        return

    parts = line.split(",", 2)
    if len(parts) != 3:
        logger(f"[BLE CMD] bad: {line}")
        return

    _, name, arg = parts

    if name == "mode":
        if arg in (MODE_IDLE, MODE_DEBUG, MODE_RUN):
            ctrl.set_mode(arg)
            logger(f"[BLE CMD] mode -> {arg}")
        else:
            logger(f"[BLE CMD] invalid mode: {arg}")
        return

    cur_idx = phase_to_index(current_phase)

    if name == "phase":
        if arg == "inc":
            ctrl.request_phase(cur_idx + 1)
            logger(f"[BLE CMD] phase request -> {cur_idx + 1}")
        elif arg == "dec":
            ctrl.request_phase(cur_idx - 1)
            logger(f"[BLE CMD] phase request -> {cur_idx - 1}")
        elif arg.startswith("set:"):
            try:
                v = int(arg.split(":", 1)[1])
                ctrl.request_phase(v)
                logger(f"[BLE CMD] phase request -> {v}")
            except ValueError:
                logger(f"[BLE CMD] invalid phase set: {arg}")
        else:
            logger(f"[BLE CMD] invalid phase arg: {arg}")
        return

    if name == "log":
        if arg == "toggle":
            new_state = ctrl.toggle_logging()
            logger(f"[BLE CMD] pi_logging -> {new_state}")
        else:
            logger(f"[BLE CMD] invalid log arg: {arg}")
        return

    logger(f"[BLE CMD] unknown: {name},{arg}")
    
def build_ble_csv_line(ctrl: BleControlState, telem_snapshot: dict) -> str:
    seq = ctrl.next_seq()
    ph = phase_to_index(telem_snapshot.get("phase", Phase.DROP))

    yaw = safe_float(telem_snapshot.get("heading"), float("nan"))
    pitch = safe_float(telem_snapshot.get("pitch"), float("nan"))
    roll = safe_float(telem_snapshot.get("roll"), float("nan"))

    lat = telem_snapshot.get("gps_lat")
    lon = telem_snapshot.get("gps_lon")
    alt_gps = telem_snapshot.get("gps_alt")

    temp = telem_snapshot.get("temp")
    press = telem_snapshot.get("press")
    hum = telem_snapshot.get("humid")
    altb = telem_snapshot.get("bme_alt")

    ax = telem_snapshot.get("ax")
    ay = telem_snapshot.get("ay")
    az = telem_snapshot.get("az")

    if ax is None or ay is None or az is None:
        a = float("nan")
    else:
        try:
            a = math.sqrt(float(ax) ** 2 + float(ay) ** 2 + float(az) ** 2)
        except Exception:
            a = float("nan")

    def fmt_num(x, nd):
        if x is None:
            return "nan"
        try:
            xf = float(x)
            if math.isnan(xf):
                return "nan"
            return f"{xf:.{nd}f}"
        except Exception:
            return "nan"

    return ",".join([
        str(seq),
        str(ph),
        fmt_num(yaw, 1),
        fmt_num(pitch, 1),
        fmt_num(roll, 1),
        fmt_num(lat, 8),
        fmt_num(lon, 8),
        fmt_num(alt_gps, 1),
        fmt_num(temp, 2),
        fmt_num(press, 2),
        fmt_num(hum, 2),
        fmt_num(altb, 2),
        fmt_num(ax, 3),
        fmt_num(ay, 3),
        fmt_num(az, 3),
        fmt_num(a, 3),
    ])
    
def open_new_ble_log():
    ts = time.strftime("%Y%m%d_%H%M%S")
    path = os.path.join(BLE_LOG_DIR, f"ble_pi_log_{ts}.csv")
    f = open(path, "w", buffering=1)
    f.write(BLE_CSV_HEADER + "\n")
    return path, f

class BleRemoteManager:
    def __init__(self, telem: TelemetryStore, ctrl: BleControlState, logger=print):
        self.telem = telem
        self.ctrl = ctrl
        self.log = logger
        self._stop_evt = threading.Event()
        self._thread = None
        self._current_phase_getter = None

    def set_phase_getter(self, getter):
        self._current_phase_getter = getter

    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._thread_main, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _thread_main(self):
        try:
            asyncio.run(self._runner())
        except Exception as e:
            self.log(f"[BLE] thread error: {e}")

    async def _find_xiao_addr(self, timeout: float = 6.0) -> Optional[str]:
        devs = await BleakScanner.discover(timeout=timeout)
        for d in devs:
            name = (d.name or "").strip()
            if name == XIAO_NAME:
                return d.address
        return None

    async def _runner(self):
        log_path, pi_log = open_new_ble_log()
        self.log(f"[BLE] pi local log: {log_path}")

        async def on_notify(_, data: bytearray):
            try:
                s = data.decode("utf-8", errors="ignore").strip()
            except Exception:
                return
            if not s:
                return

            cur_phase = Phase.DROP
            if self._current_phase_getter is not None:
                try:
                    cur_phase = self._current_phase_getter()
                except Exception:
                    pass

            for line in s.splitlines():
                handle_ble_cmd(self.ctrl, cur_phase, line, logger=self.log)

        backoff = 0.5
        backoff_max = 8.0

        while not self._stop_evt.is_set():
            addr = await self._find_xiao_addr()
            if not addr:
                self.ctrl.set_ble_connected(False)
                self.log("[BLE] XIAO not found")
                await asyncio.sleep(1.0)
                continue

            self.log(f"[BLE] found {XIAO_NAME} addr={addr}")

            try:
                async with BleakClient(addr) as client:
                    self.ctrl.set_ble_connected(True)
                    self.log("[BLE] connected")

                    await client.start_notify(UART_TX_UUID, on_notify)
                    self.log("[BLE] notify started")
                    await asyncio.sleep(1.0)

                    next_tick = time.monotonic()
                    dt = 1.0 / BLE_TELEM_HZ
                    backoff = 0.5

                    while client.is_connected and (not self._stop_evt.is_set()):
                        now = time.monotonic()
                        if now >= next_tick:
                            next_tick += dt

                            snap = self.telem.snapshot()
                            line = build_ble_csv_line(self.ctrl, snap)

                            if self.ctrl.get_logging():
                                pi_log.write(line + "\n")

                            try:
                                await client.write_gatt_char(
                                    UART_RX_UUID,
                                    (line + "\n").encode("utf-8"),
                                    response=False
                                )
                            except (BleakError, OSError) as e:
                                self.log(f"[BLE] write error: {e}")
                                break

                        await asyncio.sleep(0.001)

                    self.log("[BLE] disconnected (inner loop end)")
                    self.ctrl.set_ble_connected(False)

                    try:
                        await client.stop_notify(UART_TX_UUID)
                    except Exception:
                        pass

            except Exception as e:
                self.ctrl.set_ble_connected(False)
                self.log(f"[BLE] connect/session error: {e}")

            await asyncio.sleep(backoff)
            backoff = min(backoff_max, backoff * 2)

        try:
            pi_log.flush()
            pi_log.close()
        except Exception:
            pass

# ============================================================
# 11) フェーズ
# ============================================================
class Phase:
    DROP = "DROP"
    CALIB = "CALIB"
    GPS = "GPS"
    CAMERA = "CAMERA"
    DONE = "DONE"

def phase_to_index(phase: str) -> int:
    if phase == Phase.DROP:
        return 0
    if phase == Phase.CALIB:
        return 1
    if phase == Phase.GPS:
        return 2
    if phase == Phase.CAMERA:
        return 3
    if phase == Phase.DONE:
        return 4
    return 0

def index_to_phase(idx: int) -> str:
    idx = max(0, min(4, int(idx)))
    if idx == 0:
        return Phase.DROP
    if idx == 1:
        return Phase.CALIB
    if idx == 2:
        return Phase.GPS
    if idx == 3:
        return Phase.CAMERA
    return Phase.DONE

# ============================================================
# 12) メイン補助
# ============================================================
def gpio_pulse_high(pi: pigpio.pi, gpio_pin: int, sec: float, logger=print):
    pi.write(gpio_pin, 1)
    logger(f"[GPIO] PIN{gpio_pin} HIGH for {sec:.2f}s")
    time.sleep(sec)
    pi.write(gpio_pin, 0)
    logger(f"[GPIO] PIN{gpio_pin} LOW")

def update_basic_telem(telem: TelemetryStore, phase: str, bno, bme, gps_reader: NmeaGpsReader,
                       drive: ServoDrive, recovery: RecoveryManager, calib: CompassCalib,
                       ble_ctrl: BleControlState | None = None):
    try:
        euler = bno.euler
    except Exception:
        euler = None
    try:
        gyro = bno.gyro
    except Exception:
        gyro = None
    try:
        accel = bno.acceleration
    except Exception:
        accel = None
    try:
        mag = bno.magnetic
    except Exception:
        mag = None

    euler = safe_vec3(euler)
    gyro = safe_vec3(gyro)
    accel = safe_vec3(accel)
    mag = safe_vec3(mag)

    heading, roll, pitch = euler
    gx, gy, gz = gyro
    ax, ay, az = accel
    mx, my, mz = mag

    try:
        temp = float(bme.temperature)
    except Exception:
        temp = None
    try:
        press = float(bme.pressure)
    except Exception:
        press = None
    try:
        humid = float(bme.humidity)
    except Exception:
        humid = None
    try:
        bme_alt = float(bme.altitude)
    except Exception:
        bme_alt = None

    fix = gps_reader.get()

    telem.update(
        phase=phase,
        bme_alt=bme_alt,
        temp=temp,
        press=press,
        humid=humid,
        heading=heading,
        roll=roll,
        pitch=pitch,
        gx=gx, gy=gy, gz=gz,
        ax=ax, ay=ay, az=az,
        mx=mx, my=my, mz=mz,
        gps_lat=fix.lat,
        gps_lon=fix.lon,
        gps_alt=fix.alt,
        fixq=fix.fixq,
        nsat=fix.nsat,
        hdop=fix.hdop,
        cmd_fwd=drive.last_fwd,
        cmd_turn=drive.last_turn,
        u18=drive.last_u18,
        u12=drive.last_u12,
        recovery_busy=int(recovery.is_busy()),
        ble_mode=ble_ctrl.get_mode() if ble_ctrl is not None else MODE_RUN,
    )


# ============================================================
# 13) メイン
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

    twe_cfg = TweliteConfig(
        rx_gpio=14,
        tx_gpio=15,
        baud=38400,
        send_hz=1.0,
    )

    GPIO_LANDING_PIN = 17
    HEADING_EXTRA_OFFSET_DEG = 0.0

    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio daemonに接続できません。sudo pigpiod を確認してください。")

    pi.set_mode(GPIO_LANDING_PIN, pigpio.OUTPUT)
    pi.write(GPIO_LANDING_PIN, 0)

    servo_cfg = ServoConfig(stop_us=1490, min_us=500, max_us=2500, pin18=18, pin12=12, max_delta=500)
    drive = ServoDrive(pi, servo_cfg)

    tw = None
    downlink = None
    picam2 = None

    i2c = busio.I2C(board.SCL, board.SDA)
    bno = adafruit_bno055.BNO055_I2C(i2c, address=0x28)
    bme = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=0x76)
    bme.sea_level_pressure = fall_cfg.sea_level_hpa

    detector = FallLandingDetector(fall_cfg, logger=print)

    gps_reader = NmeaGpsReader(gps_cfg, logger=print)
    gps_reader.start()

    calib = CompassCalib(valid=False, heading_extra_offset_deg=HEADING_EXTRA_OFFSET_DEG)
    gps_guidance = GpsGuidance(gps_cfg, drive, bno, gps_reader, calib, logger=print)

    rec_cfg = RecoveryConfig()
    recovery = RecoveryManager(rec_cfg, drive, bno, logger=print)

    cam_guidance = CameraGuidance(cam_cfg, drive, logger=print)

    telem = TelemetryStore()
    
    ble_ctrl = BleControlState(mode=MODE_RUN, pi_logging=True)
    ble_remote = BleRemoteManager(telem, ble_ctrl, logger=print)

    try:
        tw = TweliteSoftUART(pi, twe_cfg)
        print(f"[TWE] soft UART open: RX=GPIO{twe_cfg.rx_gpio}, TX=GPIO{twe_cfg.tx_gpio}, {twe_cfg.baud}bps")
        downlink = DownlinkManager(tw, telem, twe_cfg, logger=print)
        downlink.start()
    except Exception as e:
        print(f"[TWE] 初期化失敗: {e}")
        print("[TWE] ダウンリンクなしで続行します。")
        tw = None
        downlink = None

    phase = Phase.DROP
    telem.update(phase=phase)
    ble_remote.set_phase_getter(lambda: phase)
    ble_remote.start()
    print("=== CanSat Integrated Program Start ===")
    print(f"Phase: {phase}")

    try:
        while phase != Phase.DONE:
            update_basic_telem(telem, phase, bno, bme, gps_reader, drive, recovery, calib, ble_ctrl)
                        # ---- BLE remote requested phase change ----
            req_idx = ble_ctrl.consume_phase_request()
            if req_idx is not None:
                new_phase = index_to_phase(req_idx)
                if new_phase != phase:
                    print(f"[BLE] force phase: {phase} -> {new_phase}")
                    drive.stop()

                    if picam2 is not None and new_phase != Phase.CAMERA:
                        try:
                            picam2.stop()
                        except Exception:
                            pass
                        picam2 = None

                    phase = new_phase
                    telem.update(phase=phase)

            # ---- BLE mode handling ----
            ble_mode = ble_ctrl.get_mode()
            telem.update(ble_mode=ble_mode)

            # idle/debug では post-landing phase を停止して待機
            if ble_mode in (MODE_IDLE, MODE_DEBUG) and phase in (Phase.CALIB, Phase.GPS, Phase.CAMERA):
                drive.stop()
                telem.update(
                    phase=phase,
                    cmd_fwd=drive.last_fwd,
                    cmd_turn=drive.last_turn,
                    u18=drive.last_u18,
                    u12=drive.last_u12,
                )
                time.sleep(0.05)
                continue

            if not recovery.is_busy() and recovery.detect_flip():
                recovery.start_recover("FLIP detected")

            if recovery.is_busy():
                recovery.tick()
                telem.update(
                    phase=phase,
                    recovery_busy=1,
                    cmd_fwd=drive.last_fwd,
                    cmd_turn=drive.last_turn,
                    u18=drive.last_u18,
                    u12=drive.last_u12,
                )
                time.sleep(0.05)
                continue

            if phase == Phase.DROP:
                alt = safe_float(bme.altitude, 0.0)
                acc = bno.acceleration
                st = detector.update(alt, acc)

                telem.update(
                    phase=phase,
                    bme_alt=alt,
                    fall_phase=st.phase,
                    freefall_confirmed=int(detector.freefall_confirmed),
                    landing_confirmed=int(st.confirmed if st.phase == "LANDING" else 0),
                )

                if st.phase == "WARMUP":
                    time.sleep(fall_cfg.dt)
                    continue

                if st.phase == "FREEFALL":
                    print(f"[DROP] FREEFALL? alt={st.alt:.2f} acc_med={st.acc_med:.2f} "
                          f"dalt_med={st.dalt_med:.3f} cond={st.cond} consec={st.consec}")
                    if st.confirmed:
                        detector.freefall_confirmed = True
                        detector.land_consec = 0
                        telem.update(freefall_confirmed=1)
                        print("[DROP] FREEFALL CONFIRMED")

                elif st.phase == "LANDING":
                    print(f"[DROP] LAND? alt={st.alt:.2f} acc_med={st.acc_med:.2f} "
                          f"dalt_med={st.dalt_med:.3f} cond={st.cond} consec={st.consec}")
                    if st.confirmed:
                        telem.update(landing_confirmed=1)
                        print("[DROP] LANDING CONFIRMED")
                        gpio_pulse_high(pi, GPIO_LANDING_PIN, 1.0, logger=print)

                        # 着地直後に2輪を前進側へ3秒回して展開/離脱動作
                        drive.set_diff(350, 0)
                        print("[DROP] drive forward 3s after landing")
                        time.sleep(3.0)
                        drive.stop()

                        phase = Phase.CALIB
                        telem.update(phase=phase)
                        print(f"Phase: {phase}")

                time.sleep(fall_cfg.dt)
                continue

            if phase == Phase.CALIB:
                drive.stop()
                telem.update(phase=phase, fall_phase="CALIB")
                time.sleep(0.2)

                run_compass_calibration(
                    drive=drive,
                    bno=bno,
                    calib=calib,
                    duration_sec=15.0,
                    turn_power=220,
                    logger=print,
                )

                phase = Phase.GPS
                telem.update(phase=phase)
                print(f"Phase: {phase}")
                continue

            if phase == Phase.GPS:
                dt = 1.0 / gps_cfg.control_hz
                t0 = time.monotonic()

                arrived, stuck = gps_guidance.step()

                telem.update(
                    phase=phase,
                    dist_m=gps_guidance.last_dist,
                    theta_goal=gps_guidance.last_theta_goal,
                    yaw_err=gps_guidance.last_err,
                    heading=gps_guidance.last_heading,
                    cmd_fwd=drive.last_fwd,
                    cmd_turn=drive.last_turn,
                    u18=drive.last_u18,
                    u12=drive.last_u12,
                )

                if stuck and not recovery.is_busy():
                    recovery.start_recover("STUCK suspected (GPS progress low)")

                if arrived:
                    phase = Phase.CAMERA
                    telem.update(phase=phase)
                    print(f"Phase: {phase}")

                time.sleep(max(0.0, dt - (time.monotonic() - t0)))
                continue

            if phase == Phase.CAMERA:
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
                if frame is None:
                    continue

                goal = cam_guidance.step(frame)

                telem.update(
                    phase=phase,
                    cam_status=cam_guidance.last_status,
                    cam_red_ratio=cam_guidance.last_red_ratio,
                    cam_area=cam_guidance.last_best_area,
                    cam_err_px=cam_guidance.last_err_px,
                    cmd_fwd=drive.last_fwd,
                    cmd_turn=drive.last_turn,
                    u18=drive.last_u18,
                    u12=drive.last_u12,
                )

                if goal:
                    phase = Phase.DONE
                    telem.update(phase=phase)
                    print(f"Phase: {phase}")
                continue

    except KeyboardInterrupt:
        print("Ctrl-C")
    finally:
        try:
            if downlink is not None:
                downlink.stop()
        except Exception:
            pass
        try:
            if tw is not None:
                tw.close()
        except Exception:
            pass
        try:
            ble_remote.stop()
        except Exception:
            pass
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
            if cam_cfg.show_debug:
                cv2.destroyAllWindows()
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