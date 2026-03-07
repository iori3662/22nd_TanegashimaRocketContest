#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import time
import math
import statistics
import threading
import csv
import os
import sys
from datetime import datetime
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


# ============================================================
# 方位設定
# ============================================================
DECLINATION_DEG = 0.0  # 現状未使用


# ============================================================
# TWELITE soft UART 設定
# ============================================================
TWELITE_RX_GPIO = 14   # Pi input  (connected to TWELITE_TX)
TWELITE_TX_GPIO = 15   # Pi output (connected to TWELITE_RX)

TWELITE_BAUD = 38400
TWELITE_DATA_BITS = 8
TWELITE_STOP_HALF_BITS = 2   # 2 => 1 stop bit
TX_LINE_ENDING = b"\r\n"
RX_BUF_MAX = 4096

DOWNLINK_HZ = 1.0


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

def safe_read_sensor(fn):
    try:
        return fn()
    except Exception:
        return None


# ============================================================
# 0.5) CSVロガー
# ============================================================
class CsvLogger:
    def __init__(self, out_dir="logs"):
        os.makedirs(out_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(out_dir, f"cansat_log_{ts}.csv")
        self.fp = open(self.path, "w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.fp)

        self.writer.writerow([
            "pc_time",
            "mono_time",
            "elapsed_sec",
            "kind",
            "phase",
            "message",

            "alt_m",
            "acc_x", "acc_y", "acc_z", "acc_norm",
            "mag_x", "mag_y", "mag_z",
            "euler_heading", "euler_roll", "euler_pitch",

            "drop_subphase",
            "drop_acc_med",
            "drop_dalt_med",
            "drop_cond",
            "drop_consec",
            "drop_confirmed",
            "drop_armed",
            "drop_baseline_alt",
            "drop_max_alt",

            "gps_lat", "gps_lon", "gps_fixq", "gps_nsat", "gps_hdop",
            "gps_dist_m", "gps_goal_bearing_deg", "gps_heading_deg", "gps_err_deg",

            "servo_fwd_cmd", "servo_turn_cmd",
            "servo_us18", "servo_us12",

            "recovery_mode",
            "recovery_busy",
            "camera_red_ratio",
            "camera_status"
        ])
        self.fp.flush()

    def log_event(self, msg: str, phase: str = "", elapsed_sec=None):
        now = datetime.now().isoformat(timespec="seconds")
        mono = time.monotonic()
        print(msg)
        self.writer.writerow([
            now, f"{mono:.3f}", elapsed_sec, "EVENT", phase, msg,
            "", "", "", "", "", "", "", "", "", "", "",
            "", "", "", "", "", "", "", "",
            "", "", "", "", "",
            "", "", "", "",
            "", "", "", "",
            "", "",
            "", ""
        ])
        self.fp.flush()

    def log_row(
        self,
        phase="",
        message="",
        elapsed_sec=None,
        alt_m=None,
        acc=None,
        mag=None,
        euler=None,
        drop_subphase="",
        drop_acc_med=None,
        drop_dalt_med=None,
        drop_cond=None,
        drop_consec=None,
        drop_confirmed=None,
        drop_armed=None,
        drop_baseline_alt=None,
        drop_max_alt=None,
        gps_fix=None,
        gps_dist_m=None,
        gps_goal_bearing_deg=None,
        gps_heading_deg=None,
        gps_err_deg=None,
        servo_fwd_cmd=None,
        servo_turn_cmd=None,
        servo_us18=None,
        servo_us12=None,
        recovery_mode="",
        recovery_busy=None,
        camera_red_ratio=None,
        camera_status=""
    ):
        now = datetime.now().isoformat(timespec="seconds")
        mono = time.monotonic()

        acc_x = acc_y = acc_z = acc_norm = ""
        if acc is not None and len(acc) == 3:
            acc_x, acc_y, acc_z = acc
            acc_norm = norm3(acc)

        mag_x = mag_y = mag_z = ""
        if mag is not None and len(mag) == 3:
            mag_x, mag_y, mag_z = mag

        euler_heading = euler_roll = euler_pitch = ""
        if euler is not None and len(euler) == 3:
            euler_heading, euler_roll, euler_pitch = euler

        gps_lat = gps_lon = gps_fixq = gps_nsat = gps_hdop = ""
        if gps_fix is not None:
            gps_lat = gps_fix.lat
            gps_lon = gps_fix.lon
            gps_fixq = gps_fix.fixq
            gps_nsat = gps_fix.nsat
            gps_hdop = gps_fix.hdop

        self.writer.writerow([
            now, f"{mono:.3f}", elapsed_sec, "ROW", phase, message,

            alt_m,
            acc_x, acc_y, acc_z, acc_norm,
            mag_x, mag_y, mag_z,
            euler_heading, euler_roll, euler_pitch,

            drop_subphase,
            drop_acc_med,
            drop_dalt_med,
            drop_cond,
            drop_consec,
            drop_confirmed,
            drop_armed,
            drop_baseline_alt,
            drop_max_alt,

            gps_lat, gps_lon, gps_fixq, gps_nsat, gps_hdop,
            gps_dist_m, gps_goal_bearing_deg, gps_heading_deg, gps_err_deg,

            servo_fwd_cmd, servo_turn_cmd,
            servo_us18, servo_us12,

            recovery_mode,
            recovery_busy,
            camera_red_ratio,
            camera_status
        ])
        self.fp.flush()

    def close(self):
        try:
            self.fp.flush()
            self.fp.close()
        except Exception:
            pass


# ============================================================
# 0.6) TWELITE soft UART
# ============================================================
class TweliteSoftUART:
    """
    pigpio-based soft UART:
      - TX: wave_add_serial (DMA)
      - RX: bb_serial_read (bit-bang)
    """
    def __init__(self, pi: pigpio.pi, rx_gpio: int, tx_gpio: int, baud: int):
        self.pi = pi
        self.rx_gpio = rx_gpio
        self.tx_gpio = tx_gpio
        self.baud = baud
        self._rx_buf = bytearray()

        self.pi.set_mode(self.tx_gpio, pigpio.OUTPUT)
        self.pi.write(self.tx_gpio, 1)

        self.pi.set_mode(self.rx_gpio, pigpio.INPUT)
        self.pi.bb_serial_read_open(self.rx_gpio, self.baud, TWELITE_DATA_BITS)
        time.sleep(0.05)

    def close(self):
        try:
            self.pi.bb_serial_read_close(self.rx_gpio)
        except pigpio.error:
            pass

    def write_line(self, line_str: str):
        payload = line_str.encode("ascii", errors="ignore") + TX_LINE_ENDING

        self.pi.wave_clear()
        self.pi.wave_add_serial(
            self.tx_gpio,
            self.baud,
            payload,
            0,
            TWELITE_DATA_BITS,
            TWELITE_STOP_HALF_BITS
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
            if len(self._rx_buf) > RX_BUF_MAX:
                self._rx_buf = self._rx_buf[-RX_BUF_MAX:]

        lines = []
        while b"\n" in self._rx_buf:
            raw, _, rest = self._rx_buf.partition(b"\n")
            self._rx_buf = bytearray(rest)
            raw = raw.rstrip(b"\r")
            lines.append(raw.decode("ascii", errors="replace").strip())
        return lines


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
        self.last_us18 = cfg.stop_us
        self.last_us12 = cfg.stop_us
        self.last_fwd = 0
        self.last_turn = 0
        self.stop()

    def _write(self, us18, us12):
        u18 = clamp_int(us18, self.cfg.min_us, self.cfg.max_us)
        u12 = clamp_int(us12, self.cfg.min_us, self.cfg.max_us)
        self.last_us18 = u18
        self.last_us12 = u12
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
# 2) 落下検知＋着地判定（30mパラシュート降下向け安全側版）
# ============================================================
@dataclass
class FallConfig:
    dt: float = 0.10

    descent_win: int = 7
    descent_req: int = 4
    descent_dalt_th: float = -0.20
    descent_from_peak_m: float = 1.5

    land_win: int = 11
    land_req: int = 8
    land_dalt_abs_th: float = 0.06
    land_alt_span_th: float = 0.30
    acc_land_min: float = 8.3
    acc_land_max: float = 11.3

    arm_alt_rise_m: float = 5.0
    arm_min_time_sec: float = 2.0

    sea_level_hpa: float = 1013.25

@dataclass
class FallStatus:
    phase: str
    alt: float | None = None
    acc_med: float | None = None
    dalt_med: float | None = None
    cond: bool | None = None
    consec: int = 0
    confirmed: bool = False
    armed: bool = False
    baseline_alt: float | None = None
    max_alt: float | None = None

class FallLandingDetector:
    def __init__(self, cfg: FallConfig, logger=print):
        self.cfg = cfg
        self.log = logger

        self.prev_alt = None
        self.t0 = time.monotonic()

        self.baseline_alt = None
        self.max_alt = None
        self.armed = False

        self.descent_confirmed = False
        self.descent_consec = 0
        self.land_consec = 0

        self.buf_acc = deque(maxlen=max(cfg.descent_win, cfg.land_win))
        self.buf_dalt_desc = deque(maxlen=cfg.descent_win)
        self.buf_dalt_land = deque(maxlen=cfg.land_win)
        self.buf_alt_land = deque(maxlen=cfg.land_win)

    def update(self, alt_m: float, acc_vec):
        c = self.cfg

        if alt_m is None:
            return FallStatus(
                phase="WARMUP",
                alt=alt_m,
                armed=self.armed,
                baseline_alt=self.baseline_alt,
                max_alt=self.max_alt,
            )

        if self.baseline_alt is None:
            self.baseline_alt = alt_m
            self.max_alt = alt_m

        self.max_alt = max(self.max_alt, alt_m)

        if self.prev_alt is None:
            self.prev_alt = alt_m
            return FallStatus(
                phase="WARMUP",
                alt=alt_m,
                armed=self.armed,
                baseline_alt=self.baseline_alt,
                max_alt=self.max_alt,
            )

        dalt = alt_m - self.prev_alt
        self.prev_alt = alt_m

        acc_n = norm3(acc_vec)
        if acc_n is not None:
            self.buf_acc.append(acc_n)

        self.buf_dalt_desc.append(dalt)
        self.buf_dalt_land.append(dalt)
        self.buf_alt_land.append(alt_m)

        dalt_desc_med = median_or_none(self.buf_dalt_desc)
        dalt_land_med = median_or_none(self.buf_dalt_land)
        acc_med = statistics.median(self.buf_acc) if len(self.buf_acc) >= 3 else None
        alt_span = None
        if len(self.buf_alt_land) == self.buf_alt_land.maxlen:
            alt_span = max(self.buf_alt_land) - min(self.buf_alt_land)

        if (not self.armed) and (self.baseline_alt is not None):
            risen = alt_m - self.baseline_alt
            elapsed = time.monotonic() - self.t0
            if (risen >= c.arm_alt_rise_m) and (elapsed >= c.arm_min_time_sec):
                self.armed = True
                self.log(f"[DROP] ARMING ON: risen={risen:.2f}m elapsed={elapsed:.1f}s")

        if dalt_desc_med is None or dalt_land_med is None:
            return FallStatus(
                phase="WARMUP",
                alt=alt_m,
                acc_med=acc_med,
                dalt_med=None,
                armed=self.armed,
                baseline_alt=self.baseline_alt,
                max_alt=self.max_alt,
            )

        if not self.descent_confirmed:
            if not self.armed:
                return FallStatus(
                    phase="ARM_WAIT",
                    alt=alt_m,
                    acc_med=acc_med,
                    dalt_med=dalt_desc_med,
                    cond=False,
                    consec=0,
                    confirmed=False,
                    armed=self.armed,
                    baseline_alt=self.baseline_alt,
                    max_alt=self.max_alt,
                )

            drop_from_peak = self.max_alt - alt_m
            descent_cond = (
                (drop_from_peak >= c.descent_from_peak_m) and
                (dalt_desc_med < c.descent_dalt_th)
            )

            self.descent_consec = self.descent_consec + 1 if descent_cond else 0
            confirmed = self.descent_consec >= c.descent_req

            if confirmed:
                self.descent_confirmed = True
                self.land_consec = 0

            return FallStatus(
                phase="DESCENT",
                alt=alt_m,
                acc_med=acc_med,
                dalt_med=dalt_desc_med,
                cond=descent_cond,
                consec=self.descent_consec,
                confirmed=confirmed,
                armed=self.armed,
                baseline_alt=self.baseline_alt,
                max_alt=self.max_alt,
            )

        acc_ok = True if acc_med is None else (c.acc_land_min <= acc_med <= c.acc_land_max)
        land_cond = (
            (abs(dalt_land_med) <= c.land_dalt_abs_th) and
            (alt_span is not None and alt_span <= c.land_alt_span_th) and
            acc_ok
        )

        self.land_consec = self.land_consec + 1 if land_cond else 0
        confirmed = self.land_consec >= c.land_req

        return FallStatus(
            phase="LANDING",
            alt=alt_m,
            acc_med=acc_med,
            dalt_med=dalt_land_med,
            cond=land_cond,
            consec=self.land_consec,
            confirmed=confirmed,
            armed=self.armed,
            baseline_alt=self.baseline_alt,
            max_alt=self.max_alt,
        )


# ============================================================
# 3) GPS誘導関連
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

    stuck_back_ms: int = 900
    stuck_turn_ms: int = 900
    stuck_fwd_ms: int = 600
    stuck_back_power: int = -260
    stuck_turn_power: int = 320
    stuck_fwd_power: int = 260

    flip_fwd_ms: int = 1200
    flip_fwd_power: int = 320

class RecoveryManager:
    def __init__(self, cfg: RecoveryConfig, drive: ServoDrive, bno, logger=print):
        self.cfg = cfg
        self.drive = drive
        self.bno = bno
        self.log = logger
        self._busy_until = 0.0
        self._state = 0
        self.mode = None

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
        self.mode = "FLIP" if "FLIP" in reason.upper() else "STUCK"
        self._set_busy(1)

    def tick(self):
        if not self.is_busy():
            return False

        if self.mode == "FLIP":
            if self._state == 0:
                self.drive.set_diff(self.cfg.flip_fwd_power, 0)
                self._state = 1
                self._set_busy(self.cfg.flip_fwd_ms)
                return True

            self.drive.stop()
            self._busy_until = 0.0
            self.mode = None
            self.log("[REC] FLIP recovery done")
            return False

        if self._state == 0:
            self.drive.set_diff(self.cfg.stuck_back_power, 0)
            self._state = 1
            self._set_busy(self.cfg.stuck_back_ms)
            return True

        if self._state == 1:
            self.drive.set_diff(0, self.cfg.stuck_turn_power)
            self._state = 2
            self._set_busy(self.cfg.stuck_turn_ms)
            return True

        if self._state == 2:
            self.drive.set_diff(self.cfg.stuck_fwd_power, 0)
            self._state = 3
            self._set_busy(self.cfg.stuck_fwd_ms)
            return True

        self.drive.stop()
        self._busy_until = 0.0
        self.mode = None
        self.log("[REC] STUCK recovery done")
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
        self.last_goal_bearing = None
        self.last_heading = None
        self.last_err = None
        self.last_cmd_fwd = 0.0
        self.last_cmd_turn = 0.0

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
            self.last_cmd_fwd = 0.0
            self.last_cmd_turn = 0.0
            return (False, False)

        dist = haversine_m(fix.lat, fix.lon, c.goal_lat, c.goal_lon)
        self.last_dist = dist

        if dist <= c.arrival_radius_m:
            self.drive.stop()
            self.log(f"[GPS] ARRIVED dist={dist:.2f}m <= {c.arrival_radius_m}m")
            return (True, False)

        theta_goal = bearing_deg(fix.lat, fix.lon, c.goal_lat, c.goal_lon)
        err = wrap_to_180(theta_goal - heading)

        self.last_goal_bearing = theta_goal
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
        self.last_cmd_turn = turn

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

        self.last_red_ratio = 0.0
        self.last_status = "INIT"

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

        if best is None:
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
# 8) フェーズ
# ============================================================
class Phase:
    DROP = "DROP"
    CALIB = "CALIB"
    GPS = "GPS"
    CAMERA = "CAMERA"
    DONE = "DONE"


# ============================================================
# 9) GPIO / 強制処理
# ============================================================
def gpio_pulse_high(pi: pigpio.pi, gpio_pin: int, sec: float, logger=print):
    pi.write(gpio_pin, 1)
    logger(f"[GPIO] PIN{gpio_pin} HIGH for {sec:.2f}s")
    time.sleep(sec)
    pi.write(gpio_pin, 0)
    logger(f"[GPIO] PIN{gpio_pin} LOW")

def force_landing_sequence(
    pi: pigpio.pi,
    landing_pin: int,
    drive: ServoDrive,
    logger=print,
    pulse_sec: float = 1.0,
    fwd_power: int = 500,
    fwd_sec: float = 3.0,
):
    logger("[FORCE] landing sequence start")
    gpio_pulse_high(pi, landing_pin, pulse_sec, logger=logger)
    drive.set_diff(fwd_power, 0)
    logger("[FORCE] positive servos...")
    time.sleep(fwd_sec)
    drive.stop()
    logger("[FORCE] landing sequence done")


# ============================================================
# 9.5) ダウンリンク生成
# ============================================================
def make_downlink_line(
    elapsed: float,
    phase: str,
    drop_state: str,
    gps_fix,
    gps_dist_m,
    heading_deg,
    recovery_mode: str | None,
    cam_status: str,
    goal_flag: bool,
):
    def nz_float(v, default=-9999.0):
        return float(v) if v is not None else default

    def nz_int(v, default=-1):
        return int(v) if v is not None else default

    lat = nz_float(gps_fix.lat if gps_fix is not None else None)
    lon = nz_float(gps_fix.lon if gps_fix is not None else None)
    fixq = nz_int(gps_fix.fixq if gps_fix is not None else None, 0)
    nsat = nz_int(gps_fix.nsat if gps_fix is not None else None, 0)
    dist = nz_float(gps_dist_m)
    head = nz_float(heading_deg)

    rec = recovery_mode if recovery_mode is not None else "NONE"
    cam = cam_status if cam_status else "NA"
    goal = 1 if goal_flag else 0

    return (
        "STAT,"
        f"{elapsed:.1f},"
        f"{phase},"
        f"{drop_state},"
        f"{lat:.8f},"
        f"{lon:.8f},"
        f"{fixq:d},"
        f"{nsat:d},"
        f"{dist:.1f},"
        f"{head:.1f},"
        f"{rec},"
        f"{cam},"
        f"{goal:d}"
    )


# ============================================================
# 10) メイン
# ============================================================
def main():
    logger = CsvLogger(out_dir="logs")
    logger.log_event("=== CanSat Integrated Program Start ===")

    fall_cfg = FallConfig(
        dt=0.10,

        descent_win=7,
        descent_req=4,
        descent_dalt_th=-0.20,
        descent_from_peak_m=1.5,

        land_win=11,
        land_req=8,
        land_dalt_abs_th=0.06,
        land_alt_span_th=0.30,
        acc_land_min=8.3,
        acc_land_max=11.3,

        arm_alt_rise_m=5.0,
        arm_min_time_sec=2.0,

        sea_level_hpa=1013.25,
    )

    gps_cfg = GpsConfig(
        goal_lat=35.66059,
        goal_lon=139.36688,
        arrival_radius_m=1.0,
        angle_ok_deg=3.0,
    )

    cam_cfg = CameraConfig(w=480, h=640, show_debug=False)

    GPIO_LANDING_PIN = 17
    HEADING_EXTRA_OFFSET_DEG = 0.0

    FORCE_GPS_AFTER_SEC = 15 * 60
    FORCE_DONE_AFTER_SEC = 25 * 60

    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio daemonに接続できません。sudo pigpiod を確認してください。")

    pi.set_mode(GPIO_LANDING_PIN, pigpio.OUTPUT)
    pi.write(GPIO_LANDING_PIN, 0)
    pi.set_mode(GPIO_LED_PIN, pigpio.OUTPUT)
    pi.write(GPIO_LED_PIN, 0)

    tw = None
    try:
        tw = TweliteSoftUART(
            pi=pi,
            rx_gpio=TWELITE_RX_GPIO,
            tx_gpio=TWELITE_TX_GPIO,
            baud=TWELITE_BAUD,
        )
        logger.log_event(
            f"[TWELITE] soft UART open RX=GPIO{TWELITE_RX_GPIO} TX=GPIO{TWELITE_TX_GPIO} {TWELITE_BAUD}bps"
        )
    except Exception as e:
        logger.log_event(f"[TWELITE] init failed: {e}")
        tw = None

    servo_cfg = ServoConfig(stop_us=1490, min_us=500, max_us=2500, pin18=18, pin12=12, max_delta=500)
    drive = ServoDrive(pi, servo_cfg)

    i2c = busio.I2C(board.SCL, board.SDA)
    bno = adafruit_bno055.BNO055_I2C(i2c, address=0x28)
    bme = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=0x76)
    bme.sea_level_pressure = fall_cfg.sea_level_hpa

    detector = FallLandingDetector(fall_cfg, logger=logger.log_event)

    gps_reader = NmeaGpsReader(gps_cfg, logger=logger.log_event)
    gps_reader.start()

    calib = CompassCalib(valid=False, heading_extra_offset_deg=HEADING_EXTRA_OFFSET_DEG)
    gps_guidance = GpsGuidance(gps_cfg, drive, bno, gps_reader, calib, logger=logger.log_event)

    rec_cfg = RecoveryConfig()
    recovery = RecoveryManager(rec_cfg, drive, bno, logger=logger.log_event)

    cam_guidance = CameraGuidance(cam_cfg, drive, logger=logger.log_event)
    picam2 = None

    mission_t0 = time.monotonic()
    forced_gps_done = False

    next_downlink_time = time.monotonic() + 1.0 / DOWNLINK_HZ
    last_drop_state = "INIT"
    goal_flag = False

    phase = Phase.DROP
    logger.log_event(f"Phase: {phase}", phase=phase, elapsed_sec=0.0)
    logger.log_event(
        f"[TIMER] force_gps={FORCE_GPS_AFTER_SEC}s force_done={FORCE_DONE_AFTER_SEC}s",
        phase=phase,
        elapsed_sec=0.0
    )

    def do_downlink(elapsed, phase, gps_fix):
        nonlocal next_downlink_time, last_drop_state, goal_flag, tw

        now_dl = time.monotonic()
        if tw is not None and now_dl >= next_downlink_time:
            next_downlink_time += 1.0 / DOWNLINK_HZ
            try:
                line = make_downlink_line(
                    elapsed=elapsed,
                    phase=phase,
                    drop_state=last_drop_state,
                    gps_fix=gps_fix,
                    gps_dist_m=gps_guidance.last_dist,
                    heading_deg=gps_guidance.last_heading,
                    recovery_mode=recovery.mode,
                    cam_status=cam_guidance.last_status if cam_guidance is not None else "NA",
                    goal_flag=goal_flag,
                )
                tw.write_line(line)
                print(line)
            except Exception as e:
                logger.log_event(f"[TWELITE] send error: {e}", phase=phase, elapsed_sec=elapsed)

        if tw is not None:
            try:
                for text in tw.read_lines():
                    if text:
                        logger.log_event(f"[TWELITE RX] {text}", phase=phase, elapsed_sec=elapsed)
            except Exception as e:
                logger.log_event(f"[TWELITE] recv error: {e}", phase=phase, elapsed_sec=elapsed)

    try:
        while phase != Phase.DONE:
            elapsed = time.monotonic() - mission_t0

            if elapsed >= FORCE_DONE_AFTER_SEC and phase != Phase.DONE:
                logger.log_event(
                    f"[TIMER] FORCE DONE at {elapsed:.1f}s (mission timeout)",
                    phase=phase,
                    elapsed_sec=elapsed
                )
                drive.stop()
                phase = Phase.DONE
                pi.write(GPIO_LED_PIN, 1)
                logger.log_event(f"Phase: {phase}", phase=phase, elapsed_sec=elapsed)
                do_downlink(elapsed, phase, gps_reader.get())
                continue

            if (elapsed >= FORCE_GPS_AFTER_SEC) and (not forced_gps_done):
                if phase in (Phase.DROP, Phase.CALIB):
                    logger.log_event(
                        f"[TIMER] FORCE GPS TRANSITION at {elapsed:.1f}s",
                        phase=phase,
                        elapsed_sec=elapsed
                    )

                    force_landing_sequence(
                        pi=pi,
                        landing_pin=GPIO_LANDING_PIN,
                        drive=drive,
                        logger=logger.log_event,
                        pulse_sec=1.0,
                        fwd_power=500,
                        fwd_sec=3.0,
                    )

                    detector.descent_confirmed = True
                    detector.land_consec = fall_cfg.land_req

                    drive.stop()
                    phase = Phase.GPS
                    forced_gps_done = True
                    logger.log_event(f"Phase: {phase}", phase=phase, elapsed_sec=elapsed)
                    do_downlink(elapsed, phase, gps_reader.get())
                    continue
                else:
                    forced_gps_done = True

            alt = safe_read_sensor(lambda: bme.altitude)
            acc = safe_read_sensor(lambda: bno.acceleration)
            mag = safe_read_sensor(lambda: bno.magnetic)
            euler = safe_read_sensor(lambda: bno.euler)
            gps_fix = gps_reader.get()

            if not recovery.is_busy() and recovery.detect_flip() and phase != Phase.DROP:
                recovery.start_recover("FLIP detected")

            if recovery.is_busy():
                recovery.tick()
                logger.log_row(
                    phase=phase,
                    message="recovering",
                    elapsed_sec=elapsed,
                    alt_m=alt,
                    acc=acc,
                    mag=mag,
                    euler=euler,
                    gps_fix=gps_fix,
                    servo_fwd_cmd=drive.last_fwd,
                    servo_turn_cmd=drive.last_turn,
                    servo_us18=drive.last_us18,
                    servo_us12=drive.last_us12,
                    recovery_mode=recovery.mode,
                    recovery_busy=True,
                )
                do_downlink(elapsed, phase, gps_fix)
                time.sleep(0.05)
                continue

            if phase == Phase.DROP:
                st = detector.update(alt, acc)
                last_drop_state = st.phase

                if st.phase == "WARMUP":
                    logger.log_row(
                        phase=phase,
                        message="warmup",
                        elapsed_sec=elapsed,
                        alt_m=alt,
                        acc=acc,
                        mag=mag,
                        euler=euler,
                        drop_subphase=st.phase,
                        drop_acc_med=st.acc_med,
                        drop_dalt_med=st.dalt_med,
                        drop_cond=st.cond,
                        drop_consec=st.consec,
                        drop_confirmed=st.confirmed,
                        drop_armed=st.armed,
                        drop_baseline_alt=st.baseline_alt,
                        drop_max_alt=st.max_alt,
                        gps_fix=gps_fix,
                        servo_fwd_cmd=drive.last_fwd,
                        servo_turn_cmd=drive.last_turn,
                        servo_us18=drive.last_us18,
                        servo_us12=drive.last_us12,
                        recovery_mode=recovery.mode,
                        recovery_busy=False,
                    )
                    do_downlink(elapsed, phase, gps_fix)
                    time.sleep(fall_cfg.dt)
                    continue

                if st.phase == "ARM_WAIT":
                    logger.log_event(
                        f"[DROP] ARM_WAIT alt={st.alt:.2f} acc_med={st.acc_med} dalt_med={st.dalt_med:.3f} "
                        f"baseline={st.baseline_alt:.2f} max_alt={st.max_alt:.2f}",
                        phase=phase,
                        elapsed_sec=elapsed
                    )
                    logger.log_row(
                        phase=phase,
                        message="arm_wait",
                        elapsed_sec=elapsed,
                        alt_m=alt,
                        acc=acc,
                        mag=mag,
                        euler=euler,
                        drop_subphase=st.phase,
                        drop_acc_med=st.acc_med,
                        drop_dalt_med=st.dalt_med,
                        drop_cond=st.cond,
                        drop_consec=st.consec,
                        drop_confirmed=st.confirmed,
                        drop_armed=st.armed,
                        drop_baseline_alt=st.baseline_alt,
                        drop_max_alt=st.max_alt,
                        gps_fix=gps_fix,
                        servo_fwd_cmd=drive.last_fwd,
                        servo_turn_cmd=drive.last_turn,
                        servo_us18=drive.last_us18,
                        servo_us12=drive.last_us12,
                        recovery_mode=recovery.mode,
                        recovery_busy=False,
                    )
                    do_downlink(elapsed, phase, gps_fix)
                    time.sleep(fall_cfg.dt)
                    continue

                if st.phase == "DESCENT":
                    logger.log_event(
                        f"[DROP] DESCENT? alt={st.alt:.2f} acc_med={st.acc_med} "
                        f"dalt_med={st.dalt_med:.3f} cond={st.cond} consec={st.consec}",
                        phase=phase,
                        elapsed_sec=elapsed
                    )
                    if st.confirmed:
                        logger.log_event("[DROP] DESCENT CONFIRMED", phase=phase, elapsed_sec=elapsed)

                elif st.phase == "LANDING":
                    logger.log_event(
                        f"[DROP] LAND? alt={st.alt:.2f} acc_med={st.acc_med} "
                        f"dalt_med={st.dalt_med:.3f} cond={st.cond} consec={st.consec}",
                        phase=phase,
                        elapsed_sec=elapsed
                    )
                    if st.confirmed:
                        logger.log_event("[DROP] LANDING CONFIRMED", phase=phase, elapsed_sec=elapsed)

                        force_landing_sequence(
                            pi=pi,
                            landing_pin=GPIO_LANDING_PIN,
                            drive=drive,
                            logger=logger.log_event,
                            pulse_sec=1.0,
                            fwd_power=500,
                            fwd_sec=3.0,
                        )

                        phase = Phase.CALIB
                        logger.log_event(f"Phase: {phase}", phase=phase, elapsed_sec=elapsed)

                logger.log_row(
                    phase=phase,
                    message="drop_step",
                    elapsed_sec=elapsed,
                    alt_m=alt,
                    acc=acc,
                    mag=mag,
                    euler=euler,
                    drop_subphase=st.phase,
                    drop_acc_med=st.acc_med,
                    drop_dalt_med=st.dalt_med,
                    drop_cond=st.cond,
                    drop_consec=st.consec,
                    drop_confirmed=st.confirmed,
                    drop_armed=st.armed,
                    drop_baseline_alt=st.baseline_alt,
                    drop_max_alt=st.max_alt,
                    gps_fix=gps_fix,
                    servo_fwd_cmd=drive.last_fwd,
                    servo_turn_cmd=drive.last_turn,
                    servo_us18=drive.last_us18,
                    servo_us12=drive.last_us12,
                    recovery_mode=recovery.mode,
                    recovery_busy=False,
                )

                do_downlink(elapsed, phase, gps_fix)
                time.sleep(fall_cfg.dt)
                continue

            if phase == Phase.CALIB:
                drive.stop()
                time.sleep(0.2)
                run_compass_calibration(
                    drive=drive,
                    bno=bno,
                    calib=calib,
                    duration_sec=15.0,
                    turn_power=220,
                    logger=logger.log_event,
                )
                phase = Phase.GPS
                logger.log_event(f"Phase: {phase}", phase=phase, elapsed_sec=elapsed)
                do_downlink(elapsed, phase, gps_fix)
                continue

            if phase == Phase.GPS:
                dt = 1.0 / gps_cfg.control_hz
                t0 = time.monotonic()

                arrived, stuck = gps_guidance.step()
                if stuck and not recovery.is_busy():
                    recovery.start_recover("STUCK suspected (GPS progress low)")
                if arrived:
                    phase = Phase.CAMERA
                    logger.log_event(f"Phase: {phase}", phase=phase, elapsed_sec=elapsed)

                logger.log_row(
                    phase=phase,
                    message="gps_step",
                    elapsed_sec=elapsed,
                    alt_m=alt,
                    acc=acc,
                    mag=mag,
                    euler=euler,
                    gps_fix=gps_fix,
                    gps_dist_m=gps_guidance.last_dist,
                    gps_goal_bearing_deg=gps_guidance.last_goal_bearing,
                    gps_heading_deg=gps_guidance.last_heading,
                    gps_err_deg=gps_guidance.last_err,
                    servo_fwd_cmd=drive.last_fwd,
                    servo_turn_cmd=drive.last_turn,
                    servo_us18=drive.last_us18,
                    servo_us12=drive.last_us12,
                    recovery_mode=recovery.mode,
                    recovery_busy=False,
                )

                do_downlink(elapsed, phase, gps_fix)
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
                    logger.log_event("[CAM] Camera started", phase=phase, elapsed_sec=elapsed)

                frame = picam2.capture_array("main")
                if frame is None:
                    logger.log_row(
                        phase=phase,
                        message="camera_frame_none",
                        elapsed_sec=elapsed,
                        alt_m=alt,
                        acc=acc,
                        mag=mag,
                        euler=euler,
                        gps_fix=gps_fix,
                        servo_fwd_cmd=drive.last_fwd,
                        servo_turn_cmd=drive.last_turn,
                        servo_us18=drive.last_us18,
                        servo_us12=drive.last_us12,
                        recovery_mode=recovery.mode,
                        recovery_busy=False,
                    )
                    do_downlink(elapsed, phase, gps_fix)
                    continue

                goal = cam_guidance.step(frame)

                logger.log_row(
                    phase=phase,
                    message="camera_step",
                    elapsed_sec=elapsed,
                    alt_m=alt,
                    acc=acc,
                    mag=mag,
                    euler=euler,
                    gps_fix=gps_fix,
                    servo_fwd_cmd=drive.last_fwd,
                    servo_turn_cmd=drive.last_turn,
                    servo_us18=drive.last_us18,
                    servo_us12=drive.last_us12,
                    recovery_mode=recovery.mode,
                    recovery_busy=False,
                    camera_red_ratio=cam_guidance.last_red_ratio,
                    camera_status=cam_guidance.last_status,
                )

                if goal:
                    goal_flag = True
                    phase = Phase.DONE
                    pi.write(GPIO_LED_PIN, 1)
                    logger.log_event(f"Phase: {phase}", phase=phase, elapsed_sec=elapsed)

                do_downlink(elapsed, phase, gps_fix)
                continue

    except KeyboardInterrupt:
        logger.log_event("Ctrl-C", elapsed_sec=(time.monotonic() - mission_t0))
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
            if cam_cfg.show_debug:
                cv2.destroyAllWindows()
        except Exception:
            pass
        try:
            if tw is not None:
                tw.close()
        except Exception:
            pass
        try:
            pi.write(GPIO_LANDING_PIN, 0)
            pi.write(GPIO_LED_PIN, 0)
            pi.stop()
        except Exception:
            pass
        logger.log_event(
            f"=== Finish === CSV: {logger.path}",
            elapsed_sec=(time.monotonic() - mission_t0)
        )
        logger.close()

if __name__ == "__main__":
    main()