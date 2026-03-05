#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
CanSat 統合スクリプト（DROP→GPS→CAMERA） + Downlink(TWELITE 1Hz) + BLE(Nordic UART 10Hz) + Piローカルログ(10Hz)

【フロー】
  Start
    -> 落下検知 + 着地判定 + GPIO17 HIGH(1秒)
    -> GPS誘導
    -> カメラ誘導（赤コーン）
    -> Done

【通信/ログ（目的：状態把握 + 低レートバックアップ + 提出用の制御履歴）】
  - “保存ログと同じ1行CSV” を 10Hzで生成 (LOG_HZ)
    - その1行を
      (a) Piローカルログに書く
      (b) BLEでXIAOへ送る（Nordic UART）
  - TWELITEは 1Hzで “最新のCSV1行” を送る（同じ列）

重要:
  - センサ読み取りはこのスクリプト内で一元化（BLE側のsensor_taskは統合しない）
  - pigpio.pi() も1つに統一（サーボ/TWELITEで共有）
"""

from __future__ import annotations

import asyncio
import math
import os
import signal
import statistics
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional

import pigpio

import board
import busio
import adafruit_bno055
from adafruit_bme280 import basic as adafruit_bme280

from smbus2 import SMBus
import pynmea2

import cv2
import numpy as np
from picamera2 import Picamera2

# --- BLE (bleak) ---
# bleakが無い環境もあり得るので、ImportErrorならBLE機能だけ無効化できるようにする
try:
    from bleak import BleakClient, BleakScanner
    from bleak.exc import BleakError
    BLE_AVAILABLE = True
except Exception:
    BLE_AVAILABLE = False
    BleakClient = None
    BleakScanner = None
    BleakError = Exception


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


# ============================================================
# 1) サーボ駆動（GPS用 set_vw / カメラ用 set_diff）
# ============================================================
@dataclass
class ServoConfig:
    stop_us: int = 1490
    min_us: int = 500
    max_us: int = 2500
    pin18: int = 18
    pin12: int = 12

    # GPS用
    scale_us: int = 500

    # カメラ用
    max_delta: int = 600


class ServoDrive:
    """
    同じサーボ(18/12)を、2種類の指令形式で動かす。

    - set_vw(v,w): GPS誘導（あなたのGPSコードの式をそのまま採用）
    - set_diff(fwd,turn): カメラ誘導（あなたのカメラコードの式を採用）

    さらに、提出/解析用に「最後に出したPWM(us)」を保持する。
    """

    def __init__(self, pi: pigpio.pi, cfg: ServoConfig):
        self.pi = pi
        self.cfg = cfg
        self.last_us18: int | None = None
        self.last_us12: int | None = None
        self.stop()

    def _write(self, us18, us12):
        u18 = clamp_int(us18, self.cfg.min_us, self.cfg.max_us)
        u12 = clamp_int(us12, self.cfg.min_us, self.cfg.max_us)
        self.pi.set_servo_pulsewidth(self.cfg.pin18, u18)
        self.pi.set_servo_pulsewidth(self.cfg.pin12, u12)

        # ★提出/解析用に保持（pigpioから読み返しにくいので自前で持つ）
        self.last_us18 = u18
        self.last_us12 = u12

    def stop(self):
        self._write(self.cfg.stop_us, self.cfg.stop_us)

    def free(self):
        self.pi.set_servo_pulsewidth(self.cfg.pin18, 0)
        self.pi.set_servo_pulsewidth(self.cfg.pin12, 0)

    def set_vw(self, v, w):
        """
        v: 前進 0..1
        w: 右旋回 +、左旋回 -
        """
        v = clamp(v, 0.0, 1.0)
        w = clamp(w, -1.0, 1.0)

        us18 = self.cfg.stop_us - v * self.cfg.scale_us + w * self.cfg.scale_us
        us12 = self.cfg.stop_us + v * self.cfg.scale_us + w * self.cfg.scale_us
        self._write(us18, us12)

    def set_diff(self, fwd, turn):
        """
        fwd  > 0 で前進
        turn > 0 で右旋回（実機に合わせてここで反転済み）
        """
        turn = -turn  # あなたの確定設定

        left_power = fwd + turn
        right_power = fwd - turn

        left_power = clamp_int(left_power, -self.cfg.max_delta, self.cfg.max_delta)
        right_power = clamp_int(right_power, -self.cfg.max_delta, self.cfg.max_delta)

        pulse18 = self.cfg.stop_us - left_power   # pin18: 小さいほど前進
        pulse12 = self.cfg.stop_us + right_power  # pin12: 大きいほど前進
        self._write(pulse18, pulse12)


# ============================================================
# 2) TWELITE soft UART（pigpio共有）
# ============================================================
class TweliteSoftUART:
    """
    pigpio soft-UART:
      - TX: wave_add_serial (DMA)
      - RX: bb_serial_read (bit-bang)

    統合のため pigpio.pi() は外から渡す（1インスタンス共有）。
    """

    def __init__(
        self,
        pi: pigpio.pi,
        rx_gpio: int,
        tx_gpio: int,
        baud: int,
        data_bits: int = 8,
        stop_half_bits: int = 2,      # pigpio仕様: 2 => 1 stop bit
        line_ending: bytes = b"\r\n",
        rx_buf_max: int = 4096,
    ):
        self.pi = pi
        self.rx_gpio = int(rx_gpio)
        self.tx_gpio = int(tx_gpio)
        self.baud = int(baud)

        self.data_bits = int(data_bits)
        self.stop_half_bits = int(stop_half_bits)
        self.line_ending = line_ending
        self.rx_buf_max = int(rx_buf_max)

        self._rx_buf = bytearray()

        # TX idle HIGH
        self.pi.set_mode(self.tx_gpio, pigpio.OUTPUT)
        self.pi.write(self.tx_gpio, 1)

        # RX bit-bang open
        self.pi.set_mode(self.rx_gpio, pigpio.INPUT)
        self.pi.bb_serial_read_open(self.rx_gpio, self.baud, self.data_bits)
        time.sleep(0.05)

    def close(self):
        try:
            self.pi.bb_serial_read_close(self.rx_gpio)
        except pigpio.error:
            pass

    def write_line(self, line: str):
        payload = line.encode("ascii", errors="ignore") + self.line_ending

        self.pi.wave_clear()
        self.pi.wave_add_serial(self.tx_gpio, self.baud, payload, 0, self.data_bits, self.stop_half_bits)
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
            if len(self._rx_buf) > self.rx_buf_max:
                self._rx_buf = self._rx_buf[-self.rx_buf_max:]

        lines = []
        while b"\n" in self._rx_buf:
            raw, _, rest = self._rx_buf.partition(b"\n")
            self._rx_buf = bytearray(rest)
            raw = raw.rstrip(b"\r")
            lines.append(raw.decode("ascii", errors="replace").strip())
        return lines


# ============================================================
# 3) 落下検知 + 着地判定
# ============================================================
@dataclass
class FallConfig:
    dt: float = 0.10
    win: int = 11
    req: int = 5
    sea_level_hpa: float = 1013.25

    freefall_acc_th: float = 3.0
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
    ff_consec: int = 0
    land_consec: int = 0
    freefall_confirmed: bool = False
    landing_confirmed: bool = False


class FallLandingDetector:
    """
    |a| と dAlt を中央値フィルタで平滑化して判定する。
    FREEFALL: (acc_med < th) OR (dalt_med < th)
    LANDING : (acc_med in [min,max]) OR (|dalt_med| < th)
    """

    def __init__(self, cfg: FallConfig):
        self.cfg = cfg
        self.buf_acc = deque(maxlen=cfg.win)
        self.buf_dalt = deque(maxlen=cfg.win)
        self.prev_alt = None

        self.freefall_confirmed = False
        self.ff_consec = 0
        self.land_consec = 0

    def update(self, alt_m: float, acc_vec) -> FallStatus:
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

            if self.ff_consec >= c.req:
                self.freefall_confirmed = True
                self.land_consec = 0

            return FallStatus(
                phase="FREEFALL",
                alt=alt_m,
                acc_med=acc_med,
                dalt_med=dalt_med,
                ff_consec=self.ff_consec,
                land_consec=self.land_consec,
                freefall_confirmed=self.freefall_confirmed,
            )

        land_cond = ((c.acc_land_min <= acc_med <= c.acc_land_max) or (abs(dalt_med) < c.land_dalt_abs_th))
        self.land_consec = (self.land_consec + 1) if land_cond else 0
        landing_confirmed = (self.land_consec >= c.req)

        return FallStatus(
            phase="LANDING",
            alt=alt_m,
            acc_med=acc_med,
            dalt_med=dalt_med,
            ff_consec=self.ff_consec,
            land_consec=self.land_consec,
            freefall_confirmed=self.freefall_confirmed,
            landing_confirmed=landing_confirmed,
        )


# ============================================================
# 4) GPS(I2C NMEA) 受信スレッド（lat/lon + GGA alt含む）
# ============================================================
@dataclass
class GpsFix:
    lat: float | None = None
    lon: float | None = None
    alt: float | None = None      # ★GGA altitude
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

    kp: float = 0.1
    w_max: float = 0.60
    w_bias: float = +0.06


class NmeaGpsReader:
    def __init__(self, cfg: GpsConfig):
        self.cfg = cfg
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

                        alt = None
                        if fixq > 0:
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
# 5) GPS誘導（制御履歴を返す）
# ============================================================
class GpsGuidance:
    def __init__(self, cfg: GpsConfig, drive: ServoDrive, bno, gps_reader: NmeaGpsReader):
        self.cfg = cfg
        self.drive = drive
        self.bno = bno
        self.gps = gps_reader
        self._last_log = time.monotonic()

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

    def step(self) -> dict:
        """
        戻り値（提出・ダウンリンク用）:
          arrived: bool
          dist, bearing_goal, heading, err, v, w
        """
        c = self.cfg
        fix = self.gps.get()

        heading = None
        try:
            e = self.bno.euler
            if e is not None:
                heading = e[0]  # 北0 東90
        except Exception:
            heading = None

        if heading is None or not self._gps_ok(fix):
            self.drive.stop()
            return {"arrived": False}

        dist = haversine_m(fix.lat, fix.lon, c.goal_lat, c.goal_lon)

        if dist <= c.arrival_radius_m:
            self.drive.stop()
            return {
                "arrived": True,
                "dist": dist,
                "bearing_goal": None,
                "heading": float(heading),
                "err": None,
                "v": 0.0,
                "w": 0.0,
            }

        theta_goal = bearing_deg(fix.lat, fix.lon, c.goal_lat, c.goal_lon)
        err = wrap_to_180(theta_goal - heading)

        if dist < c.slowdown_dist_m:
            a = clamp(dist / c.slowdown_dist_m, 0.0, 1.0)
            v = c.min_v + (c.base_v - c.min_v) * a
        else:
            v = c.base_v

        if abs(err) < c.angle_ok_deg:
            w = c.w_bias
        else:
            w = clamp(c.kp * err + c.w_bias, -c.w_max, c.w_max)

        self.drive.set_vw(v, w)

        now = time.monotonic()
        if now - self._last_log > 1.0:
            print(f"[GPS] dist={dist:6.1f}m goal={theta_goal:6.1f} head={heading:6.1f} err={err:6.1f} v={v:.2f} w={w:.2f}")
            self._last_log = now

        return {
            "arrived": False,
            "dist": dist,
            "bearing_goal": theta_goal,
            "heading": float(heading),
            "err": float(err),
            "v": float(v),
            "w": float(w),
        }


# ============================================================
# 6) カメラ誘導（モード/赤率を返す）
# ============================================================
@dataclass
class CameraConfig:
    w: int = 480
    h: int = 640
    show_debug: bool = False

    kp_turn: float = 0.8
    base_fwd: int = 250
    search_turn: int = 200
    area_track_th: int = 900
    area_fwd_th: int = 2200
    center_tol_ratio: float = 0.08

    slow_red_ratio: float = 0.35
    goal_red_ratio: float = 0.60
    goal_hold_sec: float = 0.25


class CameraGuidance:
    """
    戻り値:
      {"goal": bool, "mode": str, "red_ratio": float}
    """

    LOWER_RED1 = np.array([0, 120, 70], dtype=np.uint8)
    UPPER_RED1 = np.array([10, 255, 255], dtype=np.uint8)
    LOWER_RED2 = np.array([170, 120, 70], dtype=np.uint8)
    UPPER_RED2 = np.array([179, 255, 255], dtype=np.uint8)
    KERNEL = np.ones((5, 5), np.uint8)

    def __init__(self, cfg: CameraConfig, drive: ServoDrive):
        self.cfg = cfg
        self.drive = drive
        self._goal_start_t = None

    def _make_red_mask(self, frame_bgr):
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        m1 = cv2.inRange(hsv, self.LOWER_RED1, self.UPPER_RED1)
        m2 = cv2.inRange(hsv, self.LOWER_RED2, self.UPPER_RED2)
        mask = cv2.bitwise_or(m1, m2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.KERNEL)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.KERNEL)
        return mask

    def _is_triangle_like(self, contour, min_area):
        area = cv2.contourArea(contour)
        if area < min_area:
            return False, None, area
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
        if len(approx) != 3:
            return False, approx, area
        if not cv2.isContourConvex(approx):
            return False, approx, area
        return True, approx, area

    def step(self, frame_bgr) -> dict:
        c = self.cfg

        frame = cv2.rotate(frame_bgr, cv2.ROTATE_90_COUNTERCLOCKWISE)

        h, w = frame.shape[:2]
        cx_frame = w // 2
        center_tol_px = int(w * c.center_tol_ratio)

        mask = self._make_red_mask(frame)
        red_ratio = cv2.countNonZero(mask) / float(h * w)

        if red_ratio >= c.goal_red_ratio:
            if self._goal_start_t is None:
                self._goal_start_t = time.time()
            elif (time.time() - self._goal_start_t) >= c.goal_hold_sec:
                self.drive.stop()
                return {"goal": True, "mode": "GOAL", "red_ratio": red_ratio}
        else:
            self._goal_start_t = None

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_area = 0.0
        best_approx = None
        for ct in contours:
            ok, approx, area = self._is_triangle_like(ct, c.area_track_th)
            if ok and area > best_area:
                best = ct
                best_area = float(area)
                best_approx = approx

        if best is None:
            if red_ratio >= c.slow_red_ratio:
                self.drive.set_diff(int(c.base_fwd * 0.35), 0)
                mode = "NEAR_FWD"
            else:
                self.drive.set_diff(0, c.search_turn)
                mode = "SEARCH"
            self._debug_show(frame, mask, mode, red_ratio, cx_frame, best_approx)
            return {"goal": False, "mode": mode, "red_ratio": red_ratio}

        M = cv2.moments(best)
        if M["m00"] == 0:
            if red_ratio >= c.slow_red_ratio:
                self.drive.set_diff(int(c.base_fwd * 0.35), 0)
                mode = "NEAR_FWD"
            else:
                self.drive.set_diff(0, c.search_turn)
                mode = "BAD_MOMENT"
            self._debug_show(frame, mask, mode, red_ratio, cx_frame, best_approx)
            return {"goal": False, "mode": mode, "red_ratio": red_ratio}

        cx = int(M["m10"] / M["m00"])
        err_px = cx - cx_frame

        if abs(err_px) < center_tol_px:
            if red_ratio >= c.slow_red_ratio:
                fwd = int(c.base_fwd * 0.35)
            else:
                fwd = c.base_fwd if best_area < c.area_fwd_th else int(c.base_fwd * 0.8)
            self.drive.set_diff(fwd, 0)
            mode = "FORWARD"
        else:
            turn_gain = 0.6 if red_ratio >= c.slow_red_ratio else 1.0
            turn = int(turn_gain * c.kp_turn * err_px)
            self.drive.set_diff(0, turn)
            mode = "TURN"

        self._debug_show(frame, mask, mode, red_ratio, cx_frame, best_approx)
        return {"goal": False, "mode": mode, "red_ratio": red_ratio}

    def _debug_show(self, frame, mask, mode, red_ratio, cx_frame, best_approx):
        if not self.cfg.show_debug:
            return
        h, w = frame.shape[:2]
        disp = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        cv2.line(disp, (cx_frame, 0), (cx_frame, h), (255, 255, 255), 1)
        if best_approx is not None:
            cv2.drawContours(disp, [best_approx], -1, (0, 255, 0), 2)
        cv2.putText(disp, f"{mode} rr={red_ratio:.3f}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("Camera", disp)
        cv2.imshow("Mask", mask)
        cv2.waitKey(1)


# ============================================================
# 7) テレメトリ状態 + CSVフォーマット（保存ログと同一列）
# ============================================================
class Phase:
    DROP = "DROP"
    GPS = "GPS"
    CAMERA = "CAMERA"
    DONE = "DONE"

MODE_IDLE = "idle"
MODE_DEBUG = "debug"
MODE_RUN = "run"


@dataclass
class TelemetryState:
    """
    ここに「機体の状態/観測/操作/制御履歴」を集約して、
    - 10HzでCSV 1行にする（保存ログと同一）
    - BLE/TWELITEに流す
    """
    # record
    seq: int = 0
    phase: str = Phase.DROP

    # mode (BLEコマンドで切り替え可能)
    mode: str = MODE_RUN

    # servo actual output (us)
    servo18_us: int | None = None
    servo12_us: int | None = None

    # GPS fix
    lat: float | None = None
    lon: float | None = None
    gps_alt: float | None = None
    fixq: int = 0
    nsat: int = 0
    hdop: float | None = None

    # 必須：距離 + 制御履歴（提出向け）
    dist_to_goal_m: float | None = None
    bearing_goal_deg: float | None = None
    heading_deg: float | None = None
    err_deg: float | None = None
    v_cmd: float | None = None
    w_cmd: float | None = None

    # Drop/Landing 判定の根拠
    alt_bme_m: float | None = None
    acc_med: float | None = None
    dalt_med: float | None = None
    ff_consec: int = 0
    land_consec: int = 0

    # Camera
    cam_mode: str | None = None
    red_ratio: float | None = None

    # Environment
    temp_c: float | None = None
    press_hpa: float | None = None
    humid_pct: float | None = None


class TelemetryFormatter:
    """
    1行CSVを作る（保存ログの正本）。
    できるだけ多く入れつつ、無線向けに丸める（桁数削減）。
    """
    def header(self) -> str:
        cols = [
            "seq", "t_ms", "ph", "mode",
            "lat", "lon", "galt", "fixq", "nsat", "hdop",
            "dist", "bg", "hd", "err", "v", "w",
            "balt", "accm", "daltm", "ffc", "ldc",
            "cm", "rr",
            "tc", "phPa", "hum",
            "s18", "s12",
        ]
        return ",".join(cols)

    def _fmt(self, v, fmt: str) -> str:
        if v is None:
            return ""
        return format(v, fmt)

    def _fmt_i(self, v) -> str:
        if v is None:
            return ""
        return str(int(v))

    def format_line(self, t_ms: int, st: TelemetryState) -> str:
        ph_map = {Phase.DROP: "DR", Phase.GPS: "GP", Phase.CAMERA: "CM", Phase.DONE: "DN"}
        ph = ph_map.get(st.phase, st.phase[:2])

        parts = [
            str(st.seq),
            str(t_ms),
            ph,
            st.mode,

            self._fmt(st.lat, ".5f"),
            self._fmt(st.lon, ".5f"),
            self._fmt(st.gps_alt, ".1f"),
            str(int(st.fixq)),
            str(int(st.nsat)),
            self._fmt(st.hdop, ".1f"),

            self._fmt(st.dist_to_goal_m, ".1f"),
            self._fmt(st.bearing_goal_deg, ".1f"),
            self._fmt(st.heading_deg, ".1f"),
            self._fmt(st.err_deg, ".1f"),
            self._fmt(st.v_cmd, ".2f"),
            self._fmt(st.w_cmd, ".2f"),

            self._fmt(st.alt_bme_m, ".1f"),
            self._fmt(st.acc_med, ".2f"),
            self._fmt(st.dalt_med, ".3f"),
            str(int(st.ff_consec)),
            str(int(st.land_consec)),

            "" if st.cam_mode is None else st.cam_mode,
            self._fmt(st.red_ratio, ".3f"),

            self._fmt(st.temp_c, ".1f"),
            self._fmt(st.press_hpa, ".1f"),
            self._fmt(st.humid_pct, ".1f"),

            self._fmt_i(st.servo18_us),
            self._fmt_i(st.servo12_us),
        ]
        return ",".join(parts)


# ============================================================
# 8) BLE (Nordic UART) 統合：送信（Pi->XIAO）/ 受信（XIAO->Pi）
# ============================================================
UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
UART_RX_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  # Pi -> XIAO (Write)
UART_TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # XIAO -> Pi (Notify)
XIAO_NAME = "CanSat-Remote"


@dataclass
class BleConfig:
    enable: bool = True
    telem_hz: float = 10.0  # BLEでXIAOへ送る頻度（= ログ生成頻度と揃えるのが基本）
    scan_timeout: float = 6.0


@dataclass
class RuntimeControl:
    """
    BLEコマンドで変更される “運用状態” を集約。
    - logging: PiローカルログのON/OFF
    - mode: idle/debug/run
    - phase_override: 文字列で強制（debug用途）。Noneなら通常フロー
    """
    logging: bool = True
    mode: str = MODE_RUN
    phase_override: Optional[str] = None
    last_cmd_ts: float = 0.0


def handle_cmd(ctrl: RuntimeControl, line: str):
    """
    XIAO -> Pi コマンド（Nordic UART notify）を処理する。
    フォーマット:
      cmd,mode,run|debug|idle
      cmd,log,toggle
      cmd,phase,set:DROP|GPS|CAMERA|DONE
      cmd,phase,clear
    """
    line = (line or "").strip()
    if not line:
        return
    if not line.startswith("cmd,"):
        print(f"[BLE CMD] ignored: {line}")
        return

    parts = line.split(",", 2)
    if len(parts) != 3:
        print(f"[BLE CMD] bad: {line}")
        return

    _, name, arg = parts
    ctrl.last_cmd_ts = time.time()

    if name == "mode":
        if arg in (MODE_RUN, MODE_DEBUG, MODE_IDLE):
            ctrl.mode = arg
            print(f"[BLE CMD] mode -> {ctrl.mode}")
        else:
            print(f"[BLE CMD] mode invalid: {arg}")

    elif name == "log":
        if arg == "toggle":
            ctrl.logging = not ctrl.logging
            print(f"[BLE CMD] logging -> {ctrl.logging}")
        else:
            print(f"[BLE CMD] log invalid: {arg}")

    elif name == "phase":
        if arg == "clear":
            ctrl.phase_override = None
            print("[BLE CMD] phase override cleared")
        elif arg.startswith("set:"):
            ph = arg.split(":", 1)[1].strip().upper()
            # 受理するのはこの4つだけ（曖昧さ排除）
            if ph in (Phase.DROP, Phase.GPS, Phase.CAMERA, Phase.DONE):
                ctrl.phase_override = ph
                print(f"[BLE CMD] phase override -> {ctrl.phase_override}")
            else:
                print(f"[BLE CMD] phase invalid: {ph}")
        else:
            print(f"[BLE CMD] phase invalid: {arg}")

    else:
        print(f"[BLE CMD] unknown: {name} {arg}")


async def find_xiao_addr(timeout: float) -> Optional[str]:
    devs = await BleakScanner.discover(timeout=timeout)
    for d in devs:
        name = (d.name or "").strip()
        if name == XIAO_NAME:
            return d.address
    return None


class BleWorker(threading.Thread):
    """
    BLEはasyncioが必要なので、別スレッドで asyncio ループを回す。
    メイン（制御/センサ）とは
      - last_record_line（送信したいCSV行）
      - ctrl（コマンドで変わる状態）
    を共有する。
    """

    def __init__(
        self,
        cfg: BleConfig,
        ctrl: RuntimeControl,
        ctrl_lock: threading.Lock,
        record_line_ref: dict,
        record_lock: threading.Lock,
        stop_flag: threading.Event,
    ):
        super().__init__(daemon=True)
        self.cfg = cfg
        self.ctrl = ctrl
        self.ctrl_lock = ctrl_lock
        self.record_line_ref = record_line_ref  # {"line": str, "updated_t": float}
        self.record_lock = record_lock
        self.stop_flag = stop_flag

    def run(self):
        if not BLE_AVAILABLE:
            print("[BLE] bleak not available -> BLE disabled")
            return
        asyncio.run(self._runner())

    async def _runner(self):
        # notify callback（XIAO->Pi）
        async def on_notify(_, data: bytearray):
            try:
                s = data.decode("utf-8", errors="ignore").strip()
            except Exception:
                return
            if not s:
                return
            for ln in s.splitlines():
                with self.ctrl_lock:
                    handle_cmd(self.ctrl, ln)

        backoff = 0.5
        backoff_max = 8.0

        while not self.stop_flag.is_set():
            addr = await find_xiao_addr(timeout=self.cfg.scan_timeout)
            if not addr:
                await asyncio.sleep(1.0)
                continue

            print(f"[BLE] found {XIAO_NAME} addr={addr}")

            try:
                async with BleakClient(addr) as client:
                    print("[BLE] connected")
                    await client.start_notify(UART_TX_UUID, on_notify)
                    print("[BLE] notify started")

                    # 接続直後の周辺が忙しい瞬間を避ける（安定化）
                    await asyncio.sleep(1.0)
                    backoff = 0.5

                    dt = 1.0 / float(self.cfg.telem_hz)
                    next_tick = time.monotonic()

                    last_sent = ""  # 同じ行の連投を避けたい場合に使用（基本は送ってもOK）

                    while client.is_connected and (not self.stop_flag.is_set()):
                        now = time.monotonic()
                        if now >= next_tick:
                            next_tick += dt

                            # 送信する行は「メインが生成した last_record_line」
                            with self.record_lock:
                                line = self.record_line_ref.get("line", "")

                            if line and line != last_sent:
                                try:
                                    await client.write_gatt_char(
                                        UART_RX_UUID,
                                        (line + "\n").encode("utf-8"),
                                        response=False
                                    )
                                    last_sent = line
                                except (BleakError, OSError) as e:
                                    print(f"[BLE] write error: {e}")
                                    break

                        await asyncio.sleep(0.001)

                    try:
                        await client.stop_notify(UART_TX_UUID)
                    except Exception:
                        pass

                    print("[BLE] disconnected")

            except Exception as e:
                print(f"[BLE] connect/session error: {e}")

            await asyncio.sleep(backoff)
            backoff = min(backoff_max, backoff * 2)


# ============================================================
# 9) Piローカルログ（保存ログの正本）
# ============================================================
def open_new_pi_log(log_dir: str, header: str):
    os.makedirs(log_dir, exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")
    path = os.path.join(log_dir, f"pi_log_{ts}.csv")
    f = open(path, "w", buffering=1)
    f.write(header + "\n")
    return path, f


# ============================================================
# 10) GPIO パルス
# ============================================================
def gpio_pulse_high(pi: pigpio.pi, gpio_pin: int, sec: float):
    pi.write(gpio_pin, 1)
    print(f"[GPIO] GPIO{gpio_pin} HIGH {sec:.2f}s")
    time.sleep(sec)
    pi.write(gpio_pin, 0)
    print(f"[GPIO] GPIO{gpio_pin} LOW")


# ============================================================
# 11) メイン
# ============================================================
def main():
    # -------------------------
    # 設定（現場で調整する場所）
    # -------------------------
    fall_cfg = FallConfig(dt=0.10, win=11, req=5, sea_level_hpa=1013.25)

    gps_cfg = GpsConfig(
        goal_lat=35.66059,
        goal_lon=139.36688,
        arrival_radius_m=1.0,
        angle_ok_deg=3.0,
    )

    cam_cfg = CameraConfig(
        w=480,
        h=640,
        show_debug=False,  # 本番はFalse推奨
    )

    # --- GPIO ---
    GPIO_LANDING_PIN = 17

    # --- TWELITE soft UART pins ---
    TWELITE_RX_GPIO = 14   # Pi input  (connected to TWELITE_TX)
    TWELITE_TX_GPIO = 15   # Pi output (connected to TWELITE_RX)
    TWELITE_BAUD = 38400
    TW_SEND_HZ = 1.0

    # --- record/log/BLE ---
    LOG_DIR = "./logs"
    LOG_HZ = 10.0

    ble_cfg = BleConfig(enable=True, telem_hz=10.0, scan_timeout=6.0)

    # -------------------------
    # pigpio
    # -------------------------
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpiodに接続できません。sudo systemctl enable --now pigpiod を確認してください。")

    pi.set_mode(GPIO_LANDING_PIN, pigpio.OUTPUT)
    pi.write(GPIO_LANDING_PIN, 0)

    drive = ServoDrive(pi, ServoConfig())

    # -------------------------
    # TWELITE
    # -------------------------
    tw = TweliteSoftUART(pi, TWELITE_RX_GPIO, TWELITE_TX_GPIO, TWELITE_BAUD)

    # -------------------------
    # I2C sensors
    # -------------------------
    i2c = busio.I2C(board.SCL, board.SDA)
    bno = adafruit_bno055.BNO055_I2C(i2c, address=0x28)
    bme = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=0x76)
    bme.sea_level_pressure = fall_cfg.sea_level_hpa

    # -------------------------
    # GPS reader thread
    # -------------------------
    gps_reader = NmeaGpsReader(gps_cfg)
    gps_reader.start()

    # -------------------------
    # Modules
    # -------------------------
    detector = FallLandingDetector(fall_cfg)
    gps_guidance = GpsGuidance(gps_cfg, drive, bno, gps_reader)
    cam_guidance = CameraGuidance(cam_cfg, drive)

    # -------------------------
    # Telemetry/log record pipeline
    # -------------------------
    telem = TelemetryState()
    fmt = TelemetryFormatter()

    # last_record_line は “10Hzで生成する正本”
    record_lock = threading.Lock()
    record_line_ref = {"line": "", "updated_t": 0.0}

    # ローカルログ
    log_path, log_f = open_new_pi_log(LOG_DIR, fmt.header())
    print(f"[PI] local log: {log_path}")

    # TWELITEはヘッダ送信（地上局でCSV保存しやすい）
    try:
        tw.write_line(fmt.header())
    except Exception as e:
        print("[TW] header send error:", e)

    # -------------------------
    # BLE worker (optional)
    # -------------------------
    ctrl = RuntimeControl(logging=True, mode=MODE_RUN, phase_override=None)
    ctrl_lock = threading.Lock()

    stop_flag = threading.Event()

    if ble_cfg.enable:
        if BLE_AVAILABLE:
            ble_worker = BleWorker(
                cfg=ble_cfg,
                ctrl=ctrl,
                ctrl_lock=ctrl_lock,
                record_line_ref=record_line_ref,
                record_lock=record_lock,
                stop_flag=stop_flag,
            )
            ble_worker.start()
        else:
            print("[BLE] enable=True but bleak not installed -> BLE disabled")

    # -------------------------
    # CAMERAは必要になってから起動（負荷を下げる）
    # -------------------------
    picam2 = None

    # -------------------------
    # 周期タイマー
    # -------------------------
    next_record = time.monotonic() + 1.0 / LOG_HZ     # 10Hz: CSV正本生成 + ログ + BLE送信用更新
    next_tw_send = time.monotonic() + 1.0 / TW_SEND_HZ  # 1Hz: TWELITE

    # -------------------------
    # フェーズ
    # -------------------------
    phase = Phase.DROP
    print("=== CanSat Integrated Start ===")
    print(f"Phase: {phase}")

    # SIGINT/SIGTERMで止める
    def _sig_handler(*_):
        stop_flag.set()

    signal.signal(signal.SIGINT, _sig_handler)
    signal.signal(signal.SIGTERM, _sig_handler)

    try:
        while (not stop_flag.is_set()) and (phase != Phase.DONE):
            now = time.monotonic()

            # ========================================================
            # (A) BLEコマンド反映（mode/phase_override/logging）
            # ========================================================
            with ctrl_lock:
                telem.mode = ctrl.mode
                phase_override = ctrl.phase_override
                logging_enabled = ctrl.logging

            # debug用途：phase_override がセットされていたら優先する
            if phase_override is not None and phase_override != phase:
                phase = phase_override
                print(f"[OVERRIDE] Phase forced -> {phase}")

            # modeがidleなら安全のため停止（運用上の保険）
            if telem.mode == MODE_IDLE:
                drive.stop()

            # ========================================================
            # (B) テレメに共通値を反映（毎ループ）
            # ========================================================
            telem.phase = phase
            telem.servo18_us = drive.last_us18
            telem.servo12_us = drive.last_us12

            # BME280
            try:
                telem.temp_c = float(bme.temperature)
                telem.press_hpa = float(bme.pressure)
                telem.humid_pct = float(bme.humidity)
                telem.alt_bme_m = float(bme.altitude)
            except Exception:
                pass

            # GPS fix
            fix = gps_reader.get()
            telem.lat = fix.lat
            telem.lon = fix.lon
            telem.gps_alt = fix.alt
            telem.fixq = fix.fixq
            telem.nsat = fix.nsat
            telem.hdop = fix.hdop

            # ========================================================
            # (C) フェーズ処理
            # ========================================================
            if phase == Phase.DROP:
                alt = float(bme.altitude)
                acc = bno.acceleration
                st = detector.update(alt, acc)

                telem.acc_med = st.acc_med
                telem.dalt_med = st.dalt_med
                telem.ff_consec = st.ff_consec
                telem.land_consec = st.land_consec

                if st.phase == "LANDING" and st.landing_confirmed:
                    print("[DROP] LANDING CONFIRMED")
                    gpio_pulse_high(pi, GPIO_LANDING_PIN, 1.0)

                    phase = Phase.GPS
                    print(f"Phase: {phase}")

                # DROPはあなたのDTで回す
                time.sleep(fall_cfg.dt)

            elif phase == Phase.GPS:
                dt = 1.0 / gps_cfg.control_hz
                t0 = time.monotonic()

                out = gps_guidance.step()

                # ★必須：距離・制御履歴（提出/無線バックアップ）
                if "dist" in out:
                    telem.dist_to_goal_m = out.get("dist")
                    telem.bearing_goal_deg = out.get("bearing_goal")
                    telem.heading_deg = out.get("heading")
                    telem.err_deg = out.get("err")
                    telem.v_cmd = out.get("v")
                    telem.w_cmd = out.get("w")

                if out.get("arrived", False):
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
                    print("[CAM] camera started")

                frame = picam2.capture_array("main")
                if frame is not None:
                    out = cam_guidance.step(frame)
                    telem.cam_mode = out.get("mode")
                    telem.red_ratio = out.get("red_ratio")

                    if out.get("goal", False):
                        phase = Phase.DONE
                        telem.phase = phase
                        print(f"Phase: {phase}")
                # カメラはフレーム依存なので sleepは置かない（captureが律速）

            else:
                # 念のため
                drive.stop()
                time.sleep(0.05)

            # ========================================================
            # (D) 10Hz: “保存ログの正本” を1回だけ生成して共有
            #     - Piローカルログ
            #     - BLE送信用 line 更新
            # ========================================================
            now2 = time.monotonic()
            if now2 >= next_record:
                next_record += 1.0 / LOG_HZ

                telem.seq += 1
                t_ms = int(now2 * 1000)
                line = fmt.format_line(t_ms, telem)

                # Piローカルログ（提出向け）
                if logging_enabled:
                    try:
                        log_f.write(line + "\n")
                    except Exception as e:
                        print("[PI LOG] write error:", e)

                # BLEへ渡す（BLEはこの line をそのまま送る）
                with record_lock:
                    record_line_ref["line"] = line
                    record_line_ref["updated_t"] = now2

            # ========================================================
            # (E) 1Hz: TWELITE 送信（最新の正本 line を送る）
            # ========================================================
            if now2 >= next_tw_send:
                next_tw_send += 1.0 / TW_SEND_HZ
                with record_lock:
                    line = record_line_ref.get("line", "")

                if line:
                    try:
                        tw.write_line(line)
                    except Exception as e:
                        print("[TW] send error:", e)

                # 必要ならTWELITE受信も読む（任意）
                try:
                    for r in tw.read_lines():
                        if r:
                            print("[TW RX]", r)
                except Exception:
                    pass

    except KeyboardInterrupt:
        stop_flag.set()
        print("Ctrl-C")

    finally:
        # -------------------------
        # 安全停止
        # -------------------------
        stop_flag.set()

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
            tw.close()
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
            log_f.flush()
            log_f.close()
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