#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
CanSat 統合スクリプト
  Start
    -> 落下検知 + 着地判定 + GPIO17 HIGH(1秒)
    -> GPS誘導
    -> カメラ誘導（赤コーン）
    -> Done

想定ハード:
  - Raspberry Pi Zero 2 W
  - BME280 (I2C, 0x76) : altitude
  - BNO055 (I2C, 0x28) : acceleration / heading(euler)
  - GPS (I2C, 0x42) : NMEAを読み取る（RMC/GGA）
  - サーボ(連続回転) 2個 : pigpio でPWM(us)制御
  - GPIO17 : 着地後に1秒HIGH（例: ニクロム/イベントトリガ）

注意:
  - 本番では "cv2.imshow" が重い/表示できない場合が多いので、デバッグ表示はOFFが安全
  - BNO055のeuler[0] は None のことがある（キャリブレーション等）ので、その場合は停止
"""

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


# ============================================================
# 0) 共通ユーティリティ
# ============================================================
def clamp(x, lo, hi):
    """xを[lo, hi]に収める（floatもOK）"""
    return lo if x < lo else hi if x > hi else x

def clamp_int(x, lo, hi):
    """xをintにして[lo, hi]に収める（サーボ等で使用）"""
    return int(max(lo, min(hi, int(x))))

def wrap_to_180(a_deg):
    """角度差を -180..+180 に正規化（deg）"""
    while a_deg > 180:
        a_deg -= 360
    while a_deg < -180:
        a_deg += 360
    return a_deg

def norm3(v):
    """3軸ベクトルのノルム（Noneが混じる場合はNone）"""
    if v is None:
        return None
    x, y, z = v
    if x is None or y is None or z is None:
        return None
    return math.sqrt(x * x + y * y + z * z)

def median_or_none(buf: deque):
    """dequeが満杯なら中央値、満杯でなければNone（起動直後の安定化用）"""
    if len(buf) < buf.maxlen:
        return None
    return statistics.median(buf)

def haversine_m(lat1, lon1, lat2, lon2):
    """緯度経度2点の地表距離[m]"""
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def bearing_deg(lat1, lon1, lat2, lon2):
    """
    方位角（北=0, 東=90, 時計回り）0..360
    """
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    b = math.degrees(math.atan2(y, x))
    return (b + 360) % 360

def dm_to_deg(dm, direction):
    """
    NMEAのddmm.mmmm形式を度(deg)へ
    dm: 例 3539.1234
    direction: 'N','S','E','W'
    """
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
# 1) pigpio サーボ駆動（GPS用 / カメラ用の両方を内包）
# ============================================================
@dataclass
class ServoConfig:
    # 停止パルス
    stop_us: int = 1490
    # パルス範囲
    min_us: int = 500
    max_us: int = 2500

    # --- ピン割り当て（あなたのコードの数字に合わせる）---
    # ※名称（left/right）はコード間で揺れていたので、"ピン番号"が真実として扱う
    pin18: int = 18
    pin12: int = 12

    # --- GPS(v,w)用 ---
    scale_us: int = 500  # v,w=1で±500us
    # --- カメラ(fwd,turn)用 ---
    max_delta: int = 600  # fwd/turn の上限

class ServoDrive:
    """
    同じサーボ2つを、2種類のAPIで動かす
      - set_vw(v,w) : GPS誘導で使っていた形式
      - set_diff(fwd,turn) : カメラ誘導で使っていた形式
    """
    def __init__(self, pi: pigpio.pi, cfg: ServoConfig):
        self.pi = pi
        self.cfg = cfg
        self.stop()

    def _write(self, us18, us12):
        """内部: ピン18/ピン12にパルス幅を出す"""
        u18 = clamp_int(us18, self.cfg.min_us, self.cfg.max_us)
        u12 = clamp_int(us12, self.cfg.min_us, self.cfg.max_us)
        self.pi.set_servo_pulsewidth(self.cfg.pin18, u18)
        self.pi.set_servo_pulsewidth(self.cfg.pin12, u12)

    def stop(self):
        """停止（ニュートラル）"""
        self._write(self.cfg.stop_us, self.cfg.stop_us)

    def free(self):
        """サーボ出力解除（0us）。停止とは別物なので注意。"""
        self.pi.set_servo_pulsewidth(self.cfg.pin18, 0)
        self.pi.set_servo_pulsewidth(self.cfg.pin12, 0)

    # -------------------------
    # GPS誘導で使っていた API
    # -------------------------
    def set_vw(self, v, w):
        """
        v: 前進 0..1
        w: 右旋回 +、左旋回 -
        ※符号や割り当ては、あなたのGPSコードの式をそのまま採用
        """
        v = clamp(v, 0.0, 1.0)
        w = clamp(w, -1.0, 1.0)

        us18 = self.cfg.stop_us - v * self.cfg.scale_us + w * self.cfg.scale_us
        us12 = self.cfg.stop_us + v * self.cfg.scale_us + w * self.cfg.scale_us
        self._write(us18, us12)

    # -------------------------
    # カメラ誘導で使っていた API
    # -------------------------
    def set_diff(self, fwd, turn):
        """
        fwd  > 0 で前進
        turn > 0 で右旋回（ただし実機に合わせてここで反転済み）

        あなたのカメラコードの実測事実に合わせた変換:
          - pin18: 小さいほど前進 → STOP - power
          - pin12: 大きいほど前進 → STOP + power
        """
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

    # ---- FREEFALL (OR条件) ----
    freefall_acc_th: float = 8.5
    freefall_dalt_th: float = -0.10   # 0.1sで-0.10m → -1m/s相当（10Hz前提）

    # ---- LANDING (OR条件) ----
    acc_land_min: float = 6.0
    acc_land_max: float = 13.0
    land_dalt_abs_th: float = 0.05

@dataclass
class FallStatus:
    phase: str                 # "WARMUP" | "FREEFALL" | "LANDING"
    alt: float | None = None
    acc_med: float | None = None
    dalt_med: float | None = None
    cond: bool | None = None
    consec: int = 0
    confirmed: bool = False

class FallLandingDetector:
    """
    BME280高度差(dAlt) と BNO055加速度ノルム(|a|) の中央値フィルタで、
      - FREEFALL判定（OR）
      - LANDING判定（OR）
    を連続成立回数で確定する。
    """

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
        """
        1周期分更新して状態を返す。
        """
        c = self.cfg

        # 初回はdAltが作れないのでWARMUP
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

        # -------------------------
        # FREEFALL判定
        # -------------------------
        if not self.freefall_confirmed:
            ff_cond = (acc_med < c.freefall_acc_th) or (dalt_med < c.freefall_dalt_th)
            self.ff_consec = (self.ff_consec + 1) if ff_cond else 0
            confirmed = (self.ff_consec >= c.req)

            return FallStatus(
                phase="FREEFALL",
                alt=alt_m,
                acc_med=acc_med,
                dalt_med=dalt_med,
                cond=ff_cond,
                consec=self.ff_consec,
                confirmed=confirmed,
            )

        # -------------------------
        # LANDING判定
        # -------------------------
        land_cond = ((c.acc_land_min <= acc_med <= c.acc_land_max) or (abs(dalt_med) < c.land_dalt_abs_th))
        self.land_consec = (self.land_consec + 1) if land_cond else 0
        confirmed = (self.land_consec >= c.req)

        return FallStatus(
            phase="LANDING",
            alt=alt_m,
            acc_med=acc_med,
            dalt_med=dalt_med,
            cond=land_cond,
            consec=self.land_consec,
            confirmed=confirmed,
        )


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

    # 到達判定（あなたのコメントに矛盾があったので値を真実として扱う）
    arrival_radius_m: float = 1.0
    angle_ok_deg: float = 3.0

    # 速度制御
    base_v: float = 0.70
    min_v: float = 0.35
    slowdown_dist_m: float = 8.0

    # 旋回制御
    kp: float = 0.1
    w_max: float = 0.60
    w_bias: float = +0.06

class NmeaGpsReader:
    """GPS(I2C)のNMEAを読み、最新fixを共有する。"""

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

                    # RMC: 位置（status A）
                    if msg.sentence_type == "RMC" and getattr(msg, "status", "") == "A":
                        lat = dm_to_deg(getattr(msg, "lat", None), getattr(msg, "lat_dir", None))
                        lon = dm_to_deg(getattr(msg, "lon", None), getattr(msg, "lon_dir", None))
                        if lat is not None and lon is not None:
                            with self._lock:
                                self._fix.lat = lat
                                self._fix.lon = lon
                                self._fix.t_mono = time.monotonic()

                    # GGA: 品質
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
# 4) GPS誘導コントローラ
# ============================================================
class GpsGuidance:
    def __init__(self, cfg: GpsConfig, drive: ServoDrive, bno, gps_reader: NmeaGpsReader, logger=print):
        self.cfg = cfg
        self.drive = drive
        self.bno = bno
        self.gps = gps_reader
        self.log = logger
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

    def step(self) -> bool:
        """
        1周期分のGPS誘導を行う。
        戻り値:
          True  -> 目標に到達（カメラ誘導へ遷移してよい）
          False -> 継続
        """
        c = self.cfg
        fix = self.gps.get()

        # heading取得（Noneなら安全側で停止）
        heading = None
        try:
            e = self.bno.euler
            if e is not None:
                heading = e[0]  # 北0 東90
        except Exception:
            heading = None

        if heading is None or not self._gps_ok(fix):
            self.drive.stop()
            return False

        dist = haversine_m(fix.lat, fix.lon, c.goal_lat, c.goal_lon)
        if dist <= c.arrival_radius_m:
            self.drive.stop()
            self.log(f"[GPS] ARRIVED dist={dist:.2f}m <= {c.arrival_radius_m}m")
            return True

        # 目標方位と機体方位
        theta_goal = bearing_deg(fix.lat, fix.lon, c.goal_lat, c.goal_lon)
        err = wrap_to_180(theta_goal - heading)

        # 距離に応じて減速
        if dist < c.slowdown_dist_m:
            a = clamp(dist / c.slowdown_dist_m, 0.0, 1.0)
            v = c.min_v + (c.base_v - c.min_v) * a
        else:
            v = c.base_v

        # フローチャート条件: |err| < ANGLE_OK_DEG ならバイアスだけ
        if abs(err) < c.angle_ok_deg:
            w = c.w_bias
        else:
            w = clamp(c.kp * err + c.w_bias, -c.w_max, c.w_max)

        self.drive.set_vw(v, w)

        # 1秒に1回だけ表示（ログを軽く）
        now = time.monotonic()
        if now - self._last_log > 1.0:
            self.log(
                f"[GPS] dist={dist:6.1f}m goal={theta_goal:6.1f} head={heading:6.1f} err={err:6.1f} "
                f"v={v:.2f} w={w:.2f} nsat={fix.nsat} hdop={fix.hdop}"
            )
            self._last_log = now

        return False


# ============================================================
# 5) カメラ誘導（赤コーン）
# ============================================================
@dataclass
class CameraConfig:
    # カメラ解像度（picamera2 BGR888）
    w: int = 480
    h: int = 640

    # デバッグ表示（現場で重ければ False 推奨）
    show_debug: bool = False

    # ---- 追跡パラメータ（あなたの値を採用）----
    kp_turn: float = 0.8
    base_fwd: int = 250
    search_turn: int = 200
    area_track_th: int = 900
    area_fwd_th: int = 2200
    center_tol_ratio: float = 0.08

    # ---- 0mゴール判定（あなたの値を採用）----
    slow_red_ratio: float = 0.35
    goal_red_ratio: float = 0.60
    goal_hold_sec: float = 0.25

class CameraGuidance:
    """
    カメラの赤検出で誘導する。
      - 遠距離: 三角形っぽい輪郭を追跡し中心合わせ
      - 近距離: 形が崩れるので赤率で減速/停止
    """

    # 赤色HSV範囲（固定）
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

    def _make_red_mask(self, frame_bgr):
        """
        IMPORTANT:
          frameはBGRなので BGR2HSV を使うのが正しい
        """
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_RGB2HSV)
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

    def step(self, frame_bgr) -> bool:
        """
        1フレーム分のカメラ誘導を行う。
        戻り値:
          True  -> ゴール確定（停止して終了してよい）
          False -> 継続
        """
        c = self.cfg

        # 物理設置が時計回り90° → ソフトで反時計回り90°補正
        frame = cv2.rotate(frame_bgr, cv2.ROTATE_90_COUNTERCLOCKWISE)

        h, w = frame.shape[:2]
        cx_frame = w // 2
        center_tol_px = int(w * c.center_tol_ratio)

        mask = self._make_red_mask(frame)

        # 赤率（画面に占める赤の割合）
        red_ratio = cv2.countNonZero(mask) / float(h * w)

        # -------- 0mゴール判定 --------
        if red_ratio >= c.goal_red_ratio:
            if self._goal_start_t is None:
                self._goal_start_t = time.time()
            elif (time.time() - self._goal_start_t) >= c.goal_hold_sec:
                self.drive.stop()
                self.log(f"[CAM] GOAL red_ratio={red_ratio:.3f}")
                return True
        else:
            self._goal_start_t = None

        # -------- 輪郭から三角形を探す --------
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

        # -------- 誘導 --------
        if best is None:
            # 近距離では三角形が崩れやすいので、赤率が高ければ「減速直進」
            if red_ratio >= c.slow_red_ratio:
                self.drive.set_diff(int(c.base_fwd * 0.35), 0)
                status = "NEAR_FWD"
            else:
                self.drive.set_diff(0, c.search_turn)
                status = "SEARCH"
            self._debug_show(c, frame, mask, status, red_ratio, cx_frame, best_approx)
            return False

        M = cv2.moments(best)
        if M["m00"] == 0:
            # 重心が作れない輪郭（レア）
            if red_ratio >= c.slow_red_ratio:
                self.drive.set_diff(int(c.base_fwd * 0.35), 0)
                status = "NEAR_FWD"
            else:
                self.drive.set_diff(0, c.search_turn)
                status = "M00=0"
            self._debug_show(c, frame, mask, status, red_ratio, cx_frame, best_approx)
            return False

        cx = int(M["m10"] / M["m00"])
        err = cx - cx_frame

        if abs(err) < center_tol_px:
            # 中心に入った → 前進（近距離は減速）
            if red_ratio >= c.slow_red_ratio:
                fwd = int(c.base_fwd * 0.35)
            else:
                fwd = c.base_fwd if best_area < c.area_fwd_th else int(c.base_fwd * 0.8)

            self.drive.set_diff(fwd, 0)
            status = "FORWARD"
        else:
            # 旋回（近距離は過敏にすると暴れるので抑える）
            turn_gain = 0.6 if red_ratio >= c.slow_red_ratio else 1.0
            turn = int(turn_gain * c.kp_turn * err)
            self.drive.set_diff(0, turn)
            status = "TURN"

        self._debug_show(c, frame, mask, status, red_ratio, cx_frame, best_approx)
        return False

    def _debug_show(self, c, frame, mask, status, red_ratio, cx_frame, best_approx):
        """デバッグ表示（重いので本番はOFF推奨）"""
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
# 6) フェーズ（状態機械）
# ============================================================
class Phase:
    DROP = "DROP"        # 落下検知→着地判定
    GPS = "GPS"          # GPS誘導
    CAMERA = "CAMERA"    # カメラ誘導
    DONE = "DONE"


# ============================================================
# 7) メイン（統合）
# ============================================================
def gpio_pulse_high(pi: pigpio.pi, gpio_pin: int, sec: float, logger=print):
    """GPIOをsec秒だけHIGHにして戻す"""
    pi.write(gpio_pin, 1)
    logger(f"[GPIO] PIN{gpio_pin} HIGH for {sec:.2f}s")
    time.sleep(sec)
    pi.write(gpio_pin, 0)
    logger(f"[GPIO] PIN{gpio_pin} LOW")


def main():
    # -------------------------
    # 設定（ここが一番いじる場所）
    # -------------------------
    fall_cfg = FallConfig(
        dt=0.10,
        win=11,
        req=3,
        sea_level_hpa=1013.25,
    )

    gps_cfg = GpsConfig(
        goal_lat=35.66059,
        goal_lon=139.36688,
        arrival_radius_m=1.0,   # あなたのコードの値を採用（コメントではなく値を真実）
        angle_ok_deg=3.0,
    )

    cam_cfg = CameraConfig(
        w=480,
        h=640,
        show_debug=False,  # 現場ではFalse推奨（重い/表示できないことが多い）
    )

    GPIO_LANDING_PIN = 17

    # -------------------------
    # pigpio / センサ初期化
    # -------------------------
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio daemonに接続できません。sudo pigpiod を確認してください。")

    # GPIO17（着地後に1秒HIGH）
    pi.set_mode(GPIO_LANDING_PIN, pigpio.OUTPUT)
    pi.write(GPIO_LANDING_PIN, 0)

    # サーボ
    servo_cfg = ServoConfig(stop_us=1490, min_us=500, max_us=2500, pin18=18, pin12=12)
    drive = ServoDrive(pi, servo_cfg)

    # I2C（BME280 / BNO055）
    i2c = busio.I2C(board.SCL, board.SDA)
    bno = adafruit_bno055.BNO055_I2C(i2c, address=0x28)
    bme = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=0x76)
    bme.sea_level_pressure = fall_cfg.sea_level_hpa

    # 落下・着地検知器
    detector = FallLandingDetector(fall_cfg, logger=print)

    # GPS受信
    gps_reader = NmeaGpsReader(gps_cfg, logger=print)
    gps_reader.start()

    # GPS誘導
    gps_guidance = GpsGuidance(gps_cfg, drive, bno, gps_reader, logger=print)

    # カメラ誘導（Picamera2はカメラフェーズで初期化する：無駄な負荷を避ける）
    cam_guidance = CameraGuidance(cam_cfg, drive, logger=print)
    picam2 = None

    # -------------------------
    # フェーズ開始
    # -------------------------
    phase = Phase.DROP
    print("=== CanSat Integrated Program Start ===")
    print(f"Phase: {phase}")

    try:
        while phase != Phase.DONE:
            # -------------------------
            # 1) 落下→着地
            # -------------------------
            if phase == Phase.DROP:
                alt = bme.altitude
                acc = bno.acceleration
                st = detector.update(alt, acc)

                # 起動直後：中央値がまだ作れないので待つ
                if st.phase == "WARMUP":
                    time.sleep(fall_cfg.dt)
                    continue

                if st.phase == "FREEFALL":
                    print(f"[DROP] FREEFALL? alt={st.alt:.2f} acc_med={st.acc_med:.2f} "
                          f"dalt_med={st.dalt_med:.3f} cond={st.cond} consec={st.consec}")
                    if st.confirmed:
                        detector.freefall_confirmed = True
                        detector.land_consec = 0
                        print("[DROP] FREEFALL CONFIRMED")

                elif st.phase == "LANDING":
                    print(f"[DROP] LAND? alt={st.alt:.2f} acc_med={st.acc_med:.2f} "
                          f"dalt_med={st.dalt_med:.3f} cond={st.cond} consec={st.consec}")
                    if st.confirmed:
                        print("[DROP] LANDING CONFIRMED")

                        # 着地後イベント：GPIO17を1秒HIGH
                        gpio_pulse_high(pi, GPIO_LANDING_PIN, 1.0, logger=print)

                        # 次フェーズへ
                        phase = Phase.GPS
                        print(f"Phase: {phase}")

                time.sleep(fall_cfg.dt)
                continue

            # -------------------------
            # 2) GPS誘導
            # -------------------------
            if phase == Phase.GPS:
                dt = 1.0 / gps_cfg.control_hz
                t0 = time.monotonic()

                arrived = gps_guidance.step()
                if arrived:
                    # GPS到達後はカメラ誘導へ
                    phase = Phase.CAMERA
                    print(f"Phase: {phase}")

                # 周期をCONTROL_HZに近づける（重くても破綻しにくい形）
                time.sleep(max(0.0, dt - (time.monotonic() - t0)))
                continue

            # -------------------------
            # 3) カメラ誘導
            # -------------------------
            if phase == Phase.CAMERA:
                # カメラはここで初期化（GPS中に無駄に回さない）
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
                if goal:
                    phase = Phase.DONE
                    print(f"Phase: {phase}")
                continue

    except KeyboardInterrupt:
        print("Ctrl-C")
    finally:
        # -------------------------
        # 終了処理（安全停止を最優先）
        # -------------------------
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