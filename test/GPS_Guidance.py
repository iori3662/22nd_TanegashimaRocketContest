import time
import math
import threading

from smbus2 import SMBus
import pynmea2

import board
import busio
import adafruit_bno055

import pigpio

# ============================================================
# 1) ユーザ設定（ここを編集）
# ============================================================
GOAL_LAT = 35.00000000   # 目的地 緯度（例）
GOAL_LON = 139.00000000  # 目的地 経度（例）

STOP_RADIUS_M = 2.0      # この距離[m]以内で停止
CONTROL_HZ = 10.0        # 制御周期（BNO055を読む周波数）

# GPS品質フィルタ（厳しすぎると動かないので、現場で調整）
GPS_MIN_FIXQ = 1         # GGA gps_qual >= 1
GPS_MIN_SATS = 5         # まずは5推奨（厳しいなら4へ）
GPS_MAX_HDOP = 5.0       # まずは5.0推奨（厳しいなら8へ）
GPS_STALE_SEC = 2.5      # 何秒更新が無いGPSを無効とするか

# 速度・操舵ゲイン（現場で必ずチューニング）
BASE_SPEED = 0.55        # 0..1（前進の基本）
MIN_SPEED  = 0.30        # 0..1（近距離の最低速度）
SLOWDOWN_DIST_M = 8.0    # ここより近づくと減速

KP_HEADING = 0.015       # [1/deg] 旋回Pゲイン（大きいと蛇行/発振）
TURN_MAX = 0.45          # 0..1 旋回成分上限

# ============================================================
# 2) GPS(I2C NMEA) 設定
# ============================================================
GPS_BUS = 1
GPS_ADDR = 0x42

# ============================================================
# 3) サーボ(pigpio) 設定（あなたの値を採用）
# ============================================================
SERVO1_PIN = 18  # GPIO18
SERVO2_PIN = 12  # GPIO12

SERVO1_STOP_US = 1480
SERVO2_STOP_US = 1490

SERVO_US_MIN = 1000
SERVO_US_MAX = 2000

# サーボ指令のスケーリング（-1..+1 -> pulsewidth）
# 連続回転サーボは個体差が大きいので、必要なら左右別ゲイン化してください
SERVO_SCALE_US = 500  # 1.0 で ±500us（STOP±500=1000/2000）

# ============================================================
# 4) ユーティリティ
# ============================================================
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

def wrap_to_180(angle_deg: float) -> float:
    while angle_deg > 180.0:
        angle_deg -= 360.0
    while angle_deg < -180.0:
        angle_deg += 360.0
    return angle_deg

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def haversine_m(lat1, lon1, lat2, lon2) -> float:
    R = 6371000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlmb = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlmb/2)**2
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return R * c

def bearing_deg(lat1, lon1, lat2, lon2) -> float:
    # 北=0 東=90（時計回り） 0..360
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dlmb = math.radians(lon2 - lon1)
    y = math.sin(dlmb) * math.cos(phi2)
    x = math.cos(phi1)*math.sin(phi2) - math.sin(phi1)*math.cos(phi2)*math.cos(dlmb)
    brng = math.degrees(math.atan2(y, x))
    return (brng + 360.0) % 360.0

# ============================================================
# 5) GPS共有状態（スレッドで更新）
# ============================================================
gps_latest = {
    "lat": None, "lon": None, "alt": None,
    "fixq": 0, "nsat": 0, "hdop": None,
    "updated_monotonic": None,
}
gps_lock = threading.Lock()

def gps_reader_thread(stop_event: threading.Event):
    buf = bytearray()
    next_print = time.monotonic()

    print("[GPS] I2C(NMEA) reader start. Outdoor recommended.")

    with SMBus(GPS_BUS) as bus:
        while not stop_event.is_set():
            try:
                data = bus.read_i2c_block_data(GPS_ADDR, 0xFF, 32)
            except OSError:
                time.sleep(0.05)
                continue

            buf.extend(data)

            while b'\n' in buf:
                line, _, rest = buf.partition(b'\n')
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
                        with gps_lock:
                            gps_latest["lat"] = lat
                            gps_latest["lon"] = lon
                            gps_latest["updated_monotonic"] = time.monotonic()

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

                    with gps_lock:
                        gps_latest["fixq"] = fixq
                        gps_latest["nsat"] = nsat
                        gps_latest["hdop"] = hdop
                        if alt is not None:
                            gps_latest["alt"] = alt

            # 1 Hz print
            now = time.monotonic()
            if now >= next_print:
                next_print += 1.0
                with gps_lock:
                    lat = gps_latest["lat"]
                    lon = gps_latest["lon"]
                    fixq = gps_latest["fixq"]
                    nsat = gps_latest["nsat"]
                    hdop = gps_latest["hdop"]
                    alt = gps_latest["alt"]

                if lat is not None and lon is not None and fixq > 0:
                    print(f"[GPS] lat={lat:.8f}, lon={lon:.8f}, alt={alt if alt is not None else float('nan'):.1f} m, "
                          f"fixq={fixq}, nsat={nsat}, hdop={hdop}")
                else:
                    print("[GPS] no valid fix yet (waiting...)")

            time.sleep(0.02)

# ============================================================
# 6) pigpio サーボ差動ドライバ
# ============================================================
class DifferentialServoPigpio:
    """
    set_speeds(left, right): -1..+1
      +1 で正転(2000us側)
      -1 で逆転(1000us側)
      0 で停止(個別stop)
    """
    def __init__(self, pi, pin_left, pin_right,
                 stop_left_us, stop_right_us,
                 us_min=1000, us_max=2000,
                 scale_us=500):
        self.pi = pi
        self.pin_left = pin_left
        self.pin_right = pin_right
        self.stop_left = stop_left_us
        self.stop_right = stop_right_us
        self.us_min = us_min
        self.us_max = us_max
        self.scale = scale_us

        # 初期停止
        self.stop()

    def _cmd_to_us(self, cmd, stop_us):
        cmd = clamp(cmd, -1.0, 1.0)
        us = int(round(stop_us + cmd * self.scale))
        return int(clamp(us, self.us_min, self.us_max))

    def set_speeds(self, left, right):
        usL = self._cmd_to_us(left, self.stop_left)
        usR = self._cmd_to_us(right, self.stop_right)
        self.pi.set_servo_pulsewidth(self.pin_left, usL)
        self.pi.set_servo_pulsewidth(self.pin_right, usR)

    def stop(self):
        self.pi.set_servo_pulsewidth(self.pin_left, self.stop_left)
        self.pi.set_servo_pulsewidth(self.pin_right, self.stop_right)

    def free(self):
        self.pi.set_servo_pulsewidth(self.pin_left, 0)
        self.pi.set_servo_pulsewidth(self.pin_right, 0)

# ============================================================
# 7) メイン（誘導）
# ============================================================
def main():
    # --- pigpio ---
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio.pi() not connected. Did you start pigpiod?")

    servo = DifferentialServoPigpio(
        pi=pi,
        pin_left=SERVO1_PIN,
        pin_right=SERVO2_PIN,
        stop_left_us=SERVO1_STOP_US,
        stop_right_us=SERVO2_STOP_US,
        us_min=SERVO_US_MIN,
        us_max=SERVO_US_MAX,
        scale_us=SERVO_SCALE_US
    )

    # --- BNO055 ---
    i2c = busio.I2C(board.SCL, board.SDA)
    sensor = adafruit_bno055.BNO055_I2C(i2c, address=0x28)

    # --- GPS thread ---
    stop_event = threading.Event()
    th = threading.Thread(target=gps_reader_thread, args=(stop_event,), daemon=True)
    th.start()

    print("=== GPS Guidance Start ===")
    print(f"Goal: lat={GOAL_LAT}, lon={GOAL_LON} | stop_radius={STOP_RADIUS_M} m")
    print("Ctrl-C to stop.")

    dt = 1.0 / CONTROL_HZ
    last_log = time.monotonic()

    try:
        while True:
            t0 = time.monotonic()

            # ---- GPS snapshot ----
            with gps_lock:
                lat = gps_latest["lat"]
                lon = gps_latest["lon"]
                fixq = gps_latest["fixq"]
                nsat = gps_latest["nsat"]
                hdop = gps_latest["hdop"]
                updated = gps_latest["updated_monotonic"]

            gps_ok = (
                (lat is not None) and (lon is not None) and
                (fixq >= GPS_MIN_FIXQ) and
                (nsat >= GPS_MIN_SATS) and
                (hdop is not None) and (hdop <= GPS_MAX_HDOP) and
                (updated is not None) and ((time.monotonic() - updated) < GPS_STALE_SEC)
            )

            # ---- Heading ----
            heading = sensor.euler[0]  # deg: North=0 East=90
            if heading is None:
                servo.stop()
                time.sleep(dt)
                continue

            # ---- Safety: GPS not ready -> stop ----
            if not gps_ok:
                servo.stop()
                if time.monotonic() - last_log > 1.0:
                    print(f"[CTRL] GPS not ready. fixq={fixq}, nsat={nsat}, hdop={hdop}, heading={heading:.1f}")
                    last_log = time.monotonic()
                time.sleep(max(0.0, dt - (time.monotonic() - t0)))
                continue

            # ---- Guidance ----
            dist_m = haversine_m(lat, lon, GOAL_LAT, GOAL_LON)
            brg = bearing_deg(lat, lon, GOAL_LAT, GOAL_LON)  # 0..360
            err = wrap_to_180(brg - heading)                 # -180..180

            # ---- Arrived ----
            if dist_m <= STOP_RADIUS_M:
                servo.stop()
                print(f"[ARRIVED] dist={dist_m:.2f} m <= {STOP_RADIUS_M} m. Stop.")
                time.sleep(0.5)
                continue

            # ---- Speed schedule ----
            if dist_m < SLOWDOWN_DIST_M:
                alpha = clamp(dist_m / SLOWDOWN_DIST_M, 0.0, 1.0)
                v = MIN_SPEED + (BASE_SPEED - MIN_SPEED) * alpha
            else:
                v = BASE_SPEED

            # ---- P control ----
            w = KP_HEADING * err
            w = clamp(w, -TURN_MAX, TURN_MAX)

            # ---- Mix (差動) ----
            left = v - w
            right = v + w

            # 出力が±1を超えないよう正規化
            m = max(1.0, abs(left), abs(right))
            left /= m
            right /= m

            servo.set_speeds(left, right)

            # ---- log (1 Hz) ----
            if time.monotonic() - last_log > 1.0:
                print(f"[CTRL] dist={dist_m:6.1f} m, brg={brg:6.1f} deg, heading={heading:6.1f} deg, err={err:6.1f} deg, "
                      f"L={left:+.2f}, R={right:+.2f}, nsat={nsat}, hdop={hdop}")
                last_log = time.monotonic()

            time.sleep(max(0.0, dt - (time.monotonic() - t0)))

    except KeyboardInterrupt:
        print("Stopping (Ctrl-C)...")
    finally:
        servo.stop()
        stop_event.set()
        time.sleep(0.2)
        servo.free()
        pi.stop()
        print("Finish.")

if __name__ == "__main__":
    main()