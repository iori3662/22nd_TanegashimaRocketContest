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
# ユーザ設定
# ============================================================
GOAL_LAT = 35.00000000
GOAL_LON = 139.00000000

# フローチャート記載
ARRIVAL_RADIUS_M = 5.0      # 「目的座標との距離が5m以内か」
ANGLE_OK_DEG = 5.0          # 「|θ2-θ1| < 5」

CONTROL_HZ = 5.0            # 分岐ベースなので低めでもOK（必要なら上げる）
GPS_MIN_FIXQ = 1
GPS_MIN_SATS = 5
GPS_MAX_HDOP = 5.0
GPS_STALE_SEC = 2.5

# pigpio サーボ設定（あなたの値）
SERVO1_PIN = 18
SERVO2_PIN = 12
SERVO1_STOP_US = 1480
SERVO2_STOP_US = 1490
SERVO_US_MIN = 1000
SERVO_US_MAX = 2000
SERVO_SCALE_US = 500  # cmd=±1で±500us

# 行動の時間（フローチャートは「右折/左折/前進」なので時間で実装）
TURN_SEC = 0.35        # 旋回させる時間
FWD_SEC  = 0.50        # 前進させる時間
FWD_CMD  = 0.60        # 前進強さ(0..1)
TURN_CMD = 0.55        # 旋回強さ(0..1) ※場で調整

# GPS I2C
GPS_BUS = 1
GPS_ADDR = 0x42

# ============================================================
# ユーティリティ
# ============================================================
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def wrap_to_180(angle_deg: float) -> float:
    while angle_deg > 180.0:
        angle_deg -= 360.0
    while angle_deg < -180.0:
        angle_deg += 360.0
    return angle_deg

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
    # 北=0 東=90（時計回り）0..360
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dlmb = math.radians(lon2 - lon1)
    y = math.sin(dlmb) * math.cos(phi2)
    x = math.cos(phi1)*math.sin(phi2) - math.sin(phi1)*math.cos(phi2)*math.cos(dlmb)
    brng = math.degrees(math.atan2(y, x))
    return (brng + 360.0) % 360.0

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
# GPS共有状態
# ============================================================
gps_latest = {
    "lat": None, "lon": None, "alt": None,
    "fixq": 0, "nsat": 0, "hdop": None,
    "updated_monotonic": None,
}
gps_lock = threading.Lock()

def gps_reader_thread(stop_event: threading.Event):
    buf = bytearray()
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

            time.sleep(0.02)

# ============================================================
# サーボ（差動）
# ============================================================
class DifferentialServoPigpio:
    def __init__(self, pi):
        self.pi = pi
        self.stop()

    def _cmd_to_us(self, cmd, stop_us):
        cmd = clamp(cmd, -1.0, 1.0)
        us = int(round(stop_us + cmd * SERVO_SCALE_US))
        return int(clamp(us, SERVO_US_MIN, SERVO_US_MAX))

    def set(self, left_cmd, right_cmd):
        usL = self._cmd_to_us(left_cmd, SERVO1_STOP_US)
        usR = self._cmd_to_us(right_cmd, SERVO2_STOP_US)
        self.pi.set_servo_pulsewidth(SERVO1_PIN, usL)
        self.pi.set_servo_pulsewidth(SERVO2_PIN, usR)

    def stop(self):
        self.pi.set_servo_pulsewidth(SERVO1_PIN, SERVO1_STOP_US)
        self.pi.set_servo_pulsewidth(SERVO2_PIN, SERVO2_STOP_US)

    def free(self):
        self.pi.set_servo_pulsewidth(SERVO1_PIN, 0)
        self.pi.set_servo_pulsewidth(SERVO2_PIN, 0)

# 行動プリミティブ（フローチャートの「右折/左折/前進」）
def do_forward(servo: DifferentialServoPigpio, sec: float):
    servo.set(FWD_CMD, FWD_CMD)
    time.sleep(sec)
    servo.stop()

def do_turn_right(servo: DifferentialServoPigpio, sec: float):
    # 右折：左を前、右を後（その場旋回）
    servo.set(+TURN_CMD, -TURN_CMD)
    time.sleep(sec)
    servo.stop()

def do_turn_left(servo: DifferentialServoPigpio, sec: float):
    # 左折：左を後、右を前（その場旋回）
    servo.set(-TURN_CMD, +TURN_CMD)
    time.sleep(sec)
    servo.stop()

# ============================================================
# メイン：GPS誘導フェーズ（フローチャート準拠）
# ============================================================
def main():
    # pigpio
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio not connected. pigpiod が起動しているか確認してください。")
    servo = DifferentialServoPigpio(pi)

    # BNO055
    i2c = busio.I2C(board.SCL, board.SDA)
    sensor = adafruit_bno055.BNO055_I2C(i2c, address=0x28)

    # GPS thread start
    stop_event = threading.Event()
    th = threading.Thread(target=gps_reader_thread, args=(stop_event,), daemon=True)
    th.start()

    print("=== GPS誘導フェーズ start ===")
    print(f"Goal lat={GOAL_LAT}, lon={GOAL_LON}")
    print("※根拠：提示フローチャート（距離5m/角度差5degの分岐）")

    dt = 1.0 / CONTROL_HZ
    last_log = time.monotonic()

    try:
        while True:
            t0 = time.monotonic()

            # ---- GPS取得（フローチャート：GPSから座標を取得）----
            with gps_lock:
                lat = gps_latest["lat"]
                lon = gps_latest["lon"]
                fixq = gps_latest["fixq"]
                nsat = gps_latest["nsat"]
                hdop = gps_latest["hdop"]
                upd = gps_latest["updated_monotonic"]

            gps_ok = (
                (lat is not None) and (lon is not None) and
                (fixq >= GPS_MIN_FIXQ) and
                (nsat >= GPS_MIN_SATS) and
                (hdop is not None) and (hdop <= GPS_MAX_HDOP) and
                (upd is not None) and ((time.monotonic() - upd) < GPS_STALE_SEC)
            )

            # ---- 方位取得（フローチャート：機体の方位角θ2）----
            heading = sensor.euler[0]  # 北0東90
            if heading is None:
                servo.stop()
                time.sleep(dt)
                continue

            if not gps_ok:
                servo.stop()
                if time.monotonic() - last_log > 1.0:
                    print(f"[WAIT] GPS未準備 fixq={fixq} nsat={nsat} hdop={hdop} heading={heading:.1f}")
                    last_log = time.monotonic()
                time.sleep(max(0.0, dt - (time.monotonic() - t0)))
                continue

            # ---- 距離算出（フローチャート：目的座標との距離を算出）----
            dist = haversine_m(lat, lon, GOAL_LAT, GOAL_LON)

            # ---- 分岐：距離が5m以内か ----
            if dist <= ARRIVAL_RADIUS_M:
                servo.stop()
                print(f"[DONE] dist={dist:.2f}m <= {ARRIVAL_RADIUS_M}m → 次フェーズへ（ここで停止）")
                break

            # ---- 目標方位角θ1（フローチャート：目標座標の方位角θ1）----
            theta1 = bearing_deg(lat, lon, GOAL_LAT, GOAL_LON)   # 0..360
            theta2 = heading                                    # 0..360想定

            # フローチャートの |θ2-θ1| 判定は、0/360跨ぎを考えるとwrapが必要
            err = wrap_to_180(theta2 - theta1)  # θ2-θ1 を -180..180 へ
            err_abs = abs(err)

            # ---- 分岐：|θ2-θ1| < 5 ----
            if err_abs < ANGLE_OK_DEG:
                # 前進
                do_forward(servo, FWD_SEC)
            else:
                # ---- 以下：フローチャートの「θ2>θ1」「差が180未満」分岐に相当 ----
                # 図の右側分岐に誤記の可能性があるので、最短回転方向で安全に決める
                # err = θ2-θ1:
                #   err > 0 → 機体が目標より右に向き過ぎ → 左へ戻す（左折）
                #   err < 0 → 機体が目標より左に向き過ぎ → 右へ戻す（右折）
                if err > 0:
                    do_turn_left(servo, TURN_SEC)
                else:
                    do_turn_right(servo, TURN_SEC)

                # 旋回の後に前進（フローチャート：右折/左折 → 前進）
                do_forward(servo, FWD_SEC)

            if time.monotonic() - last_log > 1.0:
                # 監視用ログ
                print(f"[GPS] dist={dist:6.1f}m θ1={theta1:6.1f} θ2={theta2:6.1f} (θ2-θ1)={err:6.1f} "
                      f"nsat={nsat} hdop={hdop}")
                last_log = time.monotonic()

            time.sleep(max(0.0, dt - (time.monotonic() - t0)))

    except KeyboardInterrupt:
        print("Ctrl-C: stop")
    finally:
        servo.stop()
        stop_event.set()
        time.sleep(0.2)
        servo.free()
        pi.stop()
        print("Finish")

if __name__ == "__main__":
    main()