import time, math, threading
from smbus2 import SMBus
import pynmea2
import board, busio
import adafruit_bno055
import pigpio

# ========= 目的地（あなた指定） =========
GOAL_LAT = 35.66059
GOAL_LON = 139.36688

# ========= フローチャート閾値 =========
ARRIVAL_RADIUS_M = 1.0   # 距離5m以内
ANGLE_OK_DEG = 3.0       # |θ2-θ1|<5deg

# ========= 制御・フィルタ =========
CONTROL_HZ = 10.0
GPS_BUS = 1
GPS_ADDR = 0x42
GPS_MIN_FIXQ = 1
GPS_MIN_SATS = 5
GPS_MAX_HDOP = 8.0       # 厳しければ緩める
GPS_STALE_SEC = 2.5

# ========= pigpio / サーボ =========
PIN18 = 18 #右
PIN12 = 12 #左
STOP_US = 1490
US_MIN = 500
US_MAX = 2500

SCALE_US = 500           # v,w=1で±500us（要調整）
BASE_V = 0.70            # 前進(0..1)
MIN_V  = 0.35
SLOWDOWN_DIST_M = 8.0

KP = 0.1               # [1/deg] 蛇行なら下げる（0.012など）
W_MAX = 0.60

# 直進が「少し左に曲がる」→右旋回バイアス（+）
W_BIAS = +0.06           # まず+0.06、現場で微調整

# ========= util =========
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def wrap_to_180(a): #角度差を -180°～+180° に正規化
    while a > 180: a -= 360
    while a < -180: a += 360
    return a

def haversine_m(lat1, lon1, lat2, lon2): #2点間の地表距離[m]を計算
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2-lat1)
    dl = math.radians(lon2-lon1)
    a = math.sin(dp/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
    return 2*R*math.atan2(math.sqrt(a), math.sqrt(1-a))

def bearing_deg(lat1, lon1, lat2, lon2): #角度誤差の計算
    # 北=0 東=90（時計回り）0..360
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2-lon1)
    y = math.sin(dl)*math.cos(p2)
    x = math.cos(p1)*math.sin(p2) - math.sin(p1)*math.cos(p2)*math.cos(dl)
    b = math.degrees(math.atan2(y, x))
    return (b + 360) % 360

def dm_to_deg(dm, direction):
    if dm is None or direction not in ("N","S","E","W"):
        return None
    try:
        dm = float(dm)
    except ValueError:
        return None
    deg = int(dm // 100)
    minutes = dm - deg*100
    val = deg + minutes/60.0
    return -val if direction in ("S","W") else val

# ========= GPS shared =========
gps = {"lat":None,"lon":None,"fixq":0,"nsat":0,"hdop":None,"t":None}
lock = threading.Lock()

def gps_thread(stop_event):
    buf = bytearray()
    with SMBus(GPS_BUS) as bus:
        while not stop_event.is_set():
            try:
                data = bus.read_i2c_block_data(GPS_ADDR, 0xFF, 32)
            except OSError:
                time.sleep(0.05); continue

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
                    lat = dm_to_deg(getattr(msg,"lat",None), getattr(msg,"lat_dir",None))
                    lon = dm_to_deg(getattr(msg,"lon",None), getattr(msg,"lon_dir",None))
                    if lat is not None and lon is not None:
                        with lock:
                            gps["lat"]=lat; gps["lon"]=lon; gps["t"]=time.monotonic()

                elif msg.sentence_type == "GGA":
                    try: fixq=int(getattr(msg,"gps_qual",0) or 0)
                    except ValueError: fixq=0
                    try: nsat=int(getattr(msg,"num_sats",0) or 0)
                    except ValueError: nsat=0
                    try: hdop=float(getattr(msg,"horizontal_dil",0) or 0)
                    except ValueError: hdop=None
                    with lock:
                        gps["fixq"]=fixq; gps["nsat"]=nsat; gps["hdop"]=hdop

            time.sleep(0.02)

# ========= Drive =========
class Drive:
    def __init__(self, pi):
        self.pi = pi
        self.stop()

    def _write(self, us18, us12):
        self.pi.set_servo_pulsewidth(PIN18, int(clamp(us18, US_MIN, US_MAX)))
        self.pi.set_servo_pulsewidth(PIN12, int(clamp(us12, US_MIN, US_MAX)))

    def stop(self):
        self._write(STOP_US, STOP_US)

    def free(self):
        self.pi.set_servo_pulsewidth(PIN18, 0)
        self.pi.set_servo_pulsewidth(PIN12, 0)

    def set_vw(self, v, w):
        """
        v: 前進 0..1
        w: 右旋回 +、左旋回 -
        実測より:
          前進 = (PIN18↓, PIN12↑)
          右旋回 = (PIN18↑, PIN12↑)
          左旋回 = (PIN18↓, PIN12↓)
        """
        v = clamp(v, 0.0, 1.0)
        w = clamp(w, -1.0, 1.0)

        us18 = STOP_US - v*SCALE_US + w*SCALE_US
        us12 = STOP_US + v*SCALE_US + w*SCALE_US
        self._write(us18, us12)

# ========= main =========
def main():
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio not connected. pigpiod起動を確認してください。")
    drv = Drive(pi)

    i2c = busio.I2C(board.SCL, board.SDA)
    bno = adafruit_bno055.BNO055_I2C(i2c, address=0x28)

    stop_event = threading.Event()
    threading.Thread(target=gps_thread, args=(stop_event,), daemon=True).start()

    dt = 1.0/CONTROL_HZ
    last = time.monotonic()

    print("=== GPS誘導フェーズ（5m/5deg判定） ===")
    print(f"Goal: {GOAL_LAT}, {GOAL_LON}")

    try:
        while True:
            t0 = time.monotonic()

            with lock:
                lat=gps["lat"]; lon=gps["lon"]; fixq=gps["fixq"]; nsat=gps["nsat"]; hdop=gps["hdop"]; tg=gps["t"]

            gps_ok = (lat is not None and lon is not None and fixq>=GPS_MIN_FIXQ and nsat>=GPS_MIN_SATS and
                      hdop is not None and hdop<=GPS_MAX_HDOP and tg is not None and (time.monotonic()-tg)<GPS_STALE_SEC)

            heading = bno.euler[0]  # 北0東90
            if heading is None or not gps_ok:
                drv.stop()
                time.sleep(dt)
                continue

            dist = haversine_m(lat, lon, GOAL_LAT, GOAL_LON)
            if dist <= ARRIVAL_RADIUS_M:
                drv.stop()
                print(f"[ARRIVED] dist={dist:.2f}m <= {ARRIVAL_RADIUS_M}m")
                break

            theta1 = bearing_deg(lat, lon, GOAL_LAT, GOAL_LON)  # 0..360
            theta2 = heading
            # 目標-機体（右なら正）
            err = wrap_to_180(theta1 - theta2)

            # 減速
            if dist < SLOWDOWN_DIST_M:
                a = clamp(dist/SLOWDOWN_DIST_M, 0.0, 1.0)
                v = MIN_V + (BASE_V - MIN_V)*a
            else:
                v = BASE_V

            # フローチャート条件を反映
            if abs(err) < ANGLE_OK_DEG:
                w = W_BIAS
            else:
                w = clamp(KP*err + W_BIAS, -W_MAX, W_MAX)

            drv.set_vw(v, w)

            if time.monotonic()-last > 1.0:
                print(f"[CTRL] dist={dist:6.1f}m θ1={theta1:6.1f} θ2={theta2:6.1f} err={err:6.1f} "
                      f"v={v:.2f} w={w:.2f} nsat={nsat} hdop={hdop}")
                last = time.monotonic()

            time.sleep(max(0.0, dt-(time.monotonic()-t0)))

    except KeyboardInterrupt:
        print("Ctrl-C")
    finally:
        drv.stop()
        stop_event.set()
        time.sleep(0.2)
        drv.free()
        pi.stop()
        print("Finish")

if __name__ == "__main__":
    main()