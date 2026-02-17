import time
import math
import threading

from smbus2 import SMBus
import pynmea2

import board
import busio
import adafruit_bno055

from picamera2 import Picamera2
import cv2
import numpy as np

import pigpio

# ============================================================
# 1) GPS 目標・制御設定（あなたのGPSコード準拠）
# ============================================================
GOAL_LAT = 35.00000000
GOAL_LON = 139.00000000

CONTROL_HZ = 10.0

GPS_MIN_FIXQ = 1
GPS_MIN_SATS = 5
GPS_MAX_HDOP = 5.0
GPS_STALE_SEC = 2.5

BASE_SPEED = 0.55
MIN_SPEED  = 0.30
SLOWDOWN_DIST_M = 8.0

KP_HEADING = 0.015
TURN_MAX = 0.45

# ============================================================
# 2) GPS→Vision 切替条件（ユーザ指定：A）
#   A: 距離<=2mが連続
# ============================================================
SWITCH_TO_VISION_M = 2.0
SWITCH_CONFIRM_SEC = 1.0   # 2m以内がこの秒数連続でVisionへ（チャタ抑制）

# ============================================================
# 3) GPS(I2C NMEA)
# ============================================================
GPS_BUS = 1
GPS_ADDR = 0x42

# ============================================================
# 4) サーボ(pigpio)（Vision側のus差動方式に統一）
# ============================================================
SERVO_L_PIN = 18
SERVO_R_PIN = 12

STOP_L = 1480
STOP_R = 1490

MAX_DELTA = 350  # 停止からの最大変位(us)

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def set_diff_drive_us(pi, fwd_us, turn_us):
    """
    fwd_us : 前進成分(+で前進)
    turn_us: 旋回成分(+で右旋回, -で左旋回)
    """
    cmd_l = STOP_L + fwd_us + turn_us
    cmd_r = STOP_R + fwd_us - turn_us

    cmd_l = clamp(cmd_l, STOP_L - MAX_DELTA, STOP_L + MAX_DELTA)
    cmd_r = clamp(cmd_r, STOP_R - MAX_DELTA, STOP_R + MAX_DELTA)

    pi.set_servo_pulsewidth(SERVO_L_PIN, cmd_l)
    pi.set_servo_pulsewidth(SERVO_R_PIN, cmd_r)

def stop_drive(pi):
    pi.set_servo_pulsewidth(SERVO_L_PIN, STOP_L)
    pi.set_servo_pulsewidth(SERVO_R_PIN, STOP_R)

def free_drive(pi):
    pi.set_servo_pulsewidth(SERVO_L_PIN, 0)
    pi.set_servo_pulsewidth(SERVO_R_PIN, 0)

# ============================================================
# 5) GPSユーティリティ（あなたのGPSコード準拠）
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
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dlmb = math.radians(lon2 - lon1)
    y = math.sin(dlmb) * math.cos(phi2)
    x = math.cos(phi1)*math.sin(phi2) - math.sin(phi1)*math.cos(phi2)*math.cos(dlmb)
    brng = math.degrees(math.atan2(y, x))
    return (brng + 360.0) % 360.0

# ============================================================
# 6) GPS共有状態（あなたのGPSコード準拠）
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
# 7) Vision設定（あなたのVisionコード準拠）
# ============================================================
Kp_turn = 0.9
BASE_FWD = 220
SEARCH_TURN = 170

AREA_TRACK_TH = 800
AREA_FWD_TH = 1500

# ============================================================
# 8) Vision到達条件（ユーザ指定：A）
#   A: area>=閾値（現状）
# ============================================================
AREA_ARRIVE_TH = 12000     # “現状”の閾値としてここを調整する
ARRIVE_HOLD_SEC = 1.0      # 閾値以上がこの秒数連続で停止

lower_red1 = np.array([0, 120, 70], dtype=np.uint8)
upper_red1 = np.array([10, 255, 255], dtype=np.uint8)
lower_red2 = np.array([170, 120, 70], dtype=np.uint8)
upper_red2 = np.array([179, 255, 255], dtype=np.uint8)
kernel = np.ones((5, 5), np.uint8)

class Mode:
    GPS = "GPS"
    VISION = "VISION"
    STOP = "STOP"

# ============================================================
# 9) メイン
# ============================================================
def main():
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpioに接続できません。sudo pigpiod を実行してください。")

    # BNO055
    i2c = busio.I2C(board.SCL, board.SDA)
    sensor = adafruit_bno055.BNO055_I2C(i2c, address=0x28)

    # GPSスレッド
    stop_event = threading.Event()
    th = threading.Thread(target=gps_reader_thread, args=(stop_event,), daemon=True)
    th.start()

    # Picamera2
    W, H = 640, 480
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"format": "RGB888", "size": (W, H)})
    picam2.configure(config)
    picam2.start()
    time.sleep(0.2)

    stop_drive(pi)
    time.sleep(0.3)

    cx_frame = W // 2
    dt = 1.0 / CONTROL_HZ

    mode = Mode.GPS
    switch_enter_t = None
    arrive_enter_t = None
    last_log = time.monotonic()

    print("=== Integrated Guidance Start (GPS -> Vision) ===")
    print(f"Goal(lat,lon)=({GOAL_LAT},{GOAL_LON})")
    print(f"Switch: dist <= {SWITCH_TO_VISION_M} m for {SWITCH_CONFIRM_SEC} sec")
    print(f"Arrive: area >= {AREA_ARRIVE_TH} for {ARRIVE_HOLD_SEC} sec")
    print("q / ESC to stop (window). Ctrl-C also works.")

    try:
        while True:
            t0 = time.monotonic()

            # GPS snapshot
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

            # Heading
            heading = sensor.euler[0]
            if heading is None:
                stop_drive(pi)
                time.sleep(max(0.0, dt - (time.monotonic() - t0)))
                continue

            # -------------------------------
            # GPS MODE
            # -------------------------------
            if mode == Mode.GPS:
                if not gps_ok:
                    stop_drive(pi)
                    if time.monotonic() - last_log > 1.0:
                        print(f"[GPS-CTRL] GPS not ready. fixq={fixq}, nsat={nsat}, hdop={hdop}, heading={heading:.1f}")
                        last_log = time.monotonic()
                    # デバッグ用ウィンドウ維持
                    dummy = np.zeros((H, W, 3), dtype=np.uint8)
                    cv2.putText(dummy, "GPS MODE (waiting GPS)", (20, 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255,255,255), 2)
                    cv2.imshow("Camera", dummy)
                    cv2.imshow("Red Mask", np.zeros((H, W), dtype=np.uint8))
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("q") or key == 27:
                        break
                    time.sleep(max(0.0, dt - (time.monotonic() - t0)))
                    continue

                dist_m = haversine_m(lat, lon, GOAL_LAT, GOAL_LON)
                brg = bearing_deg(lat, lon, GOAL_LAT, GOAL_LON)
                err = wrap_to_180(brg - heading)

                # 切替判定：距離<=2mが連続
                if dist_m <= SWITCH_TO_VISION_M:
                    if switch_enter_t is None:
                        switch_enter_t = time.monotonic()
                    elif (time.monotonic() - switch_enter_t) >= SWITCH_CONFIRM_SEC:
                        mode = Mode.VISION
                        stop_drive(pi)
                        arrive_enter_t = None
                        print(f"[MODE] GPS -> VISION (dist={dist_m:.2f} m, confirmed {SWITCH_CONFIRM_SEC}s)")
                        time.sleep(0.2)
                else:
                    switch_enter_t = None

                # 速度スケジュール
                if dist_m < SLOWDOWN_DIST_M:
                    alpha = clamp(dist_m / SLOWDOWN_DIST_M, 0.0, 1.0)
                    v = MIN_SPEED + (BASE_SPEED - MIN_SPEED) * alpha
                else:
                    v = BASE_SPEED

                w = KP_HEADING * err
                w = clamp(w, -TURN_MAX, TURN_MAX)

                # -1..+1 → us に変換（MAX_DELTAをスケールに使用）
                fwd_us = int(v * MAX_DELTA)
                turn_us = int(w * MAX_DELTA)
                set_diff_drive_us(pi, fwd_us=fwd_us, turn_us=turn_us)

                if time.monotonic() - last_log > 1.0:
                    st = (time.monotonic() - switch_enter_t) if switch_enter_t else 0.0
                    print(f"[GPS] dist={dist_m:6.1f}m brg={brg:6.1f} head={heading:6.1f} err={err:6.1f} "
                          f"fwd_us={fwd_us:+4d} turn_us={turn_us:+4d} nsat={nsat} hdop={hdop} switch_t={st:.1f}s")
                    last_log = time.monotonic()

                # デバッグウィンドウ維持
                dummy = np.zeros((H, W, 3), dtype=np.uint8)
                cv2.putText(dummy, f"GPS MODE dist={dist_m:.1f}m", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255,255,255), 2)
                cv2.imshow("Camera", dummy)
                cv2.imshow("Red Mask", np.zeros((H, W), dtype=np.uint8))
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q") or key == 27:
                    break

            # -------------------------------
            # VISION MODE
            # -------------------------------
            elif mode == Mode.VISION:
                frame_rgb = picam2.capture_array("main")
                if frame_rgb is None or frame_rgb.size == 0:
                    stop_drive(pi)
                    time.sleep(max(0.0, dt - (time.monotonic() - t0)))
                    continue

                frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
                hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

                mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
                mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
                mask = cv2.bitwise_or(mask1, mask2)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                if contours:
                    c = max(contours, key=cv2.contourArea)
                    area = cv2.contourArea(c)

                    if area >= AREA_TRACK_TH:
                        M = cv2.moments(c)
                        if M["m00"] > 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])

                            err_n = (cx - cx_frame) / (W / 2.0)  # -1..+1
                            turn = int(Kp_turn * err_n * MAX_DELTA)

                            fwd = BASE_FWD if area < AREA_FWD_TH else int(BASE_FWD * 1.2)
                            set_diff_drive_us(pi, fwd_us=fwd, turn_us=turn)

                            # 到達判定：area>=閾値（連続保持）
                            if area >= AREA_ARRIVE_TH:
                                if arrive_enter_t is None:
                                    arrive_enter_t = time.monotonic()
                                elif (time.monotonic() - arrive_enter_t) >= ARRIVE_HOLD_SEC:
                                    mode = Mode.STOP
                                    stop_drive(pi)
                                    print(f"[ARRIVED] VISION area>={AREA_ARRIVE_TH} for {ARRIVE_HOLD_SEC}s -> STOP")
                            else:
                                arrive_enter_t = None

                            # 可視化
                            x, y, w, h = cv2.boundingRect(c)
                            cv2.rectangle(frame_bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)
                            cv2.circle(frame_bgr, (cx, cy), 4, (0, 255, 0), -1)
                            cv2.putText(frame_bgr, f"RED area={int(area)} err={err_n:.2f}",
                                        (x, max(0, y - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        else:
                            set_diff_drive_us(pi, fwd_us=0, turn_us=SEARCH_TURN)
                            arrive_enter_t = None
                    else:
                        set_diff_drive_us(pi, fwd_us=0, turn_us=SEARCH_TURN)
                        arrive_enter_t = None
                else:
                    set_diff_drive_us(pi, fwd_us=0, turn_us=SEARCH_TURN)
                    arrive_enter_t = None

                cv2.line(frame_bgr, (cx_frame, 0), (cx_frame, H), (255, 255, 255), 1)
                cv2.imshow("Camera", frame_bgr)
                cv2.imshow("Red Mask", mask)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q") or key == 27:
                    break

                if time.monotonic() - last_log > 1.0:
                    print("[VISION] tracking...")
                    last_log = time.monotonic()

            # -------------------------------
            # STOP MODE
            # -------------------------------
            else:
                stop_drive(pi)
                dummy = np.zeros((H, W, 3), dtype=np.uint8)
                cv2.putText(dummy, "STOP MODE", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255,255,255), 2)
                cv2.imshow("Camera", dummy)
                cv2.imshow("Red Mask", np.zeros((H, W), dtype=np.uint8))
                key = cv2.waitKey(50) & 0xFF
                if key == ord("q") or key == 27:
                    break

            time.sleep(max(0.0, dt - (time.monotonic() - t0)))

    except KeyboardInterrupt:
        print("Stopping (Ctrl-C)...")

    finally:
        stop_drive(pi)
        time.sleep(0.2)
        free_drive(pi)
        pi.stop()

        try:
            picam2.stop()
        except Exception:
            pass

        stop_event.set()
        time.sleep(0.2)

        cv2.destroyAllWindows()
        print("Finish.")

if __name__ == "__main__":
    main()
