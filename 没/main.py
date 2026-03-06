import time
import math
import threading
import sys
import csv
import os
import serial

from smbus2 import SMBus
import pynmea2

import board
import busio
import adafruit_bno055
from adafruit_bme280 import basic as adafruit_bme280

from picamera2 import Picamera2
import cv2
import numpy as np

import pigpio

# ============================================================
# 0) TWELITE / Logging 設定
# ============================================================
TWELITE_PORT = "/dev/serial0"
TWELITE_BAUD = 38400

SEND_HZ = 1.0          # ダウンリンク周期 [Hz]
LOG_DIR = "./logs"     # CSV保存先
os.makedirs(LOG_DIR, exist_ok=True)

# ============================================================
# 1) GPS 目標・制御設定（あなたの統合誘導準拠）
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

# GPS→Vision 切替（ユーザ指定 A）
SWITCH_TO_VISION_M = 2.0
SWITCH_CONFIRM_SEC = 1.0

# ============================================================
# 2) GPS(I2C NMEA)
# ============================================================
GPS_BUS = 1
GPS_ADDR = 0x42

# ============================================================
# 3) サーボ(pigpio)（Vision側のus差動方式）
# ============================================================
SERVO_L_PIN = 18
SERVO_R_PIN = 12

STOP_L = 1480
STOP_R = 1490
MAX_DELTA = 350

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def set_diff_drive_us(pi, fwd_us, turn_us):
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
# 4) GPSユーティリティ
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
# 5) GPS共有状態（スレッド）
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
# 6) Vision設定（あなたのVisionコード準拠）
# ============================================================
Kp_turn = 0.9
BASE_FWD = 220
SEARCH_TURN = 170

AREA_TRACK_TH = 800
AREA_FWD_TH = 1500

# 到達条件（ユーザ指定 A）
AREA_ARRIVE_TH = 12000
ARRIVE_HOLD_SEC = 1.0

lower_red1 = np.array([0, 120, 70], dtype=np.uint8)
upper_red1 = np.array([10, 255, 255], dtype=np.uint8)
lower_red2 = np.array([170, 120, 70], dtype=np.uint8)
upper_red2 = np.array([179, 255, 255], dtype=np.uint8)
kernel = np.ones((5, 5), np.uint8)

# Vision結果共有（テレメ用）
vision_latest = {
    "found": 0,
    "area": 0.0,
    "err_n": None,      # -1..1
}
vis_lock = threading.Lock()

# ============================================================
# 7) テレメ定義（CSV/TWELITE共通）
# ============================================================
TELEM_HEADER = [
    "type","seq","t_unix_ms","t_mono_s","mode",
    "gps_lat_deg","gps_lon_deg","gps_alt_m","gps_fixq","gps_nsat","gps_hdop",
    "bno_heading_deg","bno_roll_deg","bno_pitch_deg",
    "bno_gx_rps","bno_gy_rps","bno_gz_rps",
    "bno_ax_mps2","bno_ay_mps2","bno_az_mps2",
    "bno_mx_uT","bno_my_uT","bno_mz_uT",
    "bme_temp_C","bme_press_hPa","bme_humid_pct","bme_alt_m",
    "nav_dist_m","nav_brg_deg","nav_err_deg",
    "cmd_fwd_us","cmd_turn_us",
    "vis_area","vis_err_n","vis_found",
    "arrive_flag"
]

def nz(v, default=-9999.0):
    return v if v is not None else default

def safe_vec3(v):
    if v is None:
        return (0.0, 0.0, 0.0)
    return tuple(x if x is not None else 0.0 for x in v)

# ============================================================
# 8) 状態機械
# ============================================================
class Mode:
    GPS = "GPS"
    VISION = "VISION"
    STOP = "STOP"

# ============================================================
# 9) メイン
# ============================================================
def main():
    # ---- TWELITE ----
    try:
        ser = serial.Serial(TWELITE_PORT, TWELITE_BAUD, timeout=0)
    except Exception as e:
        print("TWELITEポートを開けません:", e)
        sys.exit(1)
    print(f"[TWELITE] Open {TWELITE_PORT} @ {TWELITE_BAUD}")

    # ---- CSV ----
    ts = time.strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(LOG_DIR, f"telem_{ts}.csv")
    f_csv = open(csv_path, "w", newline="")
    writer = csv.writer(f_csv)
    writer.writerow(TELEM_HEADER)
    f_csv.flush()
    print(f"[LOG] CSV -> {csv_path}")

    # ---- pigpio ----
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpioに接続できません。sudo pigpiod を実行してください。")

    # ---- I2C sensors ----
    i2c = board.I2C()
    bno055 = adafruit_bno055.BNO055_I2C(i2c, address=0x28)
    bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=0x76)
    bme280.sea_level_pressure = 1013.25

    # ---- GPS thread ----
    stop_event = threading.Event()
    th = threading.Thread(target=gps_reader_thread, args=(stop_event,), daemon=True)
    th.start()

    # ---- Camera ----
    W, H = 640, 480
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"format": "RGB888", "size": (W, H)})
    picam2.configure(config)
    picam2.start()
    time.sleep(0.2)
    cx_frame = W // 2

    # ---- init ----
    stop_drive(pi)
    time.sleep(0.3)

    mode = Mode.GPS
    switch_enter_t = None
    arrive_enter_t = None

    # last command for telemetry
    last_cmd_fwd_us = 0
    last_cmd_turn_us = 0

    # GPS-nav last (for telemetry)
    last_nav = {"dist": None, "brg": None, "err": None}

    # send schedule
    next_send_time = time.monotonic() + 1.0 / SEND_HZ
    dt = 1.0 / CONTROL_HZ
    seq = 0

    print("=== Integrated Guidance + TWELITE + CSV Start ===")
    print(f"Switch: dist<= {SWITCH_TO_VISION_M} m for {SWITCH_CONFIRM_SEC} s")
    print(f"Arrive: area>= {AREA_ARRIVE_TH} for {ARRIVE_HOLD_SEC} s")
    print("q/ESC to stop window. Ctrl-C also works.")

    try:
        while True:
            loop_t0 = time.monotonic()

            # -------- GPS snapshot --------
            with gps_lock:
                lat = gps_latest["lat"]
                lon = gps_latest["lon"]
                alt = gps_latest["alt"]
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

            # -------- Sensors --------
            euler = bno055.euler
            if euler is None:
                heading, roll, pitch = (0.0, 0.0, 0.0)
            else:
                euler = tuple(x if x is not None else 0.0 for x in euler)
                heading, roll, pitch = euler

            gx, gy, gz = safe_vec3(bno055.gyro)
            ax, ay, az = safe_vec3(bno055.acceleration)
            mx, my, mz = safe_vec3(bno055.magnetic)

            temp = float(bme280.temperature)
            press = float(bme280.pressure)
            humid = float(bme280.humidity)
            bme_alt = float(bme280.altitude)

            arrive_flag = 0

            # ===================================================
            # MODE: GPS
            # ===================================================
            if mode == Mode.GPS:
                if not gps_ok:
                    stop_drive(pi)
                    last_cmd_fwd_us = 0
                    last_cmd_turn_us = 0
                    last_nav = {"dist": None, "brg": None, "err": None}

                    dummy = np.zeros((H, W, 3), dtype=np.uint8)
                    cv2.putText(dummy, "GPS MODE (waiting GPS)", (20, 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255,255,255), 2)
                    cv2.imshow("Camera", dummy)
                    cv2.imshow("Red Mask", np.zeros((H, W), dtype=np.uint8))
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("q") or key == 27:
                        break

                else:
                    dist_m = haversine_m(lat, lon, GOAL_LAT, GOAL_LON)
                    brg = bearing_deg(lat, lon, GOAL_LAT, GOAL_LON)
                    err = wrap_to_180(brg - heading)
                    last_nav = {"dist": dist_m, "brg": brg, "err": err}

                    # switch check: dist<=2m continuous
                    if dist_m <= SWITCH_TO_VISION_M:
                        if switch_enter_t is None:
                            switch_enter_t = time.monotonic()
                        elif (time.monotonic() - switch_enter_t) >= SWITCH_CONFIRM_SEC:
                            mode = Mode.VISION
                            stop_drive(pi)
                            last_cmd_fwd_us = 0
                            last_cmd_turn_us = 0
                            arrive_enter_t = None
                            with vis_lock:
                                vision_latest["found"] = 0
                                vision_latest["area"] = 0.0
                                vision_latest["err_n"] = None
                            time.sleep(0.2)
                    else:
                        switch_enter_t = None

                    # speed schedule
                    if dist_m < SLOWDOWN_DIST_M:
                        alpha = clamp(dist_m / SLOWDOWN_DIST_M, 0.0, 1.0)
                        v = MIN_SPEED + (BASE_SPEED - MIN_SPEED) * alpha
                    else:
                        v = BASE_SPEED

                    w = KP_HEADING * err
                    w = clamp(w, -TURN_MAX, TURN_MAX)

                    # convert to us
                    fwd_us = int(v * MAX_DELTA)
                    turn_us = int(w * MAX_DELTA)

                    set_diff_drive_us(pi, fwd_us=fwd_us, turn_us=turn_us)
                    last_cmd_fwd_us = fwd_us
                    last_cmd_turn_us = turn_us

                    dummy = np.zeros((H, W, 3), dtype=np.uint8)
                    cv2.putText(dummy, f"GPS MODE dist={dist_m:.1f}m", (20, 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255,255,255), 2)
                    cv2.imshow("Camera", dummy)
                    cv2.imshow("Red Mask", np.zeros((H, W), dtype=np.uint8))
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("q") or key == 27:
                        break

            # ===================================================
            # MODE: VISION
            # ===================================================
            elif mode == Mode.VISION:
                frame_rgb = picam2.capture_array("main")
                if frame_rgb is None or frame_rgb.size == 0:
                    stop_drive(pi)
                    last_cmd_fwd_us = 0
                    last_cmd_turn_us = 0
                else:
                    frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
                    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

                    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
                    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
                    mask = cv2.bitwise_or(mask1, mask2)
                    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
                    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

                    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                    found = 0
                    area = 0.0
                    err_n = None

                    if contours:
                        c = max(contours, key=cv2.contourArea)
                        area = float(cv2.contourArea(c))

                        if area >= AREA_TRACK_TH:
                            M = cv2.moments(c)
                            if M["m00"] > 0:
                                found = 1
                                cx = int(M["m10"] / M["m00"])
                                cy = int(M["m01"] / M["m00"])
                                err_n = (cx - cx_frame) / (W / 2.0)

                                turn_us = int(Kp_turn * err_n * MAX_DELTA)
                                fwd_us = BASE_FWD if area < AREA_FWD_TH else int(BASE_FWD * 1.2)

                                set_diff_drive_us(pi, fwd_us=fwd_us, turn_us=turn_us)
                                last_cmd_fwd_us = fwd_us
                                last_cmd_turn_us = turn_us

                                # arrive check: area>=th continuous
                                if area >= AREA_ARRIVE_TH:
                                    if arrive_enter_t is None:
                                        arrive_enter_t = time.monotonic()
                                    elif (time.monotonic() - arrive_enter_t) >= ARRIVE_HOLD_SEC:
                                        arrive_flag = 1
                                        mode = Mode.STOP
                                        stop_drive(pi)
                                        last_cmd_fwd_us = 0
                                        last_cmd_turn_us = 0
                                else:
                                    arrive_enter_t = None

                                # debug draw
                                x, y, w, h = cv2.boundingRect(c)
                                cv2.rectangle(frame_bgr, (x, y), (x+w, y+h), (0,255,0), 2)
                                cv2.circle(frame_bgr, (cx, cy), 4, (0,255,0), -1)
                                cv2.putText(frame_bgr, f"area={int(area)} err={err_n:+.2f}",
                                            (x, max(0, y-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                            else:
                                set_diff_drive_us(pi, fwd_us=0, turn_us=SEARCH_TURN)
                                last_cmd_fwd_us = 0
                                last_cmd_turn_us = SEARCH_TURN
                                arrive_enter_t = None
                        else:
                            set_diff_drive_us(pi, fwd_us=0, turn_us=SEARCH_TURN)
                            last_cmd_fwd_us = 0
                            last_cmd_turn_us = SEARCH_TURN
                            arrive_enter_t = None
                    else:
                        set_diff_drive_us(pi, fwd_us=0, turn_us=SEARCH_TURN)
                        last_cmd_fwd_us = 0
                        last_cmd_turn_us = SEARCH_TURN
                        arrive_enter_t = None

                    with vis_lock:
                        vision_latest["found"] = found
                        vision_latest["area"] = area
                        vision_latest["err_n"] = err_n

                    # visualize
                    if frame_rgb is not None and frame_rgb.size > 0:
                        cv2.line(frame_bgr, (cx_frame, 0), (cx_frame, H), (255, 255, 255), 1)
                        cv2.imshow("Camera", frame_bgr)
                        cv2.imshow("Red Mask", mask)
                        key = cv2.waitKey(1) & 0xFF
                        if key == ord("q") or key == 27:
                            break

                # nav fields not meaningful in vision; keep last or blank
                last_nav = {"dist": None, "brg": None, "err": None}

            # ===================================================
            # MODE: STOP
            # ===================================================
            else:
                stop_drive(pi)
                last_cmd_fwd_us = 0
                last_cmd_turn_us = 0
                arrive_flag = 1

                dummy = np.zeros((H, W, 3), dtype=np.uint8)
                cv2.putText(dummy, "STOP MODE", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255,255,255), 2)
                cv2.imshow("Camera", dummy)
                cv2.imshow("Red Mask", np.zeros((H, W), dtype=np.uint8))
                key = cv2.waitKey(50) & 0xFF
                if key == ord("q") or key == 27:
                    break

            # ===================================================
            # 1Hz テレメ送信 + CSV保存
            # ===================================================
            now = time.monotonic()
            if now >= next_send_time:
                next_send_time += 1.0 / SEND_HZ

                with vis_lock:
                    vis_found = int(vision_latest["found"])
                    vis_area = float(vision_latest["area"])
                    vis_err_n = vision_latest["err_n"]

                # GPS values (sentinel fill)
                lat_s = nz(lat)
                lon_s = nz(lon)
                alt_s = nz(alt)
                hdop_s = nz(hdop)

                # nav fields
                nav_dist = nz(last_nav["dist"])
                nav_brg  = nz(last_nav["brg"])
                nav_err  = nz(last_nav["err"])

                # vision err
                vis_err_s = nz(vis_err_n)

                row = [
                    "TL1",
                    seq,
                    int(time.time() * 1000),
                    round(time.monotonic(), 3),
                    mode,
                    f"{lat_s:.8f}", f"{lon_s:.8f}", f"{alt_s:.1f}", int(fixq), int(nsat), f"{hdop_s:.2f}",
                    f"{heading:.2f}", f"{roll:.2f}", f"{pitch:.2f}",
                    f"{gx:.4f}", f"{gy:.4f}", f"{gz:.4f}",
                    f"{ax:.4f}", f"{ay:.4f}", f"{az:.4f}",
                    f"{mx:.4f}", f"{my:.4f}", f"{mz:.4f}",
                    f"{temp:.2f}", f"{press:.2f}", f"{humid:.2f}", f"{bme_alt:.2f}",
                    f"{nav_dist:.2f}", f"{nav_brg:.2f}", f"{nav_err:.2f}",
                    int(last_cmd_fwd_us), int(last_cmd_turn_us),
                    f"{vis_area:.1f}", f"{vis_err_s:.3f}", vis_found,
                    int(arrive_flag)
                ]

                # CSV保存
                writer.writerow(row)
                f_csv.flush()

                # TWELITE送信（CSV互換 1行）
                line = ",".join(map(str, row))
                try:
                    ser.write((line + "\r\n").encode())
                except Exception as e:
                    print("[TWELITE] send error:", e)

                # デバッグ表示
                print(line)
                seq += 1

            # ---- TWELITE RX（必要なら） ----
            try:
                if ser.in_waiting > 0:
                    rx = ser.readline()
                    if rx:
                        text = rx.decode(errors="replace").rstrip()
                        print(f"[RX] {text}")
            except Exception as e:
                print("[TWELITE] rx error:", e)

            # loop timing
            elapsed = time.monotonic() - loop_t0
            sleep_t = dt - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\n終了します。")

    finally:
        stop_drive(pi)
        time.sleep(0.2)
        free_drive(pi)
        pi.stop()

        stop_event.set()
        time.sleep(0.2)

        try:
            picam2.stop()
        except Exception:
            pass
        cv2.destroyAllWindows()

        try:
            ser.close()
        except Exception:
            pass

        try:
            f_csv.close()
        except Exception:
            pass

        print("Finish.")

if __name__ == "__main__":
    main()
