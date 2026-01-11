import time
import sys
import serial

import board
import busio
import adafruit_bno055
from adafruit_bme280 import basic as adafruit_bme280

from smbus2 import SMBus
import pynmea2

# ==========================================
# 設定
# ==========================================
TWELITE_PORT = "/dev/ttyUSB0"
TWELITE_BAUD = 38400

GPS_I2C_BUS = 1
GPS_I2C_ADDR = 0x42  # SAM-M8Q など
GPS_READ_LEN = 32    # 1回で読むバイト数

SEND_HZ = 1.0        # TWELITEへ送信周期 [Hz]
LOOP_HZ = 20.0       # メインループ周期 [Hz]


# ==========================================
# ヘルパー: 緯度経度 dm → 度
# ==========================================
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

    if direction in ("S", "W"):
        val = -val
    return val


# ==========================================
# メイン
# ==========================================
def main():
    # ---------- TWELITE シリアル ----------
    try:
        ser = serial.Serial(TWELITE_PORT, TWELITE_BAUD, timeout=0)
    except Exception as e:
        print("TWELITEポートを開けません:", e)
        sys.exit(1)

    print(f"TWELITEポート {TWELITE_PORT} をオープンしました")

    # ---------- I2C (BNO055, BME280) ----------
    i2c = busio.I2C(board.SCL, board.SDA)
    bno055 = adafruit_bno055.BNO055_I2C(i2c, address=0x28)
    bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=0x76)
    bme280.sea_level_pressure = 1013.25  # 必要に応じて現地値へ

    # ---------- I2C (GNSS) ----------
    gps_bus = SMBus(GPS_I2C_BUS)
    gps_buf = bytearray()
    latest_gps = {
        "lat": None,
        "lon": None,
        "alt": None,
        "fixq": 0,
        "nsat": 0,
        "hdop": None,
    }

    # 送信周期制御
    next_send_time = time.monotonic() + 1.0 / SEND_HZ
    loop_dt = 1.0 / LOOP_HZ

    print("計測＋TWELITEダウンリンク開始")

    try:
        while True:
            loop_start = time.monotonic()

            # ==============================
            # 1) GNSS I2C 読み出し & パース
            # ==============================
            try:
                data = gps_bus.read_i2c_block_data(GPS_I2C_ADDR, 0xFF, GPS_READ_LEN)
                gps_buf.extend(data)
            except OSError:
                # Errno 121対策など: 一瞬待ってスキップ
                pass

            # 完全な NMEA 行ごとにパース
            while b"\n" in gps_buf:
                line, _, rest = gps_buf.partition(b"\n")
                gps_buf = bytearray(rest)

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
                        latest_gps["lat"] = lat
                        latest_gps["lon"] = lon

                elif msg.sentence_type == "GGA":
                    try:
                        fixq = int(getattr(msg, "gps_qual", 0) or 0)
                    except ValueError:
                        fixq = 0
                    latest_gps["fixq"] = fixq

                    try:
                        latest_gps["nsat"] = int(getattr(msg, "num_sats", 0) or 0)
                    except ValueError:
                        latest_gps["nsat"] = 0

                    try:
                        latest_gps["hdop"] = float(getattr(msg, "horizontal_dil", 0) or 0)
                    except ValueError:
                        latest_gps["hdop"] = None

                    if fixq > 0:
                        try:
                            latest_gps["alt"] = float(getattr(msg, "altitude", 0) or 0)
                        except ValueError:
                            pass

            # ==============================
            # 2) 一定周期で TWELITE へ送信
            # ==============================
            now = time.monotonic()
            if now >= next_send_time:
                next_send_time += 1.0 / SEND_HZ

                # ----- BNO055 -----
                euler = bno055.euler    # (heading, roll, pitch)
                gyro = bno055.gyro      # (gx, gy, gz)
                accel = bno055.acceleration  # (ax, ay, az)
                mag = bno055.magnetic   # (mx, my, mz)

                # None 対策: 取得できなければ 0.0 などに置き換え
                def safe_vec3(v):
                    if v is None:
                        return (0.0, 0.0, 0.0)
                    return tuple(x if x is not None else 0.0 for x in v)

                if euler is None:
                    euler = (0.0, 0.0, 0.0)
                euler = tuple(x if x is not None else 0.0 for x in euler)

                gyro = safe_vec3(gyro)
                accel = safe_vec3(accel)
                mag = safe_vec3(mag)

                heading, roll, pitch = euler
                gx, gy, gz = gyro
                ax, ay, az = accel
                mx, my, mz = mag

                # ----- BME280 -----
                temp = bme280.temperature
                press = bme280.pressure
                humid = bme280.humidity
                bme_alt = bme280.altitude

                # ----- GNSS -----
                lat = latest_gps["lat"]
                lon = latest_gps["lon"]
                alt = latest_gps["alt"]
                fixq = latest_gps["fixq"]
                nsat = latest_gps["nsat"]
                hdop = latest_gps["hdop"]

                # None は NaN 相当値に置き換え（-9999 など好きな値でOK）
                def nz(v, default=-9999.0):
                    return v if v is not None else default

                lat = nz(lat)
                lon = nz(lon)
                alt = nz(alt)
                hdop = nz(hdop)

                # ==============================
                # 3) CSV 文字列にして TWELITE へ送信
                # ==============================
                line = (
                    "SENS,"
                    f"{lat:.8f},{lon:.8f},{alt:.1f},"
                    f"{fixq:d},{nsat:d},{hdop:.2f},"
                    f"{heading:.2f},{roll:.2f},{pitch:.2f},"
                    f"{gx:.4f},{gy:.4f},{gz:.4f},"
                    f"{ax:.4f},{ay:.4f},{az:.4f},"
                    f"{mx:.4f},{my:.4f},{mz:.4f},"
                    f"{temp:.2f},{press:.2f},{humid:.2f},{bme_alt:.2f}"
                )

                try:
                    ser.write((line + "\r\n").encode())
                except Exception as e:
                    print("シリアル送信エラー:", e)

                # デバッグ用に標準出力にも表示
                print(line)

            # ==============================
            # 4) TWELITE からの受信（必要なら）
            # ==============================
            try:
                if ser.in_waiting > 0:
                    rx = ser.readline()
                    if rx:
                        text = rx.decode(errors="replace").rstrip()
                        print(f"[RX] {text}")
            except Exception as e:
                print("シリアル受信エラー:", e)

            # ループ周期調整
            elapsed = time.monotonic() - loop_start
            sleep_time = loop_dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n終了します。")
    finally:
        try:
            gps_bus.close()
        except Exception:
            pass
        ser.close()
        print("ポートをクローズしました。")


if __name__ == "__main__":
    main()
