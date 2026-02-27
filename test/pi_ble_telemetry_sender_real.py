import asyncio
import time
from smbus2 import SMBus
import pynmea2

import board
import busio
import adafruit_bno055
from adafruit_bme280 import basic as adafruit_bme280

from bleak import BleakClient

# =======================
# BLE Config
# =======================
ADDR = "58:8C:81:AE:A8:1A"  # XIAOのMAC（あなたの環境の値）
LOG_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

SEND_HZ = 10.0  # XIAOへ送る周期（UIが滑らかになる）
SEND_DT = 1.0 / SEND_HZ

# =======================
# Sensor Config
# =======================
BUS = 1
GPS_ADDR = 0x42
BNO_ADDR = 0x28
BME_ADDR = 0x76

IMU_HZ = 10.0
GPS_HZ = 1.0
GPS_READ_LEN = 32
GPS_SLEEP = 0.02

# =======================
# Helpers
# =======================
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


def parse_nmea_from_buffer(buf, latest):
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
                latest["lat"] = lat
                latest["lon"] = lon

        elif msg.sentence_type == "GGA":
            try:
                fixq = int(getattr(msg, "gps_qual", 0) or 0)
            except ValueError:
                fixq = 0
            latest["fixq"] = fixq

            try:
                latest["nsat"] = int(getattr(msg, "num_sats", 0) or 0)
            except ValueError:
                latest["nsat"] = 0

            try:
                latest["hdop"] = float(getattr(msg, "horizontal_dil", 0) or 0)
            except ValueError:
                latest["hdop"] = None

            if fixq > 0:
                try:
                    latest["alt"] = float(getattr(msg, "altitude", 0) or 0)
                except ValueError:
                    pass

    return buf


def safe_float(x, default=None):
    try:
        return float(x)
    except Exception:
        return default


def build_ascii_line(seq: int, phase: int, vbat_mV: int,
                     yaw_deg, pitch_deg, roll_deg,
                     lat, lon, alt,
                     temp_c, press_hpa, hum_pct):
    # XIAO側のパーサに合わせたキーを最低限含める
    # 文字数削減のため小数点を抑える（必要なら増やせる）
    def f(v, fmt):
        if v is None:
            return "nan"
        return format(v, fmt)

    line = (
        f"SEQ={seq},PH={phase},VBAT={vbat_mV},"
        f"Y={f(yaw_deg, '.1f')},P={f(pitch_deg, '.1f')},R={f(roll_deg, '.1f')},"
        f"LAT={f(lat, '.6f')},LON={f(lon, '.6f')},ALT={f(alt, '.1f')},"
        f"T={f(temp_c, '.1f')},Pr={f(press_hpa, '.1f')},H={f(hum_pct, '.1f')}\n"
    )
    return line.encode("ascii", errors="ignore")


# =======================
# Main BLE loop
# =======================
async def telemetry_loop():
    # --- Sensors (CircuitPython stack) ---
    i2c = busio.I2C(board.SCL, board.SDA)
    bno055 = adafruit_bno055.BNO055_I2C(i2c, address=BNO_ADDR)
    bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=BME_ADDR)
    bme280.sea_level_pressure = 1013.25

    # --- GPS state ---
    buf = bytearray()
    latest = {"lat": None, "lon": None, "alt": None, "fixq": 0, "nsat": 0, "hdop": None}

    # --- Dummy fields you will later replace with real phase/vbat ---
    phase = 0
    vbat_mV = 7400  # TODO: 実機ではADCやPi側電圧計測値に置き換え

    seq = 0
    next_send = time.monotonic()
    next_imu_tick = time.monotonic()
    imu_dt = 1.0 / IMU_HZ

    with SMBus(BUS) as bus:
        async with BleakClient(ADDR) as client:
            await client.connect()
            print("Connected:", client.is_connected)

            while True:
                now = time.monotonic()

                # -------- GPS read/parse (continuous) --------
                try:
                    data = bus.read_i2c_block_data(GPS_ADDR, 0xFF, GPS_READ_LEN)
                    buf.extend(data)
                    buf = parse_nmea_from_buffer(buf, latest)
                except OSError:
                    pass

                # -------- IMU/BME sampling at IMU_HZ --------
                # 送信が10Hzなら、IMUも同じ周期で読む（過剰読みによるI2C負荷を避ける）
                yaw_deg = pitch_deg = roll_deg = None
                temp_c = press_hpa = hum_pct = None

                if now >= next_imu_tick:
                    next_imu_tick += imu_dt

                    euler = bno055.euler  # (heading, roll, pitch) [deg] or None
                    if euler is not None:
                        # euler[0]=heading(yaw), euler[1]=roll, euler[2]=pitch
                        yaw_deg = safe_float(euler[0])
                        roll_deg = safe_float(euler[1])
                        pitch_deg = safe_float(euler[2])

                    temp_c = safe_float(bme280.temperature)
                    press_hpa = safe_float(bme280.pressure)
                    hum_pct = safe_float(bme280.humidity)

                # -------- Send at SEND_HZ --------
                if now >= next_send:
                    next_send += SEND_DT

                    lat = latest["lat"] if latest["fixq"] > 0 else None
                    lon = latest["lon"] if latest["fixq"] > 0 else None
                    alt = latest["alt"] if latest["fixq"] > 0 else None

                    payload = build_ascii_line(
                        seq=seq,
                        phase=phase,
                        vbat_mV=vbat_mV,
                        yaw_deg=yaw_deg,
                        pitch_deg=pitch_deg,
                        roll_deg=roll_deg,
                        lat=lat,
                        lon=lon,
                        alt=alt,
                        temp_c=temp_c,
                        press_hpa=press_hpa,
                        hum_pct=hum_pct,
                    )

                    # response=Trueで確実に失敗を拾う（安定したらFalseへ）
                    await client.write_gatt_char(LOG_UUID, payload, response=True)

                    if seq % 20 == 0:
                        print(payload.decode("ascii", errors="ignore").strip())

                    seq += 1

                await asyncio.sleep(GPS_SLEEP)


async def main():
    while True:
        try:
            await telemetry_loop()
        except KeyboardInterrupt:
            print("\nStopped by user.")
            return
        except Exception as e:
            print("error (retry):", repr(e))
            await asyncio.sleep(1.0)


if __name__ == "__main__":
    asyncio.run(main())