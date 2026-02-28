import asyncio
import time
from smbus2 import SMBus
import pynmea2

import board
import busio
import adafruit_bno055
from adafruit_bme280 import basic as adafruit_bme280

from bleak import BleakClient
from bleak.exc import BleakError

# =======================
# BLE Config
# =======================
ADDR = "58:8C:81:AE:A8:1A"  # XIAOのMACアドレス
LOG_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

SEND_HZ = 10.0
SEND_DT = 1.0 / SEND_HZ

# =======================
# Sensor Config
# =======================
BUS = 1
GPS_ADDR = 0x42
BNO_ADDR = 0x28
BME_ADDR = 0x76

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


def build_line(seq, phase, vbat_mV, yaw, pitch, roll, lat, lon, alt, temp, press, hum):
    def f(v, fmt):
        if v is None:
            return "nan"
        return format(v, fmt)

    # XIAO側パーサのキーに合わせる（SEQ,PH,VBAT,Y,P,R,LAT,LON,ALT）
    # 追加でT,Pr,Hも載せる（表示/保存用）
    s = (
        f"SEQ={seq},PH={phase},VBAT={vbat_mV},"
        f"Y={f(yaw,'.1f')},P={f(pitch,'.1f')},R={f(roll,'.1f')},"
        f"LAT={f(lat,'.6f')},LON={f(lon,'.6f')},ALT={f(alt,'.1f')},"
        f"T={f(temp,'.1f')},Pr={f(press,'.1f')},H={f(hum,'.1f')}\n"
    )
    return s.encode("ascii", errors="ignore")


# =======================
# One run (connect once, loop send)
# =======================
async def telemetry_once():
    # --- CircuitPython I2C stack ---
    i2c = busio.I2C(board.SCL, board.SDA)
    bno055 = adafruit_bno055.BNO055_I2C(i2c, address=BNO_ADDR)
    bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=BME_ADDR)
    bme280.sea_level_pressure = 1013.25

    # --- GPS state ---
    buf = bytearray()
    latest = {"lat": None, "lon": None, "alt": None, "fixq": 0, "nsat": 0, "hdop": None}

    # --- placeholders: 競技の実装に合わせて置換 ---
    phase = 0          # TODO: Pi側のフェーズ管理値に置換
    vbat_mV = 7400     # TODO: 電圧計測（外付けADC等）に置換

    seq = 0
    next_send = time.monotonic()

    client = BleakClient(ADDR)

    with SMBus(BUS) as bus:
        try:
            await client.connect()
            print("Connected:", client.is_connected)

            while True:
                now = time.monotonic()

                # -------- GPS read/parse (EIOは握る) --------
                try:
                    data = bus.read_i2c_block_data(GPS_ADDR, 0xFF, GPS_READ_LEN)
                    buf.extend(data)
                    buf = parse_nmea_from_buffer(buf, latest)
                except OSError:
                    pass

                # -------- IMU read (EIO/Noneは握る) --------
                yaw = pitch = roll = None
                try:
                    euler = bno055.euler  # (heading, roll, pitch) or None
                    if euler is not None:
                        if euler[0] is not None:
                            yaw = float(euler[0])
                        if euler[1] is not None:
                            roll = float(euler[1])
                        if euler[2] is not None:
                            pitch = float(euler[2])
                except OSError:
                    pass

                # -------- BME read --------
                temp = press = hum = None
                try:
                    temp = float(bme280.temperature)
                    press = float(bme280.pressure)
                    hum = float(bme280.humidity)
                except OSError:
                    pass
                except Exception:
                    pass

                # -------- Prepare GPS fields --------
                if latest["fixq"] > 0:
                    lat = latest["lat"]
                    lon = latest["lon"]
                    alt = latest["alt"]
                else:
                    lat = lon = alt = None

                # -------- Send at 10Hz --------
                if now >= next_send:
                    next_send += SEND_DT

                    payload = build_line(
                        seq=seq,
                        phase=phase,
                        vbat_mV=vbat_mV,
                        yaw=yaw,
                        pitch=pitch,
                        roll=roll,
                        lat=lat,
                        lon=lon,
                        alt=alt,
                        temp=temp,
                        press=press,
                        hum=hum,
                    )

                    # デバッグ段階では response=True（成功/失敗が分かる）
                    await client.write_gatt_char(LOG_UUID, payload, response=True)

                    if seq % 20 == 0:
                        print(payload.decode("ascii", errors="ignore").strip())

                    seq += 1

                await asyncio.sleep(GPS_SLEEP)

        finally:
            # 例外でも切断を試みる（already connected対策）
            try:
                if client.is_connected:
                    await client.disconnect()
            except Exception as e:
                print("disconnect error (ignored):", repr(e))
            print("Disconnected.")


# =======================
# Retry loop
# =======================
async def main():
    while True:
        try:
            await telemetry_once()
        except KeyboardInterrupt:
            print("\nStopped by user.")
            return
        except BleakError as e:
            print("bleak error (retry):", repr(e))
            await asyncio.sleep(1.0)
        except OSError as e:
            print("error (retry):", repr(e))
            await asyncio.sleep(1.0)
        except Exception as e:
            print("error (retry):", repr(e))
            await asyncio.sleep(1.0)


if __name__ == "__main__":
    asyncio.run(main())