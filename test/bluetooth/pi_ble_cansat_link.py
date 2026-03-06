import asyncio
import time
import math
from dataclasses import dataclass
from typing import Optional

from bleak import BleakClient
from bleak.exc import BleakError

from smbus2 import SMBus
import pynmea2

import board
import busio
import adafruit_bno055
from adafruit_bme280 import basic as adafruit_bme280

# =======================
# BLE (Nordic UART)
# =======================
XIAO_ADDR = "58:8C:81:AE:A8:1A"  # XIAOのMAC
UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
UART_RX_UUID      = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  # Pi -> XIAO (Write)
UART_TX_UUID      = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # XIAO -> Pi (Notify)

SEND_HZ = 10.0
SEND_DT = 1.0 / SEND_HZ

# =======================
# I2C sensors
# =======================
BUS = 1
GPS_ADDR = 0x42
BNO_ADDR = 0x28
BME_ADDR = 0x76

GPS_READ_LEN = 32
GPS_SLEEP = 0.02

CSV_HEADER = (
    "seq,ph,vbat_mV,yaw_deg,pitch_deg,roll_deg,lat_deg,lon_deg,alt_gps_m,"
    "temp_C,press_hPa,hum_pct,alt_bme_m,ax,ay,az,a"
)

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


def parse_nmea_from_buffer(buf: bytearray, latest: dict) -> bytearray:
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


def build_csv_line(
    seq: int, ph: int, vbat_mV: int,
    yaw: Optional[float], pitch: Optional[float], roll: Optional[float],
    lat: Optional[float], lon: Optional[float], alt_gps: Optional[float],
    temp: Optional[float], press: Optional[float], hum: Optional[float],
    alt_bme: Optional[float],
    ax: Optional[float], ay: Optional[float], az: Optional[float], amag: Optional[float],
) -> bytes:
    def f(v, fmt):
        if v is None:
            return "nan"
        return format(v, fmt)

    s = (
        f"{seq},{ph},{vbat_mV},"
        f"{f(yaw,'.1f')},{f(pitch,'.1f')},{f(roll,'.1f')},"
        f"{f(lat,'.6f')},{f(lon,'.6f')},{f(alt_gps,'.1f')},"
        f"{f(temp,'.1f')},{f(press,'.1f')},{f(hum,'.1f')},"
        f"{f(alt_bme,'.2f')},"
        f"{f(ax,'.2f')},{f(ay,'.2f')},{f(az,'.2f')},{f(amag,'.2f')}\n"
    )
    return s.encode("ascii", errors="ignore")


# =======================
# Command handling (from XIAO buttons)
# =======================
@dataclass
class PiState:
    phase: int = 0
    mode: str = "IDLE"     # IDLE / DEBUG / RUN
    logging: bool = True   # Pi側でログも取るならここで管理（今回はダミー）
    vbat_mV: int = 7400    # TODO 実測へ


def parse_cmd_line(b: bytearray | bytes) -> tuple[str, str]:
    """
    XIAO -> Pi notify line format:
      cmd,<NAME>,<ARG>\n
    e.g.
      cmd,mode,debug
      cmd,mode,run
      cmd,phase,set:2
      cmd,log,toggle
    """
    s = b.decode("ascii", errors="ignore").strip()
    parts = [p.strip() for p in s.split(",")]
    if len(parts) < 3 or parts[0] != "cmd":
        return ("", "")
    name = parts[1]
    arg = ",".join(parts[2:])
    return (name, arg)


def apply_cmd(state: PiState, name: str, arg: str) -> None:
    if name == "mode":
        if arg in ("idle", "debug", "run"):
            state.mode = arg.upper()
            print("[CMD] mode ->", state.mode)
    elif name == "phase":
        # arg: "set:2" or "inc" or "dec"
        if arg.startswith("set:"):
            try:
                state.phase = int(arg.split(":", 1)[1])
                print("[CMD] phase set ->", state.phase)
            except ValueError:
                pass
        elif arg == "inc":
            state.phase += 1
            print("[CMD] phase inc ->", state.phase)
        elif arg == "dec":
            state.phase = max(0, state.phase - 1)
            print("[CMD] phase dec ->", state.phase)
    elif name == "log":
        if arg == "toggle":
            state.logging = not state.logging
            print("[CMD] logging ->", state.logging)
    elif name == "beep":
        # XIAO側がブザー鳴らす想定ならPi側では無視でもOK
        print("[CMD] beep request:", arg)
    else:
        print("[CMD] unknown:", name, arg)


# =======================
# Main BLE + sensor loop
# =======================
async def run_once():
    state = PiState()

    # --- CircuitPython I2C stack ---
    i2c = busio.I2C(board.SCL, board.SDA)
    bno055 = adafruit_bno055.BNO055_I2C(i2c, address=BNO_ADDR)
    bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=BME_ADDR)
    bme280.sea_level_pressure = 1013.25  # 相対変化を見る前提でOK

    # --- GPS ---
    buf = bytearray()
    latest = {"lat": None, "lon": None, "alt": None, "fixq": 0, "nsat": 0, "hdop": None}

    seq = 0
    next_send = time.monotonic()

    client = BleakClient(XIAO_ADDR)

    def on_notify(_: int, data: bytearray):
        name, arg = parse_cmd_line(data)
        if name:
            apply_cmd(state, name, arg)

    with SMBus(BUS) as bus:
        try:
            await client.connect()
            print("Connected:", client.is_connected)

            # XIAO -> Pi commands
            await client.start_notify(UART_TX_UUID, on_notify)

            # （任意）最初にヘッダをXIAOへ送る（XIAO側は受け取ったら無視する設計でもOK）
            # 今回はXIAOがNewFile時にヘッダを書き込むので、送らない。

            while True:
                now = time.monotonic()

                # ---- GPS read/parse (握る) ----
                try:
                    data = bus.read_i2c_block_data(GPS_ADDR, 0xFF, GPS_READ_LEN)
                    buf.extend(data)
                    buf = parse_nmea_from_buffer(buf, latest)
                except OSError:
                    pass

                # ---- IMU Euler ----
                yaw = pitch = roll = None
                try:
                    euler = bno055.euler
                    if euler is not None:
                        if euler[0] is not None:
                            yaw = float(euler[0])
                        if euler[1] is not None:
                            roll = float(euler[1])
                        if euler[2] is not None:
                            pitch = float(euler[2])
                except OSError:
                    pass

                # ---- Accel (fall) ----
                ax = ay = az = amag = None
                try:
                    # linear_acceleration があるなら優先
                    lin_attr = getattr(bno055, "linear_acceleration", None)
                    if lin_attr is not None:
                        v = bno055.linear_acceleration
                    else:
                        v = None

                    if v is None:
                        v = bno055.acceleration

                    if v is not None and v[0] is not None:
                        ax = float(v[0])
                        ay = float(v[1])
                        az = float(v[2])
                        amag = math.sqrt(ax*ax + ay*ay + az*az)
                except OSError:
                    pass
                except Exception:
                    pass

                # ---- BME ----
                temp = press = hum = alt_bme = None
                try:
                    temp = float(bme280.temperature)
                    press = float(bme280.pressure)
                    hum = float(bme280.humidity)
                    alt_bme = float(bme280.altitude)  # ★高度判断の主
                except OSError:
                    pass
                except Exception:
                    pass

                # ---- GPS fields ----
                if latest["fixq"] > 0:
                    lat = latest["lat"]
                    lon = latest["lon"]
                    alt_gps = latest["alt"]
                else:
                    lat = lon = alt_gps = None

                # ---- Send at 10Hz ----
                if now >= next_send:
                    next_send += SEND_DT

                    payload = build_csv_line(
                        seq=seq,
                        ph=state.phase,
                        vbat_mV=state.vbat_mV,
                        yaw=yaw, pitch=pitch, roll=roll,
                        lat=lat, lon=lon, alt_gps=alt_gps,
                        temp=temp, press=press, hum=hum,
                        alt_bme=alt_bme,
                        ax=ax, ay=ay, az=az, amag=amag,
                    )
                    # Pi -> XIAO (Write)
                    await client.write_gatt_char(UART_RX_UUID, payload, response=True)

                    if seq % 20 == 0:
                        print(payload.decode("ascii", errors="ignore").strip())

                    seq += 1

                await asyncio.sleep(GPS_SLEEP)

        finally:
            try:
                if client.is_connected:
                    try:
                        await client.stop_notify(UART_TX_UUID)
                    except Exception:
                        pass
                    await client.disconnect()
            except Exception as e:
                print("disconnect error (ignored):", repr(e))
            print("Disconnected.")


async def main():
    while True:
        try:
            await run_once()
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
    print("CSV header:", CSV_HEADER)
    asyncio.run(main())