#!/usr/bin/env python3
"""
CanSat sensor downlink over BLE (GATT UART-like) to XIAO ESP32-C3.

- ESP32-C3 acts as BLE Peripheral (server).
- Raspberry Pi acts as BLE Central (client).
- We mimic the previous TweliteSoftUART interface:
    - write_line(str)
    - read_lines() -> list[str] (from notify stream)

Requirements:
  sudo apt install -y bluetooth bluez
  python3 -m pip install bleak adafruit-circuitpython-bno055 adafruit-circuitpython-bme280 smbus2 pynmea2
"""

import asyncio
import time
import sys
from collections import deque

from bleak import BleakClient, BleakScanner

import board
import adafruit_bno055
from adafruit_bme280 import basic as adafruit_bme280
from smbus2 import SMBus
import pynmea2


# ==========================================
# 設定
# ==========================================

# --- BLE target (choose either NAME or MAC) ---
BLE_TARGET_NAME = "XIAO-C3-CANSAT"  # must match ESP32 code
BLE_TARGET_ADDR = None             # e.g. "AA:BB:CC:DD:EE:FF" if you want fixed

# --- BLE UART-like UUIDs (must match ESP32 code) ---
SVC_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
RX_UUID  = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  # Write
TX_UUID  = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # Notify

# --- GNSS (I2C) ---
GPS_I2C_BUS = 1
GPS_I2C_ADDR = 0x42
GPS_READ_LEN = 32

# --- Loop rates ---
SEND_HZ = 1.0
LOOP_HZ = 20.0

# --- framing ---
TX_LINE_ENDING = "\r\n"   # keep CRLF like TWELITE
RX_BUF_MAX_LINES = 200    # prevent unbounded growth


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
# BLE UART Client
# ==========================================
class BleUartClient:
    """
    BLE UART-like:
      - write_line(): GATT write to RX characteristic
      - read_lines(): returns accumulated lines from TX notifications
    """

    def __init__(self):
        self.client: BleakClient | None = None
        self._rx_lines = deque(maxlen=RX_BUF_MAX_LINES)
        self._notify_buf = bytearray()

    async def connect(self):
        addr = BLE_TARGET_ADDR
        if addr is None:
            dev = await self._find_by_name(BLE_TARGET_NAME, timeout=8.0)
            if dev is None:
                raise RuntimeError(f"BLE device not found by name: {BLE_TARGET_NAME}")
            addr = dev.address

        self.client = BleakClient(addr)

        await self.client.connect()
        if not await self.client.is_connected():
            raise RuntimeError("BLE connect failed")

        # subscribe notify
        await self.client.start_notify(TX_UUID, self._on_notify)

        print(f"[BLE] connected: {addr}")

    async def close(self):
        if self.client is None:
            return
        try:
            if await self.client.is_connected():
                try:
                    await self.client.stop_notify(TX_UUID)
                except Exception:
                    pass
                await self.client.disconnect()
        finally:
            self.client = None

    async def write_line(self, line_str: str):
        if self.client is None or (not await self.client.is_connected()):
            raise RuntimeError("BLE not connected")
        payload = (line_str + TX_LINE_ENDING).encode("utf-8", errors="ignore")
        # write without response is usually faster; ESP RX has WRITE_NR enabled
        await self.client.write_gatt_char(RX_UUID, payload, response=False)

    def read_lines(self):
        lines = []
        while self._rx_lines:
            lines.append(self._rx_lines.popleft())
        return lines

    async def ensure_connected(self):
        if self.client is None:
            await self.connect()
            return
        try:
            if not await self.client.is_connected():
                await self.close()
                await self.connect()
        except Exception:
            await self.close()
            await self.connect()

    async def _find_by_name(self, name: str, timeout: float = 8.0):
        def match(d, ad):
            return (d.name == name) if d else False

        return await BleakScanner.find_device_by_filter(match, timeout=timeout)

    def _on_notify(self, _char_uuid: str, data: bytearray):
        # accumulate bytes, split by '\n'
        if not data:
            return
        self._notify_buf.extend(data)

        while b"\n" in self._notify_buf:
            raw, _, rest = self._notify_buf.partition(b"\n")
            self._notify_buf = bytearray(rest)
            raw = raw.rstrip(b"\r")
            try:
                s = raw.decode("utf-8", errors="replace").strip()
            except Exception:
                s = ""
            if s:
                self._rx_lines.append(s)


# ==========================================
# メイン
# ==========================================
async def main_async():
    # ---------- BLE ----------
    ble = BleUartClient()
    try:
        await ble.connect()
    except Exception as e:
        print("BLE 初期化に失敗:", e)
        sys.exit(1)

    # ---------- I2C sensors ----------
    i2c = board.I2C()
    bno055 = adafruit_bno055.BNO055_I2C(i2c, address=0x28)
    bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=0x76)
    bme280.sea_level_pressure = 1013.25

    # ---------- GNSS ----------
    gps_bus = SMBus(GPS_I2C_BUS)
    gps_buf = bytearray()
    latest_gps = {"lat": None, "lon": None, "alt": None, "fixq": 0, "nsat": 0, "hdop": None}

    next_send_time = time.monotonic() + 1.0 / SEND_HZ
    loop_dt = 1.0 / LOOP_HZ

    print("計測＋BLEダウンリンク開始")

    try:
        while True:
            loop_start = time.monotonic()

            # 1) GNSS read/parse
            try:
                data = gps_bus.read_i2c_block_data(GPS_I2C_ADDR, 0xFF, GPS_READ_LEN)
                gps_buf.extend(data)
            except OSError:
                pass

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
                        latest_gps["fixq"] = int(getattr(msg, "gps_qual", 0) or 0)
                    except ValueError:
                        latest_gps["fixq"] = 0
                    try:
                        latest_gps["nsat"] = int(getattr(msg, "num_sats", 0) or 0)
                    except ValueError:
                        latest_gps["nsat"] = 0
                    try:
                        latest_gps["hdop"] = float(getattr(msg, "horizontal_dil", 0) or 0)
                    except ValueError:
                        latest_gps["hdop"] = None
                    if latest_gps["fixq"] > 0:
                        try:
                            latest_gps["alt"] = float(getattr(msg, "altitude", 0) or 0)
                        except ValueError:
                            pass

            # 2) periodic send
            now = time.monotonic()
            if now >= next_send_time:
                next_send_time += 1.0 / SEND_HZ

                # BNO055
                euler = bno055.euler
                gyro = bno055.gyro
                accel = bno055.acceleration
                mag = bno055.magnetic

                def safe_vec3(v):
                    if v is None:
                        return (0.0, 0.0, 0.0)
                    return tuple(x if x is not None else 0.0 for x in v)

                if euler is None:
                    euler = (0.0, 0.0, 0.0)
                euler = tuple(x if x is not None else 0.0 for x in euler)

                heading, roll, pitch = euler
                gx, gy, gz = safe_vec3(gyro)
                ax, ay, az = safe_vec3(accel)
                mx, my, mz = safe_vec3(mag)

                # BME280
                temp = float(bme280.temperature)
                press = float(bme280.pressure)
                humid = float(bme280.humidity)
                bme_alt = float(bme280.altitude)

                # GNSS
                def nz(v, default=-9999.0):
                    return v if v is not None else default

                lat = nz(latest_gps["lat"])
                lon = nz(latest_gps["lon"])
                alt = nz(latest_gps["alt"])
                hdop = nz(latest_gps["hdop"])
                fixq = int(latest_gps["fixq"])
                nsat = int(latest_gps["nsat"])

                # CSV（元コードの形式を維持）
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
                    await ble.ensure_connected()
                    await ble.write_line(line)
                except Exception as e:
                    print("BLE送信エラー:", e)

                print(line)

            # 3) receive notify lines
            try:
                for text in ble.read_lines():
                    print(f"[RX] {text}")
            except Exception as e:
                print("BLE受信エラー:", e)

            # 4) timing
            elapsed = time.monotonic() - loop_start
            sleep_time = loop_dt - elapsed
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n終了します。")
    finally:
        try:
            gps_bus.close()
        except Exception:
            pass
        await ble.close()
        print("BLE をクローズしました。")


def main():
    asyncio.run(main_async())


if __name__ == "__main__":
    main()