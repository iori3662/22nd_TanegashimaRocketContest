#!/usr/bin/env python3
"""
CanSat sensor downlink using TWELITE via pigpio "soft UART".

- Hardware UART (/dev/serial0) is NOT used.
- TX/RX are done on GPIOs with pigpio:
  - RX (bit-bang read): bb_serial_read_open()
  - TX (DMA wave):      wave_add_serial() + wave_send_once()

Your wiring-mistake case is supported:
  Twelite TX -> Pi GPIO14  (use as RX_INPUT)
  Twelite RX -> Pi GPIO15  (use as TX_OUTPUT)

Requirements:
  - pigpiod running: sudo systemctl enable --now pigpiod
  - pip install pigpio adafruit-circuitpython-bno055 adafruit-circuitpython-bme280 smbus2 pynmea2
"""

import time
import sys

import pigpio

import board
import adafruit_bno055
from adafruit_bme280 import basic as adafruit_bme280

from smbus2 import SMBus
import pynmea2


# ==========================================
# 設定
# ==========================================

# --- TWELITE soft UART GPIO (wiring-mistake friendly) ---
TWELITE_RX_GPIO = 14   # Pi input  (connected to TWELITE_TX)
TWELITE_TX_GPIO = 15   # Pi output (connected to TWELITE_RX)

TWELITE_BAUD = 38400
TWELITE_DATA_BITS = 8
TWELITE_STOP_HALF_BITS = 2  # pigpio: bb_stop is "stop half bits" (2 => 1 stop bit)

# --- GNSS (I2C) ---
GPS_I2C_BUS = 1
GPS_I2C_ADDR = 0x42  # SAM-M8Q など
GPS_READ_LEN = 32    # 1回で読むバイト数

# --- Loop rates ---
SEND_HZ = 1.0        # TWELITEへ送信周期 [Hz]
LOOP_HZ = 20.0       # メインループ周期 [Hz]

# --- Soft UART behavior ---
TX_LINE_ENDING = b"\r\n"   # TWELITE側設定に合わせる（CRLFが無難）
RX_BUF_MAX = 4096          # 受信バッファ肥大防止


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
# Soft UART for TWELITE (pigpio)
# ==========================================
class TweliteSoftUART:
    """
    pigpio-based soft UART:
      - TX: wave_add_serial (DMA)
      - RX: bb_serial_read (bit-bang)
    """

    def __init__(self, rx_gpio: int, tx_gpio: int, baud: int):
        self.rx_gpio = rx_gpio
        self.tx_gpio = tx_gpio
        self.baud = baud
        self._rx_buf = bytearray()

        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpiodに接続できません。`sudo systemctl start pigpiod` を確認してください。")

        # TX idle high
        self.pi.set_mode(self.tx_gpio, pigpio.OUTPUT)
        self.pi.write(self.tx_gpio, 1)

        # RX open (bit-bang)
        self.pi.set_mode(self.rx_gpio, pigpio.INPUT)
        self.pi.bb_serial_read_open(self.rx_gpio, self.baud, TWELITE_DATA_BITS)
        time.sleep(0.05)

    def close(self):
        try:
            self.pi.bb_serial_read_close(self.rx_gpio)
        except pigpio.error:
            pass
        try:
            self.pi.stop()
        except Exception:
            pass

    def write_line(self, line_str: str):
        """
        Send a line (ASCII) with line ending.
        """
        payload = line_str.encode("ascii", errors="ignore") + TX_LINE_ENDING

        # TX via DMA wave (robust)
        self.pi.wave_clear()
        # pigpio 1.78 signature: wave_add_serial(gpio, baud, data, offset=0, bb_bits=8, bb_stop=2)
        self.pi.wave_add_serial(self.tx_gpio, self.baud, payload, 0, TWELITE_DATA_BITS, TWELITE_STOP_HALF_BITS)
        wid = self.pi.wave_create()
        if wid < 0:
            raise RuntimeError("wave_create failed")

        self.pi.wave_send_once(wid)
        while self.pi.wave_tx_busy():
            time.sleep(0.001)
        self.pi.wave_delete(wid)

    def _read_bytes(self) -> bytes:
        cnt, data = self.pi.bb_serial_read(self.rx_gpio)
        if cnt > 0 and data:
            # data is bytearray
            return bytes(data)
        return b""

    def read_lines(self):
        """
        Non-blocking read of complete lines ending with '\n'.
        Returns list[str].
        """
        chunk = self._read_bytes()
        if chunk:
            self._rx_buf.extend(chunk)
            if len(self._rx_buf) > RX_BUF_MAX:
                self._rx_buf = self._rx_buf[-RX_BUF_MAX:]

        lines = []
        while b"\n" in self._rx_buf:
            raw, _, rest = self._rx_buf.partition(b"\n")
            self._rx_buf = bytearray(rest)
            raw = raw.rstrip(b"\r")
            lines.append(raw.decode("ascii", errors="replace").strip())
        return lines


# ==========================================
# メイン
# ==========================================
def main():
    # ---------- TWELITE Soft UART ----------
    try:
        tw = TweliteSoftUART(
            rx_gpio=TWELITE_RX_GPIO,
            tx_gpio=TWELITE_TX_GPIO,
            baud=TWELITE_BAUD,
        )
    except Exception as e:
        print("TWELITE (soft UART) 初期化に失敗:", e)
        sys.exit(1)

    print(f"TWELITE soft UART open: RX=GPIO{TWELITE_RX_GPIO}, TX=GPIO{TWELITE_TX_GPIO}, {TWELITE_BAUD}bps")

    # ---------- I2C (BNO055, BME280) ----------
    i2c = board.I2C()
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
                euler = bno055.euler         # (heading, roll, pitch)
                gyro = bno055.gyro           # (gx, gy, gz)
                accel = bno055.acceleration  # (ax, ay, az)
                mag = bno055.magnetic        # (mx, my, mz)

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
                temp = float(bme280.temperature)
                press = float(bme280.pressure)
                humid = float(bme280.humidity)
                bme_alt = float(bme280.altitude)

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
                    tw.write_line(line)
                except Exception as e:
                    print("TWELITE送信エラー:", e)

                # デバッグ用に標準出力にも表示
                print(line)

            # ==============================
            # 4) TWELITE からの受信（必要なら）
            # ==============================
            # pyserialの in_waiting/ readline 相当を "行単位" で実装
            try:
                for text in tw.read_lines():
                    if text:
                        print(f"[RX] {text}")
            except Exception as e:
                print("TWELITE受信エラー:", e)

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
        tw.close()
        print("soft UART をクローズしました。")


if __name__ == "__main__":
    main()