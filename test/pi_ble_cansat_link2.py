#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import asyncio
import math
import os
import signal
import subprocess
import time
from dataclasses import dataclass
from typing import Optional

from smbus2 import SMBus
import pynmea2

import board
import busio
import adafruit_bno055
from adafruit_bme280 import basic as adafruit_bme280

from bleak import BleakClient, BleakScanner
from bleak.exc import BleakError

# =======================
# BLE (Nordic UART)
# =======================
UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
UART_RX_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  # Pi -> XIAO (Write)
UART_TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # XIAO -> Pi (Notify)

XIAO_NAME = "CanSat-Remote"  # advertising name

# =======================
# I2C / Sensors
# =======================
BUS = 1
GPS_ADDR = 0x42
BNO_ADDR = 0x28
BME_ADDR = 0x76

GPS_READ_LEN = 32
GPS_SLEEP = 0.02

IMU_HZ = 20.0      # internal update rate (read loop)
TELEM_HZ = 10.0    # BLE telemetry send rate (you can change)
LOG_HZ = 10.0      # local log rate (normally same as TELEM_HZ)

# =======================
# Logging (Pi local)
# =======================
LOG_DIR = "./logs"
os.makedirs(LOG_DIR, exist_ok=True)

CSV_HEADER = (
    "seq,ph,yaw_deg,pitch_deg,roll_deg,"
    "lat_deg,lon_deg,alt_gps_m,"
    "temp_C,press_hPa,hum_pct,alt_bme_m,"
    "ax,ay,az,a"
)

# =======================
# Phase / Mode
# =======================
MODE_IDLE = "idle"
MODE_DEBUG = "debug"
MODE_RUN = "run"

# ここはあなたの統合システムに合わせてパスを書き換えて使う想定
# 例：フェーズ変更時に実行したいスクリプトがあるならここに登録
PHASE_HOOKS = {
    # 0: ["python3", "phase0_init.py"],
    # 1: ["python3", "phase1_drop.py"],
    # 2: ["python3", "phase2_run.py"],
}

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


def f_or_nan(x: Optional[float]) -> float:
    if x is None:
        return float("nan")
    try:
        # NaN を通す
        return float(x)
    except Exception:
        return float("nan")


def euler_to_safe_tuple(euler):
    # bno055.euler は (heading, roll, pitch) だが None混在あり得る
    if euler is None:
        return (float("nan"), float("nan"), float("nan"))
    h, r, p = euler[0], euler[1], euler[2]
    return (f_or_nan(h), f_or_nan(p), f_or_nan(r))  # yaw=H, pitch=P, roll=R


def accel_to_safe_tuple(accel):
    if accel is None:
        return (float("nan"), float("nan"), float("nan"))
    ax, ay, az = accel[0], accel[1], accel[2]
    return (f_or_nan(ax), f_or_nan(ay), f_or_nan(az))


def fmt_num(x: float, nd: int = 6) -> str:
    if x is None or (isinstance(x, float) and math.isnan(x)):
        return "nan"
    if nd == 0:
        return str(int(round(float(x))))
    return f"{float(x):.{nd}f}"


def accel_norm(ax: float, ay: float, az: float) -> float:
    if any(math.isnan(v) for v in (ax, ay, az)):
        return float("nan")
    return math.sqrt(ax * ax + ay * ay + az * az)


@dataclass
class SharedState:
    phase: int = 0
    mode: str = MODE_IDLE

    # logging control (Pi local)
    logging: bool = True

    # telemetry seq
    seq: int = 0

    # last command time
    last_cmd_ts: float = 0.0


# =======================
# Command handling
# =======================
def run_phase_hook(phase: int):
    cmd = PHASE_HOOKS.get(phase)
    if not cmd:
        return
    try:
        # あなたの統合コードに合わせてここを強化可能
        subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception as e:
        print(f"[WARN] phase hook failed: {e}")


def handle_cmd(state: SharedState, line: str):
    # expected: cmd,name,arg
    # example: cmd,phase,inc
    line = line.strip()
    if not line:
        return
    if not line.startswith("cmd,"):
        print(f"[CMD] (ignored) {line}")
        return

    parts = line.split(",", 2)
    if len(parts) != 3:
        print(f"[CMD] (bad) {line}")
        return

    _, name, arg = parts
    state.last_cmd_ts = time.time()

    if name == "mode":
        if arg in (MODE_DEBUG, MODE_RUN, MODE_IDLE):
            state.mode = arg
            print(f"[CMD] mode -> {state.mode}")
        else:
            print(f"[CMD] mode invalid: {arg}")

    elif name == "phase":
        if arg == "inc":
            state.phase += 1
            print(f"[CMD] phase -> {state.phase}")
            run_phase_hook(state.phase)
        elif arg == "dec":
            state.phase = max(0, state.phase - 1)
            print(f"[CMD] phase -> {state.phase}")
            run_phase_hook(state.phase)
        elif arg.startswith("set:"):
            try:
                v = int(arg.split(":", 1)[1])
                state.phase = max(0, v)
                print(f"[CMD] phase -> {state.phase}")
                run_phase_hook(state.phase)
            except ValueError:
                print(f"[CMD] phase set invalid: {arg}")
        else:
            print(f"[CMD] phase invalid: {arg}")

    elif name == "log":
        if arg == "toggle":
            state.logging = not state.logging
            print(f"[CMD] logging -> {state.logging}")
        else:
            print(f"[CMD] log invalid: {arg}")

    else:
        print(f"[CMD] unknown: {name} {arg}")


# =======================
# Sensor task
# =======================
async def sensor_task(shared: SharedState, latest_out: dict, stop_evt: asyncio.Event):
    i2c = busio.I2C(board.SCL, board.SDA)
    bno055 = adafruit_bno055.BNO055_I2C(i2c, address=BNO_ADDR)
    bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=BME_ADDR)
    bme280.sea_level_pressure = 1013.25

    gps_buf = bytearray()
    gps_latest = {"lat": None, "lon": None, "alt": None, "fixq": 0, "nsat": 0, "hdop": None}

    next_imu = time.monotonic()
    imu_dt = 1.0 / IMU_HZ

    with SMBus(BUS) as bus:
        while not stop_evt.is_set():
            now = time.monotonic()

            # GPS stream
            try:
                data = bus.read_i2c_block_data(GPS_ADDR, 0xFF, GPS_READ_LEN)
                gps_buf.extend(data)
                gps_buf = parse_nmea_from_buffer(gps_buf, gps_latest)
            except OSError:
                pass

            if now >= next_imu:
                next_imu += imu_dt

                euler = bno055.euler
                accel = bno055.acceleration

                yaw, pitch, roll = euler_to_safe_tuple(euler)
                ax, ay, az = accel_to_safe_tuple(accel)
                a = accel_norm(ax, ay, az)

                # BME
                temp = f_or_nan(bme280.temperature)
                press = f_or_nan(bme280.pressure)
                hum = f_or_nan(bme280.humidity)
                altb = f_or_nan(bme280.altitude)

                # GPS
                lat = gps_latest["lat"] if gps_latest["lat"] is not None else float("nan")
                lon = gps_latest["lon"] if gps_latest["lon"] is not None else float("nan")
                alt_gps = gps_latest["alt"] if gps_latest["alt"] is not None else float("nan")

                latest_out.update({
                    "yaw": yaw, "pitch": pitch, "roll": roll,
                    "lat": lat, "lon": lon, "alt_gps": alt_gps,
                    "temp": temp, "press": press, "hum": hum, "altb": altb,
                    "ax": ax, "ay": ay, "az": az, "a": a,
                    "gps_fixq": gps_latest["fixq"],
                    "gps_nsat": gps_latest["nsat"],
                    "gps_hdop": gps_latest["hdop"],
                    "ts": time.time(),
                })

            await asyncio.sleep(GPS_SLEEP)


# =======================
# BLE: find device
# =======================
async def find_xiao_addr(timeout: float = 6.0) -> Optional[str]:
    devs = await BleakScanner.discover(timeout=timeout)
    for d in devs:
        name = (d.name or "").strip()
        if name == XIAO_NAME:
            return d.address
    return None


# =======================
# Telemetry builder
# =======================
def build_csv_line(shared: SharedState, L: dict) -> str:
    shared.seq += 1
    seq = shared.seq
    ph = shared.phase

    yaw = float(L.get("yaw", float("nan")))
    pitch = float(L.get("pitch", float("nan")))
    roll = float(L.get("roll", float("nan")))

    lat = float(L.get("lat", float("nan")))
    lon = float(L.get("lon", float("nan")))
    alt_gps = float(L.get("alt_gps", float("nan")))

    temp = float(L.get("temp", float("nan")))
    press = float(L.get("press", float("nan")))
    hum = float(L.get("hum", float("nan")))
    altb = float(L.get("altb", float("nan")))

    ax = float(L.get("ax", float("nan")))
    ay = float(L.get("ay", float("nan")))
    az = float(L.get("az", float("nan")))
    a = float(L.get("a", float("nan")))

    # 値のみ（XIAOがそのままSDへ保存できる）
    return ",".join([
        str(seq),
        str(ph),
        fmt_num(yaw, 1),
        fmt_num(pitch, 1),
        fmt_num(roll, 1),
        fmt_num(lat, 8),
        fmt_num(lon, 8),
        fmt_num(alt_gps, 1),
        fmt_num(temp, 2),
        fmt_num(press, 2),
        fmt_num(hum, 2),
        fmt_num(altb, 2),
        fmt_num(ax, 3),
        fmt_num(ay, 3),
        fmt_num(az, 3),
        fmt_num(a, 3),
    ])


# =======================
# Pi local log
# =======================
def open_new_pi_log() -> tuple[str, 'io.TextIOWrapper']:
    ts = time.strftime("%Y%m%d_%H%M%S")
    path = os.path.join(LOG_DIR, f"pi_log_{ts}.csv")
    f = open(path, "w", buffering=1)
    f.write(CSV_HEADER + "\n")
    return path, f


# =======================
# BLE main loop (connect/reconnect)
# =======================
async def ble_loop(shared: SharedState, latest_in: dict, stop_evt: asyncio.Event):
    pi_log_path, pi_log = open_new_pi_log()
    print(f"[PI] local log: {pi_log_path}")

    async def on_notify(_, data: bytearray):
        # XIAO -> Pi commands arrive here
        try:
            s = data.decode("utf-8", errors="ignore").strip()
        except Exception:
            return
        if s:
            # XIAO side may send multiple lines; split robustly
            for line in s.splitlines():
                handle_cmd(shared, line)

    while not stop_evt.is_set():
        addr = await find_xiao_addr()
        if not addr:
            print("[BLE] XIAO not found. (scan...)")
            await asyncio.sleep(1.0)
            continue

        print(f"[BLE] found {XIAO_NAME} addr={addr}")
        try:
            async with BleakClient(addr) as client:
                print("[BLE] connected")

                # Subscribe to TX notify (XIAO->Pi)
                await client.start_notify(UART_TX_UUID, on_notify)
                print("[BLE] notify started")

                # send header once (XIAO ignores header anyway, but useful for debugging)
                try:
                    await client.write_gatt_char(UART_RX_UUID, (CSV_HEADER + "\n").encode("utf-8"), response=False)
                except Exception:
                    pass

                next_telem = time.monotonic()
                next_log = time.monotonic()
                telem_dt = 1.0 / TELEM_HZ
                log_dt = 1.0 / LOG_HZ

                while client.is_connected and (not stop_evt.is_set()):
                    now = time.monotonic()

                    # telemetry
                    if now >= next_telem:
                        next_telem += telem_dt
                        line = build_csv_line(shared, latest_in)
                        payload = (line + "\n").encode("utf-8")

                        # BLE write (no response for speed)
                        try:
                            await client.write_gatt_char(UART_RX_UUID, payload, response=False)
                        except (BleakError, OSError) as e:
                            print(f"[BLE] write error: {e}")
                            break

                    # Pi local log
                    if shared.logging and now >= next_log:
                        next_log += log_dt
                        line = build_csv_line(shared, latest_in)  # NOTE: increments seq again
                        # ここで seq を二重に増やしたくない場合は、上のtelemetryと共通化してください
                        # ---- 二重増加を避けるため、下では seq を戻す方式にします ----
                        #（上で増えた分を使い回す実装もできますが、簡潔性優先）
                        # ここでは「ローカルログを主」とするなら telemetry と同一lineを使うのが理想です。
                        # その場合は build_csv_line を 1箇所だけで呼ぶように変更してください。
                        pi_log.write(line + "\n")

                    await asyncio.sleep(0.001)

                print("[BLE] disconnected (inner loop end)")

        except Exception as e:
            print(f"[BLE] connect/session error: {e}")

        await asyncio.sleep(0.5)

    try:
        pi_log.flush()
        pi_log.close()
    except Exception:
        pass


# =======================
# IMPORTANT FIX: seq duplication issue
# =======================
# 上の ble_loop のままだと telemetry と Piローカルログで seq が二重増加します。
# 「提出用の制御履歴＝Piローカルログ」を正とするなら、TELEM送信も同一行を送るべきです。
# そこで、運用上は次のようにするのが正解です：
#   - 1周期で build_csv_line を 1回だけ生成
#   - それを (a) Piローカルログ に書く (b) BLE送信 に使う
#
# そのための “確定版” を下に用意して main() ではそちらを使います。

async def ble_loop_fixed(shared: SharedState, latest_in: dict, stop_evt: asyncio.Event):
    pi_log_path, pi_log = open_new_pi_log()
    print(f"[PI] local log: {pi_log_path}")

    async def on_notify(_, data: bytearray):
        try:
            s = data.decode("utf-8", errors="ignore").strip()
        except Exception:
            return
        if s:
            for line in s.splitlines():
                handle_cmd(shared, line)

    while not stop_evt.is_set():
        addr = await find_xiao_addr()
        if not addr:
            print("[BLE] XIAO not found. (scan...)")
            await asyncio.sleep(1.0)
            continue

        print(f"[BLE] found {XIAO_NAME} addr={addr}")
        try:
            async with BleakClient(addr) as client:
                print("[BLE] connected")

                await client.start_notify(UART_TX_UUID, on_notify)
                print("[BLE] notify started")

                # optional header
                try:
                    await client.write_gatt_char(UART_RX_UUID, (CSV_HEADER + "\n").encode("utf-8"), response=False)
                except Exception:
                    pass

                next_tick = time.monotonic()
                dt = 1.0 / TELEM_HZ  # telemetry/log tick unified

                while client.is_connected and (not stop_evt.is_set()):
                    now = time.monotonic()
                    if now >= next_tick:
                        next_tick += dt

                        line = build_csv_line(shared, latest_in)

                        # Pi local log (submission-oriented)
                        if shared.logging:
                            pi_log.write(line + "\n")

                        # BLE send to XIAO
                        try:
                            await client.write_gatt_char(UART_RX_UUID, (line + "\n").encode("utf-8"), response=False)
                        except (BleakError, OSError) as e:
                            print(f"[BLE] write error: {e}")
                            break

                    await asyncio.sleep(0.001)

                print("[BLE] disconnected (inner loop end)")

        except Exception as e:
            print(f"[BLE] connect/session error: {e}")

        await asyncio.sleep(0.5)

    try:
        pi_log.flush()
        pi_log.close()
    except Exception:
        pass


# =======================
# main
# =======================
def main():
    stop_evt = asyncio.Event()
    shared = SharedState(phase=0, mode=MODE_IDLE, logging=True, seq=0)
    latest = {"ts": time.time()}

    def _sig_handler(*_):
        stop_evt.set()

    signal.signal(signal.SIGINT, _sig_handler)
    signal.signal(signal.SIGTERM, _sig_handler)

    async def runner():
        t1 = asyncio.create_task(sensor_task(shared, latest, stop_evt))
        t2 = asyncio.create_task(ble_loop_fixed(shared, latest, stop_evt))
        await asyncio.wait([t1, t2], return_when=asyncio.FIRST_COMPLETED)
        stop_evt.set()
        await asyncio.gather(t1, t2, return_exceptions=True)

    asyncio.run(runner())


if __name__ == "__main__":
    main()