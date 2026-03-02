import time
import math
from collections import deque
import statistics

import board
import busio
import adafruit_bno055
from adafruit_bme280 import basic as adafruit_bme280

# =========================
# 設定
# =========================
DT = 0.10            # 10Hz
WIN = 11             # 中央値窓（奇数推奨）
REQ = 5              # 5回連続

FREEFALL_ACC_TH = 3.0   # |a| < 3.0 m/s^2 -> 自由落下候補（要調整）
LAND_VZ_ABS_TH = 0.15   # |vz| < 0.15 m/s -> ほぼ停止（要調整）

SEA_LEVEL_HPA = 1013.25 # 現地で合わせると安定（要調整）

# =========================
# センサ初期化
# =========================
i2c = busio.I2C(board.SCL, board.SDA)
bno055 = adafruit_bno055.BNO055_I2C(i2c, address=0x28)
bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=0x76)
bme280.sea_level_pressure = SEA_LEVEL_HPA

# =========================
# ユーティリティ
# =========================
def norm3(v):
    if v is None:
        return None
    x, y, z = v
    return math.sqrt(x*x + y*y + z*z)

def median_or_none(buf):
    if len(buf) < buf.maxlen:
        return None
    return statistics.median(buf)

# =========================
# バッファ
# =========================
buf_acc = deque(maxlen=WIN)
buf_vz  = deque(maxlen=WIN)

# =========================
# ループ
# =========================
prev_alt = bme280.altitude
prev_t = time.time()

freefall_confirmed = False
freefall_consec = 0
land_consec = 0

print("Start: detecting FREEFALL -> LANDING ...")

while True:
    now = time.time()
    dt = now - prev_t
    if dt <= 0:
        dt = DT
    prev_t = now

    # read sensors
    alt = bme280.altitude
    accN = norm3(bno055.acceleration)

    # vertical speed (simple diff)
    vz = (alt - prev_alt) / dt
    prev_alt = alt

    # push to buffers (ignore None for acc)
    if accN is not None:
        buf_acc.append(accN)
    buf_vz.append(vz)

    # compute medians
    acc_med = median_or_none(buf_acc)
    vz_med  = median_or_none(buf_vz)

    # wait until window filled
    if acc_med is None or vz_med is None:
        time.sleep(DT)
        continue

    # ---- FREEFALL detection (median x 5) ----
    if not freefall_confirmed:
        if acc_med < FREEFALL_ACC_TH:
            freefall_consec += 1
        else:
            freefall_consec = 0

        print(f"[FREEFALL?] alt={alt:.2f} acc_med={acc_med:.2f} vz_med={vz_med:.2f} consec={freefall_consec}")

        if freefall_consec >= REQ:
            freefall_confirmed = True
            print("FREEFALL CONFIRMED")
            # reset landing counter for safety
            land_consec = 0

    # ---- LANDING detection (only after freefall) ----
    else:
        if abs(vz_med) < LAND_VZ_ABS_TH:
            land_consec += 1
        else:
            land_consec = 0

        print(f"[LAND?] alt={alt:.2f} acc_med={acc_med:.2f} vz_med={vz_med:.2f} consec={land_consec}")

        if land_consec >= REQ:
            print("LANDING CONFIRMED")
            break

    time.sleep(DT)

print("Done.")