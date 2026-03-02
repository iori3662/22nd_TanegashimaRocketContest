import time
import math
from collections import deque
import statistics

import board
import busio
import adafruit_bno055
from adafruit_bme280 import basic as adafruit_bme280

# =========================
# 設定（あなたの前提：10Hz）
# =========================
DT = 0.10
WIN = 11
REQ = 5

SEA_LEVEL_HPA = 1013.25

# ---- 落下（FREEFALL） OR条件 ----
FREEFALL_ACC_TH  = 3.0     # |a| < 3.0 m/s^2
FREEFALL_DALT_TH = -0.10   # Δalt < -0.10 m / 0.1s（= -1.0 m/s相当）※10Hz前提

# ---- 着地（LANDING） OR条件 ----
ACC_LAND_MIN = 6.0         # |a| が1g付近（下限）
ACC_LAND_MAX = 13.0        # |a| が1g付近（上限）
LAND_DALT_ABS_TH = 0.05    # |Δalt| < 0.05 m / 0.1s で「ほぼ高度変化なし」※要調整

# =========================
# 初期化
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
    if x is None or y is None or z is None:
        return None
    return math.sqrt(x*x + y*y + z*z)

def median_or_none(buf):
    if len(buf) < buf.maxlen:
        return None
    return statistics.median(buf)

# =========================
# バッファ
# =========================
buf_acc  = deque(maxlen=WIN)
buf_dalt = deque(maxlen=WIN)

prev_alt = bme280.altitude

freefall_confirmed = False
ff_consec = 0
land_consec = 0

print("Start: FREEFALL -> LANDING (OR: accel or dAlt)")

while True:
    alt = bme280.altitude
    dalt = alt - prev_alt
    prev_alt = alt

    accN = norm3(bno055.acceleration)

    if accN is not None:
        buf_acc.append(accN)
    buf_dalt.append(dalt)

    acc_med  = median_or_none(buf_acc)
    dalt_med = median_or_none(buf_dalt)

    if acc_med is None or dalt_med is None:
        time.sleep(DT)
        continue

    # ========= FREEFALL (OR) =========
    if not freefall_confirmed:
        ff_cond = (acc_med < FREEFALL_ACC_TH) or (dalt_med < FREEFALL_DALT_TH)

        if ff_cond:
            ff_consec += 1
        else:
            ff_consec = 0

        print(f"[FREEFALL?] alt={alt:.2f} acc_med={acc_med:.2f} dalt_med={dalt_med:.3f} cond={ff_cond} consec={ff_consec}")

        if ff_consec >= REQ:
            freefall_confirmed = True
            land_consec = 0
            print("FREEFALL CONFIRMED")

    # ========= LANDING (OR) =========
    else:
        land_cond = ((ACC_LAND_MIN <= acc_med <= ACC_LAND_MAX) or (abs(dalt_med) < LAND_DALT_ABS_TH))

        if land_cond:
            land_consec += 1
        else:
            land_consec = 0

        print(f"[LAND?] alt={alt:.2f} acc_med={acc_med:.2f} dalt_med={dalt_med:.3f} cond={land_cond} consec={land_consec}")

        if land_consec >= REQ:
            print("LANDING CONFIRMED")
            break

    time.sleep(DT)

print("Done.")