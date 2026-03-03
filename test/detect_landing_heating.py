import time
import math
from collections import deque
import statistics

import board
import busio
import adafruit_bno055
from adafruit_bme280 import basic as adafruit_bme280
import pigpio

# =========================
# 設定
# =========================
DT = 0.10          # 10Hz
WIN = 11
REQ = 5

FREEFALL_ACC_TH  = 3.0      # |a| < 3
FREEFALL_DALT_TH = -0.10    # Δalt < -0.10 m/0.1s（10Hz前提）

SEA_LEVEL_HPA = 1013.25

GPIO_PIN = 17  # BCM番号

# =========================
# pigpio初期化
# =========================
pi = pigpio.pi()
if not pi.connected:
    print("pigpio daemonに接続できません")
    exit()

pi.set_mode(GPIO_PIN, pigpio.OUTPUT)
pi.write(GPIO_PIN, 0)

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
ff_consec = 0

print("Start FALL detection (OR condition)")

try:
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

        # ===== 落下検知（OR）=====
        fall_cond = (acc_med < FREEFALL_ACC_TH) or (dalt_med < FREEFALL_DALT_TH)

        if fall_cond:
            ff_consec += 1
        else:
            ff_consec = 0

        print(f"acc_med={acc_med:.2f} dalt_med={dalt_med:.3f} fall={fall_cond} consec={ff_consec}")

        if ff_consec >= REQ:
            print("FALL DETECTED → GPIO17 HIGH 1秒")

            pi.write(GPIO_PIN, 1)
            time.sleep(1.0)
            pi.write(GPIO_PIN, 0)

            print("GPIO17 LOW")
            break

        time.sleep(DT)

except KeyboardInterrupt:
    pass

finally:
    pi.write(GPIO_PIN, 0)
    pi.stop()