import time, math
from collections import deque
import statistics

DT = 0.10  # 10Hz

# median window
WIN = 11               # 奇数推奨（中央値が安定）
REQ_CONSEC = 5         # 5回連続

# thresholds (要現地チューニング)
ACC_MIN, ACC_MAX = 6.0, 13.0     # |a| が1g付近
VZ_ABS_TH = 0.10                 # |vz| がほぼ0（停止）
GYRO_ABS_TH = 0.25               # 任意：停止時の回転の小ささ

# buffers
buf_acc = deque(maxlen=WIN)
buf_vz  = deque(maxlen=WIN)
buf_gyro= deque(maxlen=WIN)

land_consec = 0

def norm3(v):
    if v is None:
        return None
    return math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)

def median_or_none(buf):
    if len(buf) < buf.maxlen:
        return None
    return statistics.median(buf)

# ループ内（例）
prev_alt = bme280.altitude
prev_t = time.time()

while True:
    now = time.time()
    dt = now - prev_t
    if dt <= 0:
        dt = DT
    prev_t = now

    alt = bme280.altitude
    accN = norm3(bno055.acceleration)
    gyroN = norm3(bno055.gyro)

    vz = (alt - prev_alt) / dt
    prev_alt = alt

    # バッファに積む（Noneは捨てる）
    if accN is not None:  buf_acc.append(accN)
    if gyroN is not None: buf_gyro.append(gyroN)
    buf_vz.append(vz)  # vzは計算できるので入れる

    # 中央値を作る（窓が埋まるまでNone）
    acc_med  = median_or_none(buf_acc)
    vz_med   = median_or_none(buf_vz)
    gyro_med = median_or_none(buf_gyro)  # gyroが取れない環境なら判定から外す設計も可

    if acc_med is None or vz_med is None:
        time.sleep(DT)
        continue

    # 着地条件（中央値ベース）
    acc_ok = (ACC_MIN <= acc_med <= ACC_MAX)
    vz_ok  = (abs(vz_med) <= VZ_ABS_TH)

    # gyroは「あるなら使う」方式（安全側に倒すなら gyro_med is None をNGに）
    gyro_ok = (gyro_med is None) or (gyro_med <= GYRO_ABS_TH)

    if acc_ok and vz_ok and gyro_ok:
        land_consec += 1
    else:
        land_consec = 0

    print(f"acc_med={acc_med:.2f} vz_med={vz_med:.2f} gyro_med={gyro_med if gyro_med is None else round(gyro_med,2)} consec={land_consec}")

    if land_consec >= REQ_CONSEC:
        print("LANDED CONFIRMED (median x 5)")
        # -> ここで pin11 HIGH（タイマで必ずOFF）
        GPIO.output(11, GPIO.HIGH)
        time.sleep(2.0)
        GPIO.output(11, GPIO.LOW)
        break

    time.sleep(DT)