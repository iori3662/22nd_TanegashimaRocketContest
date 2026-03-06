import time
import math
import board
import busio
import pynmea2
import pigpio
import adafruit_bno055
from smbus2 import SMBus

# --- 設定定数 ---
GOAL_LAT = 35.000000  # 目的地の緯度 (例)
GOAL_LON = 135.000000 # 目的地の経度 (例)
GPS_ADDR = 0x42
BNO_ADDR = 0x28
SERVO1_PIN = 18 # 右
SERVO2_PIN = 12 # 左
STOP_PULSE = 1490
FORWARD_LEFT = 2000
FORWARD_RIGHT = 1000
DISTANCE_THRESHOLD = 3.0 # 3m以内に近づいたらゴール
ANGLE_THRESHOLD = 10.0   # 10度以内の誤差なら直進
MAG_DECLINATION = 7.5    # 磁気偏角 (日本国内なら約7〜9度。地域に合わせて調整)

# --- 初期化 ---
pi = pigpio.pi()
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c, address=BNO_ADDR)

def get_gps_data():
    """GPSから最新の緯度経度を取得する"""
    buf = bytearray()
    with SMBus(1) as bus:
        # I2Cバッファを読み切る
        for _ in range(10):
            try:
                data = bus.read_i2c_block_data(GPS_ADDR, 0xFF, 32)
                buf.extend(data)
            except OSError:
                break
        
        while b'\n' in buf:
            line, _, rest = buf.partition(b'\n')
            buf = bytearray(rest)
            s = line.decode("ascii", errors="ignore").strip()
            if s.startswith("$GPRMC") or s.startswith("$GNRMC"):
                try:
                    msg = pynmea2.parse(s)
                    if msg.status == "A":
                        lat = dm_to_deg(msg.lat, msg.lat_dir)
                        lon = dm_to_deg(msg.lon, msg.lon_dir)
                        return lat, lon
                except:
                    continue
    return None, None

def dm_to_deg(dm, direction):
    if not dm: return None
    dm = float(dm)
    deg = int(dm // 100)
    val = deg + (dm - deg * 100) / 60.0
    return -val if direction in ("S", "W") else val

def get_distance_and_bearing(lat1, lon1, lat2, lon2):
    """2点間の距離と方位角を計算する"""
    # 緯度経度をラジアンに変換
    phi1, lambda1 = math.radians(lat1), math.radians(lon1)
    phi2, lambda2 = math.radians(lat2), math.radians(lon2)
    
    # 方位角の計算
    y = math.sin(lambda2 - lambda1) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(lambda2 - lambda1)
    bearing = (math.degrees(math.atan2(y, x)) + 360) % 360
    
    # 距離の計算 (簡易版: 赤道半径6378137m)
    r = 6378137
    distance = r * math.acos(math.sin(phi1)*math.sin(phi2) + math.cos(phi1)*math.cos(phi2)*math.cos(lambda2-lambda1))
    
    return distance, bearing

def control_servos(left, right):
    pi.set_servo_pulsewidth(SERVO1_PIN, right)
    pi.set_servo_pulsewidth(SERVO2_PIN, left)

def calibrate_bno():
    print("BNO055のキャリブレーションを開始します。機体を8の字に動かしてください...")
    while True:
        sys, gyro, accel, mag = sensor.calibration_status
        print(f"Calib Status: Sys={sys} Gyro={gyro} Accel={accel} Mag={mag}")
        if mag >= 3: # 磁気センサの精度が最高になれば抜ける
            print("磁気センサのキャリブレーション完了！")
            break
        time.sleep(1)

# --- メインループ ---
try:
    calibrate_bno()
    
    while True:
        # 1. 現在地取得
        lat, lon = get_gps_data()
        if lat is None:
            print("GPS待ち...")
            control_servos(STOP_PULSE, STOP_PULSE)
            time.sleep(1)
            continue
        
        # 2. 目的地までの距離と方位を計算
        dist, target_bearing = get_distance_and_bearing(lat, lon, GOAL_LAT, GOAL_LON)
        print(f"現在地: {lat:.6f}, {lon:.6f} / 距離: {dist:.2f}m / 目標方位: {target_bearing:.2f}°")
        
        if dist < DISTANCE_THRESHOLD:
            print("ゴールに到着しました！")
            break

        # 3. 向き調整ループ
        while True:
            current_heading = sensor.euler[0] # 0-360度
            if current_heading is None: continue
            
            # 磁気偏角の補正（真北に合わせる）
            current_heading = (current_heading + MAG_DECLINATION + 360) % 360
            
            # 目標との角度差を計算 (-180 to 180)
            diff = (target_bearing - current_heading + 180) % 360 - 180
            
            print(f"現在方位: {current_heading:.2f}° / 角度差: {diff:.2f}°")
            
            if abs(diff) < ANGLE_THRESHOLD:
                print("目標方向を向きました。前進します。")
                control_servos(STOP_PULSE, STOP_PULSE)
                break
            elif diff > 0:
                # 右旋回
                control_servos(2000, 2000)
            else:
                # 左旋回
                control_servos(1000, 1000)
            time.sleep(0.1)

        # 4. 前進
        control_servos(FORWARD_LEFT, FORWARD_RIGHT)
        time.sleep(3) # 3秒間前進
        control_servos(STOP_PULSE, STOP_PULSE)

except KeyboardInterrupt:
    print("停止します")
finally:
    control_servos(0, 0)
    pi.stop()