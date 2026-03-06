import time
import board
import adafruit_bno055

# --- 設定項目 ---
# 国土地理院のサイトなどで、使用場所の偏角を確認してください。
# 例：東京付近なら 約 -7.5度（西に7.5度ズレている）
# 西偏（West）の場合はマイナスの値、東偏（East）ならプラスの値を使います。
MAGNETIC_DECLINATION = -7.6 

i2c = board.I2C()
sensor = adafruit_bno055.BNO055_I2C(i2c)

def get_true_heading(mag_heading, declination):
    """磁北基準の方位を真北基準に変換する"""
    # 磁気偏角を引く（西偏がマイナスなら、結果的にプラスされる）
    true_heading = mag_heading - declination
    
    # 0〜360度の範囲に収める処理
    if true_heading < 0:
        true_heading += 360
    elif true_heading >= 360:
        true_heading -= 360
    return true_heading

while True:
    heading, roll, pitch = sensor.euler
    cal_sys, cal_gyro, cal_accel, cal_mag = sensor.calibration_status

    if heading is not None:
        # 真北の計算
        true_north = get_true_heading(heading, MAGNETIC_DECLINATION)
        
        print(f"【方位】真北基準: {true_north:>6.2f}° | 磁北基準: {heading:>6.2f}°")
        print(f"【校正状況】Mag: {cal_mag} (3が理想)")
    
    time.sleep(0.2)