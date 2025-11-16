#標準ライブラリ
import RPi.GPIO as GPIO
import time
import pyserial as serial
import numpy as np

#画像系ライブラリ
import cv2
import picamera as camera

#センサー系ライブラリ
import bno055 as bno
import bme280 as bme
import melopero_samm8q as mp
import melopero_ubx as ubx

#ディスプレイ系ライブラリ
from luma.core.interface.serial import spi
from luma.lcd.device import st7735
from PIL import Image, ImageDraw, ImageFont

#モーター制御クラス
from motor import TB6612Driver

#センサー初期化
bno_sensor = bno.BNO055()
bme_sensor = bme.BME280()
gps = mp.SAM_M8Q()

#GPIO初期化
GPIO.setmode(GPIO.BCM) #GPIO番号で指定

#モーター初期化
motor_driver = TB6612Driver(
    in1_a=17,
    in2_a=27,
    pwm_a=22,
    in1_b=23,
    in2_b=24, pwm_b=25,
    stby=4
)

#main処理

#着地フェーズ
def detect_landing():
    while detect_landing != True:
        for i in range(10): #10回測定して平均を取る
            pressure = bme_sensor.read_pressure()
            altitude = bme_sensor.pressure_to_altitude(pressure)
            altitudes.append(altitude)
            time.sleep(0.1)
            
        if altitude < LANDING_ALTITUDE_THRESHOLD:
            altitude_counter += 1
        else:
            altitude_counter = 0
    
    if altitude_counter >= LANDING_CONFIRMATION_COUNT:
        return detect_landing = True:
    

#GPS誘導フェーズ
def gps_guidance():
    gps.ubx_only()
    gps.wait_for_acknowledge(ubx.CFG_CLASS, ubx.CFG_PRT)

    gps.set_message_frequency(ubx.NAV_CLASS, ubx.NAV_PVT, 1)
    gps.wait_for_acknowledge(ubx.CFG_CLASS ,ubx.CFG_MSG)

    gps.set_measurement_frequency(100, 1)
    gps.wait_for_acknowledge(ubx.CFG_CLASS, ubx.CFG_RATE)

    while not at_target_location:
        info = gps.get_pvt()
        if info:
            current_latitude = info[gps.LATITUDE_TAG]
            current_longitude = info[gps.LONGITUDE_TAG]
            
            #目標位置までの距離と方向を計算
            distance, bearing = calculate_distance_and_bearing(current_latitude, current_longitude, TARGET_LATITUDE, TARGET_LONGITUDE)
            
            if distance < TARGET_PROXIMITY_THRESHOLD:
                at_target_location = True
                motor_driver.stop()
            else:
                adjust_course(bearing)
        time.sleep(0.1)

#カメラ誘導フェーズ
def camera_guidance():
    with camera.PiCamera() as cam:
        cam.resolution = (640, 480)
        cam.framerate = 30
        time.sleep(2)  # カメラのウォームアップ

        while not at_target_location:
            frame = np.empty((480, 640, 3), dtype=np.uint8)
            cam.capture(frame, 'rgb')
            
            target_position = detect_target_in_frame(frame)
            if target_position:
                adjust_course_towards_target(target_position)
            else:
                search_for_target()
            time.sleep(0.1)


def main():
    detect_landing()
    gps_guidance()
    camera_guidance()

if __name__ == "__main__":
    main()