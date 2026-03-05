#!/usr/bin/env python3
import time
import math
import cv2
import numpy as np
from collections import deque
from dataclasses import dataclass
import pigpio

import board
import adafruit_bno055
from adafruit_bme280 import basic as adafruit_bme280
from smbus2 import SMBus
import pynmea2
from picamera2 import Picamera2

# =====================================================
# 設定
# =====================================================

TWELITE_RX_GPIO = 14
TWELITE_TX_GPIO = 15
TWELITE_BAUD = 38400

SEND_HZ = 1.0
LOOP_HZ = 20.0

GOAL_LAT = 30.374896
GOAL_LON = 130.957641

ARRIVAL_RADIUS = 2.0

MISSION_TIMEOUT = 19 * 60

# =====================================================
# ユーティリティ
# =====================================================

def clamp(x, a, b):
    return max(a, min(b, x))


def wrap180(x):
    while x > 180:
        x -= 360
    while x < -180:
        x += 360
    return x


def haversine(lat1, lon1, lat2, lon2):
    R = 6371000
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    d1 = math.radians(lat2 - lat1)
    d2 = math.radians(lon2 - lon1)

    a = math.sin(d1/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(d2/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R*c


def bearing(lat1, lon1, lat2, lon2):
    y = math.sin(math.radians(lon2-lon1)) * math.cos(math.radians(lat2))
    x = math.cos(math.radians(lat1))*math.sin(math.radians(lat2)) - \
        math.sin(math.radians(lat1))*math.cos(math.radians(lat2))*math.cos(math.radians(lon2-lon1))
    return (math.degrees(math.atan2(y,x)) + 360) % 360


# =====================================================
# サーボ
# =====================================================

class ServoDrive:

    def __init__(self):

        self.pi = pigpio.pi()

        self.L = 18
        self.R = 12

        self.stop = 1500
        self.scale = 400

    def write(self,l,r):
        self.pi.set_servo_pulsewidth(self.L,l)
        self.pi.set_servo_pulsewidth(self.R,r)

    def stop_drive(self):
        self.write(self.stop,self.stop)

    def set_vw(self,v,w):

        v = clamp(v,-1,1)
        w = clamp(w,-1,1)

        l = self.stop - v*self.scale + w*self.scale
        r = self.stop + v*self.scale + w*self.scale

        self.write(l,r)


# =====================================================
# GPS
# =====================================================

class GPS:

    def __init__(self):

        self.bus = SMBus(1)
        self.addr = 0x42
        self.buf = bytearray()

        self.lat=None
        self.lon=None
        self.alt=None

    def update(self):

        try:
            d = self.bus.read_i2c_block_data(self.addr,0xff,32)
            self.buf.extend(d)
        except:
            return

        while b"\n" in self.buf:

            line,_,rest = self.buf.partition(b"\n")
            self.buf=bytearray(rest)

            s=line.decode(errors="ignore")

            if not s.startswith("$"):
                continue

            try:
                msg=pynmea2.parse(s)
            except:
                continue

            if msg.sentence_type=="RMC" and msg.status=="A":

                try:
                    lat=float(msg.lat)
                    lon=float(msg.lon)
                except:
                    continue

                d=int(lat/100)
                m=lat-d*100
                lat=d+m/60

                d=int(lon/100)
                m=lon-d*100
                lon=d+m/60

                if msg.lat_dir=="S": lat=-lat
                if msg.lon_dir=="W": lon=-lon

                self.lat=lat
                self.lon=lon


# =====================================================
# カメラ誘導
# =====================================================

class CameraGuide:

    def __init__(self,drive):

        self.drive=drive
        self.state="SEARCH"

        self.last_seen=0
        self.last_err=0
        self.acquire_count=0

        self.picam=Picamera2()
        self.picam.configure(self.picam.create_preview_configuration(main={"size":(640,480)}))
        self.picam.start()

    def red_mask(self,img):

        hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

        m1=cv2.inRange(hsv,(0,120,70),(10,255,255))
        m2=cv2.inRange(hsv,(170,120,70),(179,255,255))

        mask=cv2.bitwise_or(m1,m2)

        kernel=np.ones((5,5),np.uint8)

        mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)
        mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel)

        return mask

    def step(self):

        frame=self.picam.capture_array()

        mask=self.red_mask(frame)

        h,w=mask.shape

        ratio=cv2.countNonZero(mask)/(h*w)

        cnts,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

        if cnts:
            c=max(cnts,key=cv2.contourArea)
            M=cv2.moments(c)

            if M["m00"]!=0:

                cx=int(M["m10"]/M["m00"])
                err=cx-w/2

                self.last_err=err
                self.last_seen=time.time()

                seen=True
            else:
                seen=False
        else:
            seen=False

        if ratio>0.6:
            self.drive.stop_drive()
            return True

        if self.state=="SEARCH":

            if seen:
                self.state="ACQUIRE"
            else:
                self.drive.set_vw(0,0.3)

        elif self.state=="ACQUIRE":

            if seen:

                turn=clamp(err*0.003,-0.6,0.6)

                self.drive.set_vw(0,turn)

                self.acquire_count+=1

                if self.acquire_count>4:
                    self.state="TRACK"

            else:
                if time.time()-self.last_seen<0.8:
                    turn=0.3 if self.last_err>0 else -0.3
                    self.drive.set_vw(0,turn)
                else:
                    self.state="SEARCH"

        elif self.state=="TRACK":

            if seen:

                if abs(err)<50:
                    self.drive.set_vw(0.5,0)
                else:
                    turn=clamp(err*0.003,-0.6,0.6)
                    self.drive.set_vw(0,turn)

            else:

                if time.time()-self.last_seen<0.8:
                    turn=0.3 if self.last_err>0 else -0.3
                    self.drive.set_vw(0,turn)
                else:
                    self.state="SEARCH"

        return False


# =====================================================
# GPS誘導
# =====================================================

class GPSGuide:

    def __init__(self,drive,bno,gps):

        self.drive=drive
        self.bno=bno
        self.gps=gps

        self.heading_offset=180

    def step(self):

        if self.gps.lat is None:
            return None

        dist=haversine(self.gps.lat,self.gps.lon,GOAL_LAT,GOAL_LON)

        head=self.bno.euler[0]

        if head is None:
            return None

        head=(head+self.heading_offset)%360

        goal=bearing(self.gps.lat,self.gps.lon,GOAL_LAT,GOAL_LON)

        err=wrap180(goal-head)

        if dist<ARRIVAL_RADIUS:

            self.drive.stop_drive()
            return True

        if abs(err)>70:
            v=0
        else:
            v=0.7*max(0,math.cos(math.radians(err)))

        w=clamp(err*0.02,-0.6,0.6)

        self.drive.set_vw(v,w)

        return dist


# =====================================================
# TWELITE
# =====================================================

class Twelite:

    def __init__(self):

        self.pi=pigpio.pi()

        self.tx=TWELITE_TX_GPIO
        self.rx=TWELITE_RX_GPIO

        self.pi.set_mode(self.tx,pigpio.OUTPUT)
        self.pi.write(self.tx,1)

        self.pi.set_mode(self.rx,pigpio.INPUT)

    def send(self,line):

        payload=(line+"\r\n").encode()

        self.pi.wave_clear()
        self.pi.wave_add_serial(self.tx,TWELITE_BAUD,payload)
        wid=self.pi.wave_create()

        self.pi.wave_send_once(wid)

        while self.pi.wave_tx_busy():
            time.sleep(0.001)

        self.pi.wave_delete(wid)


# =====================================================
# MAIN
# =====================================================

def main():

    drive=ServoDrive()

    gps=GPS()

    bno=adafruit_bno055.BNO055_I2C(board.I2C())

    bme=adafruit_bme280.Adafruit_BME280_I2C(board.I2C())

    cam=CameraGuide(drive)

    gpsguide=GPSGuide(drive,bno,gps)

    radio=Twelite()

    start=None

    while True:

        gps.update()

        if start is None:
            start=time.time()

        if time.time()-start>MISSION_TIMEOUT:
            drive.stop_drive()
            break

        dist=gpsguide.step()

        if dist and dist<10:

            goal=cam.step()

            if goal:
                break

        e=bno.euler

        line=",".join(map(str,[

        gps.lat,
        gps.lon,
        dist,

        e[0] if e else None,
        e[1] if e else None,
        e[2] if e else None,

        bme.temperature,
        bme.pressure,
        bme.humidity

        ]))

        radio.send(line)

        print(line)

        time.sleep(1/SEND_HZ)


if __name__=="__main__":
    main()