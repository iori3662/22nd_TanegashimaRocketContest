#!/usr/bin/env python3

import pigpio
import time
import board
import adafruit_bno055
import json

# =========================
# servo
# =========================
SERVO_R = 18
SERVO_L = 12

STOP = 1490
FWD_R = 1000
FWD_L = 2000
LEFT = 1000
RIGHT = 2000

# =========================
# init
# =========================
pi = pigpio.pi()
if not pi.connected:
    print("pigpio error")
    exit()

i2c = board.I2C()
bno = adafruit_bno055.BNO055_I2C(i2c)

time.sleep(1)

# =========================
# magnetometer buffer
# =========================
mag_min = [1e9,1e9,1e9]
mag_max = [-1e9,-1e9,-1e9]

# =========================
# servo helpers
# =========================
def stop():
    pi.set_servo_pulsewidth(SERVO_R, STOP)
    pi.set_servo_pulsewidth(SERVO_L, STOP)

def forward():
    pi.set_servo_pulsewidth(SERVO_R, FWD_R)
    pi.set_servo_pulsewidth(SERVO_L, FWD_L)

def spin_left():
    pi.set_servo_pulsewidth(SERVO_R, LEFT)
    pi.set_servo_pulsewidth(SERVO_L, LEFT)

def spin_right():
    pi.set_servo_pulsewidth(SERVO_R, RIGHT)
    pi.set_servo_pulsewidth(SERVO_L, RIGHT)

# =========================
# magnetometer update
# =========================
def update_mag():

    mag = bno.magnetic

    if mag is None:
        return

    for i in range(3):
        mag_min[i] = min(mag_min[i], mag[i])
        mag_max[i] = max(mag_max[i], mag[i])

# =========================
# movement with sampling
# =========================
def run_motion(func, duration):

    func()
    start = time.time()

    while time.time() - start < duration:
        update_mag()
        time.sleep(0.05)

    stop()
    time.sleep(1)

# =========================
# calibration motion
# =========================
print("Start magnetic calibration")

# その場回転
run_motion(spin_left,5)
run_motion(spin_right,5)

# 前進
run_motion(forward,3)

# 再回転
run_motion(spin_left,5)
run_motion(spin_right,5)

stop()

# =========================
# compute offset
# =========================
offset = [
    (mag_max[i] + mag_min[i]) / 2
    for i in range(3)
]

scale = [
    (mag_max[i] - mag_min[i]) / 2
    for i in range(3)
]

print("mag_min:",mag_min)
print("mag_max:",mag_max)

print("offset:",offset)
print("scale:",scale)

# =========================
# save
# =========================
data = {
    "offset": offset,
    "scale": scale
}

with open("mag_calibration.json","w") as f:
    json.dump(data,f,indent=2)

print("saved calibration")

# =========================
# cleanup
# =========================
pi.set_servo_pulsewidth(SERVO_R,0)
pi.set_servo_pulsewidth(SERVO_L,0)

pi.stop()

print("Finish")