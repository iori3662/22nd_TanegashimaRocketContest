import pigpio
import time

SERVO1_PIN = 18  # GPIO18　右サーボ
SERVO2_PIN = 12  # GPIO12　左サーボ

pi = pigpio.pi()
if not pi.connected:
    exit()

#停止
pi.set_servo_pulsewidth(SERVO1_PIN, 1490)
pi.set_servo_pulsewidth(SERVO2_PIN, 1490)
print("Stopping servos...")
time.sleep(2)

#前進
pi.set_servo_pulsewidth(SERVO1_PIN, 1000)
pi.set_servo_pulsewidth(SERVO2_PIN, 2000)
print("Positive servos...")
time.sleep(3)

#停止
pi.set_servo_pulsewidth(SERVO1_PIN, 1490)
pi.set_servo_pulsewidth(SERVO2_PIN, 1490)
print("Stopping servos...")
time.sleep(2)

#左旋回
pi.set_servo_pulsewidth(SERVO1_PIN, 1000)
pi.set_servo_pulsewidth(SERVO2_PIN, 1000)
print("left spin servos...")
time.sleep(3)

#停止
pi.set_servo_pulsewidth(SERVO1_PIN, 1490)
pi.set_servo_pulsewidth(SERVO2_PIN, 1490)
print("Stopping servos...")
time.sleep(3)

#右旋回
pi.set_servo_pulsewidth(SERVO1_PIN, 2000)
pi.set_servo_pulsewidth(SERVO2_PIN, 2000)
print("Right spin servos...")
time.sleep(3)

pi.set_servo_pulsewidth(SERVO1_PIN, 0)
pi.set_servo_pulsewidth(SERVO2_PIN, 0)
print("free")
pi.stop()


pi.stop()
print("Finish")
