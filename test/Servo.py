import pigpio
import time

SERVO1_PIN = 18  # GPIO18
SERVO2_PIN = 12  # GPIO12

pi = pigpio.pi()
if not pi.connected:
    exit()

pi.set_servo_pulsewidth(SERVO1_PIN, 1490)
pi.set_servo_pulsewidth(SERVO2_PIN, 1490)
print("Stopping servos...")
time.sleep(2)

pi.set_servo_pulsewidth(SERVO1_PIN, 2000)
pi.set_servo_pulsewidth(SERVO2_PIN, 2000)
print("Positive servos...")
time.sleep(3)

pi.set_servo_pulsewidth(SERVO1_PIN, 1490)
pi.set_servo_pulsewidth(SERVO2_PIN, 1490)
print("Stopping servos...")
time.sleep(2)

pi.set_servo_pulsewidth(SERVO1_PIN, 1000)
pi.set_servo_pulsewidth(SERVO2_PIN, 1000)
print("Negative servos...")
time.sleep(3)

pi.set_servo_pulsewidth(SERVO1_PIN, 1490)
pi.set_servo_pulsewidth(SERVO2_PIN, 1490)
print("Stopping servos...")
time.sleep(3)

pi.set_servo_pulsewidth(SERVO1_PIN, 0)
pi.set_servo_pulsewidth(SERVO2_PIN, 0)
print("free")
pi.stop()


pi.stop()
print("Finish")
