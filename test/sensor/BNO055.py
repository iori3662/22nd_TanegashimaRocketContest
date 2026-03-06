import time
import board
import busio
import adafruit_bno055


i2c = busio.I2C(board.SCL, board.SDA)

sensor = adafruit_bno055.BNO055_I2C(i2c, address=0x28)


while True:
    print("Euler angle (deg):")
    print(f"  Heading: {sensor.euler[0]:.2f}")
    print(f"  Roll:    {sensor.euler[1]:.2f}")
    print(f"  Pitch:   {sensor.euler[2]:.2f}")

    print("Angular velocity (rad/s):")
    print(f"  Gyro: {sensor.gyro}")

    print("Acceleration (m/s^2):")
    print(f"  Accel: {sensor.acceleration}")

    print("Magnetic field (uT):")
    print(f"  Mag: {sensor.magnetic}")

    print("Calibration status (sys, gyro, accel, mag):")
    print(f"  Calib: {sensor.calibration_status}")

    print("-" * 40)
    time.sleep(1)
