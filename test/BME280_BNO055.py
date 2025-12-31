import time
import board
import busio
import adafruit_bno055
from adafruit_bme280 import basic as adafruit_bme280


i2c = busio.I2C(board.SCL, board.SDA)

bno055 = adafruit_bno055.BNO055_I2C(i2c, address=0x28)
bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=0x76)

bme280.sea_level_pressure = 1013.25


while True:
    euler = bno055.euler
    if euler is not None:
        print("Euler angle (deg):")
        print(f"  Heading: {euler[0]:.2f}")
        print(f"  Roll:    {euler[1]:.2f}")
        print(f"  Pitch:   {euler[2]:.2f}")
    else:
        print("Euler angle not available yet")


    print("Angular velocity (rad/s):")
    print(f"  Gyro: {bno055.gyro}")

    print("Acceleration (m/s^2):")
    print(f"  Accel: {bno055.acceleration}")

    print("Magnetic field (uT):")
    print(f"  Mag: {bno055.magnetic}")

    print("Calibration status (sys, gyro, accel, mag):")
    print(f"  Calib: {bno055.calibration_status}")

    print("-" * 40)
    
    print(f"Temp      = {bme280.temperature:.2f} C")
    print(f"Pressure  = {bme280.pressure:.2f} hPa")
    print(f"Humidity  = {bme280.humidity:.2f} %")
    print(f"Altitude  = {bme280.altitude:.2f} m")
    print("-" * 30)
    time.sleep(0.1)
