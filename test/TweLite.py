import serial
import time

ser = serial.Serial(
    port='/dev/ttyAMA0',   
    baudrate=115200,
    timeout=0.1
)

time.sleep(1)
print("UART connected")

try:
    while True:
        ser.write(b"HELLO TWELITE\n")

        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            print("RX:", line)

        time.sleep(1)

except KeyboardInterrupt:
    pass

finally:
    ser.close()
