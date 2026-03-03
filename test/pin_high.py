import pigpio
import time

GPIO_PIN = 17  # BCM番号

# pigpio接続
pi = pigpio.pi()

if not pi.connected:
    print("pigpio daemonに接続できません")
    exit()

# 出力設定
pi.set_mode(GPIO_PIN, pigpio.OUTPUT)

try:
    while True:
        print("HIGH")
        pi.write(GPIO_PIN, 1)   # HIGH
        time.sleep(1)

        print("LOW")
        pi.write(GPIO_PIN, 0)   # LOW
        time.sleep(1)

except KeyboardInterrupt:
    pass

# 終了処理
pi.write(GPIO_PIN, 0)  # 念のためLOWに戻す
pi.stop()