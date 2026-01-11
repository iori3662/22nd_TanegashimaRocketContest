import serial
import sys
import time
import select

PORT = "/dev/ttyUSB0"
BAUD = 38400

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0)
    except Exception as e:
        print("ポートを開けません:", e)
        sys.exit(1)

    print(f"ポート {PORT} をオープンしました")
    print("受信したデータは表示されます")
    print("キーボード入力で送信できます（Ctrl+Cで終了）")
    print("-------------------------------------------")

    try:
        while True:
            # -------------------------------------------------
            # 1) TWELITE からの受信処理
            # -------------------------------------------------
            if ser.in_waiting > 0:
                data = ser.readline()
                if data:
                    text = data.decode(errors="replace").rstrip()
                    print(f"[RX] {text}")

            # -------------------------------------------------
            # 2) キーボード入力の監視
            # -------------------------------------------------
            rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
            if sys.stdin in rlist:
                line = sys.stdin.readline().rstrip()
                if line:
                    ser.write((line + "\r\n").encode())
                    print(f"[TX] {line}")

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n終了します。")
    finally:
        ser.close()
        print("ポートをクローズしました。")

if __name__ == "__main__":
    main()
