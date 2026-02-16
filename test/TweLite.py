import serial
import time
import sys

# ===== ユーザ設定 =====
# MONOSTICK/USB接続なら: "/dev/ttyUSB0" or "/dev/ttyACM0"
# GPIO UARTなら: "/dev/serial0"
PORT = "/dev/serial0"
BAUD = 38400  # TWELITE側の設定と合わせる

def open_serial():
    ser = serial.Serial(
        PORT,
        BAUD,
        timeout=0.1,   # 読み取りの待ち時間[秒]
    )
    return ser

def main():
    try:
        ser = open_serial()
    except Exception as e:
        print("シリアルポートを開けませんでした:", e)
        sys.exit(1)

    print(f"TWELITE 親機ポートをオープンしました: {PORT} @ {BAUD}bps")
    print("Ctrl+C で終了します。")
    print("ターミナルで文字を入力すると、そのまま無線で送信されます。")

    # 簡単な送信カウンタ
    send_counter = 0
    last_send_time = time.time()

    try:
        while True:
            # -------------------------------------------------
            # 1) 受信処理: TWELITE → Raspberry Pi
            # -------------------------------------------------
            line = ser.readline()  # 行単位で読み取り（\nまで）
            if line:
                try:
                    text = line.decode("utf-8", errors="replace").rstrip()
                except UnicodeDecodeError:
                    text = repr(line)
                print(f"[RX] {text}")

            # -------------------------------------------------
            # 2) 定期送信（2秒に1回テストメッセージ）
            # -------------------------------------------------
            now = time.time()
            if now - last_send_time > 2.0:
                msg = f"RPI,{send_counter},HELLO_FROM_RPI"
                # App_UART(Eモード)では1行ずつ送るのが基本（CRLF推奨）
                ser.write((msg + "\r\n").encode("utf-8"))
                print(f"[TX] {msg}")
                send_counter += 1
                last_send_time = now

            # -------------------------------------------------
            # 3) キーボード入力があれば即送信（オプション）
            # -------------------------------------------------
            # 簡易実装なので、必要になったら select を使ってもOK
            # 今は送信はテストメッセージだけにしておく

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nユーザ操作により終了します。")
    finally:
        ser.close()
        print("シリアルポートをクローズしました。")

if __name__ == "__main__":
    main()
