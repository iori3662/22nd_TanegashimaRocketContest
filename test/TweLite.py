import time
import pigpio

# --- あなたの基板配線に合わせた「役割入れ替え」 ---
TX_PIN = 15  # Pi GPIO15 -> TWELITE RX なので「送信」に使う
RX_PIN = 14  # Pi GPIO14 <- TWELITE TX なので「受信」に使う
BAUD   = 38400

class SoftUART:
    def __init__(self, pi: pigpio.pi, tx_pin: int, rx_pin: int, baud: int):
        self.pi = pi
        self.tx = tx_pin
        self.rx = rx_pin
        self.baud = baud

        # GPIOモード設定（重要）
        self.pi.set_mode(self.tx, pigpio.OUTPUT)
        self.pi.set_mode(self.rx, pigpio.INPUT)
        self.pi.set_pull_up_down(self.rx, pigpio.PUD_UP)  # 必要ならPUD_OFFでもOK

        # 受信開始（bitbang read）
        self.pi.bb_serial_read_close(self.rx)
        self.pi.bb_serial_read_open(self.rx, self.baud)

        self._rx_buf = bytearray()

    def close(self):
        self.pi.bb_serial_read_close(self.rx)

    def write(self, data: bytes):
        # pigpio waveを使ってシリアル送信
        self.pi.wave_clear()
        self.pi.wave_add_serial(self.tx, self.baud, data)
        wid = self.pi.wave_create()
        if wid < 0:
            raise RuntimeError("wave_create failed")
        self.pi.wave_send_once(wid)
        while self.pi.wave_tx_busy():
            time.sleep(0.001)
        self.pi.wave_delete(wid)

    def readline(self) -> bytes | None:
        # 受信バッファに追加
        count, data = self.pi.bb_serial_read(self.rx)
        if count > 0:
            self._rx_buf.extend(data)

        # 1行（\n）で切り出し
        if b"\n" in self._rx_buf:
            line, _, rest = self._rx_buf.partition(b"\n")
            self._rx_buf = bytearray(rest)
            return line + b"\n"
        return None

def main():
    pi = pigpio.pi()
    if not pi.connected:
        raise SystemExit("pigpiod に接続できません。sudo systemctl status pigpiod を確認してください。")

    uart = SoftUART(pi, TX_PIN, RX_PIN, BAUD)

    n = 0
    t0 = time.monotonic()

    try:
        print("START soft UART test")
        while True:
            # 1秒ごとに送信
            if time.monotonic() - t0 >= 1.0:
                msg = f"PING,{n}\r\n".encode("ascii")
                uart.write(msg)
                print("TX:", msg.decode().strip())
                n += 1
                t0 = time.monotonic()

            # 受信
            line = uart.readline()
            if line:
                print("RX:", line.decode("utf-8", errors="replace").rstrip())

            time.sleep(0.005)

    except KeyboardInterrupt:
        pass
    finally:
        uart.close()
        pi.stop()
        print("DONE")

if __name__ == "__main__":
    main()