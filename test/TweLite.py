#!/usr/bin/env python3
import sys, time, select
import pigpio

# ========= 設定 =========
BAUD = 38400
TWE_RX_GPIO = 14  # Twelite TXが繋がっているピン（Pi本来TXだが、ここをRXとして使う）
TWE_TX_GPIO = 15  # Twelite RXが繋がっているピン（Pi本来RXだが、ここをTXとして使う）
DATA_BITS = 8
STOP_BITS = 1

READ_POLL_DT = 0.01
RX_BUF_MAX = 2048

class SoftUartTwelite:
    def __init__(self, pi: pigpio.pi, tx_gpio: int, rx_gpio: int, baud: int):
        self.pi = pi
        self.tx = tx_gpio
        self.rx = rx_gpio
        self.baud = baud
        self._rx_buf = b""

        # TX idle high
        self.pi.set_mode(self.tx, pigpio.OUTPUT)
        self.pi.write(self.tx, 1)

        # RX open (bit-bang read)
        self.pi.set_mode(self.rx, pigpio.INPUT)
        self.pi.bb_serial_read_open(self.rx, self.baud, DATA_BITS)
        time.sleep(0.05)

    def close(self):
        try:
            self.pi.bb_serial_read_close(self.rx)
        except pigpio.error:
            pass

    def send_line(self, s: str):
        # Arduinoの println 相当（CRLFで送るのが無難）
        b = (s.strip() + "\r\n").encode("ascii", errors="ignore")

        # DMA wave serial send
        self.pi.wave_clear()
        self.pi.wave_add_serial(self.tx, self.baud, DATA_BITS, STOP_BITS, 0, b)
        wid = self.pi.wave_create()
        if wid < 0:
            raise RuntimeError("wave_create failed")

        self.pi.wave_send_once(wid)
        while self.pi.wave_tx_busy():
            time.sleep(0.001)

        self.pi.wave_delete(wid)

    def read_lines(self):
        """受信バッファから \n 区切りの行を取り出す（複数行返ることあり）"""
        cnt, data = self.pi.bb_serial_read(self.rx)
        if cnt > 0 and data:
            self._rx_buf += data
            if len(self._rx_buf) > RX_BUF_MAX:
                self._rx_buf = self._rx_buf[-RX_BUF_MAX:]

        lines = []
        while b"\n" in self._rx_buf:
            line, self._rx_buf = self._rx_buf.split(b"\n", 1)
            # CR除去して文字列へ
            line = line.rstrip(b"\r")
            try:
                lines.append(line.decode("ascii", errors="replace").strip())
            except Exception:
                lines.append("")
        return lines

def stdin_readline_nonblock():
    """stdinに行が来ていれば1行読む。無ければNone"""
    r, _, _ = select.select([sys.stdin], [], [], 0)
    if r:
        return sys.stdin.readline()
    return None

def main():
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpiodに接続できません。`sudo systemctl start pigpiod` を確認してください。")

    tw = SoftUartTwelite(pi, tx_gpio=TWE_TX_GPIO, rx_gpio=TWE_RX_GPIO, baud=BAUD)

    try:
        sys.stdout.write("Bridge start: STDIN/STDOUT <-> TWELITE (soft UART)\n")
        sys.stdout.write("Type a line and press Enter to send to TWELITE.\n")
        sys.stdout.flush()

        while True:
            # PC(ターミナル) -> TWELITE
            s = stdin_readline_nonblock()
            if s is not None:
                s = s.strip()
                if s:
                    tw.send_line(s)

            # TWELITE -> PC(ターミナル)
            for line in tw.read_lines():
                if line:
                    sys.stdout.write(line + "\n")
                    sys.stdout.flush()

            time.sleep(READ_POLL_DT)

    finally:
        tw.close()
        pi.stop()

if __name__ == "__main__":
    main()