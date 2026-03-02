#!/usr/bin/env python3
"""
TWELITE software-UART (bitbang) test using pigpio.

前提（あなたのPCB配線ミスをソフトで回避する）:
- Pi GPIO14 が TWELITE_TX に接続されている（= TWELITEのTXがPiへ入る線）
- Pi GPIO15 が TWELITE_RX に接続されている（= PiからTWELITEのRXへ入る線）
→ 物理的にはクロスが成立しているので、ソフトUARTでは
   TX_PIN=15（送信） / RX_PIN=14（受信）として扱う。

使い方:
  1) sudo apt install -y pigpio python3-pigpio
  2) sudo systemctl enable --now pigpiod
  3) 可能なら raspi-config で Serial Port を無効化（GPIO14/15を解放）
  4) python3 twelite_softuart.py
"""

import time
import pigpio

# ===== ユーザ設定 =====
TX_PIN = 15      # BCM GPIO15 を「送信」に使う（TWELITE RX に繋がっている想定）
RX_PIN = 14      # BCM GPIO14 を「受信」に使う（TWELITE TX に繋がっている想定）
BAUD   = 38400   # TWELITE の UART baud と一致させる

SEND_PERIOD_SEC = 1.0


class SoftUART:
    def __init__(self, pi: pigpio.pi, tx_pin: int, rx_pin: int, baud: int,
                 rx_pull: int = pigpio.PUD_UP):
        self.pi = pi
        self.tx = tx_pin
        self.rx = rx_pin
        self.baud = baud

        # GPIOモード設定
        self.pi.set_mode(self.tx, pigpio.OUTPUT)
        self.pi.set_mode(self.rx, pigpio.INPUT)

        # RXラインが浮くと誤受信するので、基本はPUD_UP（必要に応じてPUD_OFFへ）
        self.pi.set_pull_up_down(self.rx, rx_pull)

        # 「開いてない close」で例外が出る仕様なので握りつぶす
        try:
            self.pi.bb_serial_read_close(self.rx)
        except pigpio.error:
            pass

        # bitbang 受信開始
        self.pi.bb_serial_read_open(self.rx, self.baud)
        self._rx_buf = bytearray()

    def close(self):
        try:
            self.pi.bb_serial_read_close(self.rx)
        except pigpio.error:
            pass

    def write(self, data: bytes):
        # waveを使ったbitbang送信
        self.pi.wave_clear()
        self.pi.wave_add_serial(self.tx, self.baud, data)
        wid = self.pi.wave_create()
        if wid < 0:
            raise RuntimeError("pigpio wave_create failed")
        self.pi.wave_send_once(wid)
        while self.pi.wave_tx_busy():
            time.sleep(0.001)
        self.pi.wave_delete(wid)

    def poll(self):
        """受信バッファに追加する（短周期で呼ぶ）"""
        count, data = self.pi.bb_serial_read(self.rx)
        if count > 0:
            self._rx_buf.extend(data)

    def readline(self) -> bytes | None:
        """\\n までの1行を返す（無ければ None）"""
        self.poll()
        if b"\n" in self._rx_buf:
            line, _, rest = self._rx_buf.partition(b"\n")
            self._rx_buf = bytearray(rest)
            return line + b"\n"
        return None


def main():
    pi = pigpio.pi()
    if not pi.connected:
        raise SystemExit(
            "pigpiod に接続できません。\n"
            "  sudo systemctl enable --now pigpiod\n"
            "  sudo systemctl status pigpiod\n"
            "を確認してください。"
        )

    uart = SoftUART(pi, TX_PIN, RX_PIN, BAUD)

    print("START soft-UART TWELITE test")
    print(f"TX_PIN(GPIO{TX_PIN})  RX_PIN(GPIO{RX_PIN})  BAUD={BAUD}")
    print("Ctrl+C で終了")

    n = 0
    next_send = time.monotonic() + SEND_PERIOD_SEC

    try:
        while True:
            now = time.monotonic()

            # 送信（1秒ごと）
            if now >= next_send:
                msg = f"PING,{n}\r\n".encode("ascii")
                uart.write(msg)
                print("TX:", msg.decode("ascii").strip())
                n += 1
                next_send += SEND_PERIOD_SEC

            # 受信（あれば表示）
            line = uart.readline()
            if line:
                print("RX:", line.decode("utf-8", errors="replace").rstrip())

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\nSTOP")
    finally:
        uart.close()
        pi.stop()
        print("DONE")


if __name__ == "__main__":
    main()