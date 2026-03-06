import time
import pigpio

TX_PIN = 15
RX_PIN = 14
BAUD   = 38400

class SoftUART:
    def __init__(self, pi, tx, rx, baud):
        self.pi = pi
        self.tx = tx
        self.rx = rx
        self.baud = baud
        self.pi.set_mode(self.tx, pigpio.OUTPUT)
        self.pi.write(self.tx, 1)  # UARTアイドルはHigh（重要）
        self.pi.set_mode(self.rx, pigpio.INPUT)
        self.pi.set_pull_up_down(self.rx, pigpio.PUD_OFF)

        try:
            self.pi.bb_serial_read_close(self.rx)
        except pigpio.error:
            pass
        self.pi.bb_serial_read_open(self.rx, self.baud)
        self.buf = bytearray()

    def close(self):
        try:
            self.pi.bb_serial_read_close(self.rx)
        except pigpio.error:
            pass

    def write(self, b: bytes):
        self.pi.wave_clear()
        self.pi.wave_add_serial(self.tx, self.baud, b)
        wid = self.pi.wave_create()
        if wid < 0:
            raise RuntimeError("wave_create failed")
        self.pi.wave_send_once(wid)
        while self.pi.wave_tx_busy():
            time.sleep(0.001)
        self.pi.wave_delete(wid)

    def readline(self):
        c, d = self.pi.bb_serial_read(self.rx)
        if c > 0:
            self.buf.extend(d)
        if b"\n" in self.buf:
            line, _, rest = self.buf.partition(b"\n")
            self.buf = bytearray(rest)
            return line + b"\n"
        return None

pi = pigpio.pi()
if not pi.connected:
    raise SystemExit("pigpiodに接続できません。sudo systemctl status pigpiod を確認")

u = SoftUART(pi, TX_PIN, RX_PIN, BAUD)

print("loopback test: GPIO15 <-> GPIO14 をジャンパで接続してください")
n = 0
t0 = time.monotonic()

try:
    while True:
        if time.monotonic() - t0 > 1.0:
            msg = f"PING,{n}\r\n".encode("ascii")
            u.write(msg)
            print("TX:", msg.decode().strip())
            n += 1
            t0 = time.monotonic()

        r = u.readline()
        if r:
            print("RX:", r.decode(errors="replace").rstrip())

        time.sleep(0.005)

except KeyboardInterrupt:
    pass
finally:
    u.close()
    pi.stop()
    print("done")