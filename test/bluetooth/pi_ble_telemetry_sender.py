import asyncio
import math
import time
from bleak import BleakClient

ADDR = "58:8C:81:AE:A8:1A"
LOG_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

def make_line(seq: int) -> bytes:
    t = time.monotonic()
    ph = int((t // 5) % 7)

    vbat_v = 7.40 - 0.12 * (0.5 + 0.5 * math.sin(0.15 * t))
    vbat_mV = int(vbat_v * 1000)

    yaw = 30.0 * math.sin(0.7 * t)
    pitch = 5.0 * math.sin(0.5 * t)
    roll = 7.0 * math.cos(0.4 * t)

    lat = 35.658600 + 0.000050 * math.sin(0.20 * t)
    lon = 139.745400 + 0.000050 * math.cos(0.20 * t)
    alt = 20.0 + 0.5 * math.sin(0.10 * t)

    pwml = 1480 + int(120 * math.sin(0.9 * t))
    pwmr = 1490 + int(120 * math.cos(0.9 * t))

    line = (
        f"SEQ={seq},PH={ph},VBAT={vbat_mV},"
        f"Y={yaw:.1f},P={pitch:.1f},R={roll:.1f},"
        f"LAT={lat:.6f},LON={lon:.6f},ALT={alt:.1f},"
        f"PWML={pwml},PWMR={pwmr}\n"
    )
    return line.encode("ascii")

async def run_once():
    client = BleakClient(ADDR)
    await client.connect()
    print("Connected:", client.is_connected)

    seq = 0
    try:
        while True:
            payload = make_line(seq)
            await client.write_gatt_char(LOG_UUID, payload, response=True)
            if seq % 10 == 0:
                print("sent seq", seq)
            seq += 1
            await asyncio.sleep(0.1)  # 10Hz
    finally:
        try:
            if client.is_connected:
                await client.disconnect()
        except Exception as e:
            print("disconnect error (ignored):", repr(e))
        print("Disconnected.")

async def main():
    while True:
        try:
            await run_once()
        except KeyboardInterrupt:
            print("\nStopped by user.")
            return
        except Exception as e:
            print("error (retry):", repr(e))
            await asyncio.sleep(1.0)

if __name__ == "__main__":
    asyncio.run(main())
