import asyncio
import math
import struct
import time
from bleak import BleakClient

ADDR = "58:8C:81:AE:A8:1A"   

LOG_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

def make_packet(seq: int) -> bytes:
    phase = int((time.monotonic() // 5) % 7)

    t = time.monotonic()
    vbat_v = 7.40 - 0.12 * (0.5 + 0.5 * math.sin(0.15 * t))
    vbat_mV = int(vbat_v * 1000)

    header = struct.pack("<I B H", seq, phase, vbat_mV)
    payload = b"HELLO"
    return header + payload

async def main():
    async with BleakClient(ADDR) as client:
        print("Connected:", client.is_connected)

        seq = 0
        while True:
            pkt = make_packet(seq)
            await client.write_gatt_char(LOG_UUID, pkt, response=False)
            seq += 1
            await asyncio.sleep(0.1)

if __name__ == "__main__":
    asyncio.run(main())
