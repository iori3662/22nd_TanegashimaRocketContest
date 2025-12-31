from smbus2 import SMBus
import time
import pynmea2

BUS = 1
ADDR = 0x42

def dm_to_deg(dm, direction):
    if dm is None or direction not in ("N", "S", "E", "W"):
        return None
    try:
        dm = float(dm)  
    except ValueError:
        return None
    deg = int(dm // 100)
    minutes = dm - deg * 100
    val = deg + minutes / 60.0
    return -val if direction in ("S", "W") else val

buf = bytearray()

latest = {
    "lat": None, "lon": None, "alt": None,
    "fixq": 0, "nsat": 0, "hdop": None
}

next_print = time.monotonic() 

print("I2C(NMEA) start. Output: 1 Hz (latest valid fix). Outdoor recommended.")

with SMBus(BUS) as bus:
    while True:
        # --- I2C read (robust) ---
        try:
            data = bus.read_i2c_block_data(ADDR, 0xFF, 32)
        except OSError:
            # Errno 121
            time.sleep(0.05)
            continue

        buf.extend(data)

        # --- parse complete NMEA lines ---
        while b'\n' in buf:
            line, _, rest = buf.partition(b'\n')
            buf = bytearray(rest)

            s = line.decode("ascii", errors="ignore").strip()
            if not s.startswith("$"):
                continue

            try:
                msg = pynmea2.parse(s)
            except pynmea2.ParseError:
                continue

            if msg.sentence_type == "RMC" and getattr(msg, "status", "") == "A":
                lat = dm_to_deg(getattr(msg, "lat", None), getattr(msg, "lat_dir", None))
                lon = dm_to_deg(getattr(msg, "lon", None), getattr(msg, "lon_dir", None))
                if lat is not None and lon is not None:
                    latest["lat"] = lat
                    latest["lon"] = lon

            elif msg.sentence_type == "GGA":
                try:
                    fixq = int(getattr(msg, "gps_qual", 0) or 0)
                except ValueError:
                    fixq = 0
                latest["fixq"] = fixq
                try:
                    latest["nsat"] = int(getattr(msg, "num_sats", 0) or 0)
                except ValueError:
                    latest["nsat"] = 0
                try:
                    latest["hdop"] = float(getattr(msg, "horizontal_dil", 0) or 0)
                except ValueError:
                    latest["hdop"] = None

                if fixq > 0:
                    try:
                        latest["alt"] = float(getattr(msg, "altitude", 0) or 0)
                    except ValueError:
                        pass

        # --- 1 Hz output (exactly once per second) ---
        now = time.monotonic()
        if now >= next_print:
            next_print += 1.0  

            if latest["lat"] is not None and latest["lon"] is not None and latest["fixq"] > 0:
                print(
                    f"lat={latest['lat']:.8f}, lon={latest['lon']:.8f}, alt={latest['alt']:.1f} m, "
                    f"fixq={latest['fixq']}, nsat={latest['nsat']}, hdop={latest['hdop']}"
                )
            else:
                print("no valid fix yet (waiting...)")

        time.sleep(0.05)
