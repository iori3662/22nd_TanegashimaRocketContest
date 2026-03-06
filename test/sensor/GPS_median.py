from smbus2 import SMBus
import time
import pynmea2
import statistics

BUS = 1
ADDR = 0x42

SAMPLE_N = 10   # 取得回数

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

lat_list = []
lon_list = []
alt_list = []

print("GPS sampling start")

with SMBus(BUS) as bus:
    while True:

        try:
            data = bus.read_i2c_block_data(ADDR, 0xFF, 32)
        except OSError:
            time.sleep(0.05)
            continue

        buf.extend(data)

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


            # --- RMC ---
            if msg.sentence_type == "RMC" and getattr(msg, "status", "") == "A":

                lat = dm_to_deg(msg.lat, msg.lat_dir)
                lon = dm_to_deg(msg.lon, msg.lon_dir)

                if lat is not None and lon is not None:
                    latest["lat"] = lat
                    latest["lon"] = lon


            # --- GGA ---
            elif msg.sentence_type == "GGA":

                try:
                    latest["fixq"] = int(msg.gps_qual)
                except:
                    latest["fixq"] = 0

                try:
                    latest["nsat"] = int(msg.num_sats)
                except:
                    latest["nsat"] = 0

                try:
                    latest["hdop"] = float(msg.horizontal_dil)
                except:
                    latest["hdop"] = None

                if latest["fixq"] > 0:

                    try:
                        latest["alt"] = float(msg.altitude)
                    except:
                        pass


                    if latest["lat"] is not None and latest["lon"] is not None:

                        lat_list.append(latest["lat"])
                        lon_list.append(latest["lon"])
                        alt_list.append(latest["alt"])

                        print(f"sample {len(lat_list)} / {SAMPLE_N}")

                        if len(lat_list) >= SAMPLE_N:

                            lat_mean = statistics.mean(lat_list)
                            lon_mean = statistics.mean(lon_list)
                            alt_mean = statistics.mean(alt_list)

                            lat_med = statistics.median(lat_list)
                            lon_med = statistics.median(lon_list)
                            alt_med = statistics.median(alt_list)

                            lat_std = statistics.stdev(lat_list) if len(lat_list) > 1 else 0
                            lon_std = statistics.stdev(lon_list) if len(lon_list) > 1 else 0

                            print("----- RESULT -----")
                            print(f"mean   lat={lat_mean:.8f} lon={lon_mean:.8f} alt={alt_mean:.2f}")
                            print(f"median lat={lat_med:.8f} lon={lon_med:.8f} alt={alt_med:.2f}")
                            print(f"std    lat={lat_std:.8f} lon={lon_std:.8f}")

                            lat_list.clear()
                            lon_list.clear()
                            alt_list.clear()

        time.sleep(0.05)