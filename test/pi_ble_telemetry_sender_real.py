from smbus2 import SMBus
import time
import pynmea2

import board
import busio
import adafruit_bno055
from adafruit_bme280 import basic as adafruit_bme280


# =======================
# Config
# =======================
BUS = 1
GPS_ADDR = 0x42         # SAM-M8Q I2C address
BNO_ADDR = 0x28
BME_ADDR = 0x76

IMU_HZ = 10.0           # BNO055/BME280 print rate
GPS_HZ = 1.0            # GPS print rate
GPS_READ_LEN = 32       # bytes per i2c read
GPS_SLEEP = 0.02        # loop sleep (s)


# =======================
# Helpers
# =======================
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


def parse_nmea_from_buffer(buf, latest):
    """
    buf: bytearray (will be mutated by caller)
    latest: dict updated in-place
    returns: new buf (bytearray)
    """
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

    return buf


# =======================
# Main
# =======================
def main():
    # --- Sensors (CircuitPython stack) ---
    i2c = busio.I2C(board.SCL, board.SDA)
    bno055 = adafruit_bno055.BNO055_I2C(i2c, address=BNO_ADDR)
    bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=BME_ADDR)
    bme280.sea_level_pressure = 1013.25

    # --- GPS state ---
    buf = bytearray()
    latest = {"lat": None, "lon": None, "alt": None, "fixq": 0, "nsat": 0, "hdop": None}

    next_gps_print = time.monotonic()
    next_imu_print = time.monotonic()

    print("Unified I2C start:")
    print(f"  GPS(SAM-M8Q) I2C=0x{GPS_ADDR:02X} @ {GPS_HZ:.1f} Hz (latest fix)")
    print(f"  BNO055/BME280 @ {IMU_HZ:.1f} Hz")
    print("Note: Two I2C stacks share the same bus (works often, but may cause contention).")

    with SMBus(BUS) as bus:
        while True:
            now = time.monotonic()

            # -------- GPS read/parse (continuous, robust) --------
            try:
                data = bus.read_i2c_block_data(GPS_ADDR, 0xFF, GPS_READ_LEN)
                buf.extend(data)
                buf = parse_nmea_from_buffer(buf, latest)
            except OSError:
                # e.g., Errno 121: remote I/O error
                pass

            # -------- IMU/BME print (10 Hz) --------
            if now >= next_imu_print:
                next_imu_print += 1.0 / IMU_HZ

                euler = bno055.euler
                gyro = bno055.gyro
                accel = bno055.acceleration
                mag = bno055.magnetic
                calib = bno055.calibration_status

                print("\n[IMU+BME]")
                if euler is not None:
                    print(f"  Euler(deg) H={euler[0]:6.2f} R={euler[1]:6.2f} P={euler[2]:6.2f}")
                else:
                    print("  Euler(deg) not available yet")

                print(f"  Gyro(rad/s)  {gyro}")
                print(f"  Accel(m/s^2) {accel}")
                print(f"  Mag(uT)      {mag}")
                print(f"  Calib(sys,g,a,m) {calib}")

                print(f"  Temp(C)      {bme280.temperature:.2f}")
                print(f"  Press(hPa)   {bme280.pressure:.2f}")
                print(f"  Humidity(%)  {bme280.humidity:.2f}")
                print(f"  Alt(m)       {bme280.altitude:.2f}")

            # -------- GPS print (1 Hz) --------
            if now >= next_gps_print:
                next_gps_print += 1.0 / GPS_HZ
                print("\n[GPS]")
                if latest["lat"] is not None and latest["lon"] is not None and latest["fixq"] > 0:
                    alt = latest["alt"]
                    alt_str = f"{alt:.1f} m" if alt is not None else "None"
                    print(
                        f"  lat={latest['lat']:.8f}, lon={latest['lon']:.8f}, alt={alt_str}, "
                        f"fixq={latest['fixq']}, nsat={latest['nsat']}, hdop={latest['hdop']}"
                    )
                else:
                    print("  no valid fix yet (waiting...)")

            time.sleep(GPS_SLEEP)


if __name__ == "__main__":
    main()
