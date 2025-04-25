import serial
import pynmea2
import time
import logging

# Set logging to only show INFO and above (hiding DEBUG messages)
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)


def convert_to_degrees(nmea_data):
    if not nmea_data:
        return None

    try:
        parts = str(nmea_data).split('.')
        if len(parts) != 2:
            return None

        degrees = float(parts[0][:-2])
        minutes = float(parts[0][-2:] + '.' + parts[1])
        decimal_degrees = degrees + (minutes / 60.0)
        return decimal_degrees
    except (ValueError, IndexError):
        return None


def parse_gps():
    try:
        serial_port = serial.Serial(
            port='/dev/ttyUSB0',
            baudrate=9600,
            timeout=1
        )
    except serial.SerialException as e:
        logging.error(f"Failed to open serial port: {e}")
        return

    print("Waiting for GPS fix...", flush=True)

    try:
        while True:
            try:
                line = serial_port.readline()
                if not line:
                    continue

                line = line.decode('ascii', errors='replace')

                if line.startswith('$GNGGA') or line.startswith('$GNRMC'):
                    msg = pynmea2.parse(line)

                    if msg.sentence_type == 'GGA' and msg.gps_qual > 0:
                        if hasattr(msg, 'latitude') and hasattr(msg, 'longitude'):
                            if msg.latitude and msg.longitude:
                                print(f"Position (GGA):")
                                print(f"Lat: {msg.latitude:.6f}° {msg.lat_dir}")
                                print(f"Lon: {msg.longitude:.6f}° {msg.lon_dir}")
                                print("-" * 20)

                    elif msg.sentence_type == 'RMC' and msg.status == 'A':
                        if hasattr(msg, 'latitude') and hasattr(msg, 'longitude'):
                            if msg.latitude and msg.longitude:
                                print(f"Position (RMC):")
                                print(f"Lat: {msg.latitude:.6f}° {msg.lat_dir}")
                                print(f"Lon: {msg.longitude:.6f}° {msg.lon_dir}")
                                print("-" * 20)

                time.sleep(0.1)

            except (UnicodeDecodeError, pynmea2.ParseError):
                continue
            except serial.SerialException as e:
                logging.error(f"Serial error: {e}")
                break

    except KeyboardInterrupt:
        print("\nStopping GPS reading...")
    finally:
        serial_port.close()


if __name__ == "__main__":
    print("Starting GPS reading... Press Ctrl+C to stop")
    print("Note: First fix may take a few minutes.")
    print("Make sure the GPS module has clear view of the sky.")
    parse_gps()