import serial
import time
import logging
from datetime import datetime
from pyubx2 import UBXReader
import pynmea2
from typing import Optional

class GPSReader:
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600):
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s'
        )
        self.port = port
        self.baudrate = baudrate
        self.serial_port = None
        # Current position data
        self.current_position = {
            'latitude': None,
            'longitude': None,
            'altitude': None,
            'timestamp': None
        }

    def connect(self) -> bool:
        """Establish connection to the GPS module"""
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1
            )
            logging.info(f"Connected to GPS on {self.port}")
            return True
        except serial.SerialException as e:
            logging.error(f"Failed to open serial port: {e}")
            return False

    def verify_checksum(self, sentence: str) -> bool:
        """Verify NMEA sentence checksum"""
        try:
            if '*' not in sentence:
                return False
            data, checksum = sentence.split('*')
            if not data.startswith('$'):
                return False

            calculated_checksum = 0
            for char in data[1:]:
                calculated_checksum ^= ord(char)

            return format(calculated_checksum, '02X') == checksum.strip()[:2]
        except Exception:
            return False

    def get_current_position(self):
        """Return the current position"""
        return self.current_position

    def read_gps(self):
        """Main GPS reading loop"""
        if not self.serial_port and not self.connect():
            return

        print("\nStarting GPS reading... Press Ctrl+C to stop")
        print("Note: First fix may take a few minutes.")
        print("Make sure the GPS module has clear view of the sky.\n")

        try:
            while True:
                try:
                    line = self.serial_port.readline().decode('ascii', errors='ignore')

                    if not line or not line.startswith('$'):
                        continue

                    if not self.verify_checksum(line):
                        continue

                    msg = pynmea2.parse(line)

                    if msg.sentence_type == 'GGA':
                        try:
                            # Update current position directly
                            self.current_position = {
                                'latitude': float(msg.latitude) if msg.latitude else None,
                                'longitude': float(msg.longitude) if msg.longitude else None,
                                'altitude': float(msg.altitude) if msg.altitude else None,
                                'timestamp': datetime.now()
                            }
                            self._display_position(self.current_position)
                        except (ValueError, AttributeError) as e:
                            logging.warning(f"Error parsing GGA data: {e}")
                            continue

                    time.sleep(0.1)

                except pynmea2.ParseError:
                    continue
                except serial.SerialException as e:
                    logging.error(f"Serial communication error: {e}")
                    break
                except Exception as e:
                    logging.error(f"Unexpected error: {e}", exc_info=True)
                    continue

        except KeyboardInterrupt:
            print("\nStopping GPS reading...")
        finally:
            self.cleanup()

    def _display_position(self, position: dict):
        """Display position information"""
        if position['latitude'] is not None and position['longitude'] is not None:
            print(f"\033[2J\033[H")  # Clear screen and move cursor to top
            print(f"GPS Status - {position['timestamp'].strftime('%Y-%m-%d %H:%M:%S')}")
            print("-" * 50)
            print(f"Position:")
            print(f"Latitude:  {position['latitude']:.6f}°")
            print(f"Longitude: {position['longitude']:.6f}°")
            if position['altitude']:
                print(f"Altitude:  {position['altitude']:.1f} m")
            print("-" * 50)

    def cleanup(self):
        """Clean up resources"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            logging.info("GPS connection closed")