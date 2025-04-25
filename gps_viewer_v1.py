import serial
import time
import logging
from datetime import datetime
from pyubx2 import UBXReader, UBXMessage, NMEA_PROTOCOL, UBX_PROTOCOL
from typing import Optional


class GPSReader:
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600):
        # Configure logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s'
        )
        self.port = port
        self.baudrate = baudrate
        self.serial_port = None
        self.ubx_reader = None
        self.last_valid_position = None
        self.satellites_in_view = 0
        self.hdop = 0.0
        self.fix_type = 0

    def connect(self) -> bool:
        """Establish connection to the GPS module"""
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1
            )
            # Initialize UBX reader with both NMEA and UBX protocols
            self.ubx_reader = UBXReader(
                self.serial_port,
                protfilter=(NMEA_PROTOCOL | UBX_PROTOCOL)
            )
            logging.info(f"Connected to GPS on {self.port}")
            return True
        except serial.SerialException as e:
            logging.error(f"Failed to open serial port: {e}")
            return False

    def _parse_nav_pvt(self, msg: UBXMessage) -> Optional[dict]:
        """Parse NAV-PVT message"""
        if msg.identity == "NAV-PVT":
            position = {
                'latitude': msg.lat / 10 ** 7,  # Convert to degrees
                'longitude': msg.lon / 10 ** 7,  # Convert to degrees
                'altitude': msg.height / 1000,  # Convert to meters
                'timestamp': datetime.now(),
                'fix_type': msg.fixType,
                'ground_speed': msg.gSpeed / 1000,  # Convert to m/s
                'heading': msg.headMot / 100000,  # Convert to degrees
                'num_satellites': msg.numSV,
                'pdop': msg.pDOP / 100,  # Convert to normal units
                'valid': bool(msg.valid & 0b1)  # Check valid flag
            }
            self.last_valid_position = position
            self.fix_type = msg.fixType
            self.satellites_in_view = msg.numSV
            return position
        return None

    def _parse_nav_sat(self, msg: UBXMessage) -> None:
        """Parse NAV-SAT message for satellite information"""
        if msg.identity == "NAV-SAT":
            self.satellites_in_view = msg.numSV

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
                    (raw_data, parsed_data) = self.ubx_reader.read()

                    if parsed_data:
                        if isinstance(parsed_data, UBXMessage):
                            # Handle UBX messages
                            if parsed_data.identity == "NAV-PVT":
                                position = self._parse_nav_pvt(parsed_data)
                                if position and position['valid']:
                                    self._display_position(position)
                            elif parsed_data.identity == "NAV-SAT":
                                self._parse_nav_sat(parsed_data)
                            elif parsed_data.identity == "NAV-DOP":
                                self.hdop = parsed_data.hDOP / 100

                    time.sleep(0.1)

                except (serial.SerialException, serial.SerialTimeoutException) as e:
                    logging.error(f"Serial communication error: {e}")
                    break
                except Exception as e:
                    logging.error(f"Unexpected error: {e}")
                    continue

        except KeyboardInterrupt:
            print("\nStopping GPS reading...")
        finally:
            self.cleanup()

    def _display_position(self, position: dict):
        """Display position information"""
        print(f"\033[2J\033[H")  # Clear screen and move cursor to top
        print(f"GPS Status - {position['timestamp'].strftime('%Y-%m-%d %H:%M:%S')}")
        print("-" * 50)
        print(f"Position (UBX NAV-PVT):")
        print(f"Latitude:  {position['latitude']:.6f}°")
        print(f"Longitude: {position['longitude']:.6f}°")
        print(f"Altitude:  {position['altitude']:.1f} m")
        print("-" * 50)
        print(f"Fix Type: {self._get_fix_type_string(position['fix_type'])}")
        print(f"Satellites in view: {position['num_satellites']}")
        print(f"Ground Speed: {position['ground_speed'] * 3.6:.1f} km/h")  # Convert to km/h
        print(f"Heading: {position['heading']:.1f}°")
        print(f"PDOP: {position['pdop']:.1f}")
        print("-" * 50)

    def _get_fix_type_string(self, fix_type: int) -> str:
        """Convert fix type number to descriptive string"""
        fix_types = {
            0: "No Fix",
            1: "Dead Reckoning",
            2: "2D Fix",
            3: "3D Fix",
            4: "GNSS + Dead Reckoning",
            5: "Time Only Fix"
        }
        return fix_types.get(fix_type, "Unknown")

    def cleanup(self):
        """Clean up resources"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            logging.info("GPS connection closed")


if __name__ == "__main__":
    gps = GPSReader()
    gps.read_gps()