import serial
import time
import logging
from datetime import datetime
from queue import Queue, Empty
import threading
from pyubx2 import UBXReader, UBXMessage, NMEA_PROTOCOL, UBX_PROTOCOL
from typing import Optional


class RealGPSReader:
    def __init__(self, port='/dev/ttyAMA0', baudrate=9600):
        # Configure logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler('real_gps.log'),
                logging.StreamHandler()
            ]
        )
        self.logger = logging.getLogger(__name__)

        self.port = port
        self.baudrate = baudrate
        self.serial_port = None
        self.ubx_reader = None
        self.gps_queue = Queue()
        self.running = False
        self.reader_thread = None

        # GPS state
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_alt = 0.0
        self.fix_type = 0
        self.num_satellites = 0

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
            self.logger.info(f"Connected to GPS on {self.port}")
            return True
        except serial.SerialException as e:
            self.logger.error(f"Failed to open serial port: {e}")
            return False

    def _parse_nav_pvt(self, msg: UBXMessage) -> Optional[dict]:
        """Parse NAV-PVT message and return position data in simulator format"""
        if msg.identity == "NAV-PVT":
            self.current_lat = msg.lat / 10 ** 7  # Convert to degrees
            self.current_lon = msg.lon / 10 ** 7  # Convert to degrees
            self.current_alt = msg.height / 1000  # Convert to meters
            self.fix_type = msg.fixType
            self.num_satellites = msg.numSV

            # Create position dict matching simulator format
            position = {
                'lat': self.current_lat,
                'lon': self.current_lon,
                'alt': self.current_alt,
                'timestamp': datetime.now(),
                'fix_type': self.fix_type,
                'num_satellites': self.num_satellites
            }

            return position if msg.valid & 0b1 else None
        return None

    def _read_gps_thread(self):
        """Main GPS reading thread"""
        while self.running:
            try:
                if not self.serial_port and not self.connect():
                    time.sleep(1)
                    continue

                (raw_data, parsed_data) = self.ubx_reader.read()

                if parsed_data and isinstance(parsed_data, UBXMessage):
                    if parsed_data.identity == "NAV-PVT":
                        position = self._parse_nav_pvt(parsed_data)
                        if position:
                            self.gps_queue.put(position)
                            self.logger.debug(
                                f"Position updated: Lat={position['lat']:.6f}, "
                                f"Lon={position['lon']:.6f}, Alt={position['alt']:.1f}m"
                            )

                time.sleep(0.1)

            except (serial.SerialException, serial.SerialTimeoutException) as e:
                self.logger.error(f"Serial communication error: {e}")
                self.connect()  # Try to reconnect
            except Exception as e:
                self.logger.error(f"Unexpected error: {e}")
                continue

    def start_reading(self):
        """Start GPS reading in a separate thread"""
        if not self.running:
            self.running = True
            self.reader_thread = threading.Thread(
                target=self._read_gps_thread,
                daemon=True
            )
            self.reader_thread.start()
            self.logger.info("Started GPS reading")

    def stop_reading(self):
        """Stop GPS reading"""
        self.running = False
        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=2)
        self.logger.info("Stopped GPS reading")

    def get_position(self):
        """Get the latest position from the queue (matches simulator interface)"""
        try:
            return self.gps_queue.get_nowait()
        except Empty:
            return None

    def cleanup(self):
        """Clean up resources"""
        self.stop_reading()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.logger.info("GPS connection closed")
        while not self.gps_queue.empty():
            self.gps_queue.get()


if __name__ == "__main__":
    # Example usage
    gps = RealGPSReader()

    try:
        gps.start_reading()

        # Monitor positions for 30 seconds
        start_time = time.time()
        while time.time() - start_time < 30:
            position = gps.get_position()
            if position:
                print(f"Position: Lat {position['lat']:.6f}, "
                      f"Lon {position['lon']:.6f}, Alt {position['alt']:.1f}m")
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nStopping GPS reading...")
    finally:
        gps.cleanup()