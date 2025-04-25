import serial
import time
import logging
import threading
from datetime import datetime
from queue import Queue
from pyubx2 import UBXReader, UBXMessage, NMEA_PROTOCOL, UBX_PROTOCOL
from typing import Optional
import math
from pymavlink import mavutil
from flask import Flask, jsonify, request
from flask_socketio import SocketIO

# Initialize Flask and SocketIO
app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

# Global variables
gps_data_queue = Queue()
follow_enabled = False
follow_thread = None
gps_thread = None
target_altitude = None
follow_lock = threading.Lock()
thread_lock = threading.Lock()
vehicle_lock = threading.Lock()
vehicle = None

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('gps_follow.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


class GPSReader:
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600):
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
            self.ubx_reader = UBXReader(
                self.serial_port,
                protfilter=(NMEA_PROTOCOL | UBX_PROTOCOL)
            )
            logger.info(f"Connected to GPS on {self.port}")
            return True
        except serial.SerialException as e:
            logger.error(f"Failed to open serial port: {e}")
            return False

    def _parse_nav_pvt(self, msg: UBXMessage) -> Optional[dict]:
        """Parse NAV-PVT message and put it in the queue"""
        if msg.identity == "NAV-PVT":
            if bool(msg.valid & 0b1):  # Check if position is valid
                position = {
                    'lat': msg.lat / 10 ** 7,
                    'lon': msg.lon / 10 ** 7,
                    'alt': msg.height / 1000,
                    'timestamp': datetime.now(),
                    'fix_type': msg.fixType,
                    'num_satellites': msg.numSV
                }
                self.last_valid_position = position
                self.fix_type = msg.fixType
                self.satellites_in_view = msg.numSV

                # Put the position data in the queue for the follow system
                gps_data_queue.put({
                    'lat': position['lat'],
                    'lon': position['lon'],
                    'alt': position['alt']
                })

                return position
        return None

    def read_gps_data(self):
        """Continuous GPS reading method for threading"""
        if not self.serial_port and not self.connect():
            return

        logger.info("GPS reader thread started")

        try:
            while True:
                try:
                    (raw_data, parsed_data) = self.ubx_reader.read()

                    if isinstance(parsed_data, UBXMessage):
                        if parsed_data.identity == "NAV-PVT":
                            self._parse_nav_pvt(parsed_data)

                    time.sleep(0.01)

                except (serial.SerialException, serial.SerialTimeoutException) as e:
                    logger.error(f"Serial error: {e}")
                    break
                except Exception as e:
                    logger.error(f"Unexpected error: {e}")
                    continue

        except KeyboardInterrupt:
            logger.info("Stopping GPS reading...")
        finally:
            self.cleanup()

    def cleanup(self):
        """Clean up resources"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            logger.info("GPS connection closed")


def connect_to_vehicle(connection_string='udp:127.0.0.1:14551'):
    """Connect to the vehicle"""
    global vehicle
    try:
        vehicle = mavutil.mavlink_connection(connection_string)
        vehicle.wait_heartbeat()
        logger.info(f"Connected to vehicle on {connection_string}")
        return True
    except Exception as e:
        logger.error(f"Failed to connect to vehicle: {e}")
        return False


def calculate_distance(lat1, lon1, lat2, lon2):
    """Calculate distance between two GPS coordinates"""
    R = 6371000  # Earth's radius in meters

    # Convert decimal degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.asin(math.sqrt(a))
    distance = R * c

    return distance


def start_gps_reader():
    """Start GPS reader thread using the GPSReader class"""
    gps_reader = GPSReader()

    try:
        new_thread = threading.Thread(target=gps_reader.read_gps_data)
        new_thread.daemon = True
        new_thread.start()
        return new_thread
    except Exception as e:
        logger.error(f"Failed to start GPS reader thread: {e}")
        return None


def follow_gps():
    """Follow the GPS coordinates from the queue"""
    global follow_enabled, target_altitude, vehicle

    if target_altitude is None:
        logger.error("Target altitude not set")
        return

    logger.info("GPS follow thread started")
    MINIMUM_DISTANCE = 5  # Minimum distance in meters before stopping

    while follow_enabled:
        try:
            if not gps_data_queue.empty():
                gps_data = gps_data_queue.get()

                with vehicle_lock:
                    if vehicle:
                        # Get current position from MAVLink messages
                        current_pos = None
                        msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
                        if msg:
                            current_pos = {
                                'lat': msg.lat / 1e7,
                                'lon': msg.lon / 1e7,
                                'alt': msg.relative_alt / 1000
                            }

                            # Calculate distance to target
                            distance = calculate_distance(
                                current_pos['lat'], current_pos['lon'],
                                gps_data['lat'], gps_data['lon']
                            )

                            logger.info(f"Distance to target: {distance:.2f} meters")

                            # Check if within minimum distance
                            if distance <= MINIMUM_DISTANCE:
                                logger.info(
                                    f"Within minimum distance ({MINIMUM_DISTANCE}m). Setting follow_enabled to False")
                                follow_enabled = False
                                continue

                        # Send position target command
                        vehicle.mav.set_position_target_global_int_send(
                            0,  # timestamp
                            vehicle.target_system,
                            vehicle.target_component,
                            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                            0b110111111000,  # Only positions enabled
                            int(gps_data['lat'] * 1e7),
                            int(gps_data['lon'] * 1e7),
                            target_altitude,
                            0, 0, 0,  # velocity
                            0, 0, 0,  # acceleration
                            0, 0  # yaw, yaw_rate
                        )

                        logger.info(
                            f"Following GPS position: Lat {gps_data['lat']}, Lon {gps_data['lon']}, Alt {target_altitude}")

                        # Emit position update to clients
                        emit_data = {
                            'type': 'GPS_FOLLOW',
                            'data': {
                                'lat': gps_data['lat'],
                                'lon': gps_data['lon'],
                                'alt': target_altitude
                            }
                        }
                        if current_pos:
                            emit_data['data']['distance'] = distance
                        socketio.emit('message', emit_data)

        except Exception as e:
            logger.error(f"Error in GPS follow thread: {e}")
            logger.exception(e)

        time.sleep(0.1)


def start_gps_follow(follow_altitude):
    """Start GPS following at specified altitude"""
    global follow_enabled, follow_thread, target_altitude, gps_thread

    try:
        follow_altitude = float(follow_altitude)
        if follow_altitude <= 0:
            logger.error("Invalid altitude: must be greater than 0")
            return False
    except ValueError:
        logger.error("Invalid altitude value provided")
        return False

    with thread_lock:
        if not follow_enabled:
            if gps_thread is None or not gps_thread.is_alive():
                gps_thread = start_gps_reader()
                if gps_thread is None:
                    logger.error("Failed to start GPS reader thread")
                    return False
                time.sleep(0.5)  # Give GPS thread time to initialize

            follow_enabled = True
            target_altitude = follow_altitude

            follow_thread = threading.Thread(target=follow_gps)
            follow_thread.daemon = True
            follow_thread.start()

            logger.info(f"GPS following started at altitude {target_altitude}m")
            return True
        return False


def stop_gps_follow():
    """Stop GPS following"""
    global follow_enabled, follow_thread, gps_thread

    with thread_lock:
        follow_enabled = False

        current_thread = threading.current_thread()
        if follow_thread and follow_thread.is_alive() and current_thread != follow_thread:
            follow_thread.join(timeout=2)
        follow_thread = None

        if gps_thread and gps_thread.is_alive():
            gps_thread = None

        logger.info("GPS following stopped")
        return True


@app.route('/start_gps_follow', methods=['POST'])
def handle_start_gps_follow():
    """Handle start GPS following request"""
    try:
        data = request.json
        follow_altitude = float(data.get('altitude', 10))
        if start_gps_follow(follow_altitude):
            return jsonify({'message': f'GPS following started at {follow_altitude}m altitude'})
        return jsonify({'error': 'Failed to start GPS following'}), 400
    except Exception as e:
        logger.error(f"Error starting GPS follow: {e}")
        return jsonify({'error': str(e)}), 500


@app.route('/stop_gps_follow')
def handle_stop_gps_follow():
    """Handle stop GPS following request"""
    try:
        if stop_gps_follow():
            return jsonify({'message': 'GPS following stopped'})
        return jsonify({'error': 'Failed to stop GPS following'}), 400
    except Exception as e:
        logger.error(f"Error stopping GPS follow: {e}")
        return jsonify({'error': str(e)}), 500


@app.route('/status')
def get_status():
    """Get current GPS follow status"""
    return jsonify({
        'following': follow_enabled,
        'target_altitude': target_altitude,
        'gps_active': gps_thread is not None and gps_thread.is_alive(),
        'follow_active': follow_thread is not None and follow_thread.is_alive()
    })


def main():
    # Connect to vehicle (using default SITL connection string)
    if not connect_to_vehicle():
        logger.error("Failed to connect to vehicle. Exiting...")
        return

    # Start Flask app
    socketio.run(app, host='0.0.0.0', port=5001, debug=True)


if __name__ == "__main__":
    main()