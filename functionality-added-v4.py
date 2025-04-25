import os
from pymavlink import mavutil, mavwp
from flask import Flask, render_template, jsonify, request, Response
from flask_socketio import SocketIO
import threading
import json
import logging
import sys
import errno
import random
import cv2
import serial.tools.list_ports
import time
import subprocess
import signal
import uuid
from dataclasses import dataclass
from typing import List, Optional
from enum import Enum

mavproxy_process = None

app = Flask(__name__)
app.config['JSON_AS_ASCII'] = False
app.config['JSONIFY_PRETTYPRINT_REGULAR'] = True
socketio = SocketIO(app, async_mode='threading')

connection_string = 'udpin:127.0.0.1:14550'
RTSP_URL = "rtsp:192.168.144.25:8554/main.264"
vehicle = None

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

console_handler = logging.StreamHandler(sys.stdout)
console_handler.setLevel(logging.DEBUG)
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
console_handler.setFormatter(formatter)
logger.addHandler(console_handler)

connection_attempts = 0
max_connection_attempts = 500

wp_loader = None
current_mission = []
last_mission = None

drone_parameters = {}


class WaypointType(Enum):
    WAYPOINT = "WAYPOINT"
    TAKEOFF = "TAKEOFF"
    LAND = "LAND"
    RTL = "RTL"


@dataclass
class Waypoint:
    type: WaypointType
    lat: float
    lon: float
    alt: float
    param1: float = 0
    param2: float = 0
    param3: float = 0
    param4: float = 0


@dataclass
class Mission:
    id: str
    name: str
    waypoints: List[Waypoint]
    created_at: float
    updated_at: float


class MissionPlanner:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.missions = {}

    def create_mission(self, name: str, waypoints: List[dict]) -> Mission:
        mission_id = str(uuid.uuid4())
        now = time.time()
        mission = Mission(
            id=mission_id,
            name=name,
            waypoints=[Waypoint(
                type=WaypointType(wp['type']),
                lat=wp['lat'],
                lon=wp['lon'],
                alt=wp['alt'],
                param1=wp.get('param1', 0),
                param2=wp.get('param2', 0),
                param3=wp.get('param3', 0),
                param4=wp.get('param4', 0)
            ) for wp in waypoints],
            created_at=now,
            updated_at=now
        )
        self.missions[mission_id] = mission
        return mission

    def get_mission(self, mission_id: str) -> Optional[Mission]:
        return self.missions.get(mission_id)

    def list_missions(self) -> List[Mission]:
        return list(self.missions.values())

    def update_mission(self, mission_id: str, name: str, waypoints: List[dict]) -> Optional[Mission]:
        if mission_id not in self.missions:
            return None
        mission = self.missions[mission_id]
        mission.name = name
        mission.waypoints = [Waypoint(
            type=WaypointType(wp['type']),
            lat=wp['lat'],
            lon=wp['lon'],
            alt=wp['alt'],
            param1=wp.get('param1', 0),
            param2=wp.get('param2', 0),
            param3=wp.get('param3', 0),
            param4=wp.get('param4', 0)
        ) for wp in waypoints]
        mission.updated_at = time.time()
        return mission

    def delete_mission(self, mission_id: str) -> bool:
        if mission_id in self.missions:
            del self.missions[mission_id]
            return True
        return False

    def upload_mission(self, mission_id: str) -> bool:
        mission = self.get_mission(mission_id)
        if not mission:
            return False

        try:
            # Clear existing mission
            self.vehicle.mav.mission_clear_all_send(
                self.vehicle.target_system, self.vehicle.target_component)
            ack = self.vehicle.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
            if not ack or ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                raise Exception("Failed to clear existing mission")

            # Upload new mission
            self.vehicle.mav.mission_count_send(
                self.vehicle.target_system, self.vehicle.target_component, len(mission.waypoints))

            for i, wp in enumerate(mission.waypoints):
                msg = self.vehicle.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'], blocking=True, timeout=5)
                if not msg:
                    raise Exception(f"Failed to receive mission request for item {i}")

                self.vehicle.mav.mission_item_int_send(
                    self.vehicle.target_system, self.vehicle.target_component,
                    i,  # seq
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    self.get_mavlink_command(wp.type),
                    0, 1,  # current, autocontinue
                    wp.param1, wp.param2, wp.param3, wp.param4,
                    int(wp.lat * 1e7),
                    int(wp.lon * 1e7),
                    float(wp.alt)
                )

            # Wait for mission acknowledgement
            ack = self.vehicle.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
            if not ack or ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                raise Exception(f"Mission upload failed: {ack.type if ack else 'No acknowledgement'}")

            return True
        except Exception as e:
            print(f"Error uploading mission: {str(e)}")
            return False

    def execute_mission(self, mission_id: str) -> bool:
        if not self.upload_mission(mission_id):
            return False

        try:
            self.vehicle.mav.command_long_send(
                self.vehicle.target_system, self.vehicle.target_component,
                mavutil.mavlink.MAV_CMD_MISSION_START, 0,
                0, 0, 0, 0, 0, 0, 0)

            ack = self.vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            if not ack or ack.command != mavutil.mavlink.MAV_CMD_MISSION_START or ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                raise Exception("Failed to start mission")

            return True
        except Exception as e:
            print(f"Error executing mission: {str(e)}")
            return False

    @staticmethod
    def get_mavlink_command(wp_type: WaypointType):
        command_map = {
            WaypointType.WAYPOINT: mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            WaypointType.TAKEOFF: mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            WaypointType.LAND: mavutil.mavlink.MAV_CMD_NAV_LAND,
            WaypointType.RTL: mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
        }
        return command_map[wp_type]


def start_mavproxy():
    command = "mavproxy.py --master=tcp:127.0.0.1:5760 --out=udp:localhost:14550 --aircraft MyCopter"
    process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)
    time.sleep(10)  # Wait for MAVProxy to initialize
    return process


def init_wp_loader():
    global wp_loader
    wp_loader = mavwp.MAVWPLoader()


def format_attitude(msg_dict):
    return {
        "Roll": f"{msg_dict['roll']:.2f}°",
        "Pitch": f"{msg_dict['pitch']:.2f}°",
        "Yaw": f"{msg_dict['yaw']:.2f}°",
        "Roll Speed": f"{msg_dict['rollspeed']:.4f} rad/s",
        "Pitch Speed": f"{msg_dict['pitchspeed']:.4f} rad/s",
        "Yaw Speed": f"{msg_dict['yawspeed']:.4f} rad/s"
    }


def format_global_position_int(msg_dict):
    return {
        "Latitude": f"{msg_dict['lat'] / 1e7:.7f}°",
        "Longitude": f"{msg_dict['lon'] / 1e7:.7f}°",
        "Relative Altitude": f"{msg_dict['relative_alt'] / 1000:.2f} m",
        "Ground Speed": f"{((msg_dict['vx'] ** 2 + msg_dict['vy'] ** 2) ** 0.5) / 100:.2f} m/s",
        "Heading": f"{msg_dict['hdg'] / 100:.1f}°"
    }


def format_vfr_hud(msg_dict):
    return {
        "Airspeed": f"{msg_dict['airspeed']:.2f} m/s",
        "Ground Speed": f"{msg_dict['groundspeed']:.2f} m/s",
        "Heading": f"{msg_dict['heading']}°",
        "Throttle": f"{msg_dict['throttle']}%",
        "Altitude": f"{msg_dict['alt']:.2f} m",
        "Climb Rate": f"{msg_dict['climb']:.2f} m/s"
    }


def format_sys_status(msg_dict):
    return {
        "Battery Voltage": f"{msg_dict['voltage_battery'] / 1000:.2f} V",
        "Battery Current": f"{msg_dict['current_battery'] / 100:.2f} A",
        "Battery Remaining": f"{msg_dict['battery_remaining']}%",
        "CPU Load": f"{msg_dict['load'] / 10:.1f}%",
        "Drop Rate Comm": f"{msg_dict['drop_rate_comm'] / 100:.2f}%",
        "Errors Comm": msg_dict['errors_comm']
    }


def json_serialize(msg_dict):
    def convert_value(value):
        if isinstance(value, bytearray):
            return list(value)
        return value

    return {k: convert_value(v) for k, v in msg_dict.items()}


def get_mode_id(mode_name):
    mode_map = {
        'STABILIZE': 0, 'ACRO': 1, 'ALT_HOLD': 2, 'AUTO': 3, 'GUIDED': 4, 'LOITER': 5,
        'RTL': 6, 'CIRCLE': 7, 'POSITION': 8, 'LAND': 9, 'OF_LOITER': 10, 'DRIFT': 11,
        'SPORT': 13, 'FLIP': 14, 'AUTOTUNE': 15, 'POSHOLD': 16, 'BRAKE': 17,
        'THROW': 18, 'AVOID_ADSB': 19, 'GUIDED_NOGPS': 20, 'SMART_RTL': 21,
    }
    return mode_map.get(mode_name.upper(), 0)


def get_rtsp_frame():
    cap = cv2.VideoCapture(RTSP_URL)
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error reading frame from RTSP stream")
            break
        yield frame
    cap.release()


def generate_frames():
    for frame in get_rtsp_frame():
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


mode_mapping = {
    0: 'STABILIZE', 1: 'ACRO', 2: 'ALT_HOLD', 3: 'AUTO', 4: 'GUIDED', 5: 'LOITER',
    6: 'RTL', 7: 'CIRCLE', 8: 'POSITION', 9: 'LAND', 10: 'OF_LOITER', 11: 'DRIFT',
    13: 'SPORT', 14: 'FLIP', 15: 'AUTOTUNE', 16: 'POSHOLD', 17: 'BRAKE',
    18: 'THROW', 19: 'AVOID_ADSB', 20: 'GUIDED_NOGPS', 21: 'SMART_RTL',
}


def fetch_all_parameters():
    global vehicle, drone_parameters
    if not vehicle:
        return {"error": "Vehicle not connected"}

    vehicle.mav.param_request_list_send(vehicle.target_system, vehicle.target_component)
    start_time = time.time()
    param_count = None

    while time.time() - start_time < 30:  # Wait for up to 30 seconds
        msg = vehicle.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if msg:
            drone_parameters[msg.param_id] = msg.param_value
            if param_count is None:
                param_count = msg.param_count
            if len(drone_parameters) >= param_count:
                break

    if not drone_parameters:
        return {"error": "No parameters received"}

    return drone_parameters


def is_port_available(port):
    try:
        s = serial.Serial(port)
        s.close()
        return True
    except serial.SerialException:
        return False


def list_available_ports():
    return [port.device for port in serial.tools.list_ports.comports()]


def update_last_mission(mission_type, data):
    global last_mission
    last_mission = {
        'type': mission_type,
        'data': data,
        'timestamp': time.time()
    }


def check_vehicle_readiness():
    if not vehicle:
        return "Vehicle not connected"

    heartbeat = vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
    if not heartbeat or not (heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
        return "Vehicle is not armed"

    gps = vehicle.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2)
    if not gps or gps.fix_type < 3:
        return "Inadequate GPS fix"

    ekf = vehicle.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=2)
    if not ekf or ekf.flags < 0x0F:
        return "EKF is not healthy"

    return None  # All checks passed


def is_vehicle_in_air():
    global vehicle
    if not vehicle:
        logger.warning("Vehicle not connected. Cannot determine if it's in the air.")
        return False

    try:
        # Request GLOBAL_POSITION_INT message
        vehicle.mav.request_data_stream_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            1,  # 1 Hz
            1  # Start sending
        )

        # Wait for a GLOBAL_POSITION_INT message
        start_time = time.time()
        while time.time() - start_time < 5:  # 5-second timeout
            msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if msg:
                relative_alt = msg.relative_alt / 1000  # Convert mm to meters
                logger.debug(f"Received altitude: {relative_alt} meters")

                # Check if altitude is above a threshold (e.g., 0.5 meters)
                if relative_alt > 0.5:
                    logger.info("Vehicle is in the air")
                    return True
                else:
                    logger.info("Vehicle is on the ground")
                    return False

        logger.warning("Timeout waiting for GLOBAL_POSITION_INT message")
        return False

    except Exception as e:
        logger.error(f"Error in is_vehicle_in_air: {str(e)}")
        return False
    finally:
        # Stop the data stream to avoid unnecessary traffic
        try:
            vehicle.mav.request_data_stream_send(
                vehicle.target_system,
                vehicle.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                1,  # 1 Hz
                0  # Stop sending
            )
        except Exception as e:
            logger.error(f"Error stopping data stream: {str(e)}")


def calculate_distance(lat1, lon1, lat2, lon2):
    # Haversine formula to calculate distance between two points on Earth
    R = 6371  # Earth radius in kilometers

    d_lat = math.radians(lat2 - lat1)
    d_lon = math.radians(lon2 - lon1)
    a = (math.sin(d_lat / 2) * math.sin(d_lat / 2) +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
         math.sin(d_lon / 2) * math.sin(d_lon / 2))
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c

    return distance * 1000  # Convert to meters


def check_vehicle_state_for_upload():
    if not vehicle:
        return "Vehicle not connected"

    msg = vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    if not msg:
        return "No heartbeat received from vehicle"

    if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        return "Vehicle is armed. Please disarm before uploading a mission."

    custom_mode = msg.custom_mode
    mode_mapping = vehicle.mode_mapping()
    current_mode = [k for k, v in mode_mapping.items() if v == custom_mode]
    if current_mode and current_mode[0] in ['AUTO', 'GUIDED']:
        return f"Vehicle is in {current_mode[0]} mode. Please switch to a manual mode before uploading a mission."

    return None  # Vehicle state is okay for mission upload


def check_vehicle_communication():
    if not vehicle:
        return "Vehicle not connected"

    try:
        vehicle.mav.param_request_read_send(vehicle.target_system, vehicle.target_component, b'SYSID_THISMAV', -1)
        msg = vehicle.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
        if not msg:
            return "Failed to receive parameter from vehicle"

        vehicle.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        msg = vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
        if not msg:
            return "Failed to receive heartbeat from vehicle"

        return None  # Communication is okay
    except Exception as e:
        return f"Error checking vehicle communication: {str(e)}"


def check_drone_readiness():
    if not vehicle:
        return "Vehicle not connected"

    # Check flight mode
    mode = vehicle.recv_match(type='HEARTBEAT', blocking=True).custom_mode
    if mode != mavutil.mode_mapping_bynumber(vehicle.vehicle_type)['GUIDED']:
        return "Drone not in GUIDED mode"

    # Check GPS
    gps = vehicle.recv_match(type='GPS_RAW_INT', blocking=True)
    if gps.fix_type < 3:
        return "No GPS fix"

    # Check battery
    battery = vehicle.recv_match(type='SYS_STATUS', blocking=True)
    if battery.battery_remaining < 20:
        return "Low battery"

    return None  # All checks passed


def fetch_data():
    global vehicle, drone_parameters, connection_string
    message_types = ["ATTITUDE", "GLOBAL_POSITION_INT", "VFR_HUD", "SYS_STATUS", "HEARTBEAT"]
    backoff_time = 0.1
    max_backoff_time = 5
    connection_attempts = 0
    max_connection_attempts = 500

    while True:
        if not vehicle or not vehicle.target_system:
            try:
                if connection_attempts >= max_connection_attempts:
                    socketio.emit('message', {
                        'text': f"Max connection attempts reached. Please check your drone connection and restart the application.",
                        'color': 'red'})
                    time.sleep(60)
                    connection_attempts = 0
                    continue

                socketio.emit('message', {'text': f"Attempting to connect to {connection_string}", 'color': 'blue'})
                logger.debug(f"Initiating connection to {connection_string}")

                if vehicle:
                    try:
                        vehicle.close()
                    except:
                        pass

                vehicle = mavutil.mavlink_connection(connection_string)

                socketio.emit('message', {'text': "Waiting for heartbeat...", 'color': 'blue'})
                logger.debug("Waiting for heartbeat")

                heartbeat = vehicle.wait_heartbeat(timeout=30)
                if not heartbeat:
                    raise Exception("Heartbeat timeout - no heartbeat received")

                logger.debug(f"Heartbeat received: {heartbeat}")
                socketio.emit('message', {
                    'text': f"Connected to system (system {vehicle.target_system} component {vehicle.target_component})",
                    'color': 'green'})

                vehicle.mav.request_data_stream_send(
                    vehicle.target_system, vehicle.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1
                )
                logger.debug("Data stream requested")

                connection_attempts = 0
                backoff_time = 0.1  # Reset backoff time on successful connection

            except Exception as e:
                connection_attempts += 1
                error_message = f"Connection error: {str(e)}. Attempt {connection_attempts}/{max_connection_attempts}"
                socketio.emit('message', {'text': error_message, 'color': 'red'})
                logger.error(error_message)
                time.sleep(backoff_time)
                backoff_time = min(max_backoff_time, backoff_time * 2 + random.uniform(0, 0.1))
                continue

        try:
            msg = vehicle.recv_match(type=message_types, blocking=True, timeout=1.0)
            if msg:
                msg_type = msg.get_type()
                msg_dict = msg.to_dict()

                if msg_type == "ATTITUDE":
                    formatted_data = format_attitude(msg_dict)
                elif msg_type == "GLOBAL_POSITION_INT":
                    formatted_data = format_global_position_int(msg_dict)
                elif msg_type == "VFR_HUD":
                    formatted_data = format_vfr_hud(msg_dict)
                elif msg_type == "SYS_STATUS":
                    formatted_data = format_sys_status(msg_dict)
                elif msg_type == "HEARTBEAT":
                    custom_mode = msg_dict['custom_mode']
                    mode_name = mode_mapping.get(custom_mode, "UNKNOWN")
                    formatted_data = {"Flight Mode": mode_name}
                else:
                    formatted_data = {k: v for k, v in msg_dict.items() if v is not None}

                socketio.emit('message', {
                    'type': msg_type,
                    'color': 'cyan',
                    'data': json.dumps(formatted_data)
                })

                backoff_time = 0.1  # Reset backoff time on successful read
            else:
                logger.debug("No message received")

        except IOError as e:
            if e.errno == errno.EAGAIN or e.errno == errno.EWOULDBLOCK:
                logger.warning(f"Device not ready, retrying in {backoff_time:.2f} seconds...")
                time.sleep(backoff_time)
                backoff_time = min(max_backoff_time, backoff_time * 2 + random.uniform(0, 0.1))
                continue
            elif e.errno == errno.ENODEV:
                error_message = "Device disconnected. Attempting to reconnect..."
                socketio.emit('message', {'text': error_message, 'color': 'red'})
                logger.error(error_message)
                vehicle = None
                continue
            else:
                error_message = f"IOError in fetch_data: {str(e)}"
                socketio.emit('message', {'text': error_message, 'color': 'red'})
                logger.error(error_message)
                vehicle = None
                continue
        except Exception as e:
            error_message = f"Error in fetch_data: {str(e)}"
            socketio.emit('message', {'text': error_message, 'color': 'red'})
            logger.error(error_message)
            vehicle = None
            continue

        time.sleep(0.01)


# Flask routes

@app.route('/')
def index():
    return render_template('functionality-added-v4-1.html')


@app.route('/reconnect')
def reconnect():
    global vehicle, connection_attempts
    if vehicle:
        vehicle.close()
    vehicle = None
    connection_attempts = 0
    socketio.emit('message', {'text': "Manual reconnection initiated.", 'color': 'blue'})
    return "Reconnection initiated"


@app.route('/get_drone_position')
def get_drone_position():
    if not vehicle:
        return jsonify({'error': 'Vehicle not connected'})

    try:
        msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1000  # Convert millimeters to meters
            return jsonify({'lat': lat, 'lon': lon, 'alt': alt})
        else:
            return jsonify({'error': 'No GPS data available'})
    except Exception as e:
        return jsonify({'error': str(e)})


@app.route('/set_flight_mode', methods=['POST'])
def set_flight_mode():
    if not vehicle:
        return jsonify({'error': 'Vehicle not connected'})

    mode = request.json['mode']
    try:
        vehicle.mav.command_long_send(
            vehicle.target_system, vehicle.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            get_mode_id(mode), 0, 0, 0, 0, 0)
        return jsonify({'message': f'Flight mode set to {mode}'})
    except Exception as e:
        return jsonify({'error': str(e)})


@app.route('/rtl')
def return_to_launch():
    if not vehicle:
        return jsonify({'error': 'Vehicle not connected'})

    try:
        vehicle.mav.command_long_send(
            vehicle.target_system, vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0,
            0, 0, 0, 0, 0, 0, 0)
        return jsonify({'message': 'Return to Launch command sent'})
    except Exception as e:
        return jsonify({'error': str(e)})


@app.route('/set_speed', methods=['POST'])
def set_speed():
    if not vehicle:
        return jsonify({'error': 'Vehicle not connected'})

    speed = request.json['speed']
    try:
        vehicle.mav.command_long_send(
            vehicle.target_system, vehicle.target_component,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0,
            0, speed, -1, 0, 0, 0, 0)
        return jsonify({'message': f'Speed set to {speed} m/s'})
    except Exception as e:
        return jsonify({'error': str(e)})


@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/reposition', methods=['POST'])
def reposition():
    if not vehicle:
        return jsonify({'error': 'Vehicle not connected'}), 400

    try:
        data = request.json
        if not data:
            return jsonify({'error': 'No JSON data received'}), 400

        lat = data.get('lat')
        lon = data.get('lon')
        alt = data.get('alt')

        if lat is None or lon is None or alt is None:
            return jsonify({'error': 'Missing lat, lon, or alt in request'}), 400

        # Check drone readiness
        error = check_drone_readiness()
        if error:
            return jsonify({'error': error}), 400

        # Send MAV_CMD_DO_REPOSITION command
        vehicle.mav.command_int_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            0,  # current
            0,  # autocontinue
            -1,  # param1: Ground speed, -1 means no change
            mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,  # param2: Reposition flags
            0,  # param3: Reserved
            0,  # param4: Yaw, NaN means no change
            int(lat * 1e7),  # param5: Latitude
            int(lon * 1e7),  # param6: Longitude
            float(alt)  # param7: Altitude
        )

        # Wait for command acknowledgement
        ack = vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if ack is None:
            return jsonify({'error': 'No acknowledgement received from vehicle'}), 500
        if ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            return jsonify({'error': f'Command denied: result code {ack.result}'}), 400

        return jsonify({'message': f'Reposition command sent: Lat {lat}, Lon {lon}, Alt {alt}m'})
    except Exception as e:
        app.logger.error(f'Error in reposition: {str(e)}')
        app.logger.error(traceback.format_exc())
        return jsonify({'error': f'Server error: {str(e)}'}), 500


@app.route('/fetch_all_parameters')
def fetch_all_parameters_route():
    params = fetch_all_parameters()
    return jsonify(params)


@app.route('/update_parameter', methods=['POST'])
def update_parameter():
    global vehicle
    if not vehicle:
        return jsonify({"error": "Vehicle not connected"})

    data = request.json
    param_id = data.get('param_id')
    param_value = data.get('param_value')

    if param_id is None or param_value is None:
        return jsonify({"error": "Missing param_id or param_value"})

    try:
        vehicle.mav.param_set_send(
            vehicle.target_system, vehicle.target_component,
            param_id.encode('utf-8'),
            float(param_value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )

        # Wait for confirmation
        start_time = time.time()
        while time.time() - start_time < 5:  # Wait up to 5 seconds for confirmation
            msg = vehicle.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
            if msg and msg.param_id == param_id:
                drone_parameters[param_id] = msg.param_value
                return jsonify({"message": f"Parameter {param_id} set to {msg.param_value}"})

        return jsonify({"error": "Parameter set timeout"})
    except Exception as e:
        return jsonify({"error": str(e)})


@app.route('/get_last_mission')
def get_last_mission():
    global last_mission
    if last_mission:
        return jsonify(last_mission)
    else:
        return jsonify({'error': 'No mission available'})


# Mission Planner routes

mission_planner = None


@app.route('/missions', methods=['POST'])
def create_mission():
    global mission_planner
    if not mission_planner:
        return jsonify({"error": "Mission planner not initialized"}), 500

    data = request.json
    mission = mission_planner.create_mission(data['name'], data['waypoints'])
    return jsonify(mission.__dict__), 201


@app.route('/missions', methods=['GET'])
def list_missions():
    global mission_planner
    if not mission_planner:
        return jsonify({"error": "Mission planner not initialized"}), 500

    missions = mission_planner.list_missions()
    return jsonify([mission.__dict__ for mission in missions])


@app.route('/missions/<mission_id>', methods=['GET'])
def get_mission(mission_id):
    global mission_planner
    if not mission_planner:
        return jsonify({"error": "Mission planner not initialized"}), 500

    mission = mission_planner.get_mission(mission_id)
    if mission:
        return jsonify(mission.__dict__)
    return jsonify({"error": "Mission not found"}), 404


@app.route('/missions/<mission_id>', methods=['PUT'])
def update_mission(mission_id):
    global mission_planner
    if not mission_planner:
        return jsonify({"error": "Mission planner not initialized"}), 500

    data = request.json
    mission = mission_planner.update_mission(mission_id, data['name'], data['waypoints'])
    if mission:
        return jsonify(mission.__dict__)
    return jsonify({"error": "Mission not found"}), 404


@app.route('/missions/<mission_id>', methods=['DELETE'])
def delete_mission(mission_id):
    global mission_planner
    if not mission_planner:
        return jsonify({"error": "Mission planner not initialized"}), 500

    if mission_planner.delete_mission(mission_id):
        return '', 204
    return jsonify({"error": "Mission not found"}), 404


@app.route('/missions/<mission_id>/upload', methods=['POST'])
def upload_mission(mission_id):
    global mission_planner
    if not mission_planner:
        return jsonify({"error": "Mission planner not initialized"}), 500

    # Check drone readiness
    error = check_drone_readiness()
    if error:
        return jsonify({'error': error})

    try:
        # Clear current mission
        vehicle.mav.mission_clear_all_send(vehicle.target_system, vehicle.target_component)
        time.sleep(1)

        # Upload new mission
        mission = mission_planner.get_mission(mission_id)
        if not mission:
            return jsonify({"error": "Mission not found"}), 404

        for i, wp in enumerate(mission.waypoints):
            vehicle.mav.mission_item_send(
                vehicle.target_system,
                vehicle.target_component,
                i,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, 1,  # current, autocontinue
                0, 0, 0, 0,  # param1-4
                wp.lat, wp.lon, wp.alt
            )
            time.sleep(0.1)  # Small delay between waypoints

        return jsonify({"message": "Mission uploaded successfully"})
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@socketio.on('arm')
def handle_arm():
    if not vehicle:
        socketio.emit('message', {'text': "Vehicle not connected.", 'color': 'red'})
        return

    try:
        vehicle.mav.command_long_send(
            vehicle.target_system, vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0
        )
        socketio.emit('message', {'text': "Arming command sent.", 'color': 'green'})
    except Exception as e:
        socketio.emit('message', {'text': f"Error arming vehicle: {str(e)}", 'color': 'red'})
        logger.error(f"Error arming vehicle: {str(e)}")


@socketio.on('disarm')
def handle_disarm():
    if not vehicle:
        socketio.emit('message', {'text': "Vehicle not connected.", 'color': 'red'})
        return

    try:
        vehicle.mav.command_long_send(
            vehicle.target_system, vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, 0, 0, 0, 0, 0, 0
        )
        socketio.emit('message', {'text': "Disarming command sent.", 'color': 'green'})
    except Exception as e:
        socketio.emit('message', {'text': f"Error disarming vehicle: {str(e)}", 'color': 'red'})
        logger.error(f"Error disarming vehicle: {str(e)}")


@socketio.on('takeoff')
def handle_takeoff(altitude):
    if not vehicle:
        socketio.emit('message', {'text': "Vehicle not connected.", 'color': 'red'})
        return

    try:
        # First, switch to GUIDED mode
        vehicle.mav.command_long_send(
            vehicle.target_system, vehicle.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            get_mode_id('GUIDED'), 0, 0, 0, 0, 0)

        # Wait a moment for the mode change to take effect
        time.sleep(1)

        # Now send the takeoff command
        vehicle.mav.command_long_send(
            vehicle.target_system, vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, altitude
        )
        socketio.emit('message',
                      {'text': f"Switched to GUIDED mode and sent takeoff command. Target altitude: {altitude}m",
                       'color': 'green'})
    except Exception as e:
        socketio.emit('message', {'text': f"Error during takeoff: {str(e)}", 'color': 'red'})
        logger.error(f"Error during takeoff: {str(e)}")


@socketio.on('land')
def handle_land():
    if not vehicle:
        socketio.emit('message', {'text': "Vehicle not connected.", 'color': 'red'})
        return

    try:
        vehicle.mav.command_long_send(
            vehicle.target_system, vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
            0, 0, 0, 0, 0, 0, 0
        )
        socketio.emit('message', {'text': "Land command sent.", 'color': 'green'})
    except Exception as e:
        socketio.emit('message', {'text': f"Error during landing: {str(e)}", 'color': 'red'})
        logger.error(f"Error during landing: {str(e)}")


@app.route('/change_altitude', methods=['POST'])
def change_altitude():
    if not vehicle:
        return jsonify({'error': 'Vehicle not connected'}), 400

    try:
        data = request.json
        if not data:
            return jsonify({'error': 'No JSON data received'}), 400

        new_altitude = data.get('altitude')
        if new_altitude is None:
            return jsonify({'error': 'Missing altitude in request'}), 400

        # Check drone readiness
        error = check_drone_readiness()
        if error:
            return jsonify({'error': error}), 400

        # Get current position
        pos = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        lat = pos.lat / 1e7
        lon = pos.lon / 1e7

        # Send MAV_CMD_DO_REPOSITION command
        vehicle.mav.command_int_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            0,  # current
            0,  # autocontinue
            -1,  # param1: Ground speed, -1 means no change
            mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,  # param2: Reposition flags
            0,  # param3: Reserved
            float('nan'),  # param4: Yaw, NaN means no change
            int(lat * 1e7),  # param5: Latitude (no change)
            int(lon * 1e7),  # param6: Longitude (no change)
            float(new_altitude)  # param7: New Altitude
        )

        # Wait for command acknowledgement
        ack = vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if ack is None:
            return jsonify({'error': 'No acknowledgement received from vehicle'}), 500
        if ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            return jsonify({'error': f'Command denied: result code {ack.result}'}), 400

        return jsonify({'message': f'Altitude change command sent: {new_altitude}m'})
    except Exception as e:
        app.logger.error(f'Error in change_altitude: {str(e)}')
        app.logger.error(traceback.format_exc())
        return jsonify({'error': f'Server error: {str(e)}'}), 500


@app.route('/vehicle_status')
def vehicle_status():
    if not vehicle:
        return jsonify({'error': 'Vehicle not connected'})

    heartbeat = vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
    sys_status = vehicle.recv_match(type='SYS_STATUS', blocking=True, timeout=2)

    return jsonify({
        'armed': bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
        'mode': mavutil.mode_mapping_bynumber(vehicle.vehicle_type)[heartbeat.custom_mode],
        'system_status': heartbeat.system_status
    })


@app.route('/gps_status')
def gps_status():
    if not vehicle:
        return jsonify({'error': 'Vehicle not connected'})

    gps = vehicle.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2)

    return jsonify({
        'fix_type': gps.fix_type,
        'satellites_visible': gps.satellites_visible
    })


def init_vehicle_connection():
    global vehicle, connection_string
    try:
        vehicle = mavutil.mavlink_connection(connection_string)
        vehicle.wait_heartbeat(timeout=30)
        print(f"Connected to system (system {vehicle.target_system} component {vehicle.target_component})")
        return True
    except Exception as e:
        print(f"Failed to connect to vehicle: {str(e)}")
        return False


if __name__ == '__main__':
    try:
        if init_vehicle_connection():
            mission_planner = MissionPlanner(vehicle)
            mavproxy_process = start_mavproxy()
            threading.Thread(target=fetch_data, daemon=True).start()
            logger.info("Starting the application. Check console for DEBUG messages.")
            socketio.run(app, debug=True, host='0.0.0.0', port=5000, allow_unsafe_werkzeug=True)
        else:
            print("Failed to initialize vehicle connection. Please check your setup.")
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        if mavproxy_process:
            os.killpg(os.getpgid(mavproxy_process.pid), signal.SIGTERM)
        print("MAVProxy process terminated.")
