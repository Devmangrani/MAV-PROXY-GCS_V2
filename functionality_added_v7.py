import os
import time
import logging
import threading
import json
import cv2
import signal
import subprocess
import errno
import random
import sys
import math
from flask import Flask, render_template, jsonify, request, Response
from flask_socketio import SocketIO
from flask_cors import CORS
from queue import Queue
from pymavlink import mavutil, mavwp
import serial
import pynmea2
from pyubx2 import UBXReader, UBXMessage, NMEA_PROTOCOL, UBX_PROTOCOL
from typing import Optional


logging.getLogger('functionality_added_v7').setLevel(logging.WARNING)
logging.getLogger('socketio').setLevel(logging.WARNING)
logging.getLogger('engineio').setLevel(logging.WARNING)
logging.getLogger('werkzeug').setLevel(logging.WARNING)

# Create a custom logger
logger = logging.getLogger(__name__)
logger.setLevel(logging.WARNING)  # Set to WARNING to suppress DEBUG and INFO messages

# Create handlers
c_handler = logging.StreamHandler()
f_handler = logging.FileHandler('app.log')
c_handler.setLevel(logging.WARNING)
f_handler.setLevel(logging.WARNING)

# Create formatters and add it to handlers
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
c_handler.setFormatter(formatter)
f_handler.setFormatter(formatter)

# Add handlers to the logger
logger.addHandler(c_handler)
logger.addHandler(f_handler)

app = Flask(__name__)
CORS(app)
app.config['JSON_AS_ASCII'] = False
app.config['JSONIFY_PRETTYPRINT_REGULAR'] = True
socketio = SocketIO(app,
                    async_mode='eventlet',
                    cors_allowed_origins="*",
                    manage_session=False)

# Global variables
vehicle = None
connection_string = 'udpin:127.0.0.1:14550'
RTSP_URL = "rtsp:192.168.144.25:8554/main.264"
wp_loader = mavwp.MAVWPLoader()
current_mission = []
last_mission = None
drone_parameters = {}

# Thread-safe locks
vehicle_lock = threading.Lock()
wp_loader_lock = threading.Lock()
drone_parameters_lock = threading.Lock()

# Connection management
connection_attempts = 0
max_connection_attempts = 500
backoff_time = 0.1
max_backoff_time = 5


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
    while True:  # Infinite loop to keep trying
        cap = None
        try:
            # Force open the camera
            cap = cv2.VideoCapture(1)

            # Ensure camera is opened
            if not cap.isOpened():
                logger.warning("Failed to open camera, retrying...")
                if cap is not None:
                    cap.release()
                time.sleep(1)
                continue

            # Set camera properties
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 30)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize buffer delay

            while True:  # Keep reading frames
                ret, frame = cap.read()
                if not ret or frame is None:
                    logger.warning("Failed to read frame, reopening camera...")
                    break

                yield frame

        except Exception as e:
            logger.error(f"Camera error: {str(e)}")

        finally:
            if cap is not None:
                cap.release()

        # Immediate retry
        logger.info("Restarting camera connection...")
        time.sleep(0.1)  # Brief pause before retry


def generate_frames():
    while True:  # Keep generating frames indefinitely
        for frame in get_rtsp_frame():
            if frame is None:
                continue

            try:
                ret, buffer = cv2.imencode('.jpg', frame)
                if not ret:
                    continue

                frame_bytes = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

            except Exception as e:
                logger.error(f"Error in generate_frames: {str(e)}")
                continue


mode_mapping = {
    0: 'STABILIZE', 1: 'ACRO', 2: 'ALT_HOLD', 3: 'AUTO', 4: 'GUIDED', 5: 'LOITER',
    6: 'RTL', 7: 'CIRCLE', 8: 'POSITION', 9: 'LAND', 10: 'OF_LOITER', 11: 'DRIFT',
    13: 'SPORT', 14: 'FLIP', 15: 'AUTOTUNE', 16: 'POSHOLD', 17: 'BRAKE',
    18: 'THROW', 19: 'AVOID_ADSB', 20: 'GUIDED_NOGPS', 21: 'SMART_RTL',
}


def fetch_all_parameters():
    global vehicle, drone_parameters
    with vehicle_lock, drone_parameters_lock:
        if not vehicle:
            return {"error": "Vehicle not connected"}

        try:
            vehicle.mav.param_request_list_send(vehicle.target_system, vehicle.target_component)
            start_time = time.time()
            param_count = None
            drone_parameters.clear()  # Clear existing parameters

            while time.time() - start_time < 30:  # 30 seconds timeout
                msg = vehicle.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
                if msg:
                    # Handle both string and bytes cases for param_id
                    param_id = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode('utf-8')
                    drone_parameters[param_id] = msg.param_value
                    if param_count is None:
                        param_count = msg.param_count
                    if len(drone_parameters) >= param_count:
                        break
                else:
                    logger.warning("No parameter message received in the last second")

            if not drone_parameters:
                return {"error": "No parameters received"}

            logger.info(f"Fetched {len(drone_parameters)} parameters")
            return drone_parameters
        except Exception as e:
            logger.error(f"Error in fetch_all_parameters: {str(e)}")
            return {"error": f"Failed to fetch parameters: {str(e)}"}


def update_last_mission(mission_type, data):
    global last_mission
    last_mission = {
        'type': mission_type,
        'data': data,
        'timestamp': time.time()
    }


def check_vehicle_readiness():
    with vehicle_lock:
        if not vehicle:
            return "Vehicle not connected"

        # Check arming
        heartbeat = vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if not heartbeat or not (heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            return "Vehicle is not armed"

        # Check GPS
        gps = vehicle.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2)
        if not gps or gps.fix_type < 3:
            return "Inadequate GPS fix"

        # Check EKF
        ekf = vehicle.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=2)
        if not ekf or ekf.flags < 0x0F:
            return "EKF is not healthy"

        return None  # All checks passed


def is_vehicle_in_air():
    with vehicle_lock:
        if not vehicle:
            return False

        try:
            # Request GLOBAL_POSITION_INT message
            vehicle.mav.request_data_stream_send(
                vehicle.target_system, vehicle.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_POSITION, 1, 1)

            msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
            if msg and msg.relative_alt > 1000:  # More than 1 meter above ground
                return True
        except Exception as e:
            logger.error(f"Error in is_vehicle_in_air: {str(e)}")
        return False


def connect_vehicle():
    global vehicle, connection_attempts
    while connection_attempts < max_connection_attempts:
        try:
            connection_string = 'udp:127.0.0.1:14550'  # or your appropriate connection string
            logger.info(f"Attempting to connect to {connection_string} (Attempt {connection_attempts + 1})")
            vehicle = mavutil.mavlink_connection(connection_string, autoreconnect=True, source_system=1)
            logger.info("Waiting for heartbeat...")
            vehicle.wait_heartbeat(timeout=30)
            logger.info(f"Connected to system (system {vehicle.target_system} component {vehicle.target_component})")
            connection_attempts = 0
            return True
        except Exception as e:
            connection_attempts += 1
            logger.error(f"Connection error: {str(e)}. Attempt {connection_attempts}/{max_connection_attempts}")
            time.sleep(backoff_time * (2 ** (connection_attempts - 1)))  # Exponential backoff
    logger.error("Max connection attempts reached. Unable to connect to vehicle.")
    return False


def format_message(msg_type, msg_dict):
    if msg_type == "ATTITUDE":
        return format_attitude(msg_dict)
    elif msg_type == "GLOBAL_POSITION_INT":
        return format_global_position_int(msg_dict)
    elif msg_type == "VFR_HUD":
        return format_vfr_hud(msg_dict)
    elif msg_type == "SYS_STATUS":
        return format_sys_status(msg_dict)
    elif msg_type == "HEARTBEAT":
        return {"Flight Mode": mode_mapping.get(msg_dict['custom_mode'], "UNKNOWN")}
    else:
        return {k: v for k, v in msg_dict.items() if v is not None}


def fetch_data():
    global vehicle
    message_types = ["ATTITUDE", "GLOBAL_POSITION_INT", "VFR_HUD", "SYS_STATUS", "HEARTBEAT"]

    while True:
        with vehicle_lock:
            if not vehicle or not vehicle.target_system:
                if not connect_vehicle():
                    time.sleep(reconnection_delay)
                    continue

            try:
                msg = vehicle.recv_match(type=message_types, blocking=True, timeout=1.0)
                if msg:
                    msg_type = msg.get_type()
                    msg_dict = msg.to_dict()

                    formatted_data = format_message(msg_type, msg_dict)

                    socketio.emit('message', {
                        'type': msg_type,
                        'color': 'cyan',
                        'data': json.dumps(formatted_data)
                    })
                    # logger.debug(f"Emitted {msg_type} data: {formatted_data}")
                else:
                    logger.debug("No message received")

            except Exception as e:
                logger.error(f"Error in fetch_data: {str(e)}")
                vehicle = None

        time.sleep(0.01)


@socketio.on('arm')
def handle_arm():
    with vehicle_lock:
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


@socketio.on('takeoff')
def handle_takeoff(altitude):
    with vehicle_lock:
        if not vehicle:
            socketio.emit('message', {'text': "Vehicle not connected.", 'color': 'red'})
            return

        try:
            # Check if EKF is healthy
            ekf_status = vehicle.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=1)
            if ekf_status and ekf_status.flags < 0x0F:
                socketio.emit('message', {'text': "EKF is not healthy. Cannot takeoff.", 'color': 'red'})
                return

            # Check GPS fix type
            gps_status = vehicle.recv_match(type='GPS_RAW_INT', blocking=True, timeout=1)
            if not gps_status or gps_status.fix_type < 3:
                socketio.emit('message', {'text': "No GPS fix. Cannot takeoff.", 'color': 'red'})
                return

            # Switch to GUIDED mode
            socketio.emit('message', {'text': "Switching to GUIDED mode...", 'color': 'blue'})
            vehicle.mav.command_long_send(
                vehicle.target_system, vehicle.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                4, 0, 0, 0, 0, 0)  # 4 is GUIDED mode for ArduPilot

            # Wait for mode change acknowledgement
            ack = vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            if not ack or ack.command != mavutil.mavlink.MAV_CMD_DO_SET_MODE or ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                socketio.emit('message', {'text': "Failed to switch to GUIDED mode", 'color': 'red'})
                return

            # Arm the vehicle
            socketio.emit('message', {'text': "Arming vehicle...", 'color': 'blue'})
            vehicle.mav.command_long_send(
                vehicle.target_system, vehicle.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1, 0, 0, 0, 0, 0, 0)

            # Wait for arming acknowledgement
            ack = vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            if not ack or ack.command != mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM or ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                socketio.emit('message', {'text': "Failed to arm the vehicle", 'color': 'red'})
                return

            # Send takeoff command
            socketio.emit('message', {'text': f"Sending takeoff command to {altitude}m...", 'color': 'blue'})
            vehicle.mav.command_long_send(
                vehicle.target_system, vehicle.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                0, 0, 0, 0, 0, 0, altitude)

            # Wait for takeoff acknowledgement
            ack = vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            if not ack or ack.command != mavutil.mavlink.MAV_CMD_NAV_TAKEOFF or ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                socketio.emit('message', {
                    'text': f"Takeoff command failed. Result: {ack.result if ack else 'No acknowledgement'}",
                    'color': 'red'})
                return

            socketio.emit('message',
                          {'text': f"Takeoff command accepted. Target altitude: {altitude}m", 'color': 'green'})

            # Monitor altitude
            start_time = time.time()
            while time.time() - start_time < 0.1:
                msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
                if msg:
                    current_altitude = msg.relative_alt / 1000  # Convert mm to meters
                    socketio.emit('message',
                                  {'text': f"Current altitude: {current_altitude:.2f}m", 'color': 'blue'})
                    if current_altitude >= altitude * 0.95:  # Within 5% of target altitude
                        socketio.emit('message',
                                      {'text': f"Reached target altitude of {altitude}m", 'color': 'green'})
                        return
                time.sleep(1)

            socketio.emit('message', {'text': "Takeoff timeout. Check vehicle status.", 'color': 'yellow'})

        except Exception as e:
            socketio.emit('message', {'text': f"Error during takeoff: {str(e)}", 'color': 'red'})
            logger.error(f"Error during takeoff: {str(e)}")


@socketio.on('land')
def handle_land():
    with vehicle_lock:
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


@socketio.on('set_flight_mode')
def handle_set_flight_mode(mode):
    with vehicle_lock:
        if not vehicle:
            return {'success': False, 'message': 'Vehicle not connected'}

        try:
            vehicle.mav.command_long_send(
                vehicle.target_system, vehicle.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                get_mode_id(mode), 0, 0, 0, 0, 0)
            return {'success': True, 'message': f'Flight mode set to {mode}'}
        except Exception as e:
            return {'success': False, 'message': f'Failed to set flight mode: {str(e)}'}


@socketio.on('disarm')
def handle_disarm():
    with vehicle_lock:
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


@app.route('/')
def index():
    return render_template('functionality_added_v8.html')


@app.route('/reconnect')
def reconnect():
    global vehicle, connection_attempts
    with vehicle_lock:
        if vehicle:
            vehicle.close()
        vehicle = None
        connection_attempts = 0
    socketio.emit('message', {'text': "Manual reconnection initiated.", 'color': 'blue'})
    return "Reconnection initiated"


@app.route('/get_drone_position')
def get_drone_position():
    with vehicle_lock:
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


@app.route('/add_waypoint', methods=['POST'])
def add_waypoint():
    with wp_loader_lock:
        data = request.json
        lat = float(data['lat'])
        lon = float(data['lon'])
        alt = float(data['alt'])
        command = data.get('command', mavutil.mavlink.MAV_CMD_NAV_WAYPOINT)

        # Create the new waypoint
        wp = mavutil.mavlink.MAVLink_mission_item_message(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command,
            0, 1, 0, 0, 0, 0,
            lat, lon, alt)

        # Insert the waypoint at the beginning (index 0)
        wp_loader.insert(0, wp)

        # Renumber the sequence of all waypoints
        for i, waypoint in enumerate(wp_loader.wpoints):
            waypoint.seq = i

        return jsonify({'message': f'Waypoint added at the start: Lat {lat}, Lon {lon}, Alt {alt}m'})


@app.route('/get_mission')
def handle_get_mission():
    global current_mission

    with vehicle_lock:
        if not vehicle:
            return jsonify({'error': 'Vehicle not connected'}), 400

        try:
            # Request the current mission from the vehicle
            vehicle.mav.mission_request_list_send(vehicle.target_system, vehicle.target_component)

            # Wait for the mission count
            msg = vehicle.recv_match(type=['MISSION_COUNT'], blocking=True, timeout=5)
            if not msg:
                return jsonify({'error': 'Failed to receive mission count'}), 400

            mission_count = msg.count
            current_mission = []

            # Request each mission item
            for i in range(mission_count):
                vehicle.mav.mission_request_int_send(vehicle.target_system, vehicle.target_component, i)
                msg = vehicle.recv_match(type=['MISSION_ITEM_INT'], blocking=True, timeout=5)
                if not msg:
                    return jsonify({'error': f'Failed to receive mission item {i}'}), 400

                # Determine the waypoint type based on the command
                if msg.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                    wp_type = 'takeoff'
                elif msg.command == mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH:
                    wp_type = 'rtl'
                elif msg.command == mavutil.mavlink.MAV_CMD_NAV_LAND:
                    wp_type = 'land'
                elif msg.command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
                    wp_type = 'waypoint'
                elif msg.command == mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME:
                    wp_type = 'loiter_time'
                elif msg.command == mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS:
                    wp_type = 'loiter_turns'
                else:
                    wp_type = 'unknown'

                waypoint = {
                    'seq': msg.seq,
                    'type': wp_type,
                    'command': msg.command,
                    'lat': msg.x / 1e7,
                    'lon': msg.y / 1e7,
                    'alt': msg.z
                }

                # Add specific parameters for special waypoint types
                if wp_type == 'loiter_time':
                    waypoint['time'] = msg.param1
                elif wp_type == 'loiter_turns':
                    waypoint['turns'] = msg.param1
                    waypoint['radius'] = msg.param3

                current_mission.append(waypoint)

            # Remove home position (usually the first waypoint with seq=0)
            current_mission = [wp for wp in current_mission if wp['seq'] != 0]

            # Validate and clean up the mission
            validated_mission = validate_mission(current_mission)

            return jsonify(validated_mission)
        except Exception as e:
            logger.error(f"Error fetching mission: {str(e)}")
            return jsonify({'error': f'Error fetching mission: {str(e)}'}), 500


def validate_mission(mission):
    validated_mission = []
    has_takeoff = False

    for wp in mission:
        if wp['type'] == 'takeoff':
            if not has_takeoff:
                validated_mission.append(wp)
                has_takeoff = True
        elif wp['type'] in ['rtl', 'land']:
            # Ensure RTL or LAND is only at the end
            if not validated_mission or validated_mission[-1]['type'] not in ['rtl', 'land']:
                validated_mission.append(wp)
        elif wp['type'] in ['waypoint', 'loiter_time', 'loiter_turns']:
            # Include these waypoint types without any special conditions
            validated_mission.append(wp)
        else:
            # For any other unknown types, include them as is
            validated_mission.append(wp)

    return validated_mission


@app.route('/clear_mission')
def clear_mission():
    with wp_loader_lock:
        wp_loader.clear()
        return jsonify({'message': 'Mission cleared'})


@app.route('/add_loiter_time', methods=['POST'])
def add_loiter_time():
    with wp_loader_lock:
        data = request.json
        lat = float(data['lat'])
        lon = float(data['lon'])
        alt = float(data['alt'])
        time = float(data['time'])

        # Create a loiter time waypoint
        wp = mavutil.mavlink.MAVLink_mission_item_message(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
            0, 1, time, 0, 0, 0,
            lat, lon, alt)

        # Add the loiter time waypoint to the mission
        wp_loader.add(wp)

        return jsonify({
            'message': f'Loiter Time added: Lat {lat}, Lon {lon}, Alt {alt}m, Time {time}s',
            'waypoint': {
                'type': 'loiter_time',
                'lat': lat,
                'lon': lon,
                'alt': alt,
                'time': time
            }
        })


@app.route('/add_loiter_turns', methods=['POST'])
def add_loiter_turns():
    with wp_loader_lock:
        data = request.json
        app.logger.info(f"Received data for add_loiter_turns: {data}")  # Log the entire received data

        try:
            lat = float(data['lat'])
            lon = float(data['lon'])
            alt = float(data['alt'])
            turns = int(data['turns'])
            radius = float(data['radius'])

            app.logger.info(f"Parsed values: lat={lat}, lon={lon}, alt={alt}, turns={turns}, radius={radius}")

            # Create a loiter turns waypoint
            wp = mavutil.mavlink.MAVLink_mission_item_message(
                0, 0, 0,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
                0, 1, turns, 0, radius, 0,
                lat, lon, alt)

            # Add the loiter turns waypoint to the mission
            wp_loader.add(wp)

            response = {
                'message': f'Loiter Turns added: Lat {lat}, Lon {lon}, Alt {alt}m, Turns {turns}, Radius {radius}m',
                'waypoint': {
                    'type': 'loiter_turns',
                    'radius': radius,
                    'lat': lat,
                    'lon': lon,
                    'alt': alt,
                    'turns': turns

                }
            }
            app.logger.info(f"Sending response: {response}")
            return jsonify(response)

        except KeyError as e:
            app.logger.error(f"KeyError in add_loiter_turns: {str(e)}")
            return jsonify({'error': f'Missing required parameter: {str(e)}'}), 400
        except ValueError as e:
            app.logger.error(f"ValueError in add_loiter_turns: {str(e)}")
            return jsonify({'error': f'Invalid value: {str(e)}'}), 400
        except Exception as e:
            app.logger.error(f"Unexpected error in add_loiter_turns: {str(e)}")
            return jsonify({'error': f'Unexpected error: {str(e)}'}), 500


@app.route('/upload_mission', methods=['POST'])
def upload_mission():
    with vehicle_lock:
        if not vehicle:
            return jsonify({'error': 'Vehicle not connected'}), 400

        try:
            data = request.json
            waypoints = data.get('waypoints', [])

            if not waypoints:
                return jsonify({'error': 'No waypoints received'}), 400

            logger.info(f"Uploading mission with {len(waypoints)} waypoints")

            # Clear current mission
            clear_mission()

            # Get current position
            current_pos = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
            if not current_pos:
                return jsonify({'error': 'Failed to get current position'}), 400

            # Prepare mission items
            mission_items = []

            # Add current position as home waypoint
            mission_items.append(mavutil.mavlink.MAVLink_mission_item_int_message(
                vehicle.target_system,
                vehicle.target_component,
                0,  # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, 1, 0, 0, 0, 0,
                current_pos.lat,  # Current latitude
                current_pos.lon,  # Current longitude
                0  # Altitude 0 for home position
            ))

            # Add actual waypoints
            for i, wp in enumerate(waypoints, start=1):  # Start from 1
                lat = int(float(wp['lat']) * 1e7)
                lon = int(float(wp['lon']) * 1e7)
                alt = float(wp['alt'])

                if wp['type'] == 'takeoff':
                    cmd = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
                elif wp['type'] == 'rtl':
                    cmd = mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
                elif wp['type'] == 'land':
                    cmd = mavutil.mavlink.MAV_CMD_NAV_LAND
                elif wp['type'] == 'loiter_time':
                    cmd = mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME
                    time = float(wp['time'])
                elif wp['type'] == 'loiter_turns':
                    cmd = mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS
                    turns = float(wp['turns'])
                    radius = float(wp['radius'])
                else:
                    cmd = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT

                mission_item = mavutil.mavlink.MAVLink_mission_item_int_message(
                    vehicle.target_system,
                    vehicle.target_component,
                    i,  # seq
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    cmd,
                    0, 1, 0, 0, 0, 0,
                    lat,
                    lon,
                    int(alt)
                )

                if wp['type'] == 'loiter_time':
                    mission_item.param1 = time
                elif wp['type'] == 'loiter_turns':
                    mission_item.param1 = turns
                    mission_item.param3 = radius

                mission_items.append(mission_item)

            # Upload mission
            vehicle.mav.mission_count_send(vehicle.target_system, vehicle.target_component, len(mission_items))

            for i in range(len(mission_items)):
                msg = vehicle.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'], blocking=True, timeout=5)
                if not msg:
                    return jsonify({'error': f'No MISSION_REQUEST received for item {i}'}), 400
                vehicle.mav.send(mission_items[msg.seq])

            # Wait for mission ack
            msg = vehicle.recv_match(type=['MISSION_ACK'], blocking=True, timeout=5)
            if msg:
                if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    logger.info("Mission uploaded successfully")
                    # Set the first actual waypoint as current after successful upload
                    vehicle.mav.mission_set_current_send(vehicle.target_system, vehicle.target_component, 1)
                    return jsonify({
                        'message': 'Mission uploaded successfully and set to start from first waypoint',
                        'item_count': len(waypoints),
                        'home_position': {
                            'lat': current_pos.lat / 1e7,
                            'lon': current_pos.lon / 1e7
                        }
                    })
                else:
                    return jsonify({'error': f'Mission upload failed: {msg.type}'}), 400

            return jsonify({'error': 'No MISSION_ACK received'}), 400

        except Exception as e:
            logger.error(f'Error in upload_mission: {str(e)}')
            return jsonify({'error': str(e)}), 400


@app.route('/execute_mission', methods=['POST'])
def execute_mission():
    with vehicle_lock:
        if not vehicle:
            return jsonify({'error': 'Vehicle not connected'}), 400

        try:
            # Check if the vehicle is already armed
            heartbeat = vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
            if heartbeat and not (heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                # Arm the vehicle
                vehicle.mav.command_long_send(
                    vehicle.target_system, vehicle.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                    1, 0, 0, 0, 0, 0, 0)

                # Wait for arming acknowledgement
                arm_ack = vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
                if not arm_ack or arm_ack.command != mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM or arm_ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    return jsonify({'error': 'Failed to arm the vehicle'}), 400

                logger.info("Vehicle armed successfully")
            else:
                logger.info("Vehicle is already armed")

            # Switch to AUTO mode
            vehicle.mav.command_long_send(
                vehicle.target_system, vehicle.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                3, 0, 0, 0, 0, 0)  # 3 is AUTO mode for ArduPilot

            # Wait for mode change acknowledgement
            mode_ack = vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            if mode_ack and mode_ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE and mode_ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                return jsonify({'message': 'Vehicle armed and successfully switched to AUTO mode'})
            else:
                return jsonify({'error': 'Failed to switch to AUTO mode'}), 400

        except Exception as e:
            logger.error(f'Error in execute_mission: {str(e)}')
            return jsonify({'error': str(e)}), 500


@app.route('/add_takeoff', methods=['POST'])
def add_takeoff():
    with wp_loader_lock:
        data = request.json
        lat = float(data['lat'])
        lon = float(data['lon'])
        alt = float(data['alt'])

        # Create a takeoff waypoint
        wp = mavutil.mavlink.MAVLink_mission_item_message(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 1, 0, 0, 0, 0,
            lat, lon, alt)

        # Insert the takeoff waypoint at the beginning of the mission
        wp_loader.insert(0, wp)

        # Renumber the sequence of all waypoints
        for i, waypoint in enumerate(wp_loader.wpoints):
            waypoint.seq = i

        return jsonify({
            'message': f'Takeoff added: Lat {lat}, Lon {lon}, Alt {alt}m',
            'waypoint': {
                'type': 'takeoff',
                'lat': lat,
                'lon': lon,
                'alt': alt
            }
        })


def get_home_position():
    with vehicle_lock:
        if not vehicle:
            return None
        try:
            # Request home position
            vehicle.mav.command_long_send(
                vehicle.target_system, vehicle.target_component,
                mavutil.mavlink.MAV_CMD_GET_HOME_POSITION, 0,
                0, 0, 0, 0, 0, 0, 0)

            # Wait for the home position message
            msg = vehicle.recv_match(type='HOME_POSITION', blocking=True, timeout=5)
            if msg:
                return {
                    'lat': msg.latitude / 1e7,
                    'lon': msg.longitude / 1e7,
                    'alt': msg.altitude / 1000  # Convert mm to meters
                }
            else:
                return None
        except Exception as e:
            logger.error(f"Error getting home position: {str(e)}")
            return None


# Modify the add_rtl function
@app.route('/add_rtl', methods=['POST'])
def add_rtl():
    with wp_loader_lock:
        home = get_home_position()
        if not home:
            return jsonify({'error': 'Unable to get home position'})

        # Create an RTL waypoint
        wp = mavutil.mavlink.MAVLink_mission_item_message(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0, 1, 0, 0, 0, 0,
            home['lat'], home['lon'], home['alt'])

        # Add the RTL waypoint to the end of the mission
        wp_loader.add(wp)

        return jsonify({
            'message': 'RTL added to mission',
            'waypoint': {
                'type': 'rtl',
                'lat': home['lat'],
                'lon': home['lon'],
                'alt': home['alt']
            }
        })


@app.route('/add_land', methods=['POST'])
def add_land():
    with wp_loader_lock:
        data = request.json
        lat = float(data['lat'])
        lon = float(data['lon'])

        wp = mavutil.mavlink.MAVLink_mission_item_message(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 1, 0, 0, 0, 0,
            lat, lon, 0)
        wp_loader.add(wp)

        return jsonify({
            'message': f'Landing added: Lat {lat}, Lon {lon}',
            'waypoint': {
                'type': 'land',
                'lat': lat,
                'lon': lon,
                'alt': 0
            }
        })


@app.route('/set_flight_mode', methods=['POST'])
def set_flight_mode():
    with vehicle_lock:
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
    with vehicle_lock:
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


@app.route('/mission_control', methods=['POST'])
def mission_control():
    with vehicle_lock:
        if not vehicle:
            return jsonify({'error': 'Vehicle not connected'})

        action = request.json['action']
        try:
            if action == 'start':
                vehicle.mav.command_long_send(
                    vehicle.target_system, vehicle.target_component,
                    mavutil.mavlink.MAV_CMD_MISSION_START, 0,
                    0, 0, 0, 0, 0, 0, 0)
                return jsonify({'message': 'Mission started'})
            elif action == 'pause':
                vehicle.mav.command_long_send(
                    vehicle.target_system, vehicle.target_component,
                    mavutil.mavlink.MAV_CMD_DO_PAUSE_CONTINUE, 0,
                    0, 0, 0, 0, 0, 0, 0)
                return jsonify({'message': 'Mission paused'})
            elif action == 'resume':
                vehicle.mav.command_long_send(
                    vehicle.target_system, vehicle.target_component,
                    mavutil.mavlink.MAV_CMD_DO_PAUSE_CONTINUE, 0,
                    1, 0, 0, 0, 0, 0, 0)
                return jsonify({'message': 'Mission resumed'})
            else:
                return jsonify({'error': 'Invalid action'})
        except Exception as e:
            return jsonify({'error': str(e)})


@app.route('/set_geofence', methods=['POST'])
def set_geofence():
    with vehicle_lock:
        if not vehicle:
            return jsonify({'error': 'Vehicle not connected'})

        fence_type = request.json['type']
        radius = request.json['radius']
        try:
            vehicle.mav.command_long_send(
                vehicle.target_system, vehicle.target_component,
                mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE, 0,
                1, 0, 0, 0, 0, 0, 0)  # Enable fence

            if fence_type == 'circle':
                vehicle.mav.command_int_send(
                    vehicle.target_system, vehicle.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION,
                    0, 0, 0, 0, 0, 0, 0, 0, radius)

            return jsonify({'message': f'Geofence set: {fence_type}, radius: {radius}m'})
        except Exception as e:
            return jsonify({'error': str(e)})


@app.route('/get_parameters')
def get_parameters():
    with vehicle_lock:
        if not vehicle:
            return jsonify({'error': 'Vehicle not connected'})

        try:
            vehicle.mav.param_request_list_send(vehicle.target_system, vehicle.target_component)
            start_time = time.time()
            parameters = {}
            while time.time() - start_time < 30:  # Wait for up to 30 seconds
                msg = vehicle.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
                if msg:
                    parameters[msg.param_id] = msg.param_value
                if len(parameters) >= msg.param_count:
                    break
            return jsonify(parameters)
        except Exception as e:
            return jsonify({'error': str(e)})


@app.route('/set_parameter', methods=['POST'])
def set_parameter():
    with vehicle_lock:
        if not vehicle:
            return jsonify({'error': 'Vehicle not connected'})

        param_id = request.json['param_id']
        param_value = request.json['param_value']
        try:
            vehicle.mav.param_set_send(
                vehicle.target_system, vehicle.target_component,
                param_id.encode('utf-8'), param_value,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            return jsonify({'message': f'Parameter {param_id} set to {param_value}'})
        except Exception as e:
            return jsonify({'error': str(e)})


@app.route('/change_altitude', methods=['POST'])
def change_altitude():
    with vehicle_lock:
        if not vehicle:
            return jsonify({'error': 'Vehicle not connected'})

        altitude = float(request.json['altitude'])
        try:
            # Ensure we're in GUIDED mode
            vehicle.mav.command_long_send(
                vehicle.target_system, vehicle.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                get_mode_id('GUIDED'), 0, 0, 0, 0, 0)

            # Wait for mode change acknowledgement
            ack = vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            if not ack or ack.command != mavutil.mavlink.MAV_CMD_DO_SET_MODE or ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                return jsonify({'error': 'Failed to switch to GUIDED mode'})

            # Get current position
            pos = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
            if not pos:
                return jsonify({'error': 'Failed to get current position'})

            # Send MAV_CMD_DO_REPOSITION command
            vehicle.mav.command_int_send(
                vehicle.target_system, vehicle.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                0, 0,  # Current & Autocontinue
                -1,  # Param1: Ground speed, -1 means no change
                mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,  # Param2: Reposition flags
                0, 0,  # Param3 & Param4: unused
                int(pos.lat),  # X/Latitude
                int(pos.lon),  # Y/Longitude
                float(altitude)  # Z/Altitude
            )

            # Wait for command acknowledgement
            ack = vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            if not ack or ack.command != mavutil.mavlink.MAV_CMD_DO_REPOSITION:
                return jsonify(
                    {'error': f'Reposition command failed. Result: {ack.result if ack else "No acknowledgement"}'})

            if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                # Wait a moment for the altitude change to begin
                time.sleep(2)

                # Check new altitude
                new_pos = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
                if new_pos:
                    new_altitude = new_pos.relative_alt / 1000  # Convert mm to meters
                    return jsonify({
                        'message': f'Altitude change initiated. Current altitude: {new_altitude:.2f}m, Target altitude: {altitude}m'})
                else:
                    return jsonify({'warning': 'Altitude change initiated, but unable to confirm new altitude'})
            else:
                return jsonify({'error': f'Reposition command not accepted. Result: {ack.result}'})

        except Exception as e:
            return jsonify({'error': str(e)})


@app.route('/set_speed', methods=['POST'])
def set_speed():
    with vehicle_lock:
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
    try:
        return Response(generate_frames(),
                        mimetype='multipart/x-mixed-replace; boundary=frame')
    except Exception as e:
        logger.error(f"Error in video_feed route: {str(e)}")
        return "Video feed unavailable", 503


@app.route('/reposition', methods=['POST'])
def reposition():
    with vehicle_lock:
        if not vehicle:
            return jsonify({'error': 'Vehicle not connected'})

        data = request.json
        lat = data['lat']
        lon = data['lon']
        alt = data['alt']

        try:
            # Check if the vehicle is in air
            gps = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
            if not gps:
                return jsonify({'error': 'Failed to get current position'})

            current_alt = gps.relative_alt / 1000  # Convert mm to meters

            if current_alt < 1:  # Less than 1 meter above ground
                return jsonify({'error': 'Vehicle is not in air. Please take off first.'})

            # Switch to GUIDED mode
            vehicle.mav.command_long_send(
                vehicle.target_system, vehicle.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                4,  # 4 GUIDED mode for ArduPilot
                0, 0, 0, 0, 0)

            # Wait for mode change acknowledgement
            ack = vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            if not ack or ack.command != mavutil.mavlink.MAV_CMD_DO_SET_MODE or ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                return jsonify({'error': 'Failed to switch to GUIDED mode'})

            # Send the reposition command
            vehicle.mav.command_int_send(
                vehicle.target_system,  # target_system
                vehicle.target_component,  # target_component
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
                mavutil.mavlink.MAV_CMD_DO_REPOSITION,  # command
                0,  # current
                0,  # autocontinue
                -1,  # (-1 means no change)
                0,
                0,
                0,
                int(lat * 1e7),  # x: Latitude
                int(lon * 1e7),  # y: Longitude
                float(alt)  # z: Altitude
            )

            update_last_mission('reposition', {'lat': lat, 'lon': lon, 'alt': alt})
            return jsonify(
                {'message': f'Switched to GUIDED mode and sent reposition command: Lat {lat}, Lon {lon}, Alt {alt}m'})
        except Exception as e:
            return jsonify({'error': str(e)})


@app.route('/fetch_all_parameters')
def fetch_all_parameters_route():
    params = fetch_all_parameters()
    if isinstance(params, dict) and "error" in params:
        logger.error(f"Error in fetch_all_parameters_route: {params['error']}")
        return jsonify(params), 500
    return jsonify(params)


@app.route('/update_parameter', methods=['POST'])
def update_parameter():
    with vehicle_lock, drone_parameters_lock:
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
            while time.time() - start_time < 5:
                msg = vehicle.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
                if msg and msg.param_id == param_id:
                    drone_parameters[param_id] = msg.param_value
                    return jsonify({"message": f"Parameter {param_id} set to {msg.param_value}"})

            return jsonify({"error": "Parameter set timeout"})
        except Exception as e:
            return jsonify({"error": str(e)})


def is_vehicle_armed():
    heartbeat = vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
    return heartbeat and (heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def set_mode(mode):
    try:
        mode_id = get_mode_id(mode)
        vehicle.mav.command_long_send(
            vehicle.target_system, vehicle.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id, 0, 0, 0, 0, 0)

        # Wait for mode change acknowledgement
        ack = vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            return {'success': True, 'message': f'Mode set to {mode}'}
        else:
            return {'success': False, 'message': f'Failed to set mode to {mode}'}
    except Exception as e:
        return {'success': False, 'message': str(e)}


@app.route('/set_circle_mode', methods=['POST'])
def set_circle_mode():
    with vehicle_lock:
        if not vehicle:
            return jsonify({'error': 'Vehicle not connected'}), 400

        try:
            data = request.json
            radius = float(data['radius'])
            speed = float(data['speed'])
            turns = int(data['turns'])

            # Set GUIDED mode
            vehicle.mav.command_long_send(
                vehicle.target_system, vehicle.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                4, 0, 0, 0, 0, 0)  # 4 is GUIDED mode for ArduPilot

            # Send MAV_CMD_DO_SET_MODE command with CIRCLE mode parameter
            vehicle.mav.command_long_send(
                vehicle.target_system, vehicle.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                7, 0, 0, 0, 0, 0)  # 7 is CIRCLE mode for ArduPilot

            # Set circle radius
            vehicle.mav.command_long_send(
                vehicle.target_system, vehicle.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS, 0,
                turns, 0, radius, 0, 0, 0, 0)

            # Set speed
            vehicle.mav.command_long_send(
                vehicle.target_system, vehicle.target_component,
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0,
                0, speed, -1, 0, 0, 0, 0)

            return jsonify({'message': 'Circle mode enabled successfully'}), 200

        except Exception as e:
            logger.error(f'Error in set_circle_mode: {str(e)}')
            return jsonify({'error': str(e)}), 500


# Global variables (keeping original)
gps_data_queue = Queue()
follow_enabled = False
follow_thread = None
gps_thread = None
target_altitude = None
follow_lock = threading.Lock()
thread_lock = threading.Lock()

# Configure logging (keeping original configuration)
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('gps_follow.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


class GPSDataCollector:
    def __init__(self):
        self.latest_position = None

    def collect_position(self, lat, lon, alt=None):
        self.latest_position = {
            'lat': float(lat),
            'lon': float(lon),
            'alt': float(alt) if alt else None
        }
        gps_data_queue.put(self.latest_position)


def start_gps_reader():
    """Modified GPS reader that uses UBX protocol while maintaining original function name and structure"""

    def gps_reader_thread():
        try:
            # Initialize serial connection for UBX
            serial_port = serial.Serial(
                port='/dev/ttyAMA0',
                baudrate=9600,
                timeout=1
            )
            ubx_reader = UBXReader(
                serial_port,
                protfilter=(NMEA_PROTOCOL | UBX_PROTOCOL)
            )

            print("Waiting for GPS fix...", flush=True)

            while True:
                try:
                    (raw_data, parsed_data) = ubx_reader.read()

                    if isinstance(parsed_data, UBXMessage):
                        if parsed_data.identity == "NAV-PVT":
                            # Check if position is valid
                            if bool(parsed_data.valid & 0b1):
                                # Convert to the same format as original code
                                gps_data_queue.put({
                                    'lat': parsed_data.lat / 10 ** 7,  # Convert to degrees
                                    'lon': parsed_data.lon / 10 ** 7,  # Convert to degrees
                                    'alt': parsed_data.height / 1000  # Convert to meters
                                })

                    time.sleep(0.01)

                except (serial.SerialException, serial.SerialTimeoutException) as e:
                    logger.error(f"Serial error: {e}")
                    break
                except Exception as e:
                    logger.error(f"Unexpected error: {e}")
                    continue

        except serial.SerialException as e:
            logging.error(f"Failed to open serial port: {e}")
            return
        except KeyboardInterrupt:
            print("\nStopping GPS reading...")
        finally:
            serial_port.close()

    try:
        new_thread = threading.Thread(target=gps_reader_thread)
        new_thread.daemon = True
        new_thread.start()
        return new_thread
    except Exception as e:
        logger.error(f"Failed to start GPS reader thread: {e}")
        return None


def calculate_distance(lat1, lon1, lat2, lon2):
    """Calculate distance between two GPS coordinates (keeping original function)"""
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


def follow_gps():
    """Follow the GPS coordinates from the queue (keeping original function)"""
    global follow_enabled, target_altitude

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
                            0,
                            vehicle.target_system,
                            vehicle.target_component,
                            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                            0b110111111000,
                            int(gps_data['lat'] * 1e7),
                            int(gps_data['lon'] * 1e7),
                            target_altitude,
                            0, 0, 0, 0, 0, 0, 0, 0
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

        time.sleep(0.01)


def start_gps_follow(follow_altitude):
    """Start GPS following at specified altitude (keeping original function)"""
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
    """Stop GPS following (keeping original function)"""
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
    """Handle start GPS following request (keeping original function)"""
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
    """Handle stop GPS following request (keeping original function)"""
    try:
        if stop_gps_follow():
            return jsonify({'message': 'GPS following stopped'})
        return jsonify({'error': 'Failed to stop GPS following'}), 400
    except Exception as e:
        logger.error(f"Error stopping GPS follow: {e}")
        return jsonify({'error': str(e)}), 500


def cleanup():
    global vehicle
    if vehicle:
        vehicle.close()
    logger.info("Cleanup completed.")


background_task_started = False


def start_background_task():
    """Start the fetch_data function in a background thread."""
    global background_task_started
    if not background_task_started:
        thread = threading.Thread(target=fetch_data)
        thread.daemon = True
        thread.start()
        background_task_started = True
        logger.info("Background task started")


@socketio.on('connect')
def handle_connect():
    """Handle client connection and start background task if not already started."""
    start_background_task()
    logger.info("Client connected")


@app.route('/status')
def status():
    """Return the current status of the vehicle connection."""
    global vehicle
    if vehicle and vehicle.target_system:
        return jsonify({'status': 'connected', 'target_system': vehicle.target_system})
    else:
        return jsonify({'status': 'disconnected'})


start_background_task()

if __name__ == '__main__':
    try:
        if os.environ.get('WERKZEUG_RUN_MAIN') != 'true':
            logger.info("Initializing application...")

        # Check if running under Gunicorn
        if os.environ.get('GUNICORN_CONFIG'):
            pass
        else:
            # For development server
            socketio.run(app,
                         host='0.0.0.0',
                         port=5000,
                         debug=True,
                         use_reloader=False)
    except Exception as e:
        logger.error(f"Error starting server: {e}")
