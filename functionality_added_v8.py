import os
import time
import logging
import threading
from threading import Lock
import json
import cv2
import signal
import stat
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
import traceback
from sitl_follow_test_v5 import GPSSITLFollow
from map_to_pathplanner_adapter import adapter
from path_predictor_v1 import DroneNavigator
from datetime import datetime
import hashlib
import hmac
import base64
from Crypto.Cipher import AES
from Crypto.Util.Padding import unpad
import json
import binascii
import video_streaming


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
socketio = SocketIO(
    app,
    async_mode='eventlet',
    cors_allowed_origins="*",

    manage_session=False,
    logger=False,
    engineio_logger=False
)

# Global variables
vehicle = None
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
SECRET_KEY = "RodellaAerospaceLabs2025SecretKey"


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
    """
    Format the GLOBAL_POSITION_INT message with improved data extraction
    """
    lat = msg_dict['lat'] / 1e7 if 'lat' in msg_dict else 0
    lon = msg_dict['lon'] / 1e7 if 'lon' in msg_dict else 0

    # Get both absolute (AMSL) and relative altitudes
    alt_amsl = msg_dict['alt'] / 1000.0 if 'alt' in msg_dict else 0  # Convert mm to meters
    relative_alt = msg_dict['relative_alt'] / 1000.0 if 'relative_alt' in msg_dict else 0  # Convert mm to meters

    # Calculate ground speed from velocity components if available
    ground_speed = 0
    if 'vx' in msg_dict and 'vy' in msg_dict:
        ground_speed = ((msg_dict['vx'] ** 2 + msg_dict['vy'] ** 2) ** 0.5) / 100  # Convert cm/s to m/s

    # Get heading if available
    heading = msg_dict.get('hdg', 0) / 100.0 if 'hdg' in msg_dict else 0  # Convert centi-degrees to degrees

    return {
        "Latitude": f"{lat:.7f}°",
        "Longitude": f"{lon:.7f}°",
        "Relative Altitude": f"{relative_alt:.2f} m",
        "Altitude AMSL": f"{alt_amsl:.2f} m",  # Add AMSL altitude
        "Ground Speed": f"{ground_speed:.2f} m/s",
        "Heading": f"{heading:.1f}°",
        "lat_raw": lat,  # Add raw values for distance calculations
        "lon_raw": lon,
        "alt_raw": alt_amsl,
        "rel_alt_raw": relative_alt
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


def request_data_streams():
    """
    Request data streams with improved requests
    """
    global vehicle
    if not vehicle:
        return False

    try:
        # Request all data streams except EXTENDED_STATUS
        message_rates = {
            mavutil.mavlink.MAV_DATA_STREAM_ALL: 4,
            mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS: 2,
            mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS: 2,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION: 2,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1: 2,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA2: 2,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA3: 2,
        }

        for stream_id, rate in message_rates.items():
            vehicle.mav.request_data_stream_send(
                vehicle.target_system,
                vehicle.target_component,
                stream_id,
                rate,  # Rate in Hz
                1  # Start/Stop
            )
            logger.info(f"Requested data stream {stream_id} at {rate} Hz")

        # Specifically request HOME_POSITION message
        vehicle.mav.command_long_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
            0,  # Confirmation
            0, 0, 0, 0, 0, 0, 0  # Parameters (unused)
        )
        logger.info("Requested HOME_POSITION message")

        # Specifically request GPS_RAW_INT messages
        vehicle.mav.command_long_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,  # Confirmation
            mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,  # Message ID for GPS_RAW_INT
            1000000,  # Interval in microseconds (1 Hz)
            0, 0, 0, 0, 0  # Unused parameters
        )
        logger.info("Specifically requested GPS_RAW_INT messages at 1 Hz")

        # Specifically request GLOBAL_POSITION_INT messages
        vehicle.mav.command_long_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,  # Confirmation
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,  # Message ID for GLOBAL_POSITION_INT
            200000,  # Interval in microseconds (5 Hz)
            0, 0, 0, 0, 0  # Unused parameters
        )
        logger.info("Specifically requested GLOBAL_POSITION_INT messages at 5 Hz")

        # Send our own heartbeat
        vehicle.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0)

        return True
    except Exception as e:
        logger.error(f"Failed to request data streams: {e}")
        return False


def connect_vehicle():
    global vehicle, connection_attempts
    connection_string = 'udp:127.0.0.1:14550'

    while connection_attempts < max_connection_attempts:
        try:
            if vehicle:
                vehicle.close()
                vehicle = None
                time.sleep(1)

            logger.warning(f"Connecting to {connection_string}")
            vehicle = mavutil.mavlink_connection(
                connection_string,
                autoreconnect=False,
                source_system=255,
                source_component=0,
                timeout=10
            )

            # Wait for first heartbeat
            logger.warning("Waiting for heartbeat...")
            msg = vehicle.wait_heartbeat(timeout=10)
            if msg:
                logger.warning(f"Heartbeat received from system {vehicle.target_system}")

                # Request data streams
                if request_data_streams():
                    logger.warning("Data streams requested successfully")
                    connection_attempts = 0
                    return True
                else:
                    raise Exception("Failed to request data streams")
            else:
                raise Exception("No heartbeat received")

        except Exception as e:
            connection_attempts += 1
            logger.error(f"Connection attempt {connection_attempts} failed: {e}")
            if vehicle:
                vehicle.close()
                vehicle = None
            time.sleep(min(1 * (2 ** (connection_attempts - 1)), 5))

    return False


def format_message(msg_type, msg_dict):
    """
    Format different message types with improved data extraction
    """
    try:
        if msg_type == "SYSTEM_TIME":
            # Format the system time message
            time_unix_usec = msg_dict.get('time_unix_usec', 0)
            time_boot_ms = msg_dict.get('time_boot_ms', 0)

            # Convert to readable format
            boot_time_seconds = time_boot_ms / 1000.0  # Convert to seconds
            hours = int(boot_time_seconds // 3600)
            minutes = int((boot_time_seconds % 3600) // 60)
            seconds = int(boot_time_seconds % 60)

            formatted_uptime = f"{hours:02d}:{minutes:02d}:{seconds:02d}"

            return {
                "System Uptime": formatted_uptime,
                "Boot Time (ms)": time_boot_ms,
                "Unix Time (μs)": time_unix_usec,
                "uptime_seconds": boot_time_seconds  # Raw value for calculations
            }
        elif msg_type == "ATTITUDE":
            return format_attitude(msg_dict)
        elif msg_type == "GLOBAL_POSITION_INT":
            return format_global_position_int(msg_dict)
        elif msg_type == "VFR_HUD":
            return format_vfr_hud(msg_dict)
        elif msg_type == "SYS_STATUS":
            return format_sys_status(msg_dict)
        elif msg_type == "HEARTBEAT":
            # Check if we have a custom_mode field
            if 'custom_mode' in msg_dict:
                mode = mode_mapping.get(msg_dict['custom_mode'], "UNKNOWN")
            else:
                mode = "UNKNOWN"
            return {"Flight Mode": mode}
        elif msg_type == "GPS_RAW_INT":
            return format_gps_raw_int(msg_dict)
        elif msg_type == "HOME_POSITION":
            # Process home position data
            home_lat = msg_dict.get('latitude', 0) / 1e7 if 'latitude' in msg_dict else 0
            home_lon = msg_dict.get('longitude', 0) / 1e7 if 'longitude' in msg_dict else 0
            home_alt = msg_dict.get('altitude', 0) / 1000.0 if 'altitude' in msg_dict else 0  # mm to m

            return {
                "Home Latitude": f"{home_lat:.7f}°",
                "Home Longitude": f"{home_lon:.7f}°",
                "Home Altitude": f"{home_alt:.2f} m",
                "home_lat_raw": home_lat,
                "home_lon_raw": home_lon,
                "home_alt_raw": home_alt
            }
        else:
            # For other message types, return a cleaned dictionary
            return {k: v for k, v in msg_dict.items() if v is not None}
    except Exception as e:
        logger.error(f"Error formatting message {msg_type}: {str(e)}")
        return {"error": f"Failed to format message: {str(e)}"}


def format_gps_raw_int(msg_dict):
    # Map the fix_type integer to a human-readable string
    fix_types = {
        0: "No GPS",
        1: "No Fix",
        2: "2D Fix",
        3: "3D Fix",
        4: "DGPS",
        5: "RTK Float",
        6: "RTK Fixed"
    }

    fix_type = msg_dict.get('fix_type', 0)
    fix_text = fix_types.get(fix_type, "Unknown")

    satellites_visible = msg_dict.get('satellites_visible', 0)

    return {
        "fix_type": fix_type,
        "Fix Type": fix_text,
        "Satellites": satellites_visible,
        "GPS Status": f"{fix_text} ({satellites_visible} satellites)"
    }


def fetch_data():
    """
    Modified fetch_data function to include SYSTEM_TIME message
    """
    global vehicle
    message_types = ["ATTITUDE", "GLOBAL_POSITION_INT", "VFR_HUD", "SYS_STATUS",
                     "HEARTBEAT", "GPS_RAW_INT", "HOME_POSITION", "SYSTEM_TIME"]
    last_heartbeat = time.time()
    last_data_request = time.time()
    last_home_request = time.time()
    messages_received = 0

    while True:
        current_time = time.time()

        with vehicle_lock:
            if not vehicle:
                if not connect_vehicle():
                    time.sleep(1)
                    continue

            try:
                # Request data streams as before...

                # Specifically request SYSTEM_TIME message
                vehicle.mav.command_long_send(
                    vehicle.target_system, vehicle.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,  # Confirmation
                    mavutil.mavlink.MAVLINK_MSG_ID_SYSTEM_TIME,  # Message ID for SYSTEM_TIME
                    1000000,  # Interval in microseconds (1 Hz)
                    0, 0, 0, 0, 0  # Unused parameters
                )

                msg = vehicle.recv_match(type=message_types, blocking=True, timeout=0.5)

                if msg:
                    messages_received += 1
                    msg_type = msg.get_type()

                    # Process message as before...
                    msg_dict = msg.to_dict()
                    formatted_data = format_message(msg_type, msg_dict)

                    # Only emit the message to clients without logging
                    socketio.emit('message', {
                        'type': msg_type,
                        'color': 'cyan',
                        'data': json.dumps(formatted_data)
                    })

                # Continue with other checks as before...

            except Exception as e:
                logger.error(f"Error in fetch_data: {e}")
                vehicle = None
                messages_received = 0

        time.sleep(0.01)


@app.route('/diagnostics')
def get_diagnostics():
    with vehicle_lock:
        status = {
            'connected': bool(vehicle and vehicle.target_system),
            'target_system': vehicle.target_system if vehicle else None,
            'target_component': vehicle.target_component if vehicle else None,
            'connection_attempts': connection_attempts,
            'last_heartbeat': getattr(vehicle, 'last_heartbeat', None) if vehicle else None
        }
        return jsonify(status)


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


should_stop = False


@socketio.on('set_flight_mode')
def handle_set_flight_mode(mode):
    global should_stop, active_gps_follow
    with vehicle_lock:
        if not vehicle:
            return {'success': False, 'message': 'Vehicle not connected'}

        try:
            # If changing to LOITER mode, stop the follow script
            if mode == 'LOITER':
                should_stop = True
                if follow_thread and follow_thread.is_alive():
                    follow_thread.join(timeout=2)
                active_gps_follow = None

            # Send mode change command
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
    return render_template('functionality_added_v13.html')


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
        return Response(video_streaming.generate_frames(),
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
def guided_circle_waypoints():
    with vehicle_lock:
        if not vehicle:
            return jsonify({'error': 'Vehicle not connected'}), 400

        try:
            data = request.json
            radius = float(data['radius'])
            speed = float(data.get('speed'))
            num_points = int(data.get('points', 16))  # Points per complete circle
            turns = int(data.get('turns', 1))  # Number of complete turns

            # Get current position
            gps = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
            if not gps:
                return jsonify({'error': 'Could not get current position'}), 400

            # Current position is the center of the circle
            center_lat = gps.lat / 1e7  # Convert from int to float degrees
            center_lon = gps.lon / 1e7
            altitude = gps.relative_alt / 1000.0  # Convert mm to meters

            logger.info(
                f"Creating circular flight path with {num_points} points per turn, {turns} turns, radius {radius}m")

            # Create circle waypoints for a single turn
            single_turn_waypoints = []
            for i in range(num_points):
                # Calculate point on the circle
                angle = 2.0 * math.pi * i / num_points
                dx = radius * math.cos(angle)
                dy = radius * math.sin(angle)

                # Convert to lat/lon
                lat_offset = dy / 111111.0
                lon_offset = dx / (111111.0 * math.cos(math.radians(center_lat)))

                point_lat = center_lat + lat_offset
                point_lon = center_lon + lon_offset

                single_turn_waypoints.append((point_lat, point_lon))

            # Now repeat these waypoints for the requested number of turns
            all_waypoints = []
            for turn in range(turns):
                all_waypoints.extend(single_turn_waypoints)

            # Add the first waypoint again to complete the final circle
            if len(all_waypoints) > 0:
                all_waypoints.append(single_turn_waypoints[0])

            # Switch to GUIDED mode first
            vehicle.mav.command_long_send(
                vehicle.target_system, vehicle.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                4, 0, 0, 0, 0, 0)  # 4 is GUIDED mode

            # Wait for mode change
            time.sleep(1)

            # Set speed if provided
            if speed > 0:
                vehicle.mav.command_long_send(
                    vehicle.target_system, vehicle.target_component,
                    mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0,
                    0, speed, -1, 0, 0, 0, 0)

                # Give time for speed change to take effect
                time.sleep(0.5)

            # Create mission with all waypoints
            mission_items = []
            for i, (lat, lon) in enumerate(all_waypoints):
                # Create mission item
                mission_items.append({
                    'seq': i,
                    'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    'current': 1 if i == 0 else 0,  # Set first waypoint as current
                    'autocontinue': 1,  # Auto continue to next waypoint
                    'param1': 0,  # Hold time at waypoint
                    'param2': 2.0,  # Acceptance radius (meters)
                    'param3': 0,  # Pass by waypoint (0 = normal)
                    'param4': 0,  # Desired yaw angle (0 = no change)
                    'x': lat,  # Latitude
                    'y': lon,  # Longitude
                    'z': altitude  # Altitude
                })

            # Clear any existing mission
            vehicle.mav.mission_clear_all_send(
                vehicle.target_system, vehicle.target_component
            )

            # Wait for acknowledgement
            ack = vehicle.recv_match(type='MISSION_ACK', blocking=True, timeout=3)
            if not ack or ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                logger.warning(f"Mission clear not acknowledged: {ack.type if ack else 'timeout'}")

            # Send mission count
            vehicle.mav.mission_count_send(
                vehicle.target_system, vehicle.target_component,
                len(mission_items)
            )

            # Upload mission items
            for i in range(len(mission_items)):
                # Wait for request
                req = vehicle.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT'], blocking=True, timeout=3)
                if not req:
                    logger.warning(f"No mission request received for item {i}")
                    break

                item = mission_items[req.seq]

                # Send mission item
                vehicle.mav.mission_item_send(
                    vehicle.target_system, vehicle.target_component,
                    item['seq'],
                    item['frame'],
                    item['command'],
                    item['current'],
                    item['autocontinue'],
                    item['param1'],
                    item['param2'],
                    item['param3'],
                    item['param4'],
                    item['x'],
                    item['y'],
                    item['z']
                )

            # Wait for mission acknowledgement
            ack = vehicle.recv_match(type='MISSION_ACK', blocking=True, timeout=3)
            if not ack or ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                logger.warning(f"Mission upload not acknowledged: {ack.type if ack else 'timeout'}")
                return jsonify({'error': f"Mission upload failed: {ack.type if ack else 'timeout'}"}), 400

            # Apply throttle for mode change
            vehicle.mav.rc_channels_override_send(
                vehicle.target_system, vehicle.target_component,
                0, 0, 1500, 0, 0, 0, 0, 0)  # Mid throttle

            # Switch to AUTO mode to execute the mission
            vehicle.mav.command_long_send(
                vehicle.target_system, vehicle.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                3, 0, 0, 0, 0, 0)  # 3 is AUTO mode

            # Keep throttle for a moment to ensure stable transition
            time.sleep(1)

            # Release throttle override
            vehicle.mav.rc_channels_override_send(
                vehicle.target_system, vehicle.target_component,
                0, 0, 0, 0, 0, 0, 0, 0)

            return jsonify({
                'message': f'Circular flight path with {turns} turns uploaded and started',
                'center': {'lat': center_lat, 'lon': center_lon},
                'altitude': altitude,
                'radius': radius,
                'speed': speed,
                'waypoints': len(mission_items),
                'estimated_duration': f"{(2 * math.pi * radius * turns / speed) / 60:.1f} minutes"
            }), 200

        except Exception as e:
            logger.error(f'Error in guided_circle_waypoints: {str(e)}')
            traceback.print_exc()

            # Ensure RC override is released in case of error
            try:
                vehicle.mav.rc_channels_override_send(
                    vehicle.target_system, vehicle.target_component,
                    0, 0, 0, 0, 0, 0, 0, 0)
            except:
                pass

            return jsonify({'error': str(e)}), 500


active_gps_follow = None
gps_follow_lock = Lock()
mission_started = False


class SITLFollowManager:
    _instance = None

    def __init__(self):
        self.active_gps_follow = None
        self.mission_started = False
        self.gps_follow_lock = Lock()
        logger.info("SITLFollowManager initialized")

    @classmethod
    def get_instance(cls):
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    def start_follow(self, altitude, velocity):
        with self.gps_follow_lock:
            if self.active_gps_follow:
                logger.warning("Attempting to start when already active")
                return False

            try:
                # Create GPSSITLFollow instance with both parameters
                self.active_gps_follow = GPSSITLFollow(
                    target_altitude=float(altitude),
                    target_velocity=float(velocity)
                )

                if self.active_gps_follow.start_mission():
                    self.mission_started = True
                    logger.info(f"Mission started successfully at altitude: {altitude}m with velocity: {velocity}m/s")
                    return True
                else:
                    self.active_gps_follow = None
                    self.mission_started = False
                    logger.error("Failed to start mission")
                    return False

            except Exception as e:
                logger.error(f"Error starting follow: {e}")
                self.active_gps_follow = None
                self.mission_started = False
                raise

    def stop_follow(self):
        with self.gps_follow_lock:
            if not self.active_gps_follow:
                logger.warning("Stop requested but no active instance")
                return False

            try:
                self.active_gps_follow.cleanup()
                self.mission_started = False
                self.active_gps_follow = None
                logger.info("Follow stopped successfully")
                return True

            except Exception as e:
                logger.error(f"Error stopping follow: {e}")
                # Force reset state on error
                self.mission_started = False
                self.active_gps_follow = None
                raise


# Initialize the singleton manager
sitl_manager = SITLFollowManager.get_instance()


@app.route('/start_sitl_follow_test', methods=['POST'])
def handle_start_sitl_follow_test():
    request_id = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    logger.info(f"[{request_id}] Received start request")

    try:
        data = request.get_json()
        altitude = float(data.get('altitude', 10))
        velocity = float(data.get('velocity', 10))

        # Validate parameters
        if not (1 <= altitude <= 100):
            return jsonify({
                'success': False,
                'message': 'Altitude must be between 1 and 100 meters',
                'request_id': request_id
            }), 400

        if not (1 <= velocity <= 20):
            return jsonify({
                'success': False,
                'message': 'Velocity must be between 1 and 20 m/s',
                'request_id': request_id
            }), 400

        if sitl_manager.start_follow(altitude=altitude, velocity=velocity):
            return jsonify({
                'success': True,
                'message': 'SITL follow test started',
                'altitude': altitude,
                'velocity': velocity,
                'request_id': request_id
            })
        else:
            return jsonify({
                'success': False,
                'message': 'Failed to start SITL follow',
                'request_id': request_id
            }), 400

    except Exception as e:
        logger.error(f"[{request_id}] Error: {str(e)}")
        return jsonify({
            'success': False,
            'message': f'Error: {str(e)}',
            'request_id': request_id
        }), 500


@app.route('/api/stop_sitl_follow', methods=['POST'])
def stop_sitl_follow():
    request_id = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    logger.info(f"[{request_id}] Received stop request")

    try:
        if sitl_manager.stop_follow():
            return jsonify({
                'success': True,
                'message': 'SITL follow stopped successfully',
                'request_id': request_id
            })
        else:
            return jsonify({
                'success': False,
                'message': 'No active SITL follow instance found',
                'request_id': request_id
            })

    except Exception as e:
        logger.error(f"[{request_id}] Error: {str(e)}")
        return jsonify({
            'success': False,
            'message': f'Error: {str(e)}',
            'request_id': request_id
        }), 500


@app.route('/api/sitl_status', methods=['GET'])
def get_sitl_status():
    with sitl_manager.gps_follow_lock:
        status = {
            'active': sitl_manager.active_gps_follow is not None,
            'mission_started': sitl_manager.mission_started
        }

        if sitl_manager.active_gps_follow:
            status['altitude'] = sitl_manager.active_gps_follow.target_altitude
            status['follow_enabled'] = sitl_manager.active_gps_follow.follow_enabled

    return jsonify({
        'success': True,
        'status': status
    })


VALID_USERS = {
    "admin": {
        "password_hash": "5e884898da28047151d0e56f8dc6292773603d0d6aabbdd62a11ef721d1542d8"  # SHA-256 of "password"
    },
    "operator": {
        "password_hash": "e6c83b282aeb2e022844595721cc00bbda47cb24537c1779f9bb84f04039e2a8"  # SHA-256 of "operator123"
    },
    "guest": {
        "password_hash": "84983c60f7daadc1cb8698621f802c0d9f9a3c3c295c810748fb048115c186ec"  # SHA-256 of "guest123"
    }
}


# Add this function for AES decryption
# Replace the previous decrypt_aes function with this improved version
def decrypt_aes(encrypted_data, key):
    try:
        # Convert key to bytes and hash it to get 32 bytes (256 bits)
        key_bytes = key.encode('utf-8')
        key_hash = hashlib.sha256(key_bytes).digest()

        # CryptoJS format special handling:
        # CryptoJS AES encryption produces data in format: "base64(salt + iv + ciphertext)"
        encrypted_bytes = base64.b64decode(encrypted_data)

        # Parse the CryptoJS format:
        # The string starts with "Salted__" (8 bytes), followed by 8 bytes of salt
        if encrypted_bytes[:8] == b'Salted__':
            salt = encrypted_bytes[8:16]
            # Derive key and IV using the OpenSSL EVP_BytesToKey derivation
            d = d_i = b''
            while len(d) < 48:  # We need 32 bytes for key and 16 bytes for IV
                d_i = hashlib.md5(d_i + key_bytes + salt).digest()
                d += d_i
            derived_key = d[:32]  # First 32 bytes for the key
            iv = d[32:48]  # Next 16 bytes for the IV
            ciphertext = encrypted_bytes[16:]  # Rest is ciphertext
        else:
            # Fallback for non-OpenSSL format - assume the first 16 bytes are IV
            iv = encrypted_bytes[:16]
            ciphertext = encrypted_bytes[16:]
            derived_key = key_hash

        # Create cipher object and decrypt
        cipher = AES.new(derived_key, AES.MODE_CBC, iv)

        try:
            # First try with PKCS7 padding (standard)
            decrypted_bytes = unpad(cipher.decrypt(ciphertext), AES.block_size, style='pkcs7')
        except ValueError:
            # If that fails, try without padding
            decrypted_bytes = cipher.decrypt(ciphertext)
            # Remove potential padding manually
            padding_length = decrypted_bytes[-1]
            if padding_length < AES.block_size:
                decrypted_bytes = decrypted_bytes[:-padding_length]

        # Convert bytes to string
        return decrypted_bytes.decode('utf-8')
    except Exception as e:
        logger.error(f"Decryption error: {str(e)}")
        import traceback
        logger.error(traceback.format_exc())
        return None


# Add this route for login verification
@app.route('/api/verify_login', methods=['POST'])
def verify_login():
    try:
        data = request.get_json()
        username = data.get('username')
        encrypted_password = data.get('encryptedPassword')

        if not username or not encrypted_password:
            return jsonify({'success': False, 'message': 'Missing username or password'}), 400

        # Check if username exists
        if username not in VALID_USERS:
            # Use constant time comparison to prevent timing attacks
            # even though we're returning immediately
            hmac.compare_digest('dummy', 'dummy')
            return jsonify({'success': False, 'message': 'Invalid username or password'}), 401

        # Decrypt the password
        decrypted_password = decrypt_aes(encrypted_password, SECRET_KEY)
        if not decrypted_password:
            return jsonify({'success': False, 'message': 'Decryption error'}), 400

        # Hash the decrypted password for comparison
        password_hash = hashlib.sha256(decrypted_password.encode('utf-8')).hexdigest()

        # Compare with stored hash using constant-time comparison to prevent timing attacks
        if hmac.compare_digest(password_hash, VALID_USERS[username]['password_hash']):
            # Log the successful login
            logger.info(f"Successful login for user: {username}")
            return jsonify({'success': True, 'message': 'Login successful'})
        else:
            # Log the failed attempt (but don't expose which part was wrong)
            logger.warning(f"Failed login attempt for user: {username}")
            return jsonify({'success': False, 'message': 'Invalid username or password'}), 401

    except Exception as e:
        logger.error(f"Login error: {str(e)}")
        return jsonify({'success': False, 'message': f'An error occurred: {str(e)}'}), 500


class MissionManager:
    """Singleton class to manage convoy mission state"""

    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(MissionManager, cls).__new__(cls)
            cls._instance.active_planner = None
            cls._instance.is_running = False
            cls._instance.mission_thread = None
            cls._instance.start_time = None
            cls._instance.waypoint_count = 0
        return cls._instance

    def start_mission(self, planner):
        """Set mission as started"""
        self.active_planner = planner
        self.is_running = True
        self.start_time = time.time()
        self.waypoint_count = len(planner.waypoints) if planner.waypoints else 0
        logger.info(f"Mission state set to RUNNING with {self.waypoint_count} waypoints")

    def stop_mission(self):
        """Stop the mission and clean up"""
        logger.info("Stopping mission...")
        if self.active_planner:
            try:
                logger.info("Calling cleanup on active planner")
                self.active_planner.cleanup()
                logger.info("Cleanup completed")
            except Exception as e:
                logger.error(f"Cleanup error: {str(e)}")

        # Reset state variables
        self.is_running = False
        self.active_planner = None
        logger.info("Mission stopped and state reset")
        return True

    def get_status(self):
        """Get current mission status information"""
        if not self.is_running or not self.active_planner:
            return {
                "active": False,
                "message": "No mission running"
            }

        duration = time.time() - self.start_time if self.start_time else 0
        current_index = getattr(self.active_planner, 'current_target_index', -1)
        midpoint_count = len(self.active_planner.midpoints) if hasattr(self.active_planner, 'midpoints') else 0

        return {
            "active": True,
            "duration_seconds": int(duration),
            "waypoints": self.waypoint_count,
            "midpoints": midpoint_count,
            "current_index": current_index,
            "progress": f"{current_index + 1}/{midpoint_count}" if midpoint_count > 0 and current_index >= 0 else "N/A"
        }


# Create the singleton mission manager
mission_mgr = MissionManager()


@app.route('/api/run_convoy', methods=['POST'])
def run_convoy_mission():
    """Route to run the convoy mission with map coordinates and car GPS tracking."""

    # Log the request attempt
    logger.info("Convoy mission request received")

    try:
        # Parse and validate waypoints
        data = request.json
        if not data or 'waypoints' not in data:
            logger.error("Missing waypoints in request")
            return jsonify({
                'success': False,
                'message': 'Missing waypoints data'
            }), 400

        waypoints = data.get('waypoints', [])
        if len(waypoints) < 2:
            logger.error(f"Insufficient waypoints: {len(waypoints)}")
            return jsonify({
                'success': False,
                'message': 'At least 2 waypoints are required'
            }), 400

        # Log the received waypoints
        logger.info(f"Received {len(waypoints)} waypoints for convoy mission")

        # Get mission parameters (if provided)
        target_altitude = float(data.get('altitude', 50.0))
        target_velocity = float(data.get('velocity', 10.0))
        distance_threshold = float(data.get('distance_threshold', 10.0))

        # Use the existing car GPS port from main loop
        # No port detection or setting logic here - just use what was already configured

        # Run the mission using the adapter with car tracking
        result = adapter.run_convoy_mission(
            waypoints_data=waypoints,
            altitude=target_altitude,
            velocity=target_velocity,
            distance_threshold=distance_threshold
        )

        # If successful, add additional data to response
        if result.get('success', False):
            missions_folder = "missions"
            os.makedirs(missions_folder, exist_ok=True)

            # Create a timestamp for the mission
            mission_filename = f"convoy_mission_{int(time.time())}.json"

            # Return the response
            return jsonify({
                'success': True,
                'message': result.get('message', 'Car tracking mission started'),
                'waypoints': result.get('waypoints', len(waypoints)),
                'midpoints': result.get('midpoints', 0),
                'car_gps_port': result.get('car_gps_port', 'using default'),
                'mission_file': mission_filename
            })
        else:
            # Pass through error message from adapter
            return jsonify(result), 500

    except Exception as e:
        logger.error(f"Error starting convoy mission: {str(e)}")
        return jsonify({
            'success': False,
            'message': f'Error starting convoy mission: {str(e)}'
        }), 500

def run_mission_thread(planner):
    """Thread function to execute the convoy mission sequence."""

    logger.info("Mission thread started")

    try:
        # Setup flight
        if not planner.set_guided_mode():
            logger.error("Failed to set GUIDED mode")
            mission_mgr.stop_mission()
            return

        if not planner.arm_vehicle():
            logger.error("Failed to arm vehicle")
            mission_mgr.stop_mission()
            return

        # Take off if needed
        current_pos = planner.get_drone_position()
        current_alt = current_pos.elevation if current_pos else 0

        # Get target altitude from the waypoints, or use a default
        target_altitude = planner.waypoints[0].elevation if planner.waypoints else 10.0
        logger.info(f"Using target altitude: {target_altitude}m")

        if current_alt < 1.0:
            logger.info(f"Taking off to {target_altitude}m")
            if not planner.takeoff(target_altitude):
                logger.error("Takeoff failed")
                mission_mgr.stop_mission()
                return
        else:
            logger.info(f"Already at altitude {current_alt}m, skipping takeoff")

        # Check if mission is still active
        if not mission_mgr.is_running:
            logger.info("Mission already stopped, aborting execution")
            return

        # Use midpoints if available, otherwise use main waypoints
        points_to_follow = planner.midpoints if planner.midpoints else planner.waypoints

        if not points_to_follow:
            logger.error("No waypoints or midpoints to follow")
            mission_mgr.stop_mission()
            return

        logger.info(f"Starting mission with {len(points_to_follow)} points to follow")

        # Set initial target index
        planner.current_target_index = 0

        for i, waypoint in enumerate(points_to_follow):
            # Check if mission is still active
            if not mission_mgr.is_running:
                logger.info("Mission stopped during execution")
                return

            logger.info(f"Moving to point {i + 1}/{len(points_to_follow)}: {waypoint}")
            planner.current_target_index = i

            # Use planner's send_goto_command with explicit altitude override
            planner.send_goto_command(waypoint, 10.0, override_altitude=target_altitude)

            # Wait for arrival
            arrived = False
            start_time = time.time()
            while mission_mgr.is_running and not arrived and time.time() - start_time < 60:
                pos = planner.get_drone_position()
                if pos:
                    distance = pos.distance_to(waypoint)
                    altitude_diff = abs(pos.elevation - target_altitude)
                    logger.info(f"Distance to waypoint: {distance:.2f}m, altitude difference: {altitude_diff:.2f}m")
                    if distance < 5.0 and altitude_diff < 2.0:  # 5m horizontal, 2m vertical threshold
                        arrived = True
                        logger.info(f"Arrived at point {i + 1}")
                time.sleep(1)

            if not mission_mgr.is_running:
                logger.info("Mission stopped during navigation")
                return

            if not arrived:
                logger.warning(f"Timeout waiting for arrival at point {i + 1}, continuing anyway")

        # Mission complete
        logger.info("Mission completed successfully")

    except Exception as e:
        logger.error(f"Error in mission thread: {str(e)}")
    finally:
        # Always ensure mission state is reset
        mission_mgr.stop_mission()


@app.route('/api/stop_convoy', methods=['POST'])
def stop_convoy_mission():
    """
    Stop the convoy mission
    """
    try:
        logger.warning("Stop convoy mission endpoint called")

        # Use adapter to stop the mission
        result = adapter.stop_mission()

        logger.warning("Mission stop request processed")
        return jsonify(result)

    except Exception as e:
        logger.error(f"Error stopping convoy mission: {str(e)}")
        return jsonify({
            'success': False,
            'message': f'Error: {str(e)}'
        }), 500


@app.route('/api/convoy_status', methods=['GET'])
def convoy_mission_status():
    """
    Get the current status of the convoy mission including car and drone positions
    """
    try:
        status = adapter.get_status()
        return jsonify({
            'success': True,
            'status': status
        })
    except Exception as e:
        logger.error(f"Error getting convoy mission status: {str(e)}")
        return jsonify({
            'success': False,
            'message': f'Error: {str(e)}'
        }), 500


@app.route('/api/set_car_gps_port', methods=['POST'])
def set_car_gps_port():
    """
    Set the car GPS port manually
    """
    try:
        data = request.json
        if not data or 'port' not in data:
            return jsonify({
                'success': False,
                'message': 'Missing port parameter'
            }), 400

        port = data.get('port')
        result = adapter.set_car_gps_port(port)
        return jsonify(result)

    except Exception as e:
        logger.error(f"Error setting car GPS port: {str(e)}")
        return jsonify({
            'success': False,
            'message': f'Error: {str(e)}'
        }), 500


DEFAULT_GPS_PORT = '/dev/ttyUSB0'
DEFAULT_GPS_BAUDRATE = 9600


# Function to check if the GPS port is accessible
def check_gps_port_access(port_path):
    """Check if the GPS port exists and is accessible."""
    try:
        # Check if the file exists
        if not os.path.exists(port_path):
            logger.warning(f"GPS port {port_path} does not exist")
            return False

        # Check if it's readable
        if not os.access(port_path, os.R_OK | os.W_OK):
            logger.warning(f"GPS port {port_path} is not readable/writable")

            # Try to fix permissions if running as root
            try:
                os.chmod(port_path,
                         stat.S_IRUSR | stat.S_IWUSR | stat.S_IRGRP | stat.S_IWGRP | stat.S_IROTH | stat.S_IWOTH)
                logger.info(f"Fixed permissions for {port_path}")
                return True
            except Exception as e:
                logger.error(f"Failed to fix permissions: {str(e)}")
                return False

        return True
    except Exception as e:
        logger.error(f"Error checking GPS port: {str(e)}")
        return False


# Dictionary to track active DroneNavigator instances
active_navigators = {}


@app.route('/api/stop_road_navigation', methods=['POST'])
def stop_road_navigation():
    """
    Stop any active road navigation mission and clean up resources.
    """
    global active_navigators

    logger.info("Stop road navigation endpoint called")

    try:
        # Get the active navigator instance if available
        navigator = active_navigators.get('road_navigation')

        if navigator:
            logger.info("Active navigator found, cleaning up...")
            try:
                # Call the cleanup function to stop the mission
                navigator.cleanup()
                logger.info("Navigator cleanup successful")

                # Remove from active navigators - FIX: Check if key exists
                if 'road_navigation' in active_navigators:
                    del active_navigators['road_navigation']

                return jsonify({
                    'success': True,
                    'message': 'Road navigation stopped successfully'
                })
            except Exception as e:
                logger.error(f"Error during navigator cleanup: {str(e)}")
                import traceback
                logger.error(traceback.format_exc())
                return jsonify({
                    'success': False,
                    'message': f'Error during cleanup: {str(e)}'
                }), 500
        else:
            logger.info("No active road navigation found")
            return jsonify({
                'success': True,
                'message': 'No active road navigation found to stop'
            })

    except Exception as e:
        logger.error(f"Error in stop_road_navigation: {str(e)}")
        import traceback
        logger.error(traceback.format_exc())
        return jsonify({
            'success': False,
            'message': f'Error: {str(e)}'
        }), 500


@app.route('/execute_predicted_path', methods=['POST'])
def execute_predicted_path():
    """
    Execute the path directly using the waypoints from the frontend without recalculation.
    """
    try:
        # Get GPS port from environment variable or use default
        gps_port = os.environ.get('GPS_PORT', DEFAULT_GPS_PORT)
        gps_baudrate = int(os.environ.get('GPS_BAUDRATE', DEFAULT_GPS_BAUDRATE))

        # Log GPS configuration
        logger.info(f"Using GPS port: {gps_port}, baudrate: {gps_baudrate}")

        # Check if GPS port is accessible
        if not check_gps_port_access(gps_port):
            logger.warning(f"Will attempt to connect to GPS port {gps_port} but it may not be accessible")

        data = request.json
        if not data or 'waypoints' not in data:
            return jsonify({
                'success': False,
                'message': 'Missing waypoints data'
            }), 400

        waypoints = data.get('waypoints', [])
        if len(waypoints) < 2:
            return jsonify({
                'success': False,
                'message': 'At least 2 waypoints are required'
            }), 400

        # Get mission parameters
        target_altitude = float(data.get('altitude', 30.0))
        target_velocity = float(data.get('velocity', 10.0))

        logger.info(f"Executing path with {len(waypoints)} waypoints provided by frontend")
        logger.info(f"Target altitude: {target_altitude}m, velocity: {target_velocity}m/s")

        # Create a new DroneNavigator instance with explicit GPS port
        navigator = DroneNavigator(
            max_points=1000,
            vehicle_gps_port=gps_port,
            vehicle_gps_baudrate=gps_baudrate
        )

        # Add waypoints to the navigator directly from frontend data
        for wp in waypoints:
            navigator.add_waypoint(
                lat=float(wp['lat']),
                lon=float(wp['lon']),
                elevation=float(wp.get('elevation', target_altitude)),
                name=wp.get('name', '')
            )

        # IMPORTANT: Skip path calculation and use waypoints as is
        # Instead of: navigator.calculate_drone_path()
        navigator.optimized_path = navigator.waypoints

        # Save the mission plan to a file for debugging
        missions_folder = "missions"
        os.makedirs(missions_folder, exist_ok=True)
        mission_file = f"{missions_folder}/frontend_path_{int(time.time())}.json"
        navigator.save_mission_to_json(mission_file)
        logger.info(f"Saved mission plan to {mission_file}")

        # Store the navigator instance globally
        global active_navigators
        active_navigators['road_navigation'] = navigator
        logger.info("Stored navigator in active_navigators dictionary")

        # Start the mission in a background thread
        def run_mission_thread():
            try:
                logger.info(f"Attempting to connect to vehicle GPS on {gps_port}...")
                if navigator.connect_gps():
                    logger.info("Vehicle GPS connected successfully")
                else:
                    logger.error(f"Failed to connect to vehicle GPS on {gps_port}")
                    return

                logger.info("Connecting to vehicle...")
                if navigator.connect_vehicle():
                    logger.info("Setting up vehicle...")
                    if navigator.set_guided_mode() and navigator.arm_vehicle():
                        logger.info(f"Taking off to {target_altitude}m...")
                        if navigator.takeoff(target_altitude):
                            logger.info("Starting vehicle tracking mission...")
                            navigator.execute_vehicle_tracking_mission(
                                target_altitude=target_altitude,
                                target_velocity=target_velocity
                            )
                            logger.info("Mission completed")
                        else:
                            logger.error("Takeoff failed")
                    else:
                        logger.error("Failed to set up vehicle")
                else:
                    logger.error("Failed to connect to vehicle")
            except Exception as e:
                logger.error(f"Error in mission thread: {str(e)}")
                import traceback
                logger.error(traceback.format_exc())
            finally:
                # Clean up
                try:
                    navigator.cleanup()
                    logger.info("Navigator cleanup completed")
                except Exception as cleanup_error:
                    logger.error(f"Error during cleanup: {str(cleanup_error)}")

                # Remove from active navigators if it's still there
                global active_navigators
                if 'road_navigation' in active_navigators and active_navigators['road_navigation'] == navigator:
                    del active_navigators['road_navigation']
                    logger.info("Removed navigator from active_navigators dictionary")

                logger.info("Mission thread ended")

        # Start the mission thread
        mission_thread = threading.Thread(target=run_mission_thread)
        mission_thread.daemon = True
        mission_thread.start()
        logger.info("Mission thread started")

        # Return success response
        return jsonify({
            'success': True,
            'message': 'Path execution started with frontend waypoints',
            'waypoints': len(waypoints),
            'mission_file': mission_file,
            'gps_port': gps_port
        })

    except Exception as e:
        logger.error(f"Error executing path: {str(e)}")
        import traceback
        logger.error(traceback.format_exc())
        return jsonify({
            'success': False,
            'message': f'Error: {str(e)}'
        }), 500


# Add these global variables near the top of the file with other globals
car_gps_serial = None
car_gps_thread = None
car_gps_running = False


# Add these API endpoints for car GPS visualization

@app.route('/api/car_gps/connect', methods=['POST'])
def connect_to_car_gps():
    """Connect to car GPS using the specified port"""
    try:
        data = request.json
        port = data.get('port', '/tdev/ttyUSB0')  # Default to the simulator virtual port
        baudrate = int(data.get('baudrate', 9600))

        result = start_car_gps_tracking(port, baudrate)
        return jsonify(result)
    except Exception as e:
        logger.error(f"Error connecting to car GPS: {str(e)}")
        return jsonify({'success': False, 'message': f'Error: {str(e)}'})


@app.route('/api/car_gps/disconnect', methods=['POST'])
def disconnect_car_gps():
    """Disconnect from car GPS"""
    try:
        result = stop_car_gps_tracking()
        return jsonify(result)
    except Exception as e:
        logger.error(f"Error disconnecting car GPS: {str(e)}")
        return jsonify({'success': False, 'message': f'Error: {str(e)}'})


@app.route('/api/car_gps/status', methods=['GET'])
def get_car_gps_status():
    """Get the current status of car GPS tracking"""
    global car_gps_serial, car_gps_running

    return jsonify({
        'success': True,
        'connected': car_gps_running and car_gps_serial is not None,
        'port': car_gps_serial.port if car_gps_serial else None
    })


def start_car_gps_tracking(port_path, baudrate=9600):
    """Start tracking car GPS data from the specified port"""
    global car_gps_serial, car_gps_thread, car_gps_running

    # Check if already running
    if car_gps_running and car_gps_serial:
        if car_gps_serial.port == port_path:
            return {'success': True, 'message': f'Already connected to car GPS on {port_path}'}
        else:
            # Stop existing connection if we're switching ports
            stop_car_gps_tracking()

    try:
        # Attempt to open serial port
        car_gps_serial = serial.Serial(port_path, baudrate=baudrate, timeout=1)
        car_gps_running = True

        # Start reader thread
        car_gps_thread = threading.Thread(target=read_car_gps_data)
        car_gps_thread.daemon = True
        car_gps_thread.start()

        logger.info(f"Connected to car GPS on {port_path} at {baudrate} baud")
        return {'success': True, 'message': f'Connected to car GPS on {port_path}'}
    except Exception as e:
        car_gps_running = False
        car_gps_serial = None
        logger.error(f"Failed to connect to car GPS: {str(e)}")
        return {'success': False, 'message': f'Failed to connect to car GPS: {str(e)}'}


def stop_car_gps_tracking():
    """Stop car GPS tracking and close the serial connection"""
    global car_gps_serial, car_gps_thread, car_gps_running

    if not car_gps_running:
        return {'success': True, 'message': 'Car GPS tracking not active'}

    try:
        # Stop the thread
        car_gps_running = False
        if car_gps_thread:
            car_gps_thread.join(timeout=2)

        # Close the serial port
        if car_gps_serial:
            car_gps_serial.close()
            car_gps_serial = None

        logger.info("Disconnected from car GPS")
        return {'success': True, 'message': 'Disconnected from car GPS'}
    except Exception as e:
        logger.error(f"Error stopping car GPS tracking: {str(e)}")
        return {'success': False, 'message': f'Error: {str(e)}'}


def read_car_gps_data():
    """Thread function to read and parse car GPS data"""
    global car_gps_serial, car_gps_running

    try:
        # Create a UBXReader to parse both NMEA and UBX data
        reader = UBXReader(car_gps_serial, protfilter=(NMEA_PROTOCOL | UBX_PROTOCOL))

        while car_gps_running and car_gps_serial:
            try:
                # Attempt to read GPS data
                (raw_data, parsed_data) = reader.read()

                if parsed_data:
                    # Process NMEA sentences (GGA, RMC)
                    if hasattr(parsed_data, 'lat') and hasattr(parsed_data, 'lon'):
                        # Format for NMEA data
                        gps_data = {
                            'lat': parsed_data.lat,
                            'lon': parsed_data.lon,
                            'timestamp': time.time()
                        }
                        # Add altitude if available
                        if hasattr(parsed_data, 'alt'):
                            gps_data['alt'] = parsed_data.alt

                        # Emit data to clients
                        socketio.emit('message', {
                            'type': 'car_gps_data',
                            'data': gps_data
                        })

                    # Process UBX messages with position data
                    elif hasattr(parsed_data, 'LAT') and hasattr(parsed_data, 'LON'):
                        # Format for UBX data
                        gps_data = {
                            'lat': parsed_data.LAT,
                            'lon': parsed_data.LON,
                            'timestamp': time.time()
                        }
                        # Add altitude if available
                        if hasattr(parsed_data, 'HEIGHT'):
                            gps_data['alt'] = parsed_data.HEIGHT / 1000.0  # Convert mm to m

                        # Emit data to clients
                        socketio.emit('message', {
                            'type': 'car_gps_data',
                            'data': gps_data
                        })

            except Exception as e:
                logger.error(f"Error reading car GPS data: {str(e)}")
                time.sleep(1)  # Add delay on error to avoid spam

            # Small delay to prevent high CPU usage
            time.sleep(0.1)

    except Exception as e:
        logger.error(f"Fatal error in car GPS thread: {str(e)}")
        car_gps_running = False


def cleanup():
    global vehicle
    if vehicle:
        vehicle.close()
    # Clean up video streaming resources
    video_streaming.cleanup()
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
