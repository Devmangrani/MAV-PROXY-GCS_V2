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

mavproxy_process = None

app = Flask(__name__)
socketio = SocketIO(app, async_mode='threading')

connection_string = 'udpin:127.0.0.1:14550'
RTSP_URL = "rtsp:192.168.144.25:8554/main.264"
vehicle = None

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

# Add a console handler to print DEBUG messages to console
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
        # Encode the frame in JPEG format
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


def get_current_mission():
    global vehicle
    if not vehicle:
        return {"error": "Vehicle not connected"}

    try:
        vehicle.mav.mission_request_list_send(vehicle.target_system, vehicle.target_component)
        msg = vehicle.recv_match(type=['MISSION_COUNT'], blocking=True, timeout=5)
        if not msg:
            return {"error": "Failed to get mission count"}

        waypoint_count = msg.count
        waypoints = []

        for i in range(waypoint_count):
            vehicle.mav.mission_request_int_send(vehicle.target_system, vehicle.target_component, i)
            msg = vehicle.recv_match(type=['MISSION_ITEM_INT'], blocking=True, timeout=5)
            if not msg:
                return {"error": f"Failed to get waypoint {i}"}

            waypoints.append({
                'seq': msg.seq,
                'frame': msg.frame,
                'command': msg.command,
                'current': msg.current,
                'autocontinue': msg.autocontinue,
                'param1': msg.param1,
                'param2': msg.param2,
                'param3': msg.param3,
                'param4': msg.param4,
                'x': msg.x,
                'y': msg.y,
                'z': msg.z
            })

        vehicle.mav.mission_ack_send(vehicle.target_system, vehicle.target_component,
                                     mavutil.mavlink.MAV_MISSION_ACCEPTED)

        return waypoints
    except Exception as e:
        return {"error": str(e)}


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
    except:
        pass
    return False


def calculate_distance(lat1, lon1, lat2, lon2):
    # Haversine formula to calculate distance between two points on Earth
    R = 6371  # Earth radius in kilometers

    d_lat = math.radians(lat2 - lat1)
    d_lon = math.radians(lon2 - lon1)
    a = (math.sin(d_lat/2) * math.sin(d_lat/2) +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
         math.sin(d_lon/2) * math.sin(d_lon/2))
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    distance = R * c

    return distance * 1000  # Convert to meters


def check_vehicle_state_for_upload():
    if not vehicle:
        return "Vehicle not connected"

    # Check if we're receiving heartbeats
    msg = vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    if not msg:
        return "No heartbeat received from vehicle"

    # Check if armed
    if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        return "Vehicle is armed. Please disarm before uploading a mission."

    # Check flight mode
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
        # Request a parameter to check communication
        vehicle.mav.param_request_read_send(vehicle.target_system, vehicle.target_component, b'SYSID_THISMAV', -1)
        msg = vehicle.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
        if not msg:
            return "Failed to receive parameter from vehicle"

        # Request heartbeat
        vehicle.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        msg = vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
        if not msg:
            return "Failed to receive heartbeat from vehicle"

        return None  # Communication is okay
    except Exception as e:
        return f"Error checking vehicle communication: {str(e)}"


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

                # Try to close existing connection if any
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


@socketio.on('takeoff')
def handle_takeoff(altitude):
    if not vehicle:
        socketio.emit('message', {'text': "Vehicle not connected.", 'color': 'red'})
        return

    try:
        # Switch to GUIDED mode
        vehicle.mav.command_long_send(
            vehicle.target_system, vehicle.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            get_mode_id('GUIDED'), 0, 0, 0, 0, 0)

        # Wait for mode change acknowledgement
        ack = vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if not ack or ack.command != mavutil.mavlink.MAV_CMD_DO_SET_MODE or ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise Exception("Failed to switch to GUIDED mode")

        # Check if vehicle is armed
        heartbeat = vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if not heartbeat or not heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            # Try to arm the vehicle
            vehicle.mav.command_long_send(
                vehicle.target_system, vehicle.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1, 0, 0, 0, 0, 0, 0)

            # Wait for arming acknowledgement
            ack = vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            if not ack or ack.command != mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM or ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                raise Exception("Failed to arm the vehicle")

        # Send takeoff command
        vehicle.mav.command_long_send(
            vehicle.target_system, vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, altitude
        )

        # Wait for takeoff acknowledgement
        ack = vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if not ack or ack.command != mavutil.mavlink.MAV_CMD_NAV_TAKEOFF or ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise Exception(f"Takeoff command failed. Result: {ack.result if ack else 'No acknowledgement'}")

        socketio.emit('message', {'text': f"Takeoff command accepted. Target altitude: {altitude}m", 'color': 'green'})
    except Exception as e:
        socketio.emit('message', {'text': f"Error during takeoff: {str(e)}", 'color': 'red'})
        logger.error(f"Error during takeoff: {str(e)}")

    # After takeoff attempt, check current altitude
    try:
        msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if msg:
            current_altitude = msg.relative_alt / 1000  # Convert mm to meters
            socketio.emit('message', {'text': f"Current altitude: {current_altitude:.2f}m", 'color': 'blue'})
        else:
            socketio.emit('message', {'text': "Unable to get current altitude", 'color': 'yellow'})
    except Exception as e:
        socketio.emit('message', {'text': f"Error getting altitude: {str(e)}", 'color': 'red'})


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


@socketio.on('set_flight_mode')
def handle_set_flight_mode(mode):
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
    return render_template('functionality-added-v3.html')


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


@app.route('/clear_mission')
def handle_clear_mission():
    return jsonify({'message': clear_mission()})


@app.route('/add_waypoint', methods=['POST'])
def handle_add_waypoint():
    data = request.json
    result = add_waypoint(data['lat'], data['lon'], data['alt'])
    return jsonify({'message': result})


@app.route('/get_mission')
def handle_get_mission():
    return jsonify(current_mission)


def clear_mission():
    global current_mission
    current_mission = []
    return "Mission cleared"


def add_waypoint(lat, lon, alt):
    global current_mission
    waypoint = {
        'lat': float(lat),
        'lon': float(lon),
        'alt': float(alt)
    }
    current_mission.append(waypoint)
    return f"Waypoint added: {waypoint}"


@app.route('/upload_mission', methods=['POST'])
def upload_mission():
    global vehicle, current_mission
    if not vehicle:
        return jsonify({'error': 'Vehicle not connected'})

    try:
        # Clear existing mission
        vehicle.mav.mission_clear_all_send(vehicle.target_system, vehicle.target_component)
        ack = vehicle.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
        if not ack or ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            return jsonify({'error': 'Failed to clear existing mission'})

        # Get mission data from the request
        mission_data = request.json.get('waypoints', [])
        if not mission_data:
            return jsonify({'error': 'No waypoints provided'})

        # Prepare the new mission
        mission_items = []
        for i, wp in enumerate(mission_data):
            if wp['command'] == 'TAKEOFF':
                cmd = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
            else:
                cmd = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT

            mission_items.append(mavutil.mavlink.MAVLink_mission_item_int_message(
                vehicle.target_system,
                vehicle.target_component,
                i,  # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                cmd,
                0,  # current
                1,  # autocontinue
                0, 0, 0, 0,  # param1-4
                int(float(wp['lat']) * 1e7),
                int(float(wp['lon']) * 1e7),
                float(wp['alt']),
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION))

        # Upload the mission
        vehicle.mav.mission_count_send(vehicle.target_system, vehicle.target_component, len(mission_items),
                                       mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

        for i in range(len(mission_items)):
            msg = vehicle.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'], blocking=True, timeout=5)
            if not msg:
                return jsonify({'error': f'Failed to receive mission request for item {i}'})

            vehicle.mav.send(mission_items[msg.seq])

        # Wait for mission ack
        ack = vehicle.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
        if not ack or ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            return jsonify({'error': f'Mission upload failed: {ack.type if ack else "No ACK received"}'})

        # Update current_mission
        current_mission = mission_data

        return jsonify({'message': 'Mission uploaded successfully', 'item_count': len(mission_items)})
    except Exception as e:
        return jsonify({'error': f'Unexpected error during mission upload: {str(e)}'})



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


@app.route('/mission_control', methods=['POST'])
def mission_control():
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
            return jsonify({'error': f'Reposition command failed. Result: {ack.result if ack else "No acknowledgement"}'})

        if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            # Wait a moment for the altitude change to begin
            time.sleep(2)

            # Check new altitude
            new_pos = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
            if new_pos:
                new_altitude = new_pos.relative_alt / 1000  # Convert mm to meters
                return jsonify({'message': f'Altitude change initiated. Current altitude: {new_altitude:.2f}m, Target altitude: {altitude}m'})
            else:
                return jsonify({'warning': 'Altitude change initiated, but unable to confirm new altitude'})
        else:
            return jsonify({'error': f'Reposition command not accepted. Result: {ack.result}'})

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
    global vehicle
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
            -1,  # param1: Speed (-1 means no change)
            0,  # param2: Radius (not used)
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
        while time.time() - start_time < 5:
            msg = vehicle.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
            if msg and msg.param_id == param_id:
                drone_parameters[param_id] = msg.param_value
                return jsonify({"message": f"Parameter {param_id} set to {msg.param_value}"})

        return jsonify({"error": "Parameter set timeout"})
    except Exception as e:
        return jsonify({"error": str(e)})


@app.route('/get_current_mission')
def handle_get_current_mission():
    mission = get_current_mission()
    return jsonify(mission)


@app.route('/get_last_mission')
def get_last_mission():
    global last_mission
    if last_mission:
        return jsonify(last_mission)
    else:
        return jsonify({'error': 'No mission available'})


@app.route('/execute_mission', methods=['POST'])
def execute_mission():
    if not vehicle:
        return jsonify({'error': 'Vehicle not connected'})

    try:
        # Verify mission first
        mission_verification = verify_mission()
        verification_result = json.loads(mission_verification.get_data(as_text=True))
        if 'error' in verification_result:
            return jsonify({'error': verification_result['error']})

        # Check if vehicle is already in the air
        if is_vehicle_in_air():
            # If in air, switch directly to AUTO mode
            vehicle.mav.command_long_send(
                vehicle.target_system, vehicle.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                get_mode_id('AUTO'), 0, 0, 0, 0, 0)
        else:
            # If on ground, arm and takeoff first
            vehicle.mav.command_long_send(
                vehicle.target_system, vehicle.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1, 0, 0, 0, 0, 0, 0)  # Arm

            ack = vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            if not ack or ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                return jsonify({'error': 'Failed to arm the vehicle'})

            # Switch to GUIDED mode and takeoff
            vehicle.mav.command_long_send(
                vehicle.target_system, vehicle.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                get_mode_id('GUIDED'), 0, 0, 0, 0, 0)

            ack = vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            if not ack or ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                return jsonify({'error': 'Failed to switch to GUIDED mode'})

            # Takeoff
            takeoff_alt = 10  # Use the same altitude as in the mission upload
            vehicle.mav.command_long_send(
                vehicle.target_system, vehicle.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                0, 0, 0, 0, 0, 0, takeoff_alt)

            ack = vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            if not ack or ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                return jsonify({'error': 'Failed to initiate takeoff'})

            # Wait for the vehicle to reach takeoff altitude
            while True:
                msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=10)
                if not msg:
                    return jsonify({'error': 'Timeout waiting for takeoff'})
                if msg.relative_alt >= takeoff_alt * 1000 * 0.95:  # 95% of target altitude
                    break

            # Now switch to AUTO mode to start the mission
            vehicle.mav.command_long_send(
                vehicle.target_system, vehicle.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                get_mode_id('AUTO'), 0, 0, 0, 0, 0)

        ack = vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if not ack or ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            return jsonify({'error': 'Failed to switch to AUTO mode'})

        return jsonify({'message': 'Mission execution started successfully'})
    except Exception as e:
        return jsonify({'error': f'Unexpected error: {str(e)}'})


@app.route('/verify_mission')
def verify_mission():
    if not vehicle:
        return jsonify({'error': 'Vehicle not connected'})

    try:
        # Check vehicle communication first
        comm_check = check_vehicle_communication()
        if comm_check:
            return jsonify({'error': comm_check})

        # Request mission count
        for attempt in range(3):
            vehicle.mav.mission_request_list_send(vehicle.target_system, vehicle.target_component)
            start_time = time.time()
            while time.time() - start_time < 5:  # 5-second timeout
                msg = vehicle.recv_match(type=['MISSION_COUNT'], blocking=True, timeout=1)
                if msg:
                    mission_count = msg.count
                    print(f"Received mission count: {mission_count}")
                    break
            if msg:
                break
            print(f"Attempt {attempt + 1} failed to receive mission count")

        if not msg:
            return jsonify({'error': 'Failed to receive mission count after multiple attempts'})

        if mission_count == 0:
            return jsonify({'error': 'No mission items found on the vehicle'})

        mission_items = []
        for i in range(mission_count):
            for attempt in range(3):  # Try up to 3 times for each item
                vehicle.mav.mission_request_int_send(vehicle.target_system, vehicle.target_component, i)
                msg = vehicle.recv_match(type=['MISSION_ITEM_INT'], blocking=True, timeout=5)
                if msg:
                    mission_items.append({
                        'seq': msg.seq,
                        'frame': msg.frame,
                        'command': msg.command,
                        'current': msg.current,
                        'autocontinue': msg.autocontinue,
                        'param1': msg.param1,
                        'param2': msg.param2,
                        'param3': msg.param3,
                        'param4': msg.param4,
                        'x': msg.x / 1e7,  # Convert from 1e7 int to float degree
                        'y': msg.y / 1e7,  # Convert from 1e7 int to float degree
                        'z': msg.z,
                    })
                    break
                else:
                    print(f"Attempt {attempt + 1} failed to receive mission item {i}")

            if not msg:
                return jsonify({'error': f'Failed to receive mission item {i} after multiple attempts'})

        # Send mission ack
        vehicle.mav.mission_ack_send(vehicle.target_system, vehicle.target_component,
                                     mavutil.mavlink.MAV_MISSION_ACCEPTED)

        # Check for takeoff command
        has_takeoff = any(item['command'] == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF for item in mission_items)
        if not has_takeoff:
            return jsonify({'error': 'Mission is missing a takeoff command'})

        # Check for valid waypoints
        valid_waypoints = [item for item in mission_items if item['command'] in [mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                                                                 mavutil.mavlink.MAV_CMD_NAV_TAKEOFF]]
        if len(valid_waypoints) < 2:  # At least takeoff and one waypoint
            return jsonify({'error': 'Mission should have at least one waypoint after takeoff'})

        # Check for reasonable altitude values
        max_alt = max(item['z'] for item in valid_waypoints)
        if max_alt > 122:  # 400 feet, adjust as needed
            return jsonify(
                {'warning': f'Mission includes high altitude of {max_alt}m. Please verify if this is intended.'})

        # Check for large distance between waypoints
        warnings = []
        for i in range(1, len(valid_waypoints)):
            prev = valid_waypoints[i - 1]
            curr = valid_waypoints[i]
            distance = calculate_distance(prev['x'], prev['y'], curr['x'], curr['y'])
            if distance > 1000:  # 1 km, adjust as needed
                warnings.append(
                    f'Large distance of {distance:.2f}m between waypoints {i} and {i + 1}. Please verify if this is intended.')

        return jsonify({
            'message': 'Mission verification successful',
            'items': mission_items,
            'valid_waypoints': len(valid_waypoints),
            'max_altitude': max_alt,
            'warnings': warnings
        })

    except Exception as e:
        return jsonify({'error': f'Unexpected error during mission verification: {str(e)}'})

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

if __name__ == '__main__':
    try:
        mavproxy_process = start_mavproxy()
        threading.Thread(target=fetch_data, daemon=True).start()
        logger.info("Starting the application. Check console for DEBUG messages.")
        socketio.run(app, debug=True, host='0.0.0.0', port=5000, allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        if mavproxy_process:
            os.killpg(os.getpgid(mavproxy_process.pid), signal.SIGTERM)
        print("MAVProxy process terminated.")
