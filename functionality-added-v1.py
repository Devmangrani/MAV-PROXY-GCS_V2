from pymavlink import mavutil, mavwp
from flask import Flask, render_template, jsonify, request, Response
from flask_socketio import SocketIO
import threading
import time
import json
import logging
import sys
import errno
import random
import cv2
import numpy as np

app = Flask(__name__)
socketio = SocketIO(app, async_mode='threading')

connection_string = 'tcp:127.0.0.1:5760'
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


def fetch_data():
    global vehicle
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
                    time.sleep(60)  # Wait for a minute before trying again
                    connection_attempts = 0
                    continue

                socketio.emit('message', {'text': f"Attempting to connect to {connection_string}", 'color': 'blue'})
                logger.debug(f"Initiating connection to {connection_string}")

                vehicle = mavutil.mavlink_connection(connection_string, autoreconnect=True, timeout=60, baudrate=57600)

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
            msg = vehicle.recv_match(type=message_types, blocking=False)
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
                time.sleep(0.01)  # Small delay when no message is received

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
        vehicle.mav.command_long_send(
            vehicle.target_system, vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, altitude
        )
        socketio.emit('message', {'text': f"Takeoff command sent. Target altitude: {altitude}m", 'color': 'green'})
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
    return render_template('functionality-added-v1.html')


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


@app.route('/upload_mission')
def handle_upload_mission():
    result = upload_mission()
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


def upload_mission():
    global vehicle, wp_loader, current_mission
    if not vehicle:
        return "Vehicle not connected"

    init_wp_loader()

    for i, wp in enumerate(current_mission):
        if i == 0:
            wp_loader.add(mavutil.mavlink.MAVLink_mission_item_message(
                vehicle.target_system, vehicle.target_component,
                i, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0,
                wp['lat'], wp['lon'], wp['alt']))
        else:
            wp_loader.add(mavutil.mavlink.MAVLink_mission_item_message(
                vehicle.target_system, vehicle.target_component,
                i, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, 0, 0, 0, 0, 0,
                wp['lat'], wp['lon'], wp['alt']))

    msg = vehicle.mav.mission_count_send(
        vehicle.target_system, vehicle.target_component, wp_loader.count())

    for i in range(wp_loader.count()):
        msg = vehicle.recv_match(type=['MISSION_REQUEST'], blocking=True)
        wp_loader.send(vehicle.mav, msg.seq)

    msg = vehicle.recv_match(type=['MISSION_ACK'], blocking=True)
    if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
        return "Mission uploaded successfully"
    else:
        return f"Mission upload failed: {msg.type}"


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

    altitude = request.json['altitude']
    try:
        vehicle.mav.command_long_send(
            vehicle.target_system, vehicle.target_component,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE, 0,
            altitude, 3, 0, 0, 0, 0, 0)
        return jsonify({'message': f'Changing altitude to {altitude}m'})
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
    alt = data['alt']

    try:
        # Fetch the last known position
        msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if not msg:
            return jsonify({'error': 'Could not get current position'})

        current_lat = msg.lat / 1e7  # Convert from int to float degree
        current_lon = msg.lon / 1e7  # Convert from int to float degree

        # Set to GUIDED mode
        vehicle.mav.command_long_send(
            vehicle.target_system, vehicle.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4, 0, 0, 0, 0, 0)  # 4 is GUIDED mode

        # Send reposition command
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
            int(current_lat * 1e7),  # x: Latitude
            int(current_lon * 1e7),  # y: Longitude
            float(alt)  # z: Altitude
        )

        print(f"Reposition command sent: Lat: {current_lat}, Lon: {current_lon}, Alt: {alt}")

        return jsonify({
            'message': f'Repositioning to current position at altitude: {alt}m',
            'lat': current_lat,
            'lon': current_lon,
            'alt': alt
        })
    except Exception as e:
        print(f"Error in reposition: {str(e)}")
        return jsonify({'error': str(e)})


if __name__ == '__main__':
    threading.Thread(target=fetch_data, daemon=True).start()
    logger.info("Starting the application. Check console for DEBUG messages.")
    socketio.run(app, debug=True, host='0.0.0.0', port=5000)
