from socket import socket

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

connection_string = '/dev/ttyACM1'
RTSP_URL = "rtsp://192.168.144.25:8554/main.264"
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


def send_command_ack(vehicle, command_function, *args, **kwargs):
    try:
        command_function(*args, **kwargs)
        ack = vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if ack:
            result = mavutil.mavlink.enums['MAV_RESULT'][ack.result].description
            logger.info(f"Command {command_function.__name__} acknowledged with result: {result}")
            return result == "Accepted"
        else:
            logger.warning(f"Command {command_function.__name__} not acknowledged")
            return False
    except Exception as e:
        logger.error(f"Error executing command {command_function.__name__}: {str(e)}")
        return False


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


def connect_to_vehicle(connection_string):
    global vehicle
    global connection_attempts
    while connection_attempts < max_connection_attempts:
        try:
            vehicle = mavutil.mavlink_connection(connection_string)
            vehicle.wait_heartbeat()
            logger.info("Connected to vehicle")
            break
        except Exception as e:
            connection_attempts += 1
            logger.warning(f"Attempt {connection_attempts}: Connection to vehicle failed: {str(e)}")
            time.sleep(2)
    if connection_attempts == max_connection_attempts:
        logger.error("Max connection attempts reached. Exiting.")
        sys.exit(1)


threading.Thread(target=connect_to_vehicle, args=(connection_string,), daemon=True).start()


def fetch_data():
    global vehicle
    while True:
        try:
            msg = vehicle.recv_match(blocking=True)
            if msg:
                formatted_data = None
                if msg.get_type() == 'ATTITUDE':
                    formatted_data = format_attitude(json_serialize(msg.to_dict()))
                elif msg.get_type() == 'GLOBAL_POSITION_INT':
                    formatted_data = format_global_position_int(json_serialize(msg.to_dict()))
                elif msg.get_type() == 'VFR_HUD':
                    formatted_data = format_vfr_hud(json_serialize(msg.to_dict()))
                elif msg.get_type() == 'SYS_STATUS':
                    formatted_data = format_sys_status(json_serialize(msg.to_dict()))

                if formatted_data:
                    socketio.emit('message', {
                        'type': msg.get_type(),
                        'data': json.dumps(formatted_data, indent=4)
                    })
        except socket.error as e:
            if e.errno != errno.EAGAIN:
                raise
        except Exception as e:
            logger.error(f"Unexpected error while fetching data: {str(e)}")
            socketio.emit('message', {'text': f"Unexpected error while fetching data: {str(e)}", 'color': 'red'})


@app.route('/')
def index():
    return render_template('functionality-added-v2.html')


@app.route('/stream')
def stream():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


@socketio.on('connect')
def on_connect():
    print('Client connected')
    socketio.emit('message', {'text': 'Connected to server', 'color': 'green'})


@socketio.on('disconnect')
def on_disconnect():
    print('Client disconnected')


@app.route('/api/status', methods=['GET'])
def api_status():
    if not vehicle:
        return jsonify({'error': 'Not connected to any vehicle'}), 400

    status = {
        "system_status": vehicle.sysid,
        "component_status": vehicle.compid,
        "mavlink_version": vehicle.version
    }
    return jsonify(status)


@app.route('/api/set_mode', methods=['POST'])
def set_mode():
    mode = request.json.get('mode')
    if not mode:
        return jsonify({'error': 'No mode specified'}), 400

    mode_id = get_mode_id(mode)

    if send_command_ack(vehicle, vehicle.mav.set_mode_send, vehicle.target_system,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id):
        return jsonify({'message': f'Mode changed to {mode}'})
    else:
        return jsonify({'error': f'Failed to change mode to {mode}'}), 500


@app.route('/api/arm', methods=['POST'])
def arm():
    if send_command_ack(vehicle, vehicle.mav.command_long_send, vehicle.target_system, vehicle.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0):
        return jsonify({'message': 'Vehicle armed'})
    else:
        return jsonify({'error': 'Failed to arm vehicle'}), 500


@app.route('/api/disarm', methods=['POST'])
def disarm():
    if send_command_ack(vehicle, vehicle.mav.command_long_send, vehicle.target_system, vehicle.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0):
        return jsonify({'message': 'Vehicle disarmed'})
    else:
        return jsonify({'error': 'Failed to disarm vehicle'}), 500


@app.route('/api/takeoff', methods=['POST'])
def takeoff():
    altitude = request.json.get('altitude', 10)  # Default to 10 meters if not specified
    if send_command_ack(vehicle, vehicle.mav.command_long_send, vehicle.target_system, vehicle.target_component,
                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, altitude):
        return jsonify({'message': 'Takeoff initiated'})
    else:
        return jsonify({'error': 'Failed to initiate takeoff'}), 500


@app.route('/api/mission', methods=['POST'])
def mission():
    global wp_loader, current_mission

    mission_data = request.json.get('mission')
    if not mission_data:
        return jsonify({'error': 'No mission data provided'}), 400

    wp_loader.clear()
    for wp in mission_data:
        waypoint = mavutil.mavlink.MAVLink_mission_item_message(
            target_system=vehicle.target_system,
            target_component=vehicle.target_component,
            seq=wp['seq'],
            frame=wp['frame'],
            command=wp['command'],
            current=wp['current'],
            autocontinue=wp['autocontinue'],
            param1=wp['param1'],
            param2=wp['param2'],
            param3=wp['param3'],
            param4=wp['param4'],
            x=wp['x'],
            y=wp['y'],
            z=wp['z']
        )
        wp_loader.add(waypoint)

    vehicle.waypoint_clear_all_send()
    for i in range(wp_loader.count()):
        waypoint = wp_loader.wp(i)
        vehicle.mav.send(waypoint)

    current_mission = mission_data
    return jsonify({'message': 'Mission uploaded'})


@app.route('/api/set_speed', methods=['POST'])
def set_speed():
    speed = request.json.get('speed')
    if speed is None:
        return jsonify({'error': 'No speed specified'}), 400

    if send_command_ack(vehicle, vehicle.mav.command_long_send, vehicle.target_system, vehicle.target_component,
                        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 1, speed, 0, 0, 0, 0, 0):
        return jsonify({'message': f'Speed set to {speed} m/s'})
    else:
        return jsonify({'error': f'Failed to set speed to {speed} m/s'}), 500


@app.route('/api/change_altitude', methods=['POST'])
def change_altitude():
    altitude = request.json.get('altitude')
    if altitude is None:
        return jsonify({'error': 'No altitude specified'}), 400

    if send_command_ack(vehicle, vehicle.mav.command_long_send, vehicle.target_system, vehicle.target_component,
                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 3, 0, 0, 0, 0, 0, altitude):
        return jsonify({'message': f'Altitude changed to {altitude} meters'})
    else:
        return jsonify({'error': f'Failed to change altitude to {altitude} meters'}), 500


if __name__ == '__main__':
    init_wp_loader()
    socketio.start_background_task(fetch_data)
    socketio.run(app, host='0.0.0.0', port=5000)
