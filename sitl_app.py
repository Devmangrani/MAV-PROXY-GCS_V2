from pymavlink import mavutil
from flask import Flask, render_template, jsonify
from flask_socketio import SocketIO
import threading
import time
import json
import logging
import sys

app = Flask(__name__)
socketio = SocketIO(app, async_mode='threading')

# Update this to the SITL connection string
connection_string = 'tcp:127.0.0.1:5760'
vehicle = None

# Set up logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

# Add a console handler to print DEBUG messages to console
console_handler = logging.StreamHandler(sys.stdout)
console_handler.setLevel(logging.DEBUG)
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
console_handler.setFormatter(formatter)
logger.addHandler(console_handler)

connection_attempts = 0
max_connection_attempts = 5


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


def connect_vehicle():
    global vehicle, connection_attempts
    try:
        if connection_attempts >= max_connection_attempts:
            socketio.emit('message', {
                'text': f"Max connection attempts reached. Please check your drone connection and restart the application.",
                'color': 'red'})
            return False

        socketio.emit('message', {'text': f"Attempting to connect to {connection_string}", 'color': 'blue'})
        logger.debug(f"Initiating connection to {connection_string}")

        vehicle = mavutil.mavlink_connection(connection_string, timeout=10)

        socketio.emit('message', {'text': "Waiting for heartbeat...", 'color': 'blue'})
        logger.debug("Waiting for heartbeat")

        heartbeat = vehicle.wait_heartbeat(timeout=15)
        if not heartbeat:
            raise Exception("Heartbeat timeout")

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
        return True
    except Exception as e:
        connection_attempts += 1
        socketio.emit('message',
                      {'text': f"Connection error: {str(e)}. Attempt {connection_attempts}/{max_connection_attempts}",
                       'color': 'red'})
        logger.error(f"Connection error: {str(e)}")
        return False


def fetch_data():
    global vehicle
    message_types = ["ATTITUDE", "GLOBAL_POSITION_INT", "VFR_HUD", "SYS_STATUS", "STATUSTEXT"]

    while True:
        if not vehicle or not vehicle.target_system:
            if not connect_vehicle():
                time.sleep(5)
                continue

        try:
            msg = vehicle.recv_match(type=message_types, blocking=True, timeout=1)
            if msg:
                msg_type = msg.get_type()
                msg_dict = msg.to_dict()
                if msg_type == "STATUSTEXT":
                    formatted_data = {"Status": msg_dict['text']}
                elif msg_type.lower() in globals():
                    formatted_data = globals()[f"format_{msg_type.lower()}"](msg_dict)
                else:
                    formatted_data = {k: v for k, v in msg_dict.items() if v is not None}

                socketio.emit('message', {
                    'type': msg_type,
                    'color': 'cyan',
                    'data': json.dumps(json_serialize(formatted_data))
                })
            else:
                logger.debug("No message received in the last second")
        except Exception as e:
            socketio.emit('message', {'text': f"Error: {str(e)}", 'color': 'red'})
            logger.error(f"Error in fetch_data: {str(e)}")
            vehicle = None
            time.sleep(5)


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


@app.route('/')
def index():
    return render_template('sitl_app.html')


@app.route('/reconnect')
def reconnect():
    global vehicle, connection_attempts
    if vehicle:
        vehicle.close()
    vehicle = None
    connection_attempts = 0
    socketio.emit('message', {'text': "Manual reconnection initiated.", 'color': 'blue'})
    return "Reconnection initiated"


@app.route('/get_gps')
def get_gps():
    if not vehicle:
        return jsonify({'error': 'Vehicle not connected'})

    try:
        msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            return jsonify({'lat': lat, 'lon': lon})
        else:
            return jsonify({'error': 'No GPS data available'})
    except Exception as e:
        return jsonify({'error': str(e)})


if __name__ == '__main__':
    threading.Thread(target=fetch_data, daemon=True).start()
    logger.info("Starting the application. Check console for DEBUG messages.")
    socketio.run(app, debug=True, host='0.0.0.0', port=5000)