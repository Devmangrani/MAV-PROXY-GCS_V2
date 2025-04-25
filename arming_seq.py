from pymavlink import mavutil
from flask import Flask, render_template
from flask_socketio import SocketIO
import threading
import time
import json
import logging

app = Flask(__name__)
socketio = SocketIO(app, async_mode='threading')

connection_string = '/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_1D0048000D51393239383638-if00'
vehicle = None

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

connection_attempts = 0
max_connection_attempts = 3


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
        vehicle = mavutil.mavlink_connection(connection_string, baud=115200, autoreconnect=True)
        socketio.emit('message', {'text': "Waiting for heartbeat...", 'color': 'blue'})
        vehicle.wait_heartbeat(timeout=10)
        socketio.emit('message', {
            'text': f"Connected to system (system {vehicle.target_system} component {vehicle.target_component})",
            'color': 'green'})
        connection_attempts = 0  # Reset attempts on successful connection
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
            for msg_type in message_types:
                msg = vehicle.recv_match(type=msg_type, blocking=False)
                if msg:
                    msg_dict = msg.to_dict()
                    if msg_type == "STATUSTEXT":
                        formatted_data = {"Status": msg_dict['text']}
                    elif msg_type in globals():
                        formatted_data = globals()[f"format_{msg_type.lower()}"](msg_dict)
                    else:
                        formatted_data = {k: v for k, v in msg_dict.items() if v is not None}

                    socketio.emit('message', {
                        'type': msg_type,
                        'color': 'cyan',
                        'data': json.dumps(json_serialize(formatted_data))
                    })

            time.sleep(0.01)
        except OSError as e:
            if "returned no data" in str(e):
                logger.warning("No data received from device. Attempting to reconnect...")
                vehicle = None
            else:
                socketio.emit('message', {'text': f"Error: {str(e)}", 'color': 'red'})
                logger.error(f"Error in fetch_data: {str(e)}")
                vehicle = None
            time.sleep(5)  # Add a delay before attempting to reconnect
        except Exception as e:
            socketio.emit('message', {'text': f"Error: {str(e)}", 'color': 'red'})
            logger.error(f"Error in fetch_data: {str(e)}")
            vehicle = None
            time.sleep(250)  # Add a delay before attempting to reconnect


@socketio.on('arm')
def handle_arm():
    if not vehicle:
        socketio.emit('message', {'text': "Vehicle not connected.", 'color': 'red'})
        return

    try:
        # Set mode to GUIDED
        vehicle.mav.command_long_send(
            vehicle.target_system, vehicle.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_GUIDED_ARMED, 0, 0, 0, 0, 0, 0
        )
        time.sleep(1)

        # Send arming command
        vehicle.mav.command_long_send(
            vehicle.target_system, vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0
        )
        socketio.emit('message', {'text': "Arming command sent.", 'color': 'green'})
    except Exception as e:
        socketio.emit('message', {'text': f"Error arming vehicle: {str(e)}", 'color': 'red'})
        logger.error(f"Error arming vehicle: {str(e)}")


@app.route('/')
def index():
    return render_template('arming_seq.html')


@app.route('/reconnect')
def reconnect():
    global vehicle
    vehicle = None
    socketio.emit('message', {'text': "Manual reconnection initiated.", 'color': 'blue'})
    return "Reconnection initiated"


if __name__ == '__main__':
    threading.Thread(target=fetch_data, daemon=True).start()
    socketio.run(app, debug=True, host='0.0.0.0', port=5000)