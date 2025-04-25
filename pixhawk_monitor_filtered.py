from pymavlink import mavutil
from flask import Flask, render_template
from flask_socketio import SocketIO
import threading
import time
import json

app = Flask(__name__)
socketio = SocketIO(app)

connection_string = '/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_1D0048000D51393239383638-if00'

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
        "Ground Speed": f"{((msg_dict['vx']**2 + msg_dict['vy']**2)**0.5) / 100:.2f} m/s",
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
        "CPU Load": f"{msg_dict['load'] / 10:.1f}%"
    }

def format_gps_raw_int(msg_dict):
    fix_type = ["No GPS", "No Fix", "2D Fix", "3D Fix", "DGPS", "RTK Float", "RTK Fixed", "Static", "PPP"][msg_dict['fix_type']]
    return {
        "Fix Type": fix_type,
        "Satellites Visible": msg_dict['satellites_visible'],
        "GPS HDOP": f"{msg_dict['eph'] / 100:.2f}",
        "GPS VDOP": f"{msg_dict['epv'] / 100:.2f}"
    }

def fetch_data():
    try:
        socketio.emit('message', {'text': f"Attempting to connect to {connection_string}", 'color': 'blue'})
        vehicle = mavutil.mavlink_connection(connection_string, baud=115200)
        socketio.emit('message', {'text': "Waiting for heartbeat...", 'color': 'blue'})
        vehicle.wait_heartbeat(timeout=10)
        socketio.emit('message', {
            'text': f"Connected to system (system {vehicle.target_system} component {vehicle.target_component})",
            'color': 'green'})

        while True:
            msg = vehicle.recv_match(blocking=True, timeout=1.0)
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
                elif msg_type == "GPS_RAW_INT":
                    formatted_data = format_gps_raw_int(msg_dict)
                elif msg_type == "STATUSTEXT":
                    formatted_data = {"Status": msg_dict['text']}
                else:
                    formatted_data = {k: v for k, v in msg_dict.items() if v is not None}

                socketio.emit('message', {
                    'type': msg_type,
                    'color': 'cyan',
                    'data': json.dumps(formatted_data)
                })
            else:
                socketio.emit('message', {'text': "No message received", 'color': 'gray'})

            time.sleep(0.01)

    except Exception as e:
        socketio.emit('message', {'text': f"Error: {e}", 'color': 'red'})
    finally:
        if 'vehicle' in locals():
            vehicle.close()
        socketio.emit('message', {'text': "Connection closed.", 'color': 'red'})

@app.route('/')
def index():
    return render_template('index2.html')

if __name__ == '__main__':
    threading.Thread(target=fetch_data, daemon=True).start()
    socketio.run(app, debug=True, host='0.0.0.0', port=5000)