from pymavlink import mavutil
from flask import Flask, render_template
from flask_socketio import SocketIO
import threading
import time
import json

app = Flask(__name__)
socketio = SocketIO(app)
connection_string = '/dev/ttyACM0'


def format_attitude(msg_dict):
    return {
        "Roll": f"{msg_dict['roll']:.2f}°",
        "Pitch": f"{msg_dict['pitch']:.2f}°",
        "Yaw": f"{msg_dict['yaw']:.2f}°",
        "Roll Speed": f"{msg_dict['rollspeed']:.4f} rad/s",
        "Pitch Speed": f"{msg_dict['pitchspeed']:.4f} rad/s",
        "Yaw Speed": f"{msg_dict['yawspeed']:.4f} rad/s"
    }


def fetch_data():
    try:
        vehicle = mavutil.mavlink_connection(connection_string, autoreconnect=True, timeout=60)
        vehicle.wait_heartbeat()

        while True:
            msg = vehicle.recv_match(type='ATTITUDE', blocking=True)
            if msg:
                msg_dict = msg.to_dict()
                formatted_data = format_attitude(msg_dict)
                socketio.emit('attitude', formatted_data)

            time.sleep(0.01)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'vehicle' in locals():
            vehicle.close()


@app.route('/')
def index():
    return render_template('index_attitude.html')


if __name__ == '__main__':
    threading.Thread(target=fetch_data, daemon=True).start()
    socketio.run(app, debug=True, host='0.0.0.0', port=5000)