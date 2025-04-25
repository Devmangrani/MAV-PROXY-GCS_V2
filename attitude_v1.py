from pymavlink import mavutil
from flask import Flask, render_template
from flask_socketio import SocketIO
import threading
import time
import json
import random
import errno
import logging

app = Flask(__name__)
socketio = SocketIO(app)
connection_string = '/dev/ttyACM0'

# Set up logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

# Global vehicle variable
vehicle = None

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
    global vehicle
    backoff_time = 0.1
    max_backoff_time = 5
    connection_attempts = 0
    max_connection_attempts = 500

    while True:
        if not vehicle or not getattr(vehicle, 'target_system', None):
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
            msg = vehicle.recv_match(type='ATTITUDE', blocking=False)
            if msg:
                msg_dict = msg.to_dict()
                formatted_data = format_attitude(msg_dict)
                socketio.emit('attitude', formatted_data)
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

@app.route('/')
def index():
    return render_template('index_attitude.html')

if __name__ == '__main__':
    threading.Thread(target=fetch_data, daemon=True).start()
    socketio.run(app, debug=True, host='0.0.0.0', port=5000)