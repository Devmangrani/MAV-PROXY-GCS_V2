from pymavlink import mavutil
from flask import Flask, render_template
from flask_socketio import SocketIO
import json
import threading
import time

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

# Global variable to store the connection
connection = None

def establish_connection():
    global connection
    print("Attempting to connect...")
    try:
        connection = mavutil.mavlink_connection('/dev/ttyACM0', baudrate=115200)
        print("Connection object created")
        connection.wait_heartbeat(timeout=10)
        print("Heartbeat received")
        return True
    except Exception as e:
        print(f"Connection failed: {e}")
        return False

def fetch_data():
    global connection
    while True:
        if connection is None or not establish_connection():
            print("No connection. Retrying in 5 seconds...")
            time.sleep(5)
            continue

        try:
            msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if msg:
                data = {
                    'lat': msg.lat / 1e7,
                    'lon': msg.lon / 1e7,
                    'alt': msg.alt / 1000.0
                }
                print(f"Received data: {data}")  # For debugging
                socketio.emit('gps_data', json.dumps(data))
            else:
                print("No message received")
        except Exception as e:
            print(f"Error in fetch_data: {e}")
            connection = None  # Reset connection to attempt reconnection
        time.sleep(1)

@app.route('/')
def index():
    return render_template('index.html')

if __name__ == '__main__':
    threading.Thread(target=fetch_data, daemon=True).start()
    socketio.run(app, debug=True, host='0.0.0.0', port=5000)
