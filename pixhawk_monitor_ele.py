from pymavlink import mavutil
from flask import Flask, render_template
from flask_socketio import SocketIO
import threading
import time
import json

app = Flask(__name__)
socketio = SocketIO(app)

connection_string = '/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_1D0048000D51393239383638-if00'

def fetch_data():
    try:
        socketio.emit('message', {'text': f"Attempting to connect to {connection_string}", 'color': 'blue'})
        vehicle = mavutil.mavlink_connection(connection_string, baud=115200)
        socketio.emit('message', {'text': "Waiting for heartbeat...", 'color': 'blue'})
        vehicle.wait_heartbeat(timeout=10)
        socketio.emit('message', {
            'text': f"Connected to system (system {vehicle.target_system} component {vehicle.target_component})",
            'color': 'green'})

        bad_data_count = 0
        while True:
            msg = vehicle.recv_match(blocking=True, timeout=1.0)
            if msg:
                msg_type = msg.get_type()
                if msg_type == "BAD_DATA":
                    bad_data_count += 1
                    if bad_data_count % 10 == 0:  # Log every 10th BAD_DATA message
                        try:
                            raw_data = msg.get_msgbuf()
                            socketio.emit('message', {
                                'text': f"BAD_DATA received. Raw data: {' '.join([hex(x) for x in raw_data])}",
                                'color': 'red'})
                        except:
                            socketio.emit('message',
                                          {'text': f"BAD_DATA received. Unable to get raw data.", 'color': 'red'})
                else:
                    bad_data_count = 0  # Reset counter when we get a good message
                    msg_dict = msg.to_dict()
                    socketio.emit('message', {
                        'type': msg_type,
                        'color': 'cyan',
                        'data': json.dumps(msg_dict)
                    })

                    if msg_type == "STATUSTEXT":
                        text = msg.text.decode('ascii', 'ignore').strip()
                        socketio.emit('message', {'text': f"STATUSTEXT: {text}", 'color': 'white'})
            else:
                socketio.emit('message', {'text': "No message received", 'color': 'gray'})

            time.sleep(0.1)

    except Exception as e:
        socketio.emit('message', {'text': f"Error: {e}", 'color': 'red'})
    finally:
        if 'vehicle' in locals():
            vehicle.close()
        socketio.emit('message', {'text': "Connection closed.", 'color': 'red'})

@app.route('/')
def index():
    return render_template('index1.html')

if __name__ == '__main__':
    threading.Thread(target=fetch_data, daemon=True).start()
    socketio.run(app, debug=True, host='0.0.0.0', port=5000)