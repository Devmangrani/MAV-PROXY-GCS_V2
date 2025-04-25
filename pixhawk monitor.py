from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink
from flask import Flask, render_template
from flask_socketio import SocketIO
import threading
import time
import json
import base64

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

connection_string = '/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_1D0048000D51393239383638-if00'


def try_reassemble_message(mav, raw_data):
    mav.buf.extend(raw_data)
    try:
        m = mav.parse_char('')
        if m is not None:
            return m
    except mavlink.MAVError:
        pass
    return None

def fetch_data():
    vehicle = None
    try:
        socketio.emit('message', {'text': json.dumps({"Status": f"Attempting to connect to {connection_string}"}),
                                  'color': 'blue'})

        # Try different baud rates
        baud_rates = [115200, 57600, 921600]
        for baud in baud_rates:
            try:
                vehicle = mavutil.mavlink_connection(connection_string, baud=baud, source_system=1, source_component=1,
                                                     force_connected=True)
                socketio.emit('message',
                              {'text': json.dumps({"Status": f"Connected with baud rate {baud}"}), 'color': 'green'})
                break
            except Exception as e:
                socketio.emit('message',
                              {'text': json.dumps({"Error": f"Failed to connect with baud {baud}: {str(e)}"}),
                               'color': 'red'})

        if vehicle is None:
            socketio.emit('message',
                          {'text': json.dumps({"Error": "Failed to connect with any baud rate"}), 'color': 'red'})
            return

        mav = mavlink.MAVLink(vehicle)
        socketio.emit('message', {'text': json.dumps({"Status": "Waiting for heartbeat..."}), 'color': 'blue'})
        vehicle.wait_heartbeat(timeout=15)
        socketio.emit('message', {'text': json.dumps(
            {"Status": f"Connected to system (system {vehicle.target_system} component {vehicle.target_component})"}),
                                  'color': 'green'})

        while True:
            try:
                msg = vehicle.recv_match(blocking=True, timeout=1.0)
                if msg:
                    if msg.get_type() != "BAD_DATA":
                        message_dict = msg.to_dict()
                        # Filter out None values, format floats, and convert bytearrays to base64
                        filtered_dict = {}
                        for k, v in message_dict.items():
                            if v is not None:
                                if isinstance(v, float):
                                    filtered_dict[k] = round(v, 4)
                                elif isinstance(v, bytearray):
                                    filtered_dict[k] = base64.b64encode(v).decode('utf-8')
                                else:
                                    filtered_dict[k] = v
                        socketio.emit('message',
                                      {'text': json.dumps({msg.get_type(): filtered_dict}), 'color': 'green'})
                else:
                    socketio.emit('message', {'text': json.dumps({"Status": "No message received"}), 'color': 'gray'})
            except Exception as e:
                socketio.emit('message',
                              {'text': json.dumps({"Error": f"Error receiving message: {str(e)}"}), 'color': 'red'})

            time.sleep(0.1)

    except Exception as e:
        socketio.emit('message', {'text': json.dumps({"Error": str(e)}), 'color': 'red'})
    finally:
        if vehicle is not None:
            vehicle.close()
        socketio.emit('message', {'text': json.dumps({"Status": "Connection closed."}), 'color': 'red'})


@app.route('/')
def index():
    return render_template('index.html')


if __name__ == '__main__':
    threading.Thread(target=fetch_data, daemon=True).start()
    socketio.run(app, debug=True, host='0.0.0.0', port=5000)