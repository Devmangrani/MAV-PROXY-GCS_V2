from pymavlink import mavutil
from flask import Flask, render_template, jsonify
from flask_socketio import SocketIO
import threading
import time
import json
import logging

# Set up logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

# Flask and SocketIO setup
app = Flask(__name__)
socketio = SocketIO(app, async_mode='threading')

# Global variables
vehicle = None
connection_string = 'tcp:127.0.0.1:5760'


def connect_vehicle():
    global vehicle
    try:
        logger.info(f"Attempting to connect to {connection_string}")
        vehicle = mavutil.mavlink_connection(connection_string, autoreconnect=True, timeout=60)
        logger.info("Waiting for heartbeat")
        vehicle.wait_heartbeat(timeout=30)
        logger.info("Connected successfully")
        return True
    except Exception as e:
        logger.error(f"Connection failed: {str(e)}")
        return False


def fetch_data():
    global vehicle
    while True:
        if not vehicle or not vehicle.target_system:
            if not connect_vehicle():
                time.sleep(5)
                continue

        try:
            msg = vehicle.recv_match(blocking=True, timeout=5)
            if msg:
                msg_type = msg.get_type()
                msg_dict = msg.to_dict()
                logger.debug(f"Received message: {msg_type}")
                socketio.emit('drone_data', {'type': msg_type, 'data': json.dumps(msg_dict)})
            else:
                logger.warning("No message received")
                socketio.emit('message', {'text': "No data received from drone", 'color': 'orange'})
        except Exception as e:
            logger.error(f"Error in fetch_data: {str(e)}")
            vehicle = None
            socketio.emit('message', {'text': f"Error: {str(e)}", 'color': 'red'})
            time.sleep(5)


@app.route('/')
def index():
    return render_template('drone-control.html')


@socketio.on('connect')
def handle_connect():
    logger.info("Client connected")
    socketio.emit('message', {'text': "Connected to server", 'color': 'green'})


@socketio.on('disconnect')
def handle_disconnect():
    logger.info("Client disconnected")


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


if __name__ == '__main__':
    # Start the data fetching thread
    threading.Thread(target=fetch_data, daemon=True).start()

    # Run the Flask application
    socketio.run(app, debug=True, host='0.0.0.0', port=5000)