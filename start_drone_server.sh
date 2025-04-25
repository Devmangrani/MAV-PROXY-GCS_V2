#!/bin/bash

# Set variables
MAVPROXY_CONNECTION="/dev/ttyACM0"  # or your serial port
MAVPROXY_OUTPUT="127.0.0.1:14550"
GUNICORN_HOST="127.0.0.1"
GUNICORN_PORT="5000"
APP_NAME="functionality_added_v6:app"

# Function to check if a process is running
is_running() {
    pgrep -f "$1" > /dev/null
}

# Kill existing processes
if is_running "mavproxy.py"; then
    echo "Killing existing MAVProxy process..."
    pkill -f "mavproxy.py"
fi

if is_running "gunicorn"; then
    echo "Killing existing Gunicorn process..."
    pkill gunicorn
fi

# Start MAVProxy
echo "Starting MAVProxy..."
mavproxy.py --master=$MAVPROXY_CONNECTION --out=$MAVPROXY_OUTPUT --daemon &

# Wait for MAVProxy to initialize
sleep 5

# Start Gunicorn
echo "Starting Gunicorn..."
gunicorn --worker-class eventlet -w 1 --bind $GUNICORN_HOST:$GUNICORN_PORT $APP_NAME

# Keep the script running
wait
