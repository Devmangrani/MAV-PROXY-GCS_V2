#!/bin/bash
# Simple script to run the GPS simulator and connect to the follower

# Step 1: Install socat if needed
if ! command -v socat &> /dev/null; then
    echo "Installing socat..."
    sudo apt-get update && sudo apt-get install -y socat
fi

# Step 2: Set up virtual serial ports
PORT1="/tmp/vgps1"
PORT2="/tmp/vgps2"

# Remove old ports if they exist
[ -e "$PORT1" ] && rm "$PORT1"
[ -e "$PORT2" ] && rm "$PORT2"

# Create virtual serial ports
echo "Creating virtual serial ports..."
socat -d -d pty,link=$PORT1,raw,echo=0 pty,link=$PORT2,raw,echo=0 &
SOCAT_PID=$!

# Give socat time to set up
sleep 1

# Step 3: Start the GPS simulator
echo "Starting GPS simulator..."
python ublox_m8n_sim.py --port $PORT1 --lat 37.7749 --lon -122.4194 --pattern circle --radius 50 --rate 2.0 &
GPS_PID=$!

# Step 4: Show instructions for the user
echo ""
echo "==== GPS Simulator Running ===="
echo "The GPS simulator is now running and sending NMEA data to a virtual port."
echo ""
echo "To connect your follower code, run:"
echo "python sitl_follow_test_v5.py --gps-port=$PORT2"
echo ""
echo "Press CTRL+C to stop the simulator."
echo "==============================="

# Wait for user to stop
trap "kill $SOCAT_PID $GPS_PID; echo 'Simulator stopped.'; exit 0" INT
wait
