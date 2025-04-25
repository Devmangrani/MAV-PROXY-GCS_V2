#!/bin/bash
# run_simple.sh - Minimal script to run the drone application

# Virtual GPS port settings
GPS_PORT="/tmp/vgps2"

# Make sure GPS port has correct permissions
echo "Setting permissions for $GPS_PORT"
if [ -e "$GPS_PORT" ]; then
    chmod 666 "$GPS_PORT" 2>/dev/null || sudo chmod 666 "$GPS_PORT"
    echo "Current permissions:"
    ls -la "$GPS_PORT"
else
    echo "Warning: $GPS_PORT does not exist."
fi

# Create missions directory
mkdir -p missions

# Set environment variables and run the app
echo "Starting application..."
export GPS_PORT="$GPS_PORT"
export GPS_BAUDRATE=9600

# Check if we should use Gunicorn or Flask
if command -v gunicorn &> /dev/null; then
    echo "Running with Gunicorn"
    gunicorn --worker-class eventlet \
         --workers=1 \
         --threads 1 \
         --worker-connections 2000 \
         --timeout 0 \
         --max-requests 1000 \
         --max-requests-jitter 50 \
         --bind 0.0.0.0:5000 \
	functionality_added_v8:app
else
    echo "Running with Flask"
    python3 functionality_added_v8.py
fi
