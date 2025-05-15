#!/bin/bash

# Print server information
echo "======== Rodella Aerospace GCS Server Information ========"
echo "Hostname: rodellaaerospacegcs"
echo "Server will be accessible at: http://rodellaaerospacegcs.local:5000"
echo "=========================================================="

# Start the application with Gunicorn
gunicorn --worker-class eventlet \
         --workers=3 \
         --worker-connections 1000 \
         --timeout 120 \
         --max-requests 1000 \
         --max-requests-jitter 50 \
         --bind 0.0.0.0:5000 \
         functionality_added_v8:app
