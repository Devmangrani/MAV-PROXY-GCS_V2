import math
import json
import time
import logging
import threading
import serial
import pynmea2
import webbrowser
import os
import http.server
import socketserver
import folium
from collections import deque

# Import from the path prediction file
from path_prediction_mid_point_v2 import GeoPoint, DronePathPlanner, CarGPSReceiver

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('position_tracker.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


class RealTimePositionTracker:
    """Class to track and visualize vehicle positions in real-time using existing PathPlanner."""

    def __init__(self, car_gps_port=None, car_gps_baudrate=9600,
                 drone_connection='udp:127.0.0.1:14551',
                 update_frequency=1.0, trail_length=100):

        # Create the drone path planner with car GPS
        self.planner = DronePathPlanner(
            max_points=100,
            car_gps_port=car_gps_port,
            car_gps_baudrate=car_gps_baudrate
        )

        # Set connection string
        self.drone_connection = drone_connection

        # Position history
        self.car_positions = deque(maxlen=trail_length)
        self.drone_positions = deque(maxlen=trail_length)

        # For map update
        self.map = None
        self.html_file = "position_tracker.html"
        self.update_frequency = update_frequency
        self.running = False
        self.update_thread = None

        # HTTP server for live updates
        self.server = None
        self.server_thread = None
        self.port = 8080

        # Mission thread
        self.mission_thread = None

    def add_waypoints(self, waypoints_list):
        """Add waypoints from a list of (lat, lon, alt, name) tuples."""
        for lat, lon, alt, name in waypoints_list:
            self.planner.add_waypoint(lat, lon, alt, name)

        # Calculate paths and midpoints
        self.planner.calculate_paths()
        self.planner.calculate_midpoints()

        # Print mission summary
        logger.info(self.planner.print_mission_summary())

    def _create_base_map(self):
        """Create the base map centered at the first waypoint or current position."""
        # Determine map center
        if self.planner.waypoints:
            center = [self.planner.waypoints[0].lat, self.planner.waypoints[0].lon]
        elif self.planner.car_gps and self.planner.car_gps.has_position:
            car_pos = self.planner.car_gps.get_position()
            if car_pos:
                center = [car_pos.lat, car_pos.lon]
            else:
                # Default to Bhopal coordinates
                center = [23.191849, 77.364854]
        else:
            # Default to Bhopal coordinates
            center = [23.191849, 77.364854]

        # Create map
        m = folium.Map(location=center, zoom_start=16)

        # Add waypoints
        for wp in self.planner.waypoints:
            folium.Marker(
                location=[wp.lat, wp.lon],
                tooltip=f"Waypoint: {wp.name}",
                icon=folium.Icon(color='blue', icon='info-sign')
            ).add_to(m)

        # Add midpoints
        for mp in self.planner.midpoints:
            folium.Marker(
                location=[mp.lat, mp.lon],
                tooltip=f"Midpoint: {mp.name}",
                icon=folium.Icon(color='green', icon='flag')
            ).add_to(m)

        # Add waypoint path
        if len(self.planner.waypoints) > 1:
            waypoint_path = [(wp.lat, wp.lon) for wp in self.planner.waypoints]
            folium.PolyLine(
                waypoint_path,
                color='blue',
                weight=2,
                opacity=0.7,
                dash_array='5, 5'
            ).add_to(m)

        return m

    def _update_positions(self, m):
        """Update vehicle positions on the map."""
        # Get current car position
        car_pos = self.planner.get_car_position()
        if car_pos:
            self.car_positions.append((car_pos.lat, car_pos.lon))

            # Add car marker
            folium.Marker(
                location=[car_pos.lat, car_pos.lon],
                tooltip=f"Car: Speed={self.planner.car_gps.speed:.1f}m/s, Alt={car_pos.elevation:.1f}m",
                icon=folium.Icon(color='red', icon='car', prefix='fa')
            ).add_to(m)

            # Add car trail
            if len(self.car_positions) > 1:
                folium.PolyLine(
                    list(self.car_positions),
                    color='red',
                    weight=3,
                    opacity=0.8
                ).add_to(m)

        # Get current drone position
        drone_pos = self.planner.get_drone_position()
        if drone_pos:
            self.drone_positions.append((drone_pos.lat, drone_pos.lon))

            # Add drone marker
            folium.Marker(
                location=[drone_pos.lat, drone_pos.lon],
                tooltip=f"Drone: Alt={drone_pos.elevation:.1f}m",
                icon=folium.Icon(color='purple', icon='plane', prefix='fa')
            ).add_to(m)

            # Add drone trail
            if len(self.drone_positions) > 1:
                folium.PolyLine(
                    list(self.drone_positions),
                    color='purple',
                    weight=3,
                    opacity=0.8
                ).add_to(m)

        # Add current target indicator if we're in a mission
        if hasattr(self.planner, 'current_target_index') and self.planner.current_target_index >= 0:
            if self.planner.current_target_index < len(self.planner.midpoints):
                current_target = self.planner.midpoints[self.planner.current_target_index]
                folium.CircleMarker(
                    location=[current_target.lat, current_target.lon],
                    radius=10,
                    color='red',
                    fill=True,
                    fill_color='red',
                    fill_opacity=0.5,
                    tooltip=f"Current Target: {current_target.name}"
                ).add_to(m)

        return m

    def _update_map(self):
        """Update the map with current positions and save to HTML."""
        while self.running:
            try:
                # Create new map
                m = self._create_base_map()

                # Update positions
                m = self._update_positions(m)

                # Add mission status
                if hasattr(self.planner, 'mission_complete'):
                    status = "Complete" if self.planner.mission_complete else "In Progress"
                    # Add mission status to map
                    folium.LayerControl().add_to(m)
                    title_html = f'''
                         <h3 align="center" style="font-size:16px">
                         <b>Mission Status: {status}</b></h3>
                         '''
                    m.get_root().html.add_child(folium.Element(title_html))

                # Save map to HTML
                m.save(self.html_file)

                # Sleep for update interval
                time.sleep(1.0 / self.update_frequency)

            except Exception as e:
                logger.error(f"Error updating map: {e}")
                time.sleep(1.0)

    def _start_http_server(self):
        """Start HTTP server for serving the map HTML."""
        # Create a simple HTTP server
        handler = http.server.SimpleHTTPRequestHandler

        # Start server in separate thread
        self.server = socketserver.TCPServer(("", self.port), handler)
        self.server_thread = threading.Thread(target=self.server.serve_forever)
        self.server_thread.daemon = True
        self.server_thread.start()

        logger.info(f"HTTP server started on port {self.port}")

    def _run_mission(self):
        """Execute the car tracking mission using the path planner."""
        try:
            # Set up flight
            if self.planner.set_guided_mode() and self.planner.arm_vehicle():
                # Takeoff
                altitude = 10.0  # Default altitude in meters
                if self.planner.takeoff(altitude):
                    # Execute the car tracking mission
                    self.planner.execute_car_tracking_mission(target_altitude=altitude, target_velocity=5.0)
                else:
                    logger.error("Takeoff failed")
            else:
                logger.error("Failed to set up vehicle")
        except Exception as e:
            logger.error(f"Error in mission: {e}")
            import traceback
            logger.error(traceback.format_exc())
        finally:
            # Ensure cleanup
            if self.planner:
                self.planner.cleanup()

        delta_lambda = math.radians(lon2 - lon1)

        a = (math.sin(delta_phi / 2) * math.sin(delta_phi / 2) +
             math.cos(phi1) * math.cos(phi2) *
             math.sin(delta_lambda / 2) * math.sin(delta_lambda / 2))
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        distance = R * c
        return distance

    def _path_tracker(self):
        """Track car position and move drone to midpoints."""
        # Reset tracking parameters
        self.current_target_index = 0
        self.mission_complete = False
        self.at_midpoint = False
        self.min_distance_recorded = float('inf')
        self.distance_increasing = False
        self.min_distance_point_passed = False
        self.distance_buffer.clear()

        # Log mission start
        logger.info(f"Starting car tracking mission with {len(self.midpoints)} midpoints")

        # Check if we have midpoints
        if not self.midpoints:
            logger.error("No midpoints available for mission")
            return

        # Get initial target
        current_target = self.midpoints[self.current_target_index]
        logger.info(f"First target: {current_target}")

        # Main tracking loop
        last_log_time = 0
        log_frequency = 2.0  # 0.5 Hz

        while not self.mission_complete and self.running:
            current_time = time.time()

            # Limit logging frequency
            if current_time - last_log_time >= log_frequency:
                last_log_time = current_time

                # Get current positions
                drone_pos = self.drone_tracker.get_position()
                car_pos = self.car_gps.get_position()

                if not drone_pos or not car_pos:
                    continue

                # Get current target midpoint
                current_target = self.midpoints[self.current_target_index]

                # Calculate distances
                drone_to_midpoint = self._calculate_distance(
                    drone_pos.lat, drone_pos.lon,
                    current_target.lat, current_target.lon
                )

                car_to_midpoint = self._calculate_distance(
                    car_pos.lat, car_pos.lon,
                    current_target.lat, current_target.lon
                )

                # Log positions
                logger.info(
                    f"Drone to midpoint {self.current_target_index + 1}/{len(self.midpoints)} "
                    f"distance: {drone_to_midpoint:.2f}m"
                )
                logger.info(
                    f"Car to midpoint {self.current_target_index + 1} distance: "
                    f"{car_to_midpoint:.2f}m, Min: {self.min_distance_recorded:.2f}m"
                )

                # Move drone to current midpoint if not there
                if drone_to_midpoint > 3.0:  # If drone is not at midpoint yet (>3m away)
                    # Command drone to go to midpoint
                    self.drone_tracker.send_goto_command(
                        current_target.lat,
                        current_target.lon,
                        current_target.elevation
                    )
                    self.at_midpoint = False
                else:
                    # Drone is at midpoint
                    if not self.at_midpoint:
                        logger.info(f"Drone reached midpoint {self.current_target_index + 1}")
                        self.at_midpoint = True

                # Check if car passed the midpoint
                if self._detect_car_passing_midpoint(car_to_midpoint):
                    logger.info(f"Car passed through midpoint {self.current_target_index + 1}")

                    # Move to next midpoint
                    self.current_target_index += 1
                    self.at_midpoint = False
                    self.is_hovering = False

                    # Reset tracking parameters
                    self.min_distance_recorded = float('inf')
                    self.distance_increasing = False
                    self.min_distance_point_passed = False
                    self.distance_buffer.clear()

                    # Check if mission complete
                    if self.current_target_index >= len(self.midpoints):
                        logger.info("Mission complete! All midpoints visited by car.")
                        self.mission_complete = True
                        break

                    # Get next target
                    current_target = self.midpoints[self.current_target_index]
                    logger.info(f"Moving to next midpoint: {current_target}")

                    # Command drone to go to next midpoint
                    self.drone_tracker.send_goto_command(
                        current_target.lat,
                        current_target.lon,
                        current_target.elevation
                    )

            # Small delay to prevent CPU overuse
            time.sleep(0.05)

        logger.info("Path tracking complete")

    def start(self, run_mission=False):
        """
        Start tracking and visualization.

        Args:
            run_mission: Whether to run the car tracking mission
        """
        # Connect to car GPS
        if not self.planner.connect_car_gps():
            logger.error("Failed to connect to car GPS")
            return False

        # Connect to vehicle if needed
        if run_mission and not self.planner.connect_vehicle(self.drone_connection):
            logger.error("Failed to connect to vehicle")
            return False

        # Create initial map
        self.map = self._create_base_map()
        self.map.save(self.html_file)

        # Start HTTP server
        self._start_http_server()

        # Start map update thread
        self.running = True
        self.update_thread = threading.Thread(target=self._update_map)
        self.update_thread.daemon = True
        self.update_thread.start()

        # Start mission if requested
        if run_mission:
            self.mission_thread = threading.Thread(target=self._run_mission)
            self.mission_thread.daemon = True
            self.mission_thread.start()
            logger.info("Mission thread started")

        # Open browser
        webbrowser.open(f"http://localhost:{self.port}/{self.html_file}")

        logger.info("Real-time position tracker started")
        return True

    def stop(self):
        """Stop tracking and visualization."""
        self.running = False

        # Stop update thread
        if self.update_thread:
            self.update_thread.join(timeout=2.0)
            self.update_thread = None

        # Stop mission thread
        if self.mission_thread:
            self.mission_thread.join(timeout=2.0)
            self.mission_thread = None

        # Stop HTTP server
        if self.server:
            self.server.shutdown()
            self.server_thread.join(timeout=2.0)
            self.server = None
            self.server_thread = None

        # Clean up planner
        if self.planner:
            self.planner.cleanup()

        logger.info("Real-time position tracker stopped")


def main():
    import argparse
    import sys
    import signal

    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Real-time Position Tracker')
    parser.add_argument('--car-gps-port', type=str, default='/tmp/vgps2',
                        help='Serial port for car GPS (e.g., /tmp/vgps2)')
    parser.add_argument('--car-gps-baud', type=int, default=9600,
                        help='Baud rate for car GPS')
    parser.add_argument('--drone-connection', type=str, default='udp:127.0.0.1:14551',
                        help='MAVLink connection string for drone')
    parser.add_argument('--update-rate', type=float, default=1.0,
                        help='Map update frequency in Hz')
    parser.add_argument('--trail-length', type=int, default=100,
                        help='Length of position history trail')
    parser.add_argument('--port', type=int, default=8080,
                        help='HTTP server port')
    parser.add_argument('--run-mission', action='store_true',
                        help='Run the car tracking mission')
    parser.add_argument('--threshold', type=float, default=10.0,
                        help='Distance threshold for car detection (meters)')
    parser.add_argument('--altitude', type=float, default=10.0,
                        help='Drone altitude for mission (meters)')

    args = parser.parse_args()

    # Create tracker
    tracker = RealTimePositionTracker(
        car_gps_port=args.car_gps_port,
        car_gps_baudrate=args.car_gps_baud,
        drone_connection=args.drone_connection,
        update_frequency=args.update_rate,
        trail_length=args.trail_length
    )
    tracker.port = args.port

    # Set the distance threshold for car detection
    tracker.planner.distance_threshold = args.threshold

    # Define waypoints (Bhopal coordinates from your scripts)
    waypoints_data = [
        (23.191849, 77.364854, args.altitude, "Point A"),
        (23.190986, 77.365937, args.altitude, "Point B"),
        (23.190355, 77.367997, args.altitude, "Point C"),
        (23.192386, 77.370830, args.altitude, "Point D"),
        (23.195517, 77.371050, args.altitude, "Point E"),
        (23.197110, 77.369097, args.altitude, "Point F")
    ]

    # Add waypoints to tracker
    tracker.add_waypoints(waypoints_data)

    # Handle keyboard interrupt
    def signal_handler(sig, frame):
        logger.info("Stopping tracker...")
        tracker.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    # Start tracker with mission if requested
    if tracker.start(run_mission=args.run_mission):
        logger.info("Tracker running. Press Ctrl+C to stop.")

        # Keep main thread alive
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            logger.info("Stopping tracker...")
        finally:
            tracker.stop()
    else:
        logger.error("Failed to start tracker")


if __name__ == "__main__":
    main()