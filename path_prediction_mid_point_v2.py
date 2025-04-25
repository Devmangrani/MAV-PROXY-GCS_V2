import math
import json
import time
import logging
import threading
import serial
import pynmea2
from pymavlink import mavutil
from collections import deque
import os

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('car_tracking_mission.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


class GeoPoint:
    """Class to represent a geographical point with latitude, longitude, and optional elevation."""

    def __init__(self, lat, lon, elevation=0, name=""):
        self.lat = lat
        self.lon = lon
        self.elevation = elevation
        self.name = name

    def distance_to(self, other_point):
        """Calculate the distance between this point and another using the Haversine formula."""
        R = 6371000  # Earth radius in meters

        phi1 = math.radians(self.lat)
        phi2 = math.radians(other_point.lat)
        delta_phi = math.radians(other_point.lat - self.lat)
        delta_lambda = math.radians(other_point.lon - self.lon)

        a = (math.sin(delta_phi / 2) * math.sin(delta_phi / 2) +
             math.cos(phi1) * math.cos(phi2) *
             math.sin(delta_lambda / 2) * math.sin(delta_lambda / 2))
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        distance = R * c
        return distance

    def midpoint_to(self, other_point):
        """Find the midpoint between this point and another point."""
        phi1 = math.radians(self.lat)
        lambda1 = math.radians(self.lon)
        phi2 = math.radians(other_point.lat)
        lambda2 = math.radians(other_point.lon)

        Bx = math.cos(phi2) * math.cos(lambda2 - lambda1)
        By = math.cos(phi2) * math.sin(lambda2 - lambda1)

        phi3 = math.atan2(math.sin(phi1) + math.sin(phi2),
                          math.sqrt((math.cos(phi1) + Bx) * (math.cos(phi1) + Bx) + By * By))
        lambda3 = lambda1 + math.atan2(By, math.cos(phi1) + Bx)

        mid_lat = math.degrees(phi3)
        mid_lon = math.degrees(lambda3)

        # For elevation, take the average
        mid_elev = (self.elevation + other_point.elevation) / 2

        name = f"Midpoint {self.name}-{other_point.name}"
        return GeoPoint(mid_lat, mid_lon, mid_elev, name)

    def bearing_to(self, other_point):
        """Calculate bearing from this point to another point"""
        lat1 = math.radians(self.lat)
        lon1 = math.radians(self.lon)
        lat2 = math.radians(other_point.lat)
        lon2 = math.radians(other_point.lon)

        y = math.sin(lon2 - lon1) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(lon2 - lon1)
        bearing = math.atan2(y, x)

        # Convert to degrees
        bearing = math.degrees(bearing)
        # Normalize to 0-360
        bearing = (bearing + 360) % 360

        return bearing

    def __str__(self):
        return f"{self.name} ({self.lat:.6f}, {self.lon:.6f}, {self.elevation:.1f}m)"

    def to_dict(self):
        """Convert the point to a dictionary for JSON serialization."""
        return {
            "name": self.name,
            "lat": self.lat,
            "lon": self.lon,
            "elevation": self.elevation
        }


class CarGPSReceiver:
    """Class to handle the GPS receiver in the car."""

    def __init__(self, port='/dev/ttyUSB0', baudrate=9600):
        """Initialize the GPS receiver."""
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.running = False
        self.thread = None

        # Current position data
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.speed = 0.0
        self.heading = 0.0
        self.fix_quality = 0
        self.num_satellites = 0

        # Position state
        self.has_position = False
        self.last_update_time = 0

    def connect(self):
        """Connect to the GPS device."""
        try:
            # Close any existing connection first
            if self.serial_conn:
                try:
                    self.serial_conn.close()
                except:
                    pass
                self.serial_conn = None
                # Small delay to ensure port is released
                time.sleep(0.5)

            # Check if the port exists
            if not os.path.exists(self.port):
                logger.error(f"GPS port {self.port} does not exist")
                return False

            # Use a longer timeout for initial connection (5 seconds)
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=5)

            # Clear any stale data in the buffer
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()

            # Verify the connection works by trying to read a bit of data
            try:
                self.serial_conn.timeout = 1  # Shorter timeout for test read
                test_data = self.serial_conn.read(10)  # Try to read 10 bytes
                # Success even if we don't get data - connection is active
            except serial.SerialException as se:
                if "device reports readiness to read but returned no data" in str(se):
                    # This specific error often occurs on initial connection
                    # We'll ignore it and consider the connection successful anyway
                    logger.warning("Device reported readiness but no data during connection test - continuing anyway")
                else:
                    raise  # Re-raise other serial exceptions

            logger.info(f"Connected to car GPS on {self.port} at {self.baudrate} baud")
            return True

        except serial.SerialException as e:
            logger.error(f"Serial exception connecting to car GPS: {e}")
            if self.serial_conn:
                try:
                    self.serial_conn.close()
                except:
                    pass
                self.serial_conn = None
            return False

        except Exception as e:
            logger.error(f"Failed to connect to car GPS: {e}")
            if self.serial_conn:
                try:
                    self.serial_conn.close()
                except:
                    pass
                self.serial_conn = None
            return False

    def start(self):
        """Start the GPS receiver thread."""
        if self.running:
            logger.warning("Car GPS receiver already running")
            return False

        # Connect if not already connected
        if not self.serial_conn:
            if not self.connect():
                return False

        # Start the receiver thread
        self.running = True
        self.thread = threading.Thread(target=self._read_gps_data)
        self.thread.daemon = True
        self.thread.start()

        logger.info("Car GPS receiver started")
        return True

    def stop(self):
        """Stop the GPS receiver."""
        self.running = False

        if self.thread:
            self.thread.join(timeout=2.0)
            self.thread = None

        if self.serial_conn:
            self.serial_conn.close()
            self.serial_conn = None

        logger.info("Car GPS receiver stopped")

    def _read_gps_data(self):
        """Read and parse NMEA sentences from the GPS."""
        consecutive_errors = 0
        max_consecutive_errors = 5
        empty_reads = 0
        max_empty_reads = 10

        while self.running:
            try:
                # Check if serial connection is still valid
                if not self.serial_conn or not self.serial_conn.is_open:
                    logger.warning("Serial connection lost or not open. Attempting to reconnect...")
                    try:
                        if self.serial_conn:
                            self.serial_conn.close()
                        self.serial_conn = None

                        # Try to reconnect
                        if self.connect():
                            logger.info("Reconnected to car GPS")
                            consecutive_errors = 0
                            empty_reads = 0
                    except Exception as e:
                        logger.error(f"Failed to reconnect to car GPS: {e}")

                    time.sleep(1)
                    continue

                # Check if data is available to read
                try:
                    in_waiting = self.serial_conn.in_waiting
                    if in_waiting > 0:
                        # Use a shorter timeout for reading to avoid blocking too long
                        self.serial_conn.timeout = 0.5
                        line = self.serial_conn.readline()

                        # Handle the case where readline() returns empty data even though in_waiting > 0
                        if not line:
                            empty_reads += 1
                            if empty_reads > max_empty_reads:
                                logger.warning(
                                    f"Received {empty_reads} consecutive empty reads. Resetting connection...")
                                self.serial_conn.close()
                                self.serial_conn = None
                                empty_reads = 0
                                time.sleep(1)
                                continue
                        else:
                            # Successfully read data, reset empty reads counter
                            empty_reads = 0

                            try:
                                line_decoded = line.decode('ascii', errors='replace').strip()
                                if line_decoded:
                                    self._parse_nmea(line_decoded)
                                    consecutive_errors = 0  # Reset error count on successful read
                            except UnicodeDecodeError:
                                # Just ignore decode errors and continue
                                pass
                    else:
                        # No data available, just sleep briefly
                        time.sleep(0.1)
                except serial.SerialException as se:
                    if "device reports readiness to read but returned no data" in str(se):
                        logger.warning("Device reported readiness but returned no data. Resetting connection...")
                        # This specific error requires a reset of the connection
                        try:
                            self.serial_conn.close()
                        except:
                            pass
                        self.serial_conn = None
                        time.sleep(1)
                    else:
                        logger.error(f"Serial exception while reading GPS data: {se}")
                        self.serial_conn.close()
                        self.serial_conn = None
                        time.sleep(1)

            except Exception as e:
                logger.error(f"Error reading car GPS data: {e}")
                consecutive_errors += 1

                if consecutive_errors > max_consecutive_errors:
                    logger.warning("Too many consecutive errors. Resetting connection...")
                    try:
                        if self.serial_conn:
                            self.serial_conn.close()
                    except:
                        pass
                    self.serial_conn = None
                    consecutive_errors = 0

                time.sleep(0.5)

    def _parse_nmea(self, nmea_str):
        """Parse NMEA sentence and update position data."""
        try:
            if not nmea_str.startswith('$'):
                return

            msg = pynmea2.parse(nmea_str)

            # Parse GGA message (position fix)
            if isinstance(msg, pynmea2.GGA):
                if msg.latitude and msg.longitude:
                    self.lat = msg.latitude
                    self.lon = msg.longitude
                    self.alt = float(msg.altitude) if msg.altitude else 0.0
                    self.fix_quality = int(msg.gps_qual) if msg.gps_qual else 0
                    self.num_satellites = int(msg.num_sats) if msg.num_sats else 0
                    self.has_position = True
                    self.last_update_time = time.time()

            # Parse RMC message (recommended minimum data)
            elif isinstance(msg, pynmea2.RMC):
                if msg.status == 'A':  # A=Active, V=Void
                    self.lat = msg.latitude
                    self.lon = msg.longitude
                    self.speed = float(
                        msg.spd_over_grnd) * 0.514444 if msg.spd_over_grnd else 0.0  # Convert knots to m/s
                    self.heading = float(msg.true_course) if msg.true_course else 0.0
                    self.has_position = True
                    self.last_update_time = time.time()

            # Parse VTG message (track made good and speed over ground)
            elif isinstance(msg, pynmea2.VTG):
                if msg.spd_over_grnd_kmph:
                    self.speed = float(msg.spd_over_grnd_kmph) / 3.6  # Convert km/h to m/s
                if msg.true_track:
                    self.heading = float(msg.true_track)

        except Exception as e:
            logger.debug(f"Error parsing NMEA sentence: {e}")

    def get_position(self):
        """Get the current GPS position."""
        if not self.has_position:
            return None

        # Check if position is stale (no updates in 5 seconds)
        if time.time() - self.last_update_time > 5.0:
            logger.warning("Car GPS position data is stale")

        return GeoPoint(self.lat, self.lon, self.alt, "Car")


class DronePathPlanner:
    """Class to plan drone paths between waypoints and calculate midpoints."""

    def __init__(self, max_points=100, car_gps_port=None, car_gps_baudrate=9600):
        """
        Initialize the drone path planner.

        Args:
            max_points: Maximum number of waypoints to plan
            car_gps_port: Serial port for car GPS (e.g., '/dev/ttyACM0')
            car_gps_baudrate: Baud rate for car GPS connection
        """
        self.waypoints = []
        self.paths = []
        self.midpoints = []
        self.max_points = max_points
        self.current_target_index = -1
        self.mission_complete = False
        self.distance_threshold = 10.0  # Meters - threshold for car detection
        self.vehicle = None

        # Car GPS setup
        self.car_gps = None
        if car_gps_port:
            self.car_gps = CarGPSReceiver(port=car_gps_port, baudrate=car_gps_baudrate)

        # For midpoint detection
        self.min_distance_recorded = float('inf')
        self.distance_increasing = False
        self.min_distance_point_passed = False
        self.distance_buffer = deque(maxlen=5)  # Store last 5 distances for smoothing

        # For drone position at midpoint
        self.at_midpoint = False
        self.is_hovering = False

    def add_waypoint(self, lat, lon, elevation=0, name=None):
        """Add a waypoint to the planner."""
        if len(self.waypoints) >= self.max_points:
            raise ValueError(f"Maximum number of waypoints ({self.max_points}) reached")

        if name is None:
            name = f"Point {len(self.waypoints) + 1}"

        point = GeoPoint(lat, lon, elevation, name)
        self.waypoints.append(point)

        # Reset calculated paths and midpoints when adding a new waypoint
        self.paths = []
        self.midpoints = []

        return point

    def calculate_paths(self):
        """Calculate shortest paths between consecutive waypoints."""
        self.paths = []

        if len(self.waypoints) < 2:
            return self.paths

        for i in range(len(self.waypoints) - 1):
            start = self.waypoints[i]
            end = self.waypoints[i + 1]

            # Calculate straight-line distance
            distance = start.distance_to(end)
            midpoint = start.midpoint_to(end)

            self.paths.append({
                "start": start,
                "end": end,
                "distance": distance,
                "midpoint": midpoint
            })

        return self.paths

    def calculate_midpoints(self):
        """Calculate midpoints for all paths."""
        if not self.paths:
            self.calculate_paths()

        self.midpoints = [path["midpoint"] for path in self.paths]
        return self.midpoints

    def connect_vehicle(self, connection_string='udp:127.0.0.1:14551'):
        """Connect to the vehicle using MAVLink."""
        logger.info(f"Connecting to vehicle on {connection_string}")
        try:
            self.vehicle = mavutil.mavlink_connection(connection_string)
            self.vehicle.wait_heartbeat()
            logger.info("Connected to vehicle")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to vehicle: {e}")
            return False

    def connect_car_gps(self):
        """Connect to the car GPS device."""
        if not self.car_gps:
            logger.error("No car GPS configured")
            return False

        if self.car_gps.start():
            logger.info("Car GPS started")
            return True
        else:
            logger.error("Failed to start car GPS")
            return False

    def print_mission_summary(self):
        """Print a summary of the mission."""
        total_distance = sum(path["distance"] for path in self.paths)

        summary = f"Mission with {len(self.waypoints)} waypoints and {len(self.midpoints)} midpoints\n"
        summary += f"Total path distance: {total_distance / 1000:.2f} km\n\n"

        summary += "Waypoints:\n"
        for wp in self.waypoints:
            summary += f"- {wp}\n"

        summary += "\nPath Segments:\n"
        for i, path in enumerate(self.paths):
            summary += f"- Path {i + 1}: {path['start'].name} to {path['end'].name} ({path['distance'] / 1000:.2f} km)\n"
            summary += f"  Midpoint: {path['midpoint']}\n"

        return summary

    def save_mission_to_json(self, filename):
        """Save the mission plan to a JSON file."""
        # Convert to serializable format
        serializable_waypoints = [wp.to_dict() for wp in self.waypoints]
        serializable_paths = [{
            "start": path["start"].to_dict(),
            "end": path["end"].to_dict(),
            "distance": path["distance"],
            "midpoint": path["midpoint"].to_dict()
        } for path in self.paths]

        mission_data = {
            "waypoints": serializable_waypoints,
            "paths": serializable_paths,
            "midpoints": [mp.to_dict() for mp in self.midpoints],
        }

        with open(filename, 'w') as f:
            json.dump(mission_data, f, indent=2)

        logger.info(f"Mission saved to {filename}")

    def load_mission_from_json(self, filename):
        """Load mission data from a JSON file."""
        with open(filename, 'r') as f:
            mission_data = json.load(f)

        # Clear existing data
        self.waypoints = []
        self.paths = []
        self.midpoints = []

        # Load waypoints
        for wp_data in mission_data["waypoints"]:
            self.add_waypoint(
                wp_data["lat"],
                wp_data["lon"],
                wp_data["elevation"],
                wp_data["name"]
            )

        # Recalculate paths and midpoints
        self.calculate_paths()
        self.calculate_midpoints()

        logger.info(f"Mission loaded from {filename}")

    def get_drone_position(self):
        """
        Get current drone position from MAVLink.

        Returns:
            GeoPoint object with current position or None if position not available
        """
        if not self.vehicle:
            logger.error("No vehicle connection")
            return None

        msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            # Convert from 1e7 to decimal degrees
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000.0  # mm to meters
            return GeoPoint(lat, lon, alt, "Drone")

        return None

    def get_car_position(self):
        """
        Get current car position from the car GPS.

        Returns:
            GeoPoint object with car position or None if not available
        """
        if not self.car_gps:
            logger.error("No car GPS configured")
            return None

        return self.car_gps.get_position()

    def set_guided_mode(self):
        """Set vehicle mode to GUIDED."""
        if not self.vehicle:
            logger.error("No vehicle connection")
            return False

        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4,  # GUIDED mode
            0, 0, 0, 0, 0
        )

        # Wait for mode change
        for _ in range(30):
            msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg and msg.custom_mode == 4:  # GUIDED mode
                logger.info("Vehicle mode set to GUIDED")
                return True
            time.sleep(0.1)

        logger.error("Failed to set GUIDED mode")
        return False

    def arm_vehicle(self):
        """Arm the vehicle."""
        if not self.vehicle:
            logger.error("No vehicle connection")
            return False

        # Check if already armed
        msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            logger.info("Vehicle already armed")
            return True

        # Arm the vehicle
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )

        # Wait for arming
        for _ in range(30):
            msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                logger.info("Vehicle armed")
                return True
            time.sleep(0.1)

        logger.error("Failed to arm vehicle")
        return False

    def takeoff(self, target_altitude):
        """Takeoff to the specified altitude."""
        if not self.vehicle:
            logger.error("No vehicle connection")
            return False

        # Check if already flying
        current_alt = self.get_drone_position().elevation if self.get_drone_position() else 0
        if current_alt > 1.0:
            logger.info(f"Already flying at {current_alt:.1f}m, target is {target_altitude:.1f}m")
            return True

        # Command takeoff
        logger.info(f"Sending takeoff command to altitude: {target_altitude:.1f}m")
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system, self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, target_altitude  # Explicitly pass the target_altitude here
        )

        # Wait for takeoff
        start_time = time.time()
        while time.time() - start_time < 30:
            position = self.get_drone_position()
            if position and position.elevation >= target_altitude - 1.0:
                logger.info(
                    f"Takeoff complete - altitude: {position.elevation:.1f}m, target was: {target_altitude:.1f}m")
                return True
            time.sleep(0.5)

        logger.error("Takeoff timed out")
        return False

    def _get_smoothed_distance(self, new_distance):
        """Use a moving average to smooth distance measurements."""
        self.distance_buffer.append(new_distance)
        return sum(self.distance_buffer) / len(self.distance_buffer)

    def _detect_car_passing_midpoint(self, current_distance):
        """
        Detect when the car is passing through a midpoint by monitoring distance changes.
        Returns True when the car has passed the minimum distance point.
        """
        # Smooth the distance to reduce noise
        smoothed_distance = self._get_smoothed_distance(current_distance)

        # Update minimum distance seen
        if smoothed_distance < self.min_distance_recorded:
            self.min_distance_recorded = smoothed_distance
            self.distance_increasing = False
            return False

        # Check if distance is now increasing after decreasing
        if not self.distance_increasing and smoothed_distance > self.min_distance_recorded * 1.05:
            # Distance has increased by at least 5% from minimum
            self.distance_increasing = True

            # Consider midpoint passed when distance starts increasing
            logger.info(f"Car passed midpoint: minimum distance was {self.min_distance_recorded:.2f}m")
            self.min_distance_point_passed = True
            return True

        return False

    def _is_drone_at_midpoint(self, midpoint):
        """Check if the drone is at the specified midpoint."""
        drone_pos = self.get_drone_position()
        if not drone_pos:
            return False

        distance = drone_pos.distance_to(midpoint)
        return distance <= 3.0  # Consider "at midpoint" if within 3 meters

    def execute_car_tracking_mission(self, target_altitude=None, target_velocity=10.0):
        """
        Execute a mission where the drone moves to midpoints and waits for the car to pass by.

        The drone will:
        1. Move to the first midpoint
        2. Hover there until the car passes by
        3. Move to the next midpoint
        4. Repeat until all midpoints are visited
        """
        if not self.midpoints:
            logger.error("No midpoints available for mission")
            return False

        if not self.car_gps:
            logger.error("No car GPS configured - cannot track car position")
            return False

        self.current_target_index = 0
        self.mission_complete = False
        self.at_midpoint = False
        self.is_hovering = False

        # Set altitude if provided
        if target_altitude is not None:
            logger.info(f"Setting all waypoints and midpoints to altitude: {target_altitude}m")
            # Set altitude for both waypoints and midpoints to ensure consistency
            for waypoint in self.waypoints:
                waypoint.elevation = target_altitude
                logger.debug(f"Set waypoint {waypoint.name} altitude to {waypoint.elevation}m")

            # Recalculate paths with the new waypoint altitudes
            self.calculate_paths()

            # Set midpoint altitudes explicitly to match target_altitude
            for midpoint in self.midpoints:
                midpoint.elevation = target_altitude
                logger.debug(f"Set midpoint {midpoint.name} altitude to {midpoint.elevation}m")

        # Initial target is the first midpoint
        current_target = self.midpoints[self.current_target_index]
        logger.info(f"Starting car tracking mission with {len(self.midpoints)} midpoints")
        logger.info(f"First target: {current_target} at altitude {current_target.elevation}m")

        # Send initial command to the first midpoint with explicit altitude
        self.send_goto_command(current_target, target_velocity, override_altitude=target_altitude)

        # Position tracking
        last_command_time = 0
        command_frequency = 0.5  # 2 Hz

        while not self.mission_complete:
            current_time = time.time()

            # Update commands at specified frequency
            if current_time - last_command_time >= command_frequency:
                last_command_time = current_time

                # Get current drone position
                drone_pos = self.get_drone_position()
                if not drone_pos:
                    logger.warning("Could not get current drone position")
                    time.sleep(0.5)
                    continue

                # Get current target midpoint
                current_target = self.midpoints[self.current_target_index]

                # Check if drone has reached the midpoint
                drone_to_midpoint_distance = drone_pos.distance_to(current_target)

                # Also check altitude difference
                altitude_diff = abs(drone_pos.elevation - current_target.elevation)

                # Log drone status
                logger.info(
                    f"Drone to midpoint {self.current_target_index + 1}/{len(self.midpoints)} "
                    f"distance: {drone_to_midpoint_distance:.2f}m, "
                    f"altitude difference: {altitude_diff:.2f}m, "
                    f"current alt: {drone_pos.elevation:.2f}m, "
                    f"target alt: {current_target.elevation:.2f}m"
                )

                # If drone is not at the midpoint yet, keep moving there
                if not self._is_drone_at_midpoint(current_target):
                    if self.at_midpoint:
                        logger.warning("Drone drifted from midpoint - repositioning")
                        self.at_midpoint = False
                        self.is_hovering = False

                    # Continue moving to midpoint with explicit altitude
                    self.send_goto_command(current_target, target_velocity, override_altitude=target_altitude)
                    continue

                # Mark that drone is at midpoint if not already set
                if not self.at_midpoint:
                    logger.info(f"Drone reached midpoint {self.current_target_index + 1}: {current_target}")
                    self.at_midpoint = True

                    # Start hovering
                    self.send_hover_command()
                    self.is_hovering = True

                    # Reset car tracking variables
                    self.min_distance_recorded = float('inf')
                    self.distance_increasing = False
                    self.min_distance_point_passed = False
                    self.distance_buffer.clear()

                # Get current car position
                car_pos = self.get_car_position()
                if not car_pos:
                    logger.warning("Could not get current car position")
                    # Continue hovering at midpoint
                    continue

                # Calculate distance from car to midpoint
                car_to_midpoint_distance = car_pos.distance_to(current_target)

                # Log car status
                logger.info(
                    f"Car to midpoint {self.current_target_index + 1} distance: "
                    f"{car_to_midpoint_distance:.2f}m, "
                    f"Min: {self.min_distance_recorded:.2f}m"
                )

                # Check if car has passed the midpoint
                if self._detect_car_passing_midpoint(car_to_midpoint_distance):
                    logger.info(f"Car passed through midpoint {self.current_target_index + 1}")

                    # Move to next midpoint
                    self.current_target_index += 1
                    self.at_midpoint = False
                    self.is_hovering = False

                    # Check if mission complete
                    if self.current_target_index >= len(self.midpoints):
                        logger.info("Mission complete! All midpoints visited by car.")

                        # Proceed to final waypoint if available
                        if self.waypoints:
                            final_waypoint = self.waypoints[-1]
                            logger.info(f"Moving to final waypoint: {final_waypoint}")
                            self.send_goto_command(final_waypoint, target_velocity)

                        self.mission_complete = True
                        break

                    # Get next target midpoint
                    current_target = self.midpoints[self.current_target_index]
                    logger.info(f"Moving to next midpoint: {current_target} at altitude {current_target.elevation}m")

                    # Send command to go to the next midpoint
                    self.send_goto_command(current_target, target_velocity)

            # Small delay to prevent CPU overuse
            time.sleep(0.05)

        return self.mission_complete

    def send_goto_command(self, target_point, target_velocity, override_altitude=None):
        """
        Send a goto command to the vehicle using SET_POSITION_TARGET_GLOBAL_INT.

        Args:
            target_point: The GeoPoint target to navigate to
            target_velocity: The target velocity in m/s
            override_altitude: If provided, this altitude will be used instead of the target_point's elevation
        """
        if not self.vehicle:
            logger.error("No vehicle connection")
            return

        try:
            # Convert lat/lon to int * 1e7 format
            lat_int = int(target_point.lat * 1e7)
            lon_int = int(target_point.lon * 1e7)

            # Use override_altitude if provided, otherwise use target_point's elevation
            if override_altitude is not None:
                alt = float(override_altitude)
                logger.info(f"Overriding target altitude: using {alt}m instead of {target_point.elevation}m")
            else:
                alt = float(target_point.elevation)

            # Create SET_POSITION_TARGET_GLOBAL_INT message
            # Critically important: using the proper mask to ensure altitude is respected
            # We're only setting position, ignoring velocity, acceleration and yaw
            mask = (
                    0b0000111111111000 |  # ignore velocity
                    0b0001000000000000 |  # ignore acceleration
                    0b0010000000000000  # ignore yaw
            )

            # Send position target message
            self.vehicle.mav.set_position_target_global_int_send(
                0,  # time_boot_ms
                self.vehicle.target_system,
                self.vehicle.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mask,  # type_mask
                lat_int,  # lat_int
                lon_int,  # lon_int
                alt,  # alt - using possibly overridden altitude
                0, 0, 0,  # vx, vy, vz
                0, 0, 0,  # afx, afy, afz
                0, 0  # yaw, yaw_rate
            )

            logger.info(
                f"Sent GOTO command to target: "
                f"lat={target_point.lat}, lon={target_point.lon}, alt={alt}m"
            )

            # Log the exact object contents for debugging
            logger.debug(f"Target point details: {target_point}")

        except Exception as e:
            logger.error(f"Error sending GOTO command: {str(e)}")

    def send_hover_command(self):
        """Send command to hover at current position."""
        if not self.vehicle:
            logger.error("No vehicle connection")
            return

        try:
            # Get current position
            current_pos = self.get_drone_position()
            if not current_pos:
                logger.error("Failed to get current position for hover command")
                return

            # Convert lat/lon to int * 1e7 format
            lat_int = int(current_pos.lat * 1e7)
            lon_int = int(current_pos.lon * 1e7)
            alt = float(current_pos.elevation)

            # Create mask to hover at current position
            mask = (
                    0b0000111111111000 |  # ignore velocity
                    0b0001000000000000 |  # ignore acceleration
                    0b0010000000000000  # ignore yaw
            )

            # Send position target message to current position (hover)
            self.vehicle.mav.set_position_target_global_int_send(
                0,  # time_boot_ms
                self.vehicle.target_system,
                self.vehicle.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mask,  # type_mask
                lat_int,  # lat_int
                lon_int,  # lon_int
                alt,  # alt
                0, 0, 0,  # vx, vy, vz
                0, 0, 0,  # afx, afy, afz
                0, 0  # yaw, yaw_rate
            )

            logger.info(
                f"Sent hover command at current position: lat={current_pos.lat}, lon={current_pos.lon}, alt={alt}")

        except Exception as e:
            logger.error(f"Error sending hover command: {str(e)}")

    def cleanup(self):
        """Clean up resources and stop mission execution"""
        logger.warning("DronePathPlanner cleanup called - forcing mission to stop")

        # Set flag to ensure the mission stops
        self.mission_complete = True

        # Stop car GPS if running
        if self.car_gps:
            self.car_gps.stop()

        # Additionally, signal to external systems (mission_mgr) that we want to stop
        # This line forces the run_mission_thread to terminate
        if 'mission_mgr' in globals():
            globals()['mission_mgr'].is_running = False
            logger.warning("Set mission_mgr.is_running to False")

        # Send a position hold command at the current position
        if self.vehicle:
            try:
                # Get current position for position hold
                position = self.get_drone_position()
                if position:
                    # Send hover command
                    self.send_hover_command()
                    logger.warning("Sent position hold command at current position")
                else:
                    logger.warning("Could not get current position for position hold command")

            except Exception as e:
                logger.error(f"Error sending position hold command: {str(e)}")

        logger.warning("DronePathPlanner cleanup completed")
        return True


def main():
    import argparse

    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Drone Car Tracking System')
    parser.add_argument('--connection', type=str, default='udp:127.0.0.1:14551',
                        help='MAVLink connection string')
    parser.add_argument('--car-gps-port', type=str, default='/dev/ttyUSB0',
                        help='Serial port for car GPS (e.g., /temp/vgps2)')
    parser.add_argument('--car-gps-baud', type=int, default=9600,
                        help='Baud rate for car GPS')
    parser.add_argument('--threshold', type=float, default=10.0,
                        help='Distance threshold in meters to consider car passing a midpoint')
    parser.add_argument('--altitude', type=float, default=70.0,
                        help='Drone flight altitude in meters')
    parser.add_argument('--save-mission', type=str, default=None,
                        help='Save mission to JSON file')
    parser.add_argument('--load-mission', type=str, default=None,
                        help='Load mission from JSON file')

    args = parser.parse_args()

    # Log the parsed arguments
    logger.info(f"Starting mission with parameters:")
    logger.info(f"Connection: {args.connection}")
    logger.info(f"Car GPS port: {args.car_gps_port}")
    logger.info(f"Car GPS baud rate: {args.car_gps_baud}")
    logger.info(f"Distance threshold: {args.threshold}")
    logger.info(f"Target altitude: {args.altitude} meters")

    # Create the path planner with car GPS
    planner = DronePathPlanner(
        max_points=1000,
        car_gps_port=args.car_gps_port,
        car_gps_baudrate=args.car_gps_baud
    )

    # Update distance threshold if specified
    if args.threshold:
        planner.distance_threshold = args.threshold
        logger.info(f"Using distance threshold: {planner.distance_threshold}m")

    try:
        # Load mission or add default waypoints
        if args.load_mission:
            planner.load_mission_from_json(args.load_mission)
            logger.info(f"Loaded mission from {args.load_mission}")

            # Important: Update all waypoint altitudes to match the specified altitude
            for waypoint in planner.waypoints:
                waypoint.elevation = args.altitude
                logger.info(f"Updated waypoint {waypoint.name} altitude to {args.altitude}m")
        else:
            # Add example waypoints with the specified altitude
            planner.add_waypoint(23.191849, 77.364854, args.altitude, "Point A")
            planner.add_waypoint(23.190986, 77.365937, args.altitude, "Point B")
            planner.add_waypoint(23.190355, 77.367997, args.altitude, "Point C")
            planner.add_waypoint(23.192386, 77.370830, args.altitude, "Point D")
            planner.add_waypoint(23.195517, 77.371050, args.altitude, "Point E")
            planner.add_waypoint(23.197110, 77.369097, args.altitude, "Point F")
            logger.info(f"Added default waypoints at altitude {args.altitude}m")

        # Calculate midpoints
        planner.calculate_paths()
        planner.calculate_midpoints()

        # Important: Explicitly ensure all midpoints also have the correct altitude
        for midpoint in planner.midpoints:
            if midpoint.elevation != args.altitude:
                logger.warning(
                    f"Midpoint {midpoint.name} had incorrect altitude {midpoint.elevation}m. Updating to {args.altitude}m")
                midpoint.elevation = args.altitude

        # Print the mission summary
        print(planner.print_mission_summary())

        # Save the mission to a file if requested
        if args.save_mission:
            planner.save_mission_to_json(args.save_mission)

        # Connect to car GPS
        if not planner.connect_car_gps():
            logger.error("Failed to connect to car GPS. Aborting mission.")
            return

        # Connect to the vehicle
        if planner.connect_vehicle(args.connection):
            # Setup flight
            if planner.set_guided_mode() and planner.arm_vehicle():
                # Takeoff
                if planner.takeoff(args.altitude):
                    # Execute the car tracking mission with the specified altitude and velocity
                    logger.info(f"Starting car tracking mission at altitude {args.altitude}m")
                    planner.execute_car_tracking_mission(target_altitude=args.altitude, target_velocity=10.0)
                else:
                    logger.error("Takeoff failed")
            else:
                logger.error("Failed to set up vehicle")
        else:
            logger.error("Failed to connect to vehicle")

    except KeyboardInterrupt:
        logger.info("Mission interrupted by user")
    except Exception as e:
        logger.error(f"Error in mission: {e}")
        import traceback
        logger.error(traceback.format_exc())
    finally:
        # Clean up
        planner.cleanup()
        logger.info("Mission ended")


if __name__ == "__main__":
    main()