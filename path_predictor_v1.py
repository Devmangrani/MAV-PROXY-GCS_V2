import math
import time
import json
import logging
import threading
import serial
import pynmea2
from pymavlink import mavutil
import os
import numpy as np
from collections import deque

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('drone_path_navigator.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


class GeoPoint:
    """Class to represent a geographical point with latitude, longitude, and optional elevation."""

    def __init__(self, lat, lon, elevation=0, name="", speed=0.0):
        self.lat = lat
        self.lon = lon
        self.elevation = elevation
        self.name = name
        self.speed = speed  # New speed attribute

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
        return {
            "name": self.name,
            "lat": self.lat,
            "lon": self.lon,
            "elevation": self.elevation,
            "speed": self.speed  # Added speed serialization
        }


class VehicleGPSReceiver:
    """Class to handle the vehicle GPS receiver."""

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
                time.sleep(0.5)

            # Check if the port exists
            if not os.path.exists(self.port):
                logger.error(f"GPS port {self.port} does not exist")
                return False

            # Use a longer timeout for initial connection
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=5)

            # Clear any stale data in the buffer
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()

            # Verify the connection works
            try:
                self.serial_conn.timeout = 1  # Shorter timeout for test read
                test_data = self.serial_conn.read(10)
            except serial.SerialException as se:
                if "device reports readiness to read but returned no data" in str(se):
                    logger.warning("Device reported readiness but no data - continuing anyway")
                else:
                    raise

            logger.info(f"Connected to vehicle GPS on {self.port} at {self.baudrate} baud")
            return True

        except Exception as e:
            logger.error(f"Failed to connect to vehicle GPS: {e}")
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
            logger.warning("GPS receiver already running")
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

        logger.info("Vehicle GPS receiver started")
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

        logger.info("Vehicle GPS receiver stopped")

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
                    logger.warning("Serial connection lost. Attempting to reconnect...")
                    try:
                        if self.serial_conn:
                            self.serial_conn.close()
                        self.serial_conn = None

                        # Try to reconnect
                        if self.connect():
                            logger.info("Reconnected to vehicle GPS")
                            consecutive_errors = 0
                            empty_reads = 0
                    except Exception as e:
                        logger.error(f"Failed to reconnect to vehicle GPS: {e}")

                    time.sleep(1)
                    continue

                # Check if data is available to read
                try:
                    in_waiting = self.serial_conn.in_waiting
                    if in_waiting > 0:
                        # Use a shorter timeout for reading
                        self.serial_conn.timeout = 0.5
                        line = self.serial_conn.readline()

                        # Handle empty reads
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
                logger.error(f"Error reading GPS data: {e}")
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
        """Get the current GPS position with speed."""
        if not self.has_position:
            return None

        # Check if position is stale (no updates in 5 seconds)
        if time.time() - self.last_update_time > 5.0:
            logger.warning("Vehicle GPS position data is stale")

        # Create GeoPoint with speed data
        return GeoPoint(self.lat, self.lon, self.alt, "Vehicle", self.speed)


class DroneNavigator:
    """Class for drone navigation with path optimization from the HTML logic."""

    def __init__(self, max_points=100, vehicle_gps_port=None, vehicle_gps_baudrate=9600):
        self.waypoints = []
        self.optimized_path = []
        self.max_points = max_points
        self.current_target_index = -1
        self.mission_complete = False
        self.vehicle = None
        self.distance_threshold = 10.0  # Meters for vehicle detection
        self.vehicle_gps = VehicleGPSReceiver(port=vehicle_gps_port,
                                              baudrate=vehicle_gps_baudrate) if vehicle_gps_port else None
        self.waypoint_skip_lookahead = 5
        self.skip_check_interval = 2.0
        self.last_skip_check = 0
        self.final_waypoint_timeout = 300  # 5 minutes in seconds
        self.at_waypoint = False
        self.is_hovering = False
        self.min_distance_recorded = float('inf')
        self.distance_increasing = False
        self.distance_buffer = deque(maxlen=5)
        self.waypoint_min_distances = {}  # {index: min_distance}
        self.lookahead_window = 5  # Check next 5 waypoints for skips

    def add_waypoint(self, lat, lon, elevation=0, name=None):
        """Add a waypoint to the navigator."""
        if len(self.waypoints) >= self.max_points:
            raise ValueError(f"Maximum number of waypoints ({self.max_points}) reached")

        if name is None:
            name = f"Point {len(self.waypoints) + 1}"

        point = GeoPoint(lat, lon, elevation, name)
        self.waypoints.append(point)
        return point

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

    def connect_gps(self):
        """Connect to the vehicle GPS device."""
        if not self.vehicle_gps:
            logger.error("No vehicle GPS configured")
            return False

        if self.vehicle_gps.start():
            logger.info("Vehicle GPS started")
            return True
        else:
            logger.error("Failed to start vehicle GPS")
            return False

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

    def get_vehicle_position(self):
        """
        Get current vehicle position from the GPS.

        Returns:
            GeoPoint object with vehicle position or None if not available
        """
        if not self.vehicle_gps:
            logger.error("No vehicle GPS configured")
            return None

        return self.vehicle_gps.get_position()

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
        current_pos = self.get_drone_position()
        current_alt = current_pos.elevation if current_pos else 0

        if current_alt > 1.0:
            logger.info(f"Already flying at {current_alt:.1f}m, target is {target_altitude:.1f}m")

            # If already flying but not at target altitude, command climb to target altitude
            if abs(current_alt - target_altitude) > 1.0:
                logger.info(f"Commanding climb to target altitude: {target_altitude:.1f}m")

                # Send position target to current lat/lon but with target altitude
                lat_int = int(current_pos.lat * 1e7)
                lon_int = int(current_pos.lon * 1e7)

                # Set position only, ignoring velocity, acceleration and yaw
                mask = (
                        0b0000111111111000 |  # ignore velocity
                        0b0001000000000000 |  # ignore acceleration
                        0b0010000000000000  # ignore yaw
                )

                # Send position target message to climb
                self.vehicle.mav.set_position_target_global_int_send(
                    0,  # time_boot_ms
                    self.vehicle.target_system,
                    self.vehicle.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    mask,  # type_mask
                    lat_int,  # lat_int
                    lon_int,  # lon_int
                    target_altitude,  # alt
                    0, 0, 0,  # vx, vy, vz
                    0, 0, 0,  # afx, afy, afz
                    0, 0  # yaw, yaw_rate
                )

                # Wait for the drone to reach target altitude
                start_time = time.time()
                logger.info("Waiting for drone to reach target altitude...")

                while time.time() - start_time < 60:  # 60 second timeout for climb
                    position = self.get_drone_position()
                    if position and abs(position.elevation - target_altitude) < 1.0:
                        logger.info(f"Target altitude reached: {position.elevation:.1f}m")
                        return True

                    # Re-send command every 5 seconds to ensure it's received
                    if (time.time() - start_time) % 5 < 0.1:
                        self.vehicle.mav.set_position_target_global_int_send(
                            0, self.vehicle.target_system, self.vehicle.target_component,
                            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                            mask, lat_int, lon_int, target_altitude,
                            0, 0, 0, 0, 0, 0, 0, 0
                        )

                    time.sleep(0.5)

                logger.error(f"Failed to reach target altitude. Current: {self.get_drone_position().elevation:.1f}m")
                return False
            else:
                # Already at target altitude (±1m)
                return True

        # Command takeoff if not already flying
        logger.info(f"Sending takeoff command to altitude: {target_altitude:.1f}m")
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system, self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, target_altitude
        )

        # Wait for takeoff
        start_time = time.time()
        while time.time() - start_time < 300:
            position = self.get_drone_position()
            if position and position.elevation >= target_altitude - 1.0:
                logger.info(f"Takeoff complete - altitude: {position.elevation:.1f}m")
                return True
            time.sleep(0.5)

        logger.error("Takeoff timed out")
        return False

    def send_goto_command(self, target_point, target_velocity=10.0, override_altitude=None):
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
            # Set position only, ignoring velocity, acceleration and yaw
            mask = (
                    0b0000111111111000 |  # ignore velocity
                    0b0001000000000000  # ignore acceleration
                    # 0b0010000000000000  # ignore yaw
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
                alt,  # alt
                target_velocity, 0, 0,  # vx, vy, vz
                0, 0, 0,  # afx, afy, afz
                0, 0  # yaw, yaw_rate
            )

            logger.info(f"Sent GOTO command to: lat={target_point.lat}, lon={target_point.lon}, alt={alt}m")

        except Exception as e:
            logger.error(f"Error sending GOTO command: {str(e)}")

    def send_hover_command(self):
        """Force immediate hover with zero velocity at current position"""
        if not self.vehicle:
            logger.error("No vehicle connection for hover command")
            return

        try:
            current_pos = self.get_drone_position()
            if not current_pos:
                logger.error("Can't hover - no position data")
                return

            # Mask to set velocity to zero while maintaining position
            mask = (
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
            )

            self.vehicle.mav.set_position_target_global_int_send(
                0,  # time_boot_ms
                self.vehicle.target_system,
                self.vehicle.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mask,
                int(current_pos.lat * 1e7),
                int(current_pos.lon * 1e7),
                current_pos.elevation,
                0,  # vx (0 m/s)
                0,  # vy (0 m/s)
                0,  # vz (0 m/s)
                0, 0, 0,  # afx, afy, afz (ignored)
                0,  # yaw (ignored)
                0  # yaw_rate (ignored)
            )
            logger.warning("EMERGENCY HOVER COMMAND SENT")

        except Exception as e:
            logger.error(f"Critical hover failure: {str(e)}")
            import traceback
            logger.error(traceback.format_exc())

    # --- From HTML: Path Optimization Functions ---

    def calculate_drone_path(self, min_waypoint_distance=200):
        """
        Calculate an optimized drone path using the algorithm from the HTML file.
        This generates waypoints based on the route's geometry and curves.

        Args:
            min_waypoint_distance: Minimum distance between waypoints in meters

        Returns:
            List of GeoPoint objects representing the optimized path
        """
        if len(self.waypoints) < 2:
            logger.error("Need at least 2 waypoints to calculate a path")
            return []

        # Convert waypoints to the format expected by the algorithm
        route_coordinates = [[wp.lat, wp.lon] for wp in self.waypoints]

        # Step 1: Apply Douglas-Peucker algorithm for initial simplification
        epsilon = 0.1  # Simplification threshold in meters
        simplified_points = self.douglas_peucker_simplify(route_coordinates, epsilon)

        # Step 2: Ensure waypoints are at least minWaypointDistance apart
        final_points = [simplified_points[0]]  # Start with the first point

        # Process each point after the first one
        for i in range(1, len(simplified_points)):
            last_added_point = final_points[-1]
            current_point = simplified_points[i]

            # Calculate distance between the last added point and current point
            distance = self.calculate_haversine_distance(
                last_added_point[0], last_added_point[1],
                current_point[0], current_point[1]
            ) * 1000  # Convert km to meters

            # If the distance is at least the minimum required, add the point
            if distance >= min_waypoint_distance:
                final_points.append(current_point)
            elif i == len(simplified_points) - 1:
                # Always include the last point regardless of distance
                final_points.append(current_point)

        # Step 3: Add intermediate points for segments that are too far apart
        distance_adjusted_points = []
        for i in range(len(final_points) - 1):
            start_point = final_points[i]
            end_point = final_points[i + 1]
            distance_adjusted_points.append(start_point)

            # Calculate distance between consecutive points
            distance = self.calculate_haversine_distance(
                start_point[0], start_point[1],
                end_point[0], end_point[1]
            ) * 1000  # Convert km to meters

            # If the distance is too large, add intermediate points
            max_distance = min_waypoint_distance * 1.5  # Allow up to 1.5x the minimum distance
            if distance > max_distance:
                # Calculate how many intermediate points to add
                num_intermediate_points = int(distance / min_waypoint_distance) - 1

                for j in range(1, num_intermediate_points + 1):
                    fraction = j / (num_intermediate_points + 1)
                    intermediate_point = [
                        start_point[0] + fraction * (end_point[0] - start_point[0]),
                        start_point[1] + fraction * (end_point[1] - start_point[1])
                    ]
                    distance_adjusted_points.append(intermediate_point)

        # Add the final point
        distance_adjusted_points.append(final_points[-1])

        # Step 4: Check for sharp curves and add waypoints if necessary
        sharp_curve_threshold = 15  # Angle in degrees
        curve_adjusted_points = []

        for i in range(len(distance_adjusted_points) - 2):
            curve_adjusted_points.append(distance_adjusted_points[i])

            bearing1 = self.calculate_bearing(
                distance_adjusted_points[i][0], distance_adjusted_points[i][1],
                distance_adjusted_points[i + 1][0], distance_adjusted_points[i + 1][1]
            )

            bearing2 = self.calculate_bearing(
                distance_adjusted_points[i + 1][0], distance_adjusted_points[i + 1][1],
                distance_adjusted_points[i + 2][0], distance_adjusted_points[i + 2][1]
            )

            # Calculate the absolute bearing change
            change = abs(bearing2 - bearing1)
            if change > 180:
                change = 360 - change

            # Add waypoint at significant bearing changes
            if change > sharp_curve_threshold:
                curve_point = distance_adjusted_points[i + 1]
                curve_adjusted_points.append(curve_point)

        # Add the last two points
        if len(distance_adjusted_points) >= 2:
            curve_adjusted_points.append(distance_adjusted_points[-2])
            curve_adjusted_points.append(distance_adjusted_points[-1])

        # Convert back to GeoPoint objects with altitude from original waypoints
        optimized_path = []
        default_alt = self.waypoints[0].elevation  # Use first waypoint's altitude as default

        for i, point in enumerate(curve_adjusted_points):
            # Create a name for the waypoint
            name = f"WP{i + 1}"
            geo_point = GeoPoint(point[0], point[1], default_alt, name)
            optimized_path.append(geo_point)

        # Update the class attribute
        self.optimized_path = optimized_path

        logger.info(f"Generated {len(optimized_path)} waypoints for drone path")
        return optimized_path

    def douglas_peucker_simplify(self, points, epsilon):
        """Douglas-Peucker algorithm for path simplification."""
        if len(points) <= 2:
            return points

        # Find the point with the maximum distance
        max_distance = 0
        max_distance_index = 0

        for i in range(1, len(points) - 1):
            distance = self.perpendicular_distance(
                points[i],
                points[0],
                points[-1]
            )

            if distance > max_distance:
                max_distance = distance
                max_distance_index = i

        # If max distance is greater than epsilon, recursively simplify
        if max_distance > epsilon:
            # Recursive call
            first_line = self.douglas_peucker_simplify(
                points[:max_distance_index + 1],
                epsilon
            )
            second_line = self.douglas_peucker_simplify(
                points[max_distance_index:],
                epsilon
            )

            # Concat the two simplified segments
            return first_line[:-1] + second_line
        else:
            # Return the end points of the line
            return [points[0], points[-1]]

    def perpendicular_distance(self, point, line_start, line_end):
        """Calculate perpendicular distance from point to line."""
        # Convert lat/lng to meters for proper distance
        x0 = point[1]  # longitude
        y0 = point[0]  # latitude
        x1 = line_start[1]  # longitude
        y1 = line_start[0]  # latitude
        x2 = line_end[1]  # longitude
        y2 = line_end[0]  # latitude

        # Calculate the denominator of the formula
        denominator = math.sqrt(
            math.pow(y2 - y1, 2) + math.pow(x2 - x1, 2)
        )

        # Avoid division by zero
        if denominator == 0:
            return 0

        # Calculate perpendicular distance
        numerator = abs(
            (y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1
        )

        # Convert to kilometers
        distance_in_degrees = numerator / denominator
        return distance_in_degrees * 111.32  # Rough conversion to km at equator

    def calculate_haversine_distance(self, lat1, lon1, lat2, lon2):
        """Calculate distance between two points using the Haversine formula."""
        R = 6371  # Earth radius in km
        dLat = math.radians(lat2 - lat1)
        dLon = math.radians(lon2 - lon1)

        a = (math.sin(dLat / 2) * math.sin(dLat / 2) +
             math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
             math.sin(dLon / 2) * math.sin(dLon / 2))
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        distance = R * c  # Distance in km
        return distance

    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        """Calculate bearing between two points."""
        lat1 = math.radians(lat1)
        lon1 = math.radians(lon1)
        lat2 = math.radians(lat2)
        lon2 = math.radians(lon2)

        y = math.sin(lon2 - lon1) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(lon2 - lon1)
        bearing = math.atan2(y, x)

        bearing = math.degrees(bearing)
        bearing = (bearing + 360) % 360  # Normalize to 0-360

        return bearing

    def _is_drone_at_waypoint(self, waypoint):
        """Check if drone is within 3m of waypoint"""
        drone_pos = self.get_drone_position()
        if not drone_pos:
            return False
        return drone_pos.distance_to(waypoint) <= 3.0

    def _get_smoothed_distance(self, new_distance):
        """Use a moving average to smooth distance measurements."""
        self.distance_buffer.append(new_distance)
        return sum(self.distance_buffer) / len(self.distance_buffer)

    def _detect_vehicle_passing_waypoint(self, current_distance):
        """Detect waypoint passage using minimum distance + 5% threshold"""
        # Update minimum distance
        if current_distance < self.min_distance_recorded:
            self.min_distance_recorded = current_distance
            self.distance_increasing = False
            logger.debug(f"New minimum distance: {self.min_distance_recorded:.2f}m")
            return False

        # Calculate 5% threshold from minimum
        threshold_distance = self.min_distance_recorded * 1.05

        # Check if we've passed the threshold
        if not self.distance_increasing and current_distance > threshold_distance:
            logger.info(f"Vehicle passed waypoint! Current: {current_distance:.2f}m, "
                        f"Threshold: {threshold_distance:.2f}m (+5%)")
            self.distance_increasing = True
            return True

        return False

    def _check_skipped_waypoints(self, vehicle_pos):
        """Check skipped waypoints using dynamic threshold"""
        if not vehicle_pos or self.current_target_index < 0:
            return False

        # Check next 5 waypoints (excluding final)
        start_idx = self.current_target_index
        end_idx = min(start_idx + self.waypoint_skip_lookahead, len(self.optimized_path) - 2)

        for idx in range(start_idx, end_idx + 1):
            waypoint = self.optimized_path[idx]
            distance = vehicle_pos.distance_to(waypoint)

            # Track minimum distance for this waypoint
            if distance < self.min_distance_recorded:
                self.min_distance_recorded = distance
                self.distance_increasing = False

            # Check 5% threshold
            if distance > self.min_distance_recorded * 1.05:
                logger.info(f"Vehicle passed WP{idx + 1} (distance: {distance:.2f}m, "
                            f"min: {self.min_distance_recorded:.2f}m)")
                self.current_target_index = idx + 1
                self._reset_tracking_vars()
                return True

        return False

    def _update_navigation(self, target_velocity, target_altitude):
        """Handle navigation updates and waypoint progression"""
        drone_pos = self.get_drone_position()
        vehicle_pos = self.get_vehicle_position()

        if not drone_pos or not vehicle_pos:
            return

        current_target = self.optimized_path[self.current_target_index]

        # Check if drone has reached current waypoint
        if self._is_drone_at_waypoint(current_target):
            vehicle_distance = vehicle_pos.distance_to(current_target)

            # Check if vehicle passed waypoint
            if self._detect_vehicle_passing_waypoint(vehicle_distance):
                self._advance_to_next_waypoint(target_velocity, target_altitude)
        else:
            # Continue navigating to current waypoint
            self.send_goto_command(current_target, target_velocity, target_altitude)

    def _advance_to_next_waypoint(self, target_velocity, target_altitude):
        """Progress to next waypoint in mission"""
        self.current_target_index += 1

        if self.current_target_index >= len(self.optimized_path):
            self.mission_complete = True
            return

        new_target = self.optimized_path[self.current_target_index]
        logger.info(f"Proceeding to WP{self.current_target_index + 1} ({new_target.name})")
        self.send_goto_command(new_target, target_velocity, target_altitude)
        self.at_waypoint = False
        self.is_hovering = False
        self._reset_tracking_vars()

    def _should_skip_for_speed(self, vehicle_pos, drone_pos):
        """Check if we should skip waypoint based on vehicle speed and distance."""
        if not vehicle_pos or not drone_pos:
            return False

        # Calculate distance between drone and vehicle
        distance = drone_pos.distance_to(vehicle_pos)
        vehicle_speed = vehicle_pos.speed

        logger.debug(f"Vehicle speed: {vehicle_speed:.2f} m/s, Distance: {distance:.2f}m")

        # Check skip conditions
        if distance <= 200 and vehicle_speed > 10:
            logger.info(f"Speed trigger: Vehicle moving at {vehicle_speed:.2f} m/s within {distance:.2f}m")
            return True
        return False

    def execute_vehicle_tracking_mission(self, target_altitude=None, target_velocity=10.0):
        """Main mission execution with speed-based navigation and trajectory enforcement"""
        if not self.optimized_path:
            logger.error("No flight path available")
            return False

        logger.info("Starting enhanced vehicle tracking mission")

        # Initialize mission control variables
        self.current_target_index = 0
        self.mission_complete = False
        self.waypoint_min_distances.clear()
        self.last_speed_skip_time = 0  # Track last speed-based skip
        speed_skip_cooldown = 10  # Minimum seconds between speed skips

        logger.info(f"Mission plan contains {len(self.optimized_path)} waypoints")

        # Set uniform altitude if specified
        if target_altitude:
            logger.info(f"Setting all waypoints to {target_altitude}m altitude")
            for wp in self.optimized_path:
                wp.elevation = target_altitude

        # Initialize navigation to first waypoint
        start_wp = self.optimized_path[0]
        logger.info(f"Initial target: {start_wp.name} (WP1)")
        self.send_goto_command(start_wp, target_velocity, target_altitude)
        time.sleep(2)  # Allow command processing

        mission_start_time = time.time()
        while not self.mission_complete:
            try:
                # Get current positions
                vehicle_pos = self.get_vehicle_position()
                drone_pos = self.get_drone_position()

                if not vehicle_pos or not drone_pos:
                    logger.warning("Position data unavailable, retrying...")
                    time.sleep(1)
                    continue

                # ================== Speed-Based Navigation Check ==================
                current_time = time.time()
                drone_vehicle_distance = drone_pos.distance_to(vehicle_pos)
                vehicle_speed = vehicle_pos.speed

                # Speed skip conditions (with cooldown and single-waypoint enforcement)
                if (drone_vehicle_distance <= 200 and
                        vehicle_speed > 10 and
                        current_time - self.last_speed_skip_time > speed_skip_cooldown):

                    # Calculate safe next index (never skip more than 1 waypoint)
                    new_index = min(self.current_target_index + 1, len(self.optimized_path) - 1)

                    if new_index != self.current_target_index:
                        logger.warning(f"🚨 Speed trigger: {vehicle_speed:.1f}m/s @ {drone_vehicle_distance:.0f}m")
                        logger.info(f"Advancing from WP{self.current_target_index + 1} to WP{new_index + 1}")

                        self.current_target_index = new_index
                        next_wp = self.optimized_path[self.current_target_index]
                        self.send_goto_command(next_wp, target_velocity, target_altitude)

                        self.last_speed_skip_time = current_time
                        self._reset_tracking_vars()
                        time.sleep(1)  # Command processing buffer
                        continue  # Skip normal processing this iteration
                # ================== End Speed Check ==================

                # ================== Normal Waypoint Processing ==================
                # Find next valid waypoint using standard logic
                calculated_target = self._find_next_unpassed_waypoint(vehicle_pos)

                # Enforce single-waypoint progression
                if calculated_target != self.current_target_index:
                    safe_target = min(calculated_target, self.current_target_index + 1)

                    if safe_target != self.current_target_index:
                        logger.info(f"Standard advance to WP{safe_target + 1}")
                        self.current_target_index = safe_target
                        next_wp = self.optimized_path[self.current_target_index]
                        self.send_goto_command(next_wp, target_velocity, target_altitude)

                # ================== Position Monitoring ==================
                # Log detailed status every 10 seconds
                if int(time.time()) % 10 == 0:
                    logger.info(
                        f"Drone: {drone_pos.lat:.6f}, {drone_pos.lon:.6f} "
                        f"| Vehicle: {vehicle_pos.lat:.6f}, {vehicle_pos.lon:.6f} "
                        f"| Speed: {vehicle_speed:.1f}m/s | Distance: {drone_vehicle_distance:.0f}m"
                    )

                    if self.current_target_index < len(self.optimized_path):
                        current_target = self.optimized_path[self.current_target_index]
                        target_distance = drone_pos.distance_to(current_target)
                        logger.info(
                            f"Current target (WP{self.current_target_index + 1}): {target_distance:.0f}m remaining")

                # ================== Final Waypoint Handling ==================
                if self.current_target_index >= len(self.optimized_path) - 1:
                    final_wp = self.optimized_path[-1]
                    if self._is_drone_at_waypoint(final_wp):
                        logger.info("✅ Final waypoint achieved")
                        self.mission_complete = True
                    else:
                        self.send_goto_command(final_wp, target_velocity, target_altitude)

                # ================== Mission Timeout ==================
                if time.time() - mission_start_time > 3600:  # 1 hour timeout
                    logger.error("⏰ Mission timeout reached!")
                    break

                time.sleep(0.5)  # Main loop interval

            except Exception as e:
                logger.error(f"Mission error: {str(e)}")
                time.sleep(1)

        # Final verification and cleanup
        logger.warning("Executing final stop commands")
        for _ in range(2):
            self.send_hover_command()
            time.sleep(0.1)

        return self._ensure_final_waypoint_reached(target_velocity, target_altitude)

    def _find_next_unpassed_waypoint(self, vehicle_pos):
        """Find the first waypoint the vehicle hasn't passed"""
        start_idx = self.current_target_index
        end_idx = min(start_idx + self.lookahead_window, len(self.optimized_path))

        for idx in range(start_idx, end_idx):
            wp = self.optimized_path[idx]
            distance = vehicle_pos.distance_to(wp)

            # Update minimum distance
            if idx not in self.waypoint_min_distances or distance < self.waypoint_min_distances[idx]:
                self.waypoint_min_distances[idx] = distance

            # Check 5% threshold
            threshold = self.waypoint_min_distances[idx] * 1.05
            if distance > threshold:
                logger.debug(f"WP{idx + 1} passed (current: {distance:.2f}m, threshold: {threshold:.2f}m)")
                continue

            return idx  # First unpassed waypoint

        # If all in window are passed, jump ahead
        return min(end_idx, len(self.optimized_path) - 1)

    def _handle_waypoint_skips(self, target_velocity, target_altitude):
        """Process vehicle-triggered waypoint skips"""
        vehicle_pos = self.get_vehicle_position()
        if not vehicle_pos:
            return

        if self._check_skipped_waypoints(vehicle_pos):
            if self.current_target_index < len(self.optimized_path):
                new_target = self.optimized_path[self.current_target_index]
                logger.info(f"🔄 New target: WP{self.current_target_index + 1} ({new_target.name})")
                self._send_waypoint_command(new_target, target_velocity, target_altitude)
                self.at_waypoint = False
                self.is_hovering = False

    def _send_waypoint_command(self, waypoint, velocity, altitude):
        """Unified waypoint navigation command"""
        try:
            self.vehicle.mav.command_long_send(
                self.vehicle.target_system,
                self.vehicle.target_component,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,  # Confirmation
                0,  # Hold time
                5,  # Acceptance radius (m)
                10,  # Pass radius (m)
                float('nan'),  # Yaw
                waypoint.lat,
                waypoint.lon,
                altitude if altitude else waypoint.elevation
            )
            logger.info(f"📍 Navigating to {waypoint.name} (WP{self.current_target_index + 1})")
        except Exception as e:
            logger.error(f"Command failed: {str(e)}")

    def _reset_tracking_vars(self):
        """Reset tracking variables for new waypoint"""
        self.min_distance_recorded = float('inf')
        self.distance_increasing = False
        self.distance_buffer.clear()
        logger.debug("Reset tracking variables for new waypoint")

    def _set_waypoint_altitudes(self, altitude):
        """Apply altitude override to all waypoints"""
        if altitude:
            logger.info(f"⚙️ Setting all waypoints to {altitude}m")
            for wp in self.optimized_path:
                wp.elevation = altitude

    def _ensure_final_waypoint_reached(self, target_velocity, target_altitude):
        """Guarantee drone reaches final waypoint"""
        final_wp = self.optimized_path[-1]
        start_time = time.time()

        while not self._is_drone_at_waypoint(final_wp):
            if time.time() - start_time > 300:
                logger.error("Timeout reaching final waypoint")
                return False
            self.send_goto_command(final_wp, target_velocity, target_altitude)
            time.sleep(1)

        logger.info("✅ Final waypoint confirmed")
        return True

    def save_mission_to_json(self, filename):
        """Save the mission plan to a JSON file."""
        # Convert to serializable format
        serializable_waypoints = [wp.to_dict() for wp in self.waypoints]
        serializable_optimized_path = [wp.to_dict() for wp in self.optimized_path]

        mission_data = {
            "waypoints": serializable_waypoints,
            "optimized_path": serializable_optimized_path
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
        self.optimized_path = []

        # Load waypoints
        for wp_data in mission_data["waypoints"]:
            self.add_waypoint(
                wp_data["lat"],
                wp_data["lon"],
                wp_data["elevation"],
                wp_data["name"]
            )

        # Load optimized path if available
        if "optimized_path" in mission_data:
            for wp_data in mission_data["optimized_path"]:
                geo_point = GeoPoint(
                    wp_data["lat"],
                    wp_data["lon"],
                    wp_data["elevation"],
                    wp_data["name"]
                )
                self.optimized_path.append(geo_point)

        logger.info(f"Mission loaded from {filename}")

    def cleanup(self):
        """Force immediate mission termination and vehicle stop"""
        logger.warning("INITIATING EMERGENCY STOP PROCEDURE")

        # Atomic mission termination
        self.mission_complete = True

        # Stop GPS updates
        if self.vehicle_gps:
            self.vehicle_gps.stop()

        # Force vehicle stop
        if self.vehicle:
            # Send 3 emergency stop commands
            for i in range(3):
                try:
                    # MAVLink override command
                    self.vehicle.mav.command_long_send(
                        self.vehicle.target_system,
                        self.vehicle.target_component,
                        mavutil.mavlink.MAV_CMD_OVERRIDE_GOTO,
                        0,
                        mavutil.mavlink.MAV_GOTO_DO_HOLD,  # Immediate stop
                        0, 0, 0, 0, 0, 0, 0
                    )
                    # Send hover command
                    self.send_hover_command()
                    logger.warning(f"Emergency stop sequence {i + 1}/3 sent")
                    time.sleep(0.1)
                except Exception as e:
                    logger.error(f"Stop command failed: {str(e)}")

        # Final hover command
        self.send_hover_command()
        logger.warning("CLEANUP COMPLETED - DRONE SHOULD BE STOPPED")
        return True


def main():
    import argparse

    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Drone Path Navigator')
    parser.add_argument('--connection', type=str, default='udp:127.0.0.1:14551',
                        help='MAVLink connection string')
    parser.add_argument('--gps-port', type=str, default='/dev/ttyUSB0',
                        help='Serial port for vehicle GPS')
    parser.add_argument('--gps-baud', type=int, default=9600,
                        help='Baud rate for GPS')
    parser.add_argument('--altitude', type=float, default=70.0,
                        help='Drone flight altitude in meters')
    parser.add_argument('--waypoint-distance', type=float, default=200.0,
                        help='Minimum distance between waypoints in meters')
    parser.add_argument('--threshold', type=float, default=10.0,
                        help='Distance threshold in meters to consider vehicle passing a waypoint')
    parser.add_argument('--save-mission', type=str, default=None,
                        help='Save mission to JSON file')
    parser.add_argument('--load-mission', type=str, default=None,
                        help='Load mission from JSON file')

    args = parser.parse_args()

    # Log the parsed arguments
    logger.info(f"Starting mission with parameters:")
    logger.info(f"Connection: {args.connection}")
    logger.info(f"Vehicle GPS port: {args.gps_port}")
    logger.info(f"Vehicle GPS baud rate: {args.gps_baud}")
    logger.info(f"Target altitude: {args.altitude} meters")
    logger.info(f"Minimum waypoint distance: {args.waypoint_distance} meters")
    logger.info(f"Vehicle detection threshold: {args.threshold} meters")

    # Create the drone navigator
    navigator = DroneNavigator(
        max_points=1000,
        vehicle_gps_port=args.gps_port,
        vehicle_gps_baudrate=args.gps_baud
    )

    # Update distance threshold if specified
    if args.threshold:
        navigator.distance_threshold = args.threshold
        logger.info(f"Using distance threshold: {navigator.distance_threshold}m")

    try:
        # Load mission or add default waypoints
        if args.load_mission:
            navigator.load_mission_from_json(args.load_mission)
            logger.info(f"Loaded mission from {args.load_mission}")
        else:
            # Add example waypoints (same as in the original file)
            navigator.add_waypoint(23.191849, 77.364854, args.altitude, "Point A")
            navigator.add_waypoint(23.190986, 77.365937, args.altitude, "Point B")
            navigator.add_waypoint(23.190355, 77.367997, args.altitude, "Point C")
            navigator.add_waypoint(23.192386, 77.370830, args.altitude, "Point D")
            navigator.add_waypoint(23.195517, 77.371050, args.altitude, "Point E")
            navigator.add_waypoint(23.197110, 77.369097, args.altitude, "Point F")
            logger.info(f"Added default waypoints at altitude {args.altitude}m")

        # Calculate the optimized drone path
        optimized_path = navigator.calculate_drone_path(min_waypoint_distance=args.waypoint_distance)
        logger.info(f"Generated optimized path with {len(optimized_path)} waypoints")

        # Print path summary
        summary = f"Drone Mission with {len(navigator.waypoints)} input waypoints\n"
        summary += f"Optimized to {len(navigator.optimized_path)} flight waypoints\n\n"

        summary += "Input Waypoints:\n"
        for wp in navigator.waypoints:
            summary += f"- {wp}\n"

        summary += "\nFlight Waypoints:\n"
        for i, wp in enumerate(navigator.optimized_path):
            summary += f"- {i + 1}. {wp}\n"

        print(summary)

        # Save the mission to a file if requested
        if args.save_mission:
            navigator.save_mission_to_json(args.save_mission)

        # Connect to vehicle GPS
        if not navigator.connect_gps():
            logger.error("Failed to connect to vehicle GPS. Aborting mission.")
            return

        # Connect to the vehicle
        if navigator.connect_vehicle(args.connection):
            # Setup flight
            if navigator.set_guided_mode() and navigator.arm_vehicle():
                # Takeoff
                if navigator.takeoff(args.altitude):
                    # Execute the vehicle tracking mission
                    logger.info(f"Starting vehicle tracking mission at altitude {args.altitude}m")
                    navigator.execute_vehicle_tracking_mission(target_altitude=args.altitude, target_velocity=10.0)
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
        navigator.cleanup()
        logger.info("Mission ended")


if __name__ == "__main__":
    main()