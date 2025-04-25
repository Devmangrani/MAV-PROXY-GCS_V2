import time
import logging
import threading
import serial
import pynmea2
from pymavlink import mavutil
import argparse
import math
import numpy as np

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('gps_sitl.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


class GPSSITLFollow:
    def __init__(self, target_altitude=10, target_velocity=5, gps_port='/tmp/vgps1', gps_baudrate=9600,
                 distance_threshold=5.0):
        # SITL settings
        self.vehicle = None
        self._target_altitude = target_altitude
        self._target_velocity = target_velocity
        self.follow_enabled = False
        self.has_taken_off = False
        self.is_armed = False
        self.is_guided = False

        # Distance threshold in meters
        self.distance_threshold = distance_threshold

        # Current drone position
        self.drone_lat = None
        self.drone_lon = None
        self.drone_alt = None

        # Initialize serial port attributes
        self.serial_port = None
        self.gps_port = gps_port
        self.gps_baudrate = gps_baudrate

        self.last_command_time = 0
        self.command_frequency = 0.05  # 20Hz command rate

        # Updated position+velocity bitmask (0x0DC0 = 3520 decimal)
        # This enables position targets while also allowing velocity commands
        self.position_velocity_mask = 0b0000110111000000  # 0x0DC0

        # For smoothing acceleration/deceleration
        self.last_velocity_north = 0
        self.last_velocity_east = 0
        self.max_accel = 1.0  # m/s² - maximum acceleration change

    def is_vehicle_flying(self):
        """Check if vehicle is currently flying by checking armed state and altitude"""
        if not self.vehicle:
            return False

        # Check armed state
        self.check_vehicle_state()
        if not self.is_armed:
            return False

        # Check current altitude
        current_alt = self.get_current_altitude()
        if current_alt is None or current_alt < 1.0:  # Consider vehicle flying if above 1 meter
            return False

        return True

    def adjust_altitude(self, new_altitude):
        """Adjust vehicle altitude using set_position_target_global_int"""
        if not self.is_vehicle_flying():
            logger.warning("Vehicle not flying - cannot adjust altitude")
            return False

        try:
            # Get current position
            pos_msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if not pos_msg:
                logger.error("Could not get current position")
                return False

            # Log current and target altitudes
            current_alt = pos_msg.relative_alt / 1000.0  # Convert mm to meters
            logger.info(f"Current altitude: {current_alt:.1f}m, Target altitude: {new_altitude:.1f}m")

            # Only adjust if altitude difference is significant (more than 0.5m)
            if abs(current_alt - new_altitude) <= 0.5:
                logger.info("Already at target altitude")
                return True

            # Use set_position_target_global_int instead of DO_REPOSITION
            # Keep current lat/lon, only change altitude
            type_mask = (
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
                0,  # timestamp
                self.vehicle.target_system,
                self.vehicle.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                type_mask,
                pos_msg.lat,  # Current latitude
                pos_msg.lon,  # Current longitude
                new_altitude,  # New target altitude
                0, 0, 0,  # velocity
                0, 0, 0,  # acceleration
                0, 0  # yaw, yaw_rate
            )

            # Monitor altitude change
            start_time = time.time()
            while time.time() - start_time < 120:  # 10 second timeout
                current_pos = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
                if current_pos:
                    current_alt = current_pos.relative_alt / 1000.0
                    logger.info(f"Adjusting altitude: Current: {current_alt:.1f}m, Target: {new_altitude:.1f}m")
                    if abs(current_alt - new_altitude) <= 0.5:
                        logger.info(f"Successfully adjusted to target altitude: {current_alt:.1f}m")
                        return True
                time.sleep(0.5)

            logger.error("Timeout waiting for altitude adjustment")
            return False

        except Exception as e:
            logger.error(f"Error adjusting altitude: {e}")
            return False

    def update_drone_position(self):
        """Update stored drone position from GLOBAL_POSITION_INT messages
        Note: This method is not used in the new implementation to avoid simultaneous reads
        """
        if not self.vehicle:
            return False

        pos_msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if pos_msg:
            # Update stored position (convert from 1e7 to decimal degrees)
            self.drone_lat = pos_msg.lat / 1e7
            self.drone_lon = pos_msg.lon / 1e7
            self.drone_alt = pos_msg.relative_alt / 1000.0  # mm to meters
            return True
        return False

    def calculate_distance(self, lat1, lon1, lat2, lon2):
        """
        Calculate distance between two GPS coordinates using Haversine formula
        Returns distance in meters
        """
        # Earth radius in meters
        R = 6371000.0

        # Convert decimal degrees to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)

        # Differences
        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad

        # Haversine formula
        a = math.sin(dlat / 2) ** 2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c

        return distance

    @property
    def target_velocity(self):
        return self._target_velocity

    @target_velocity.setter
    def target_velocity(self, value):
        if not isinstance(value, (int, float)):
            raise ValueError("Velocity must be a number")
        if value < 1 or value > 20:  # Safety limits
            raise ValueError("Velocity must be between 1 and 20 meters per second")
        self._target_velocity = float(value)
        logger.info(f"Target velocity set to: {self._target_velocity}m/s")

    @property
    def target_altitude(self):
        return self._target_altitude

    @target_altitude.setter
    def target_altitude(self, value):
        """Set target altitude with proper validation and adjustment"""
        if not isinstance(value, (int, float)):
            raise ValueError("Altitude must be a number")
        if value < 1 or value > 100:  # Safety limits
            raise ValueError("Altitude must be between 1 and 100 meters")

        # Store new target altitude
        old_altitude = self._target_altitude
        self._target_altitude = float(value)
        logger.info(f"Target altitude updated: {old_altitude:.1f}m -> {self._target_altitude:.1f}m")

        # If vehicle is flying, adjust to new altitude
        if self.is_vehicle_flying():
            self.adjust_altitude(self._target_altitude)

    def connect_vehicle(self):
        connection_string = 'udp:127.0.0.1:14551'
        logger.info(f"Connecting to SITL on {connection_string}")

        try:
            self.vehicle = mavutil.mavlink_connection(connection_string)
            self.vehicle.wait_heartbeat()
            logger.info("Connected to vehicle")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to vehicle: {e}")
            return False

    def connect_gps(self):
        """Connect to GPS device"""
        try:
            self.serial_port = serial.Serial(
                port=self.gps_port,
                baudrate=self.gps_baudrate,
                timeout=1
            )
            logger.info(f"Connected to GPS on {self.gps_port}")
            return True
        except serial.SerialException as e:
            logger.error(f"Failed to open GPS serial port: {e}")
            return False

    def verify_checksum(self, sentence):
        """Verify NMEA sentence checksum"""
        try:
            if '*' not in sentence:
                return False
            data, checksum = sentence.split('*')
            if not data.startswith('$'):
                return False

            calculated_checksum = 0
            for char in data[1:]:
                calculated_checksum ^= ord(char)

            return format(calculated_checksum, '02X') == checksum.strip()[:2]
        except Exception:
            return False

    def check_vehicle_state(self):
        """Check current vehicle state"""
        if not self.vehicle:
            return False

        msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg:
            self.is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            self.is_guided = msg.custom_mode == 4  # GUIDED mode
            return True
        return False

    def set_guided_mode(self):
        """Set vehicle mode to GUIDED with state checking"""
        if not self.vehicle:
            return False

        # Check if already in GUIDED mode
        self.check_vehicle_state()
        if self.is_guided:
            logger.info("Vehicle already in GUIDED mode")
            return True

        # Original guided mode setting code
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
            if self.check_vehicle_state() and self.is_guided:
                logger.info("Vehicle mode set to GUIDED")
                return True
            time.sleep(0.1)

        logger.error("Failed to set GUIDED mode")
        return False

    def arm_vehicle(self):
        """Arm the vehicle with state checking"""
        if not self.vehicle:
            return False

        # Check if already armed
        self.check_vehicle_state()
        if self.is_armed:
            logger.info("Vehicle already armed")
            return True

        # Original arming code
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )

        # Wait for arming
        for _ in range(30):
            if self.check_vehicle_state() and self.is_armed:
                logger.info("Vehicle armed")
                return True
            time.sleep(0.1)

        logger.error("Failed to arm vehicle")
        return False

    def get_current_altitude(self):
        """Get current relative altitude"""
        if not self.vehicle:
            return None

        msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            self.current_alt = msg.relative_alt / 1000.0
            return self.current_alt
        return None

    def follow_gps(self):
        if not self.serial_port or not self.serial_port.is_open:
            if not self.connect_gps():
                logger.error("Failed to connect to GPS")
                return

        self.follow_enabled = True
        logger.info(f"Starting GPS follow with position+velocity control")

        # Store latest drone position and lock to prevent simultaneous reads
        drone_position_lock = threading.Lock()
        latest_drone_position = {
            'lat': None,
            'lon': None,
            'alt': None
        }

        # Initialize Kalman filter state
        # State: [lat, lon, v_lat, v_lon]
        x = np.zeros((4, 1))
        P = np.eye(4)  # Covariance matrix

        # Base filter parameters
        R_base = np.array([[0.005, 0], [0, 0.005]])  # Base measurement noise
        Q = np.array([
            [0.0005, 0, 0, 0],
            [0, 0.0005, 0, 0],
            [0, 0, 0.005, 0],
            [0, 0, 0, 0.005]
        ])

        # Measurement matrix
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])

        # Position history for additional smoothing
        position_history = []
        default_history_length = 10  # Base history length that gets adjusted dynamically
        current_max_history = default_history_length  # Initialize with default value

        # Previous GPS reading for velocity calculation
        prev_lat = None
        prev_lon = None
        prev_time = time.time()
        filter_initialized = False

        # For velocity calculation and smoothing
        last_velocity_north = 0
        last_velocity_east = 0
        max_accel = self.max_accel if hasattr(self, 'max_accel') else 1.0  # m/s²

        # Store GPS-provided velocity
        gps_vel_north = None
        gps_vel_east = None
        last_velocity_update = 0

        # Initialize current_speed with a default value
        current_speed = 0
        LOW_SPEED_THRESHOLD = 1.0  # m/s

        # Start separate thread for position monitoring to avoid simultaneous socket reads
        def update_position_thread():
            while self.follow_enabled:
                try:
                    if self.vehicle:
                        msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
                        if msg:
                            with drone_position_lock:
                                latest_drone_position['lat'] = msg.lat / 1e7
                                latest_drone_position['lon'] = msg.lon / 1e7
                                latest_drone_position['alt'] = msg.relative_alt / 1000.0
                    time.sleep(0.1)  # Small delay to prevent excessive CPU usage
                except Exception as e:
                    logger.warning(f"Position update thread error: {e}")
                    time.sleep(0.5)  # Longer delay if there's an error

        # Start position update thread
        position_thread = threading.Thread(target=update_position_thread)
        position_thread.daemon = True
        position_thread.start()

        while self.follow_enabled:
            try:
                current_time = time.time()
                dt = current_time - prev_time  # Time delta for Kalman filter

                if current_time - self.last_command_time >= self.command_frequency:
                    line = self.serial_port.readline().decode('ascii', errors='ignore')

                    if not line or not line.startswith('$'):
                        continue

                    if not self.verify_checksum(line):
                        continue

                    try:
                        msg = pynmea2.parse(line)

                        # Extract velocity data from RMC sentence
                        if msg.sentence_type == 'RMC' and hasattr(msg, 'spd_over_grnd') and hasattr(msg, 'true_course'):
                            # RMC sentence has speed in knots, convert to m/s
                            if msg.spd_over_grnd is not None and msg.true_course is not None:
                                current_speed = float(msg.spd_over_grnd) * 0.514444  # knots to m/s
                                current_course = math.radians(float(msg.true_course))

                                # Calculate velocity components from speed and course
                                gps_vel_north = current_speed * math.cos(current_course)
                                gps_vel_east = current_speed * math.sin(current_course)

                                # Apply acceleration limiting for smooth transitions
                                dt_cmd = current_time - last_velocity_update if last_velocity_update > 0 else self.command_frequency
                                last_velocity_update = current_time

                                # Limit acceleration in north direction
                                delta_v_north = gps_vel_north - last_velocity_north
                                if abs(delta_v_north) > max_accel * dt_cmd:
                                    delta_v_north = math.copysign(max_accel * dt_cmd, delta_v_north)
                                gps_vel_north = last_velocity_north + delta_v_north

                                # Limit acceleration in east direction
                                delta_v_east = gps_vel_east - last_velocity_east
                                if abs(delta_v_east) > max_accel * dt_cmd:
                                    delta_v_east = math.copysign(max_accel * dt_cmd, delta_v_east)
                                gps_vel_east = last_velocity_east + delta_v_east

                                # Store for next iteration
                                last_velocity_north = gps_vel_north
                                last_velocity_east = gps_vel_east

                                logger.debug(
                                    f"GPS Velocity: {current_speed:.2f} m/s, Course: {math.degrees(current_course):.1f}°, "
                                    f"Components: N={gps_vel_north:.2f}, E={gps_vel_east:.2f}")

                        # Alternatively, extract velocity from VTG sentence
                        elif msg.sentence_type == 'VTG' and hasattr(msg, 'spd_over_grnd_kmph') and hasattr(msg,
                                                                                                           'true_track'):
                            # VTG has speed in km/h, convert to m/s
                            if msg.spd_over_grnd_kmph is not None and msg.true_track is not None:
                                current_speed = float(msg.spd_over_grnd_kmph) / 3.6  # km/h to m/s
                                current_course = math.radians(float(msg.true_track))

                                # Calculate velocity components from speed and course
                                gps_vel_north = current_speed * math.cos(current_course)
                                gps_vel_east = current_speed * math.sin(current_course)

                                # Apply acceleration limiting for smooth transitions
                                dt_cmd = current_time - last_velocity_update if last_velocity_update > 0 else self.command_frequency
                                last_velocity_update = current_time

                                # Limit acceleration in north direction
                                delta_v_north = gps_vel_north - last_velocity_north
                                if abs(delta_v_north) > max_accel * dt_cmd:
                                    delta_v_north = math.copysign(max_accel * dt_cmd, delta_v_north)
                                gps_vel_north = last_velocity_north + delta_v_north

                                # Limit acceleration in east direction
                                delta_v_east = gps_vel_east - last_velocity_east
                                if abs(delta_v_east) > max_accel * dt_cmd:
                                    delta_v_east = math.copysign(max_accel * dt_cmd, delta_v_east)
                                gps_vel_east = last_velocity_east + delta_v_east

                                # Store for next iteration
                                last_velocity_north = gps_vel_north
                                last_velocity_east = gps_vel_east

                                logger.debug(
                                    f"GPS Velocity (VTG): {current_speed:.2f} m/s, Track: {math.degrees(current_course):.1f}°, "
                                    f"Components: N={gps_vel_north:.2f}, E={gps_vel_east:.2f}")

                        # Process position data from GGA sentence
                        elif msg.sentence_type == 'GGA':
                            if msg.latitude and msg.longitude:
                                # Convert GPS coordinates to float
                                raw_lat = float(msg.latitude)
                                raw_lon = float(msg.longitude)

                                # ----- Kalman filtering implementation -----
                                if not filter_initialized:
                                    # First measurement - initialize state
                                    x[0, 0] = raw_lat
                                    x[1, 0] = raw_lon
                                    prev_lat = raw_lat
                                    prev_lon = raw_lon
                                    filter_initialized = True
                                else:
                                    # Calculate current speed if we have previous positions and we don't have a GPS-provided speed
                                    if prev_lat is not None and prev_lon is not None and (
                                            current_time - last_velocity_update) >= 2.0:
                                        # Estimate velocity from previous position
                                        v_lat = (raw_lat - prev_lat) / dt
                                        v_lon = (raw_lon - prev_lon) / dt

                                        # Estimate speed in meters/second (approximate)
                                        # 1 degree latitude ≈ 111km, 1 degree longitude varies with latitude
                                        lat_meters_per_degree = 111000  # approximate
                                        lon_meters_per_degree = 111000 * math.cos(math.radians(raw_lat))

                                        v_lat_mps = v_lat * lat_meters_per_degree
                                        v_lon_mps = v_lon * lon_meters_per_degree
                                        current_speed = math.sqrt(v_lat_mps ** 2 + v_lon_mps ** 2)

                                        # Update previous position for next iteration
                                        prev_lat = raw_lat
                                        prev_lon = raw_lon
                                    elif prev_lat is None or prev_lon is None:
                                        # Initialize previous position if not set
                                        prev_lat = raw_lat
                                        prev_lon = raw_lon

                                    # Adaptive filtering based on speed
                                    if current_speed < LOW_SPEED_THRESHOLD:
                                        # More aggressive filtering for slow speeds
                                        # Scale inversely with speed - slower = more filtering
                                        speed_factor = max(0.1, current_speed / LOW_SPEED_THRESHOLD)

                                        # Increase measurement noise at low speeds (trust GPS less)
                                        R_adaptive = R_base / speed_factor

                                        # Also modify max history length for more smoothing at low speeds
                                        current_max_history = min(15, int(default_history_length / speed_factor))
                                    else:
                                        # Normal filtering at higher speeds
                                        R_adaptive = R_base
                                        current_max_history = 5  # Less smoothing at higher speeds

                                    # Update state transition matrix with dt
                                    F = np.array([
                                        [1, 0, dt, 0],
                                        [0, 1, 0, dt],
                                        [0, 0, 1, 0],
                                        [0, 0, 0, 1]
                                    ])

                                    # 1. Prediction step
                                    x = F @ x
                                    P = F @ P @ F.T + Q

                                    # 2. Update step with measurement
                                    z = np.array([[raw_lat], [raw_lon]])
                                    y = z - H @ x  # Measurement residual
                                    S = H @ P @ H.T + R_adaptive  # Innovation covariance
                                    K = P @ H.T @ np.linalg.inv(S)  # Kalman gain

                                    x = x + K @ y  # Updated state estimate
                                    P = (np.eye(4) - K @ H) @ P  # Updated covariance

                                # Extract filtered position
                                filtered_lat = float(x[0, 0])
                                filtered_lon = float(x[1, 0])

                                # Add to position history with adaptive length
                                position_history.append((filtered_lat, filtered_lon))
                                if len(position_history) > current_max_history:
                                    position_history.pop(0)

                                # Apply weighted moving average for final smoothing
                                if len(position_history) > 1:
                                    # More weight to recent positions at higher speeds
                                    if current_speed >= LOW_SPEED_THRESHOLD:
                                        weights = np.linspace(0.5, 1.5, len(position_history))
                                    else:
                                        # More weight to older positions at lower speeds
                                        weights = np.linspace(1.5, 0.5, len(position_history))

                                    weights = weights / np.sum(weights)  # Normalize weights

                                    smoothed_lat = sum(w * p[0] for w, p in zip(weights, position_history))
                                    smoothed_lon = sum(w * p[1] for w, p in zip(weights, position_history))
                                else:
                                    smoothed_lat, smoothed_lon = filtered_lat, filtered_lon

                                # ----- End of filtering implementation -----

                                # Get current drone position safely
                                with drone_position_lock:
                                    drone_lat = latest_drone_position['lat']
                                    drone_lon = latest_drone_position['lon']

                                # Skip if we don't have current drone position yet
                                if drone_lat is None or drone_lon is None:
                                    logger.debug("Waiting for drone position data...")
                                    continue

                                # Calculate distance between target and current position
                                distance = self.calculate_distance(
                                    drone_lat, drone_lon,
                                    smoothed_lat, smoothed_lon  # Use smoothed coordinates
                                )

                                # Determine if we should use GPS-provided velocity or calculate from bearing
                                use_gps_velocity = (gps_vel_north is not None and gps_vel_east is not None and
                                                    (
                                                                current_time - last_velocity_update) < 2.0)  # Use GPS velocity if fresh

                                if not use_gps_velocity:
                                    # Fall back to calculated velocity from bearing and distance
                                    # Calculate bearing to target
                                    lat1_rad = math.radians(drone_lat)
                                    lon1_rad = math.radians(drone_lon)
                                    lat2_rad = math.radians(smoothed_lat)
                                    lon2_rad = math.radians(smoothed_lon)

                                    # Calculate bearing
                                    y = math.sin(lon2_rad - lon1_rad) * math.cos(lat2_rad)
                                    x_bearing = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(
                                        lat2_rad) * math.cos(lon2_rad - lon1_rad)
                                    bearing = math.atan2(y, x_bearing)

                                    # Calculate velocity components based on distance (linear scaling)
                                    max_speed = self._target_velocity
                                    min_speed = 0.5  # Minimum speed to maintain when close to target

                                    # Scale speed based on distance, but keep a minimum to avoid stopping completely
                                    if distance > 20:
                                        target_speed = max_speed
                                    else:
                                        # Linear interpolation: speed = min_speed + (max_speed - min_speed) * (distance / 20)
                                        target_speed = min_speed + (max_speed - min_speed) * (distance / 20)

                                    # Calculate velocity components
                                    velocity_north = target_speed * math.cos(bearing)
                                    velocity_east = target_speed * math.sin(bearing)

                                    # Apply acceleration limiting to smooth velocity changes
                                    dt_cmd = self.command_frequency  # Time delta between commands

                                    # Limit acceleration in north direction
                                    delta_v_north = velocity_north - last_velocity_north
                                    if abs(delta_v_north) > max_accel * dt_cmd:
                                        delta_v_north = math.copysign(max_accel * dt_cmd, delta_v_north)
                                    velocity_north = last_velocity_north + delta_v_north

                                    # Limit acceleration in east direction
                                    delta_v_east = velocity_east - last_velocity_east
                                    if abs(delta_v_east) > max_accel * dt_cmd:
                                        delta_v_east = math.copysign(max_accel * dt_cmd, delta_v_east)
                                    velocity_east = last_velocity_east + delta_v_east

                                    # Store for next iteration
                                    last_velocity_north = velocity_north
                                    last_velocity_east = velocity_east
                                else:
                                    # Use GPS-provided velocity (already smoothed with acceleration limiting)
                                    velocity_north = gps_vel_north
                                    velocity_east = gps_vel_east

                                # Log the calculated distance and velocity
                                logger.info(
                                    f"Distance to target: {distance:.2f}m\n"
                                    f"Raw GPS: {raw_lat:.6f}, {raw_lon:.6f}\n"
                                    f"Filtered: {filtered_lat:.6f}, {filtered_lon:.6f}\n"
                                    f"Smoothed: {smoothed_lat:.6f}, {smoothed_lon:.6f}\n"
                                    f"Velocity Source: {'GPS' if use_gps_velocity else 'Calculated'}\n"
                                    f"Velocity (N,E): {velocity_north:.2f}, {velocity_east:.2f} m/s"
                                )

                                # Use the position+velocity bitmask (0x0DC0)
                                position_velocity_mask = 0b0000110111000000  # 0x0DC0

                                # Send position target command with velocity components
                                self.vehicle.mav.set_position_target_global_int_send(
                                    0,  # timestamp
                                    self.vehicle.target_system,
                                    self.vehicle.target_component,
                                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                    position_velocity_mask,  # Position + velocity control
                                    int(smoothed_lat * 1e7),  # Target latitude
                                    int(smoothed_lon * 1e7),  # Target longitude
                                    self.target_altitude,  # Target altitude
                                    velocity_north, velocity_east, 0,  # Velocity components (NED)
                                    0, 0, 0,  # acceleration (not used)
                                    0, 0  # yaw, yaw_rate (not used)
                                )

                                self.last_command_time = current_time
                                prev_time = current_time  # Update for next Kalman cycle

                                # Log movement command
                                logger.info(
                                    f"Position+Velocity command sent (distance: {distance:.2f}m):\n"
                                    f"Current Position: {drone_lat:.6f}, {drone_lon:.6f}\n"
                                    f"Target Position: {smoothed_lat:.6f}, {smoothed_lon:.6f}\n"
                                    f"Target Altitude: {self.target_altitude:.1f}m\n"
                                    f"Velocity Vector: N={velocity_north:.2f} m/s, E={velocity_east:.2f} m/s"
                                )

                    except pynmea2.ParseError:
                        continue
                    except ValueError as e:
                        logger.warning(f"Error parsing GPS data: {e}")
                        continue
                    except np.linalg.LinAlgError as e:
                        logger.warning(f"Matrix calculation error: {e}")
                        continue

            except serial.SerialException as e:
                logger.error(f"GPS Serial error: {e}")
                break
            except Exception as e:
                logger.error(f"Error in follow thread: {e}")
                logger.exception(e)
                continue

    def check_and_handle_altitude(self):
        """Check current state and handle altitude appropriately"""
        if not self.vehicle:
            logger.error("No vehicle connection")
            return False

        # First check if we're already flying
        flying = self.is_vehicle_flying()
        current_alt = self.get_current_altitude()

        if current_alt is None:
            logger.error("Could not get current altitude")
            return False

        logger.info(
            f"Current state - Flying: {flying}, Altitude: {current_alt:.1f}m, Target: {self.target_altitude:.1f}m")

        if flying:
            # Vehicle is already flying, use position target if needed
            if abs(current_alt - self.target_altitude) > 0.5:
                logger.info("Vehicle already flying - adjusting altitude")
                return self.adjust_altitude(self.target_altitude)
            else:
                logger.info("Already at target altitude")
                return True
        else:
            # Not flying, need to takeoff
            logger.info("Vehicle not flying - initiating takeoff")
            return self.initiate_takeoff()

    def initiate_takeoff(self):
        """Handle the actual takeoff command"""
        logger.info(f"Initiating takeoff to {self.target_altitude}m")
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0, self.target_altitude
        )

        # Monitor takeoff progress
        start_time = time.time()
        while time.time() - start_time < 30:
            current_alt = self.get_current_altitude()
            if current_alt is not None:
                logger.info(f"Current altitude during takeoff: {current_alt:.1f}m")
                if abs(current_alt - self.target_altitude) < 1.0:
                    self.has_taken_off = True
                    logger.info(f"Reached target altitude: {self.target_altitude}m")
                    return True
            time.sleep(0.1)

        logger.error("Takeoff timeout")
        return False

    def takeoff(self):
        """Smart takeoff with state checking"""
        return self.check_and_handle_altitude()

    # Update start_mission to use this logic
    def start_mission(self):
        """Start the GPS follow mission"""
        # Connect to vehicle
        if not self.connect_vehicle():
            return False

        # Set up vehicle
        if not self.set_guided_mode() or not self.arm_vehicle():
            return False

        # Use new altitude check and handling
        if not self.check_and_handle_altitude():
            return False

        # Start follow thread
        follow_thread = threading.Thread(target=self.follow_gps)
        follow_thread.daemon = True
        follow_thread.start()

        return True

    def cleanup(self):
        """Clean up resources and hold current position"""
        # Signal threads to terminate
        self.follow_enabled = False

        # Allow threads time to notice the termination signal
        time.sleep(0.5)

        # Get and store current position - use non-blocking to avoid collision
        current_position = None
        if self.vehicle:
            try:
                # Use non-blocking recv_match to avoid simultaneous reads
                msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
                if msg:
                    current_position = {
                        'lat': msg.lat,
                        'lon': msg.lon,
                        'alt': self.target_altitude  # Keep the target altitude
                    }
                    logger.warning(f"Current position captured - lat: {current_position['lat'] / 1e7:.6f}, "
                                   f"lon: {current_position['lon'] / 1e7:.6f}, alt: {current_position['alt']:.1f}m")

                    # Position hold command removed as requested

                else:
                    logger.warning("Could not get current position for hold")
            except Exception as e:
                logger.error(f"Error during cleanup position capture: {e}")

        # Close GPS connection
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
                logger.info("GPS connection closed")
            except Exception as e:
                logger.error(f"Error closing GPS connection: {e}")


def main():
    try:
        parser = argparse.ArgumentParser(description='GPS Follow with position+velocity control')
        parser.add_argument('--altitude', type=float, default=10.0,
                            help='Target altitude in meters (1-100)')
        parser.add_argument('--velocity', type=float, default=5.0,
                            help='Target velocity in m/s (1-20)')
        parser.add_argument('--gps-port', type=str, default='/tmp/vgps1',
                            help='GPS device port')
        parser.add_argument('--gps-baud', type=int, default=9600,
                            help='GPS baud rate')
        parser.add_argument('--distance-threshold', type=float, default=1.0,
                            help='Distance threshold in meters (default: 5.0)')
        parser.add_argument('--max-accel', type=float, default=1.0,
                            help='Maximum acceleration in m/s² (default: 1.0)')
        parser.add_argument('--debug', action='store_true',
                            help='Enable debug logging')
        args = parser.parse_args()

        if args.debug:
            logger.setLevel(logging.DEBUG)

        logger.info("Starting GPS Follow with parameters:")
        logger.info(f"  Altitude: {args.altitude}m")
        logger.info(f"  Target Velocity: {args.velocity}m/s")
        logger.info(f"  GPS Port: {args.gps_port}")
        logger.info(f"  GPS Baud Rate: {args.gps_baud}")
        logger.info(f"  Distance Threshold: {args.distance_threshold}m")
        logger.info(f"  Maximum Acceleration: {args.max_accel}m/s²")
        logger.info(f"  Control Type: Position + Velocity (0x0DC0)")

        gps_follow = GPSSITLFollow(
            target_altitude=args.altitude,
            target_velocity=args.velocity,
            gps_port=args.gps_port,
            gps_baudrate=args.gps_baud,
            distance_threshold=args.distance_threshold
        )

        # Set maximum acceleration if provided
        gps_follow.max_accel = args.max_accel

        if gps_follow.start_mission():
            logger.info("Mission started successfully - using position+velocity control for smoother movement")
            try:
                while True:
                    time.sleep(1)
            except KeyboardInterrupt:
                logger.info("Mission interrupted by user")
        else:
            logger.error("Failed to start mission")

    except Exception as e:
        logger.error(f"Mission failed: {e}")
        logger.exception(e)
    finally:
        if 'gps_follow' in locals():
            gps_follow.cleanup()
        logger.info("Cleanup completed")


if __name__ == "__main__":
    main()