import time
import logging
import threading
import serial
import pynmea2
from pymavlink import mavutil
import argparse
import math
import os
import numpy as np
from datetime import datetime as dt


def setup_logging(log_dir=None, use_timestamps=True):
    """
    Configure logging with timestamped files or in a timestamp-based directory

    Args:
        log_dir (str): Directory to store logs. If None, logs are stored in 'logs' directory
        use_timestamps (bool): If True, create a new log file with timestamp for each run
                              If False, create a timestamp directory and save log in it

    Returns:
        logger: Configured logger instance
    """
    import logging
    import os
    from datetime import datetime as dt

    # Create base logs directory if it doesn't exist
    base_log_dir = log_dir if log_dir else 'logs'
    if not os.path.exists(base_log_dir):
        os.makedirs(base_log_dir)

    # Get current timestamp
    timestamp = dt.now().strftime("%Y%m%d_%H%M%S")

    if use_timestamps:
        # Option 1: Create a timestamped log file in the base logs directory
        log_file = os.path.join(base_log_dir, f"gps_sitl_{timestamp}.log")
    else:
        # Option 2: Create a timestamped directory and save log file inside
        log_subdir = os.path.join(base_log_dir, timestamp)
        if not os.path.exists(log_subdir):
            os.makedirs(log_subdir)
        log_file = os.path.join(log_subdir, "gps_sitl.log")

    # Configure logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler(log_file),
            logging.StreamHandler()
        ]
    )
    logger = logging.getLogger(__name__)  # FIXED: removed the 'self.' prefix
    logger.info(f"Logging initialized. Log file: {log_file}")

    return logger


class GPSSITLFollow:
    def __init__(self, target_altitude=10, target_velocity=5, gps_port='/tmp/vgps1', gps_baudrate=9600,
                 distance_threshold=5.0, logger=None):
        # Setup logging first
        self.logger = logger if logger else setup_logging()

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
        self.max_accel = 2.5  # m/s² - maximum acceleration change

        self.Kp_position = 0.001  # Proportional gain for position

        self.Ki_position = 0.0001  # Integral gain for position

        self.Kd_position = 2  # Derivative gain for position

        self.Kp_velocity = 0.001  # Proportional gain for velocity

        self.Ki_velocity = 0.0001  # Integral gain for velocity

        self.Kd_velocity = 2  # Derivative gain for velocity

        # Integral error accumulators

        self.integral_position_north = 0

        self.integral_position_east = 0

        self.integral_velocity_north = 0

        self.integral_velocity_east = 0

        # Error tracking for PD controllers
        self.last_position_error_north = 0
        self.last_position_error_east = 0
        self.last_velocity_error_north = 0
        self.last_velocity_error_east = 0
        self.last_position_error_time = 0
        self.last_velocity_error_time = 0

        # Attitude correction parameters
        self.attitude_source = None
        self.attitude_receive_count = 0
        self.pitch_accumulator = 0.0
        self.roll_accumulator = 0.0
        self.enable_attitude_correction = True
        self.attitude_correction_gain = 0.34
        self.max_pitch = 6.0  # Maximum allowed pitch in degrees
        self.max_roll = 5.0  # Maximum allowed roll in degrees
        self._last_attitude_log = 0

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
            self.logger.warning("Vehicle not flying - cannot adjust altitude")
            return False

        try:
            # Get current position
            pos_msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if not pos_msg:
                self.logger.error("Could not get current position")
                return False

            # Log current and target altitudes
            current_alt = pos_msg.relative_alt / 1000.0  # Convert mm to meters
            self.logger.info(f"Current altitude: {current_alt:.1f}m, Target altitude: {new_altitude:.1f}m")

            # Only adjust if altitude difference is significant (more than 0.5m)
            if abs(current_alt - new_altitude) <= 0.5:
                self.logger.info("Already at target altitude")
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
                    self.logger.info(f"Adjusting altitude: Current: {current_alt:.1f}m, Target: {new_altitude:.1f}m")
                    if abs(current_alt - new_altitude) <= 0.5:
                        self.logger.info(f"Successfully adjusted to target altitude: {current_alt:.1f}m")
                        return True
                time.sleep(0.5)

            self.logger.error("Timeout waiting for altitude adjustment")
            return False

        except Exception as e:
            self.logger.error(f"Error adjusting altitude: {e}")
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

    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        """
        Calculate bearing from point 1 to point 2
        Returns bearing in radians
        """
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)

        y = math.sin(lon2_rad - lon1_rad) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(
            lon2_rad - lon1_rad)
        bearing = math.atan2(y, x)

        return bearing

    def apply_pd_control_position(self, current_lat, current_lon, target_lat, target_lon, dt):

        """

        PID control for position errors (renamed but now implements PID)

        Returns velocity north, velocity east in m/s

        """

        # Convert lat/lon differences to meters (approximate)

        lat_meters_per_degree = 111000

        lon_meters_per_degree = 111000 * math.cos(math.radians(current_lat))

        # Calculate position errors in meters

        error_north = (target_lat - current_lat) * lat_meters_per_degree

        error_east = (target_lon - current_lon) * lon_meters_per_degree

        # Calculate derivative of error

        if self.last_position_error_time == 0:

            d_error_north = 0

            d_error_east = 0

        else:

            d_error_north = (error_north - self.last_position_error_north) / dt

            d_error_east = (error_east - self.last_position_error_east) / dt

        # Update integral terms

        self.integral_position_north += error_north * dt

        self.integral_position_east += error_east * dt

        # Apply PID formula

        velocity_north = (self.Kp_position * error_north +

                          self.Ki_position * self.integral_position_north +

                          self.Kd_position * d_error_north)

        velocity_east = (self.Kp_position * error_east +

                         self.Ki_position * self.integral_position_east +

                         self.Kd_position * d_error_east)

        # Cap speed to target velocity

        speed = math.sqrt(velocity_north ** 2 + velocity_east ** 2)

        if speed > self._target_velocity:
            scale_factor = self._target_velocity / speed

            velocity_north *= scale_factor

            velocity_east *= scale_factor

        # Store current errors

        self.last_position_error_north = error_north

        self.last_position_error_east = error_east

        self.last_position_error_time = time.time()

        return velocity_north, velocity_east

    def apply_pd_control_velocity(self, current_vel_north, current_vel_east, target_vel_north, target_vel_east, dt):

        """

        PID control for velocity errors (renamed but now implements PID)

        Returns adjusted velocity north, velocity east in m/s

        """

        # Calculate velocity errors

        error_north = target_vel_north - current_vel_north

        error_east = target_vel_east - current_vel_east

        # Calculate derivative of error

        if self.last_velocity_error_time == 0:

            d_error_north = 0

            d_error_east = 0

        else:

            d_error_north = (error_north - self.last_velocity_error_north) / dt

            d_error_east = (error_east - self.last_velocity_error_east) / dt

        # Update integral terms

        self.integral_velocity_north += error_north * dt

        self.integral_velocity_east += error_east * dt

        # Apply PID formula

        accel_north = (self.Kp_velocity * error_north +

                       self.Ki_velocity * self.integral_velocity_north +

                       self.Kd_velocity * d_error_north)

        accel_east = (self.Kp_velocity * error_east +

                      self.Ki_velocity * self.integral_velocity_east +

                      self.Kd_velocity * d_error_east)

        # Limit acceleration

        accel_magnitude = math.sqrt(accel_north ** 2 + accel_east ** 2)

        if accel_magnitude > self.max_accel:
            scale_factor = self.max_accel / accel_magnitude

            accel_north *= scale_factor

            accel_east *= scale_factor

        # Calculate new velocity

        new_vel_north = current_vel_north + accel_north * dt

        new_vel_east = current_vel_east + accel_east * dt

        # Store current errors

        self.last_velocity_error_north = error_north

        self.last_velocity_error_east = error_east

        self.last_velocity_error_time = time.time()

        return new_vel_north, new_vel_east

    def apply_pd_control_with_attitude_correction(self, current_lat, current_lon, target_lat, target_lon,
                                                  current_pitch, current_roll, dt):
        """
        Enhanced PID control that accounts for and corrects excessive pitch and roll

        Args:
            current_lat, current_lon: Current drone position
            target_lat, target_lon: Target position
            current_pitch: Current pitch in degrees (positive is nose up)
            current_roll: Current roll in degrees (positive is right wing down)
            dt: Time delta since last control update

        Returns:
            velocity_north, velocity_east: Corrected velocity commands in m/s
        """
        # First, calculate base velocity commands using standard position PID control via the original function name
        base_vel_north, base_vel_east = self.apply_pd_control_position(
            current_lat, current_lon, target_lat, target_lon, dt
        )

        # Define pitch/roll limits (in degrees)
        max_pitch = self.max_pitch  # Maximum acceptable pitch
        max_roll = self.max_roll  # Maximum acceptable roll

        # Calculate pitch/roll correction factors
        pitch_error = abs(current_pitch) - max_pitch
        roll_error = abs(current_roll) - max_roll

        # Only apply corrections if pitch/roll exceed maximum values
        pitch_correction = 0.0
        roll_correction = 0.0

        if pitch_error > 0:
            # Calculate correction factor based on how much pitch exceeds the limit
            # Use a proportional correction: the more excessive the pitch, the stronger the correction
            pitch_correction_factor = self.attitude_correction_gain * pitch_error

            # Determine correction direction (oppose the pitch direction)
            if current_pitch > 0:  # Nose up
                # Reduce forward velocity to bring nose down
                pitch_correction = -pitch_correction_factor
            else:  # Nose down
                # Increase forward velocity to bring nose up
                pitch_correction = pitch_correction_factor

        if roll_error > 0:
            # Calculate correction factor for roll
            roll_correction_factor = self.attitude_correction_gain * roll_error

            # Determine correction direction (oppose the roll direction)
            if current_roll > 0:  # Right wing down
                # Apply left correction
                roll_correction = -roll_correction_factor
            else:  # Left wing down
                # Apply right correction
                roll_correction = roll_correction_factor

        # Decompose corrections into north/east components based on current heading
        # Get current heading (from velocity vector)
        if base_vel_north != 0 or base_vel_east != 0:
            heading = math.atan2(base_vel_east, base_vel_north)
        else:
            heading = 0  # Default heading if velocity is zero

        # Apply corrections in the appropriate directions
        correction_north = pitch_correction * math.cos(heading) - roll_correction * math.sin(heading)
        correction_east = pitch_correction * math.sin(heading) + roll_correction * math.cos(heading)

        # Apply corrections to base velocities
        corrected_vel_north = base_vel_north + correction_north
        corrected_vel_east = base_vel_east + correction_east

        # Log significant corrections
        if abs(pitch_correction) > 0.2 or abs(roll_correction) > 0.2:
            self.logger.info(
                f"Attitude correction applied - Pitch: {current_pitch:.1f}° (corr: {pitch_correction:.2f}), "
                f"Roll: {current_roll:.1f}° (corr: {roll_correction:.2f})"
            )

        return corrected_vel_north, corrected_vel_east
    def request_attitude_data(self):
        """
        Explicitly request attitude data from the vehicle
        This might be needed for some systems where attitude isn't streamed automatically
        """
        if not self.vehicle:
            self.logger.error("Cannot request attitude data - no vehicle connection")
            return False

        try:
            # Request ATTITUDE message at 10Hz
            self.vehicle.mav.request_data_stream_send(
                self.vehicle.target_system,
                self.vehicle.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,  # ATTITUDE is in EXTRA1
                10,  # 10Hz
                1  # Start
            )

            # Also request extended status (may include attitude on some systems)
            self.vehicle.mav.request_data_stream_send(
                self.vehicle.target_system,
                self.vehicle.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
                10,  # 10Hz
                1  # Start
            )

            self.logger.info("Requested attitude data streams")
            return True
        except Exception as e:
            self.logger.error(f"Failed to request attitude data: {e}")
            return False

    def get_attitude_data(self):
        """
        Get the latest attitude data using multiple methods
        Returns: (pitch, roll) in degrees or (None, None) if not available
        """
        if not self.vehicle:
            return None, None

        try:
            # Method 1: Try ATTITUDE message directly
            att_msg = self.vehicle.recv_match(type='ATTITUDE', blocking=False)
            if att_msg:
                pitch = math.degrees(att_msg.pitch)
                roll = math.degrees(att_msg.roll)
                self.logger.debug(f"Got attitude from ATTITUDE message: Pitch={pitch:.1f}°, Roll={roll:.1f}°")
                return pitch, roll

            # Method 2: Try AHRS2 message
            ahrs_msg = self.vehicle.recv_match(type='AHRS2', blocking=False)
            if ahrs_msg:
                pitch = math.degrees(ahrs_msg.pitch)
                roll = math.degrees(ahrs_msg.roll)
                self.logger.debug(f"Got attitude from AHRS2 message: Pitch={pitch:.1f}°, Roll={roll:.1f}°")
                return pitch, roll

            # Method 3: Try VFR_HUD message which may contain pitch on some systems
            vfr_msg = self.vehicle.recv_match(type='VFR_HUD', blocking=False)
            if vfr_msg and hasattr(vfr_msg, 'pitch'):
                pitch = vfr_msg.pitch  # Already in degrees
                # Roll might not be available in VFR_HUD
                self.logger.debug(f"Got partial attitude from VFR_HUD: Pitch={pitch:.1f}°")
                return pitch, 0.0

            # No attitude data available from any source
            return None, None

        except Exception as e:
            self.logger.error(f"Error getting attitude data: {e}")
            return None, None

    def _process_attitude_data(self, pitch, roll):
        """Process attitude data and update accumulators for persistent offsets"""
        # Only accumulate if the attitude has been consistently off
        pitch_threshold = 5.0  # Lower threshold to catch your 15° issue earlier
        roll_threshold = 5.0

        if abs(pitch) > pitch_threshold:
            self.pitch_accumulator += 0.015 * pitch  # Increased rate of accumulation
            self.pitch_accumulator = max(min(self.pitch_accumulator, 10.0), -10.0)  # Limit range

        if abs(roll) > roll_threshold:
            self.roll_accumulator += 0.01 * roll
            self.roll_accumulator = max(min(self.roll_accumulator, 10.0), -10.0)  # Limit range

        # Log significant attitude values
        if abs(pitch) > 10.0 or abs(roll) > 10.0 or abs(self.pitch_accumulator) > 2.0:
            self.logger.info(
                f"Significant attitude: Pitch={pitch:.1f}°, Roll={roll:.1f}°, "
                f"Accumulators: P={self.pitch_accumulator:.1f}, R={self.roll_accumulator:.1f}"
            )

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
        self.logger.info(f"Target velocity set to: {self._target_velocity}m/s")

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
        self.logger.info(f"Target altitude updated: {old_altitude:.1f}m -> {self._target_altitude:.1f}m")

        # If vehicle is flying, adjust to new altitude
        if self.is_vehicle_flying():
            self.adjust_altitude(self._target_altitude)

    def connect_vehicle(self):
        connection_string = 'udp:127.0.0.1:14551'
        self.logger.info(f"Connecting to SITL on {connection_string}")

        try:
            self.vehicle = mavutil.mavlink_connection(connection_string)
            self.vehicle.wait_heartbeat()
            self.logger.info("Connected to vehicle")
            return True
        except Exception as e:
            self.logger.error(f"Failed to connect to vehicle: {e}")
            return False

    def connect_gps(self):
        """Connect to GPS device"""
        try:
            self.serial_port = serial.Serial(
                port=self.gps_port,
                baudrate=self.gps_baudrate,
                timeout=1
            )
            self.logger.info(f"Connected to GPS on {self.gps_port}")
            return True
        except serial.SerialException as e:
            self.logger.error(f"Failed to open GPS serial port: {e}")
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
            self.logger.info("Vehicle already in GUIDED mode")
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
                self.logger.info("Vehicle mode set to GUIDED")
                return True
            time.sleep(0.1)

        self.logger.error("Failed to set GUIDED mode")
        return False

    def arm_vehicle(self):
        """Arm the vehicle with state checking"""
        if not self.vehicle:
            return False

        # Check if already armed
        self.check_vehicle_state()
        if self.is_armed:
            self.logger.info("Vehicle already armed")
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
                self.logger.info("Vehicle armed")
                return True
            time.sleep(0.1)

        self.logger.error("Failed to arm vehicle")
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
                self.logger.error("Failed to connect to GPS")
                return

        self.follow_enabled = True
        self.logger.info(f"Starting GPS follow with PD control and attitude correction")

        # Initialize attitude tracking variables
        self.attitude_source = None
        self.attitude_receive_count = 0
        self.pitch_accumulator = 0.0
        self.roll_accumulator = 0.0
        self._last_attitude_log = 0
        self._message_types_logged = False

        # Initialize pitch/roll correction accumulators
        if not hasattr(self, 'pitch_accumulator'):
            self.pitch_accumulator = 0.0
        if not hasattr(self, 'roll_accumulator'):
            self.roll_accumulator = 0.0
        if not hasattr(self, 'max_pitch'):
            self.max_pitch = 5.0
        if not hasattr(self, 'max_roll'):
            self.max_roll = 5.0
        if not hasattr(self, 'attitude_correction_gain'):
            self.attitude_correction_gain = 0.5
        if not hasattr(self, 'enable_attitude_correction'):
            self.enable_attitude_correction = True

        # Store latest drone position and lock to prevent simultaneous reads
        drone_position_lock = threading.Lock()
        latest_drone_position = {
            'lat': None,
            'lon': None,
            'alt': None,
            'vel_north': 0,
            'vel_east': 0,
            'pitch': 0,
            'roll': 0
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
        default_history_length = 30  # Base history length that gets adjusted dynamically
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

        # Start separate thread for position and attitude monitoring
        # Use the improved position thread function that properly handles attitude data
        def update_position_thread():
            """Modified position update thread with improved attitude handling"""
            # Request attitude data at the beginning
            self.request_attitude_data()

            # Track attitude statistics
            self.attitude_receive_count = 0
            self._last_attitude_log = time.time()

            while self.follow_enabled:
                try:
                    if self.vehicle:
                        # Get position data
                        pos_msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
                        if pos_msg:
                            with drone_position_lock:
                                latest_drone_position['lat'] = pos_msg.lat / 1e7
                                latest_drone_position['lon'] = pos_msg.lon / 1e7
                                latest_drone_position['alt'] = pos_msg.relative_alt / 1000.0

                                # Extract velocity if available
                                if hasattr(pos_msg, 'vx') and hasattr(pos_msg, 'vy'):
                                    # Convert from cm/s to m/s
                                    latest_drone_position['vel_north'] = pos_msg.vx / 100.0
                                    latest_drone_position['vel_east'] = pos_msg.vy / 100.0

                        # Get attitude data using all possible methods
                        # Method 1: Try ATTITUDE message directly
                        att_msg = self.vehicle.recv_match(type='ATTITUDE', blocking=False)
                        if att_msg:
                            with drone_position_lock:
                                pitch = math.degrees(att_msg.pitch)
                                roll = math.degrees(att_msg.roll)
                                latest_drone_position['pitch'] = pitch
                                latest_drone_position['roll'] = roll
                                self.attitude_source = 'ATTITUDE'
                                self.attitude_receive_count += 1

                                # Process attitude data for PD controller - FIXED CALL
                                _process_attitude_data(pitch, roll)

                        else:
                            # Method 2: Try AHRS2 message
                            ahrs_msg = self.vehicle.recv_match(type='AHRS2', blocking=False)
                            if ahrs_msg:
                                with drone_position_lock:
                                    pitch = math.degrees(ahrs_msg.pitch)
                                    roll = math.degrees(ahrs_msg.roll)
                                    latest_drone_position['pitch'] = pitch
                                    latest_drone_position['roll'] = roll
                                    self.attitude_source = 'AHRS2'
                                    self.attitude_receive_count += 1

                                    # Process attitude data - FIXED CALL
                                    _process_attitude_data(pitch, roll)

                            else:
                                # Method 3: Try VFR_HUD message which may contain pitch on some systems
                                vfr_msg = self.vehicle.recv_match(type='VFR_HUD', blocking=False)
                                if vfr_msg and hasattr(vfr_msg, 'pitch'):
                                    with drone_position_lock:
                                        pitch = vfr_msg.pitch  # Already in degrees
                                        roll = latest_drone_position.get('roll', 0)  # Keep existing roll
                                        latest_drone_position['pitch'] = pitch
                                        self.attitude_source = 'VFR_HUD'
                                        self.attitude_receive_count += 1

                                        # Process pitch data - FIXED CALL
                                        _process_attitude_data(pitch, roll)

                        # Periodically re-request attitude data and log status
                        current_time = time.time()
                        if current_time - self._last_attitude_log > 5.0:
                            self._last_attitude_log = current_time

                            # Re-request data streams to ensure we keep getting attitude
                            self.request_attitude_data()

                            # Log current attitude status
                            with drone_position_lock:
                                self.logger.info(
                                    f"Attitude status: Source={self.attitude_source if hasattr(self, 'attitude_source') else 'None'}, "
                                    f"Received count={self.attitude_receive_count}, "
                                    f"Current values: Pitch={latest_drone_position.get('pitch', 'N/A')}, "
                                    f"Roll={latest_drone_position.get('roll', 'N/A')}, "
                                    f"Accumulators: P={self.pitch_accumulator:.1f}, R={self.roll_accumulator:.1f}"
                                )

                    time.sleep(0.1)  # Small delay to prevent excessive CPU usage
                except Exception as e:
                    self.logger.warning(f"Position update thread error: {e}")
                    time.sleep(0.5)  # Longer delay if there's an error

        # Helper method to process attitude data
        def _process_attitude_data(pitch, roll):
            """Process attitude data and update accumulators for persistent offsets"""
            # Only accumulate if the attitude has been consistently off
            pitch_threshold = 5.0  # Lower threshold to catch your 15° issue earlier
            roll_threshold = 5.0

            if abs(pitch) > pitch_threshold:
                self.pitch_accumulator += 0.015 * pitch  # Increased rate of accumulation
                self.pitch_accumulator = max(min(self.pitch_accumulator, 10.0), -10.0)  # Limit range

            if abs(roll) > roll_threshold:
                self.roll_accumulator += 0.01 * roll
                self.roll_accumulator = max(min(self.roll_accumulator, 10.0), -10.0)  # Limit range

            # Log significant attitude values
            if abs(pitch) > 10.0 or abs(roll) > 10.0 or abs(self.pitch_accumulator) > 2.0:
                self.logger.info(
                    f"Significant attitude: Pitch={pitch:.1f}°, Roll={roll:.1f}°, "
                    f"Accumulators: P={self.pitch_accumulator:.1f}, R={self.roll_accumulator:.1f}"
                )

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
                                last_velocity_update = current_time

                                self.logger.debug(
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
                                last_velocity_update = current_time

                                self.logger.debug(
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
                                        R_adaptive = R_base / (speed_factor * 0.8)

                                        # Also modify max history length for more smoothing at low speeds
                                        current_max_history = min(15, int(default_history_length / speed_factor))
                                    else:
                                        # Normal filtering at higher speeds
                                        R_adaptive = R_base
                                        current_max_history = 20  # Less smoothing at higher speeds

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
                                        weights = np.linspace(0.7, 1.3, len(position_history))
                                    else:
                                        # More weight to older positions at lower speeds
                                        weights = np.linspace(1.5, 0.5, len(position_history))

                                    weights = weights / np.sum(weights)  # Normalize weights

                                    smoothed_lat = sum(w * p[0] for w, p in zip(weights, position_history))
                                    smoothed_lon = sum(w * p[1] for w, p in zip(weights, position_history))
                                else:
                                    smoothed_lat, smoothed_lon = filtered_lat, filtered_lon

                                # ----- End of filtering implementation -----

                                # Get current drone position and attitude safely
                                with drone_position_lock:
                                    drone_lat = latest_drone_position['lat']
                                    drone_lon = latest_drone_position['lon']
                                    drone_vel_north = latest_drone_position.get('vel_north', 0)
                                    drone_vel_east = latest_drone_position.get('vel_east', 0)
                                    current_pitch = latest_drone_position.get('pitch', 0)
                                    current_roll = latest_drone_position.get('roll', 0)

                                # Skip if we don't have current drone position yet
                                if drone_lat is None or drone_lon is None:
                                    self.logger.debug("Waiting for drone position data...")
                                    continue

                                # Calculate distance between target and current position
                                distance = self.calculate_distance(
                                    drone_lat, drone_lon,
                                    smoothed_lat, smoothed_lon  # Use smoothed coordinates
                                )

                                # Calculate bearing to target for additional context
                                bearing = self.calculate_bearing(drone_lat, drone_lon, smoothed_lat, smoothed_lon)
                                bearing_degrees = math.degrees(bearing)

                                # Apply PD control with attitude correction if enabled
                                if self.enable_attitude_correction:
                                    # Use the attitude-corrected PD controller
                                    target_vel_north, target_vel_east = self.apply_pd_control_with_attitude_correction(
                                        drone_lat, drone_lon,
                                        smoothed_lat, smoothed_lon,
                                        current_pitch + self.pitch_accumulator,
                                        current_roll + self.roll_accumulator,
                                        dt
                                    )
                                else:
                                    # Use standard position PD controller without attitude correction
                                    target_vel_north, target_vel_east = self.apply_pd_control_position(
                                        drone_lat, drone_lon,
                                        smoothed_lat, smoothed_lon,
                                        dt
                                    )

                                # Get current velocity components
                                current_vel_north = drone_vel_north
                                current_vel_east = drone_vel_east

                                # If we have fresh GPS velocity data, incorporate it
                                use_gps_velocity = (gps_vel_north is not None and
                                                    gps_vel_east is not None and
                                                    (current_time - last_velocity_update) < 2.0)

                                if use_gps_velocity:
                                    # Apply second PD controller for velocity to smooth acceleration
                                    velocity_north, velocity_east = self.apply_pd_control_velocity(
                                        current_vel_north, current_vel_east,
                                        target_vel_north, target_vel_east,
                                        dt
                                    )
                                else:
                                    # Without GPS velocity, use position-based PD control only with limits
                                    velocity_north, velocity_east = target_vel_north, target_vel_east

                                    # Apply acceleration limiting to smooth changes
                                    dt_cmd = current_time - self.last_command_time

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

                                # Determine speed to use based on distance to target
                                current_speed = math.sqrt(velocity_north ** 2 + velocity_east ** 2)

                                # Dynamic speed scaling based on distance
                                # - Maximum speed when far away
                                # - Progressively slower as we approach target
                                max_speed = self._target_velocity
                                min_speed = 0.5  # Minimum speed in m/s

                                # Distance-based speed scaling
                                if distance > 20:
                                    # Full speed when far away
                                    target_speed = max_speed
                                elif distance < self.distance_threshold:
                                    # Minimum speed when very close
                                    target_speed = min_speed
                                else:
                                    # Linear scaling between min and max speed
                                    # when distance is between threshold and 20m
                                    target_speed = min_speed + (max_speed - min_speed) * (
                                            (distance - self.distance_threshold) / (20 - self.distance_threshold)
                                    )

                                # Scale velocity components to target speed
                                if current_speed > 0:
                                    scale_factor = target_speed / current_speed
                                    velocity_north *= scale_factor
                                    velocity_east *= scale_factor

                                # Store for next iteration
                                last_velocity_north = velocity_north
                                last_velocity_east = velocity_east

                                # Log the calculated distance and velocity components
                                if self.enable_attitude_correction:
                                    self.logger.info(
                                        f"Distance to target: {distance:.2f}m, Bearing: {bearing_degrees:.1f}°\n"
                                        f"Attitude: Pitch: {current_pitch:.1f}° (acc: {self.pitch_accumulator:.1f}°), "
                                        f"Roll: {current_roll:.1f}° (acc: {self.roll_accumulator:.1f}°)\n"
                                        f"Target velocity: {target_speed:.2f} m/s\n"
                                        f"PD Control velocities (N,E): {velocity_north:.2f}, {velocity_east:.2f} m/s"
                                    )
                                else:
                                    self.logger.info(
                                        f"Distance to target: {distance:.2f}m, Bearing: {bearing_degrees:.1f}°\n"
                                        f"Target velocity: {target_speed:.2f} m/s\n"
                                        f"PD Control velocities (N,E): {velocity_north:.2f}, {velocity_east:.2f} m/s"
                                    )

                                # Use the position+velocity bitmask
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
                                self.logger.info(
                                    f"PD Control: Movement command sent (distance: {distance:.2f}m):\n"
                                    f"  Current Position: {drone_lat:.6f}, {drone_lon:.6f}\n"
                                    f"  Target Position: {smoothed_lat:.6f}, {smoothed_lon:.6f}\n"
                                    f"  Target Altitude: {self.target_altitude:.1f}m\n"
                                    f"  Velocity Vector: N={velocity_north:.2f} m/s, E={velocity_east:.2f} m/s"
                                )

                    except pynmea2.ParseError:
                        continue
                    except ValueError as e:
                        self.logger.warning(f"Error parsing GPS data: {e}")
                        continue
                    except np.linalg.LinAlgError as e:
                        self.logger.warning(f"Matrix calculation error: {e}")
                        continue

            except serial.SerialException as e:
                self.logger.error(f"GPS Serial error: {e}")
                break
            except Exception as e:
                self.logger.error(f"Error in follow thread: {e}")
                self.logger.exception(e)
                continue

    def check_and_handle_altitude(self):
        """Check current state and handle altitude appropriately"""
        if not self.vehicle:
            self.logger.error("No vehicle connection")
            return False

        # First check if we're already flying
        flying = self.is_vehicle_flying()
        current_alt = self.get_current_altitude()

        if current_alt is None:
            self.logger.error("Could not get current altitude")
            return False

        self.logger.info(
            f"Current state - Flying: {flying}, Altitude: {current_alt:.1f}m, Target: {self.target_altitude:.1f}m")

        if flying:
            # Vehicle is already flying, use position target if needed
            if abs(current_alt - self.target_altitude) > 0.5:
                self.logger.info("Vehicle already flying - adjusting altitude")
                return self.adjust_altitude(self.target_altitude)
            else:
                self.logger.info("Already at target altitude")
                return True
        else:
            # Not flying, need to takeoff
            self.logger.info("Vehicle not flying - initiating takeoff")
            return self.initiate_takeoff()

    def initiate_takeoff(self):
        """Handle the actual takeoff command"""
        self.logger.info(f"Initiating takeoff to {self.target_altitude}m")
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
                self.logger.info(f"Current altitude during takeoff: {current_alt:.1f}m")
                if abs(current_alt - self.target_altitude) < 1.0:
                    self.has_taken_off = True
                    self.logger.info(f"Reached target altitude: {self.target_altitude}m")
                    return True
            time.sleep(0.1)

        self.logger.error("Takeoff timeout")
        return False

    def takeoff(self):
        """Smart takeoff with state checking"""
        return self.check_and_handle_altitude()

    # Update start_mission to use this logic
    def start_mission(self):

        """Start the GPS follow mission"""

        # Reset integral terms when starting new mission

        self.integral_position_north = 0

        self.integral_position_east = 0

        self.integral_velocity_north = 0

        self.integral_velocity_east = 0

        # Rest of existing start_mission code...

        if not self.connect_vehicle():
            return False

        if not self.set_guided_mode() or not self.arm_vehicle():
            return False

        if not self.check_and_handle_altitude():
            return False

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
                    self.logger.warning(f"Current position captured - lat: {current_position['lat'] / 1e7:.6f}, "
                                        f"lon: {current_position['lon'] / 1e7:.6f}, alt: {current_position['alt']:.1f}m")

                    # Position hold command removed as requested

                else:
                    self.logger.warning("Could not get current position for hold")
            except Exception as e:
                self.logger.error(f"Error during cleanup position capture: {e}")

        # Close GPS connection
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
                self.logger.info("GPS connection closed")
            except Exception as e:
                self.logger.error(f"Error closing GPS connection: {e}")


def main():
    try:
        parser = argparse.ArgumentParser(description='GPS Follow with PD Control for smooth movement')
        parser.add_argument('--altitude', type=float, default=10.0,
                            help='Target altitude in meters (1-100)')
        parser.add_argument('--velocity', type=float, default=10.0,
                            help='Target velocity in m/s (1-20)')
        parser.add_argument('--gps-port', type=str, default='/dev/ttyUSB0',
                            help='GPS device port')
        parser.add_argument('--gps-baud', type=int, default=9600,
                            help='GPS baud rate')
        parser.add_argument('--distance-threshold', type=float, default=1.0,
                            help='Distance threshold in meters (default: 1.0)')
        parser.add_argument('--max-accel', type=float, default=0.5,
                            help='Maximum acceleration in m/s² (default: 1.0)')
        parser.add_argument('--kp-position', type=float, default=8,
                            help='Proportional gain for position control (default: 1.0)')
        parser.add_argument('--kd-position', type=float, default=0,
                            help='Derivative gain for position control (default: 0.5)')
        parser.add_argument('--kp-velocity', type=float, default=15,
                            help='Proportional gain for velocity control (default: 2.0)')
        parser.add_argument('--kd-velocity', type=float, default=0,
                            help='Derivative gain for velocity control (default: 0.8)')
        parser.add_argument('--ki-position', type=float, default=0,
                            help='Integral gain for position control (default: 0.0001)')
        parser.add_argument('--ki-velocity', type=float, default=0,
                            help='Integral gain for velocity control (default: 0.0001)')
        parser.add_argument('--attitude-correction', type=bool, default=True,
                            help='Enable attitude correction (default: True)')
        parser.add_argument('--attitude-gain', type=float, default=0.5,
                            help='Gain for attitude correction (default: 0.5)')
        parser.add_argument('--max-pitch', type=float, default=3.5,
                            help='Maximum allowed pitch in degrees (default: 5.0)')
        parser.add_argument('--max-roll', type=float, default=3.5,
                            help='Maximum allowed roll in degrees (default: 5.0)')
        parser.add_argument('--debug', action='store_true',
                            help='Enable debug logging')
        parser.add_argument('--log-dir', type=str, default='logs',
                            help='Directory to store log files (default: logs)')
        parser.add_argument('--use-timestamp-dir', action='store_true',
                            help='Use timestamped directory instead of timestamped filename')
        args = parser.parse_args()

        # Setup logging
        logger = setup_logging(
            log_dir=args.log_dir,
            use_timestamps=not args.use_timestamp_dir  # If use_timestamp_dir is True, use_timestamps should be False
        )

        if args.debug:
            logger.setLevel(logging.DEBUG)

        logger.info("Starting GPS Follow with PD Control and Attitude Correction:")
        logger.info(f"  Altitude: {args.altitude}m")
        logger.info(f"  Target Velocity: {args.velocity}m/s")
        logger.info(f"  GPS Port: {args.gps_port}")
        logger.info(f"  GPS Baud Rate: {args.gps_baud}")
        logger.info(f"  Distance Threshold: {args.distance_threshold}m")
        logger.info(f"  Maximum Acceleration: {args.max_accel}m/s²")
        logger.info(f"  PID Control Parameters:")
        logger.info(f"    Position: Kp={args.kp_position}, Ki={args.ki_position}, Kd={args.kd_position}")
        logger.info(f"    Velocity: Kp={args.kp_velocity}, Ki={args.ki_velocity}, Kd={args.kd_velocity}")
        logger.info(f"  Attitude Correction:")
        logger.info(f"    Enabled: {args.attitude_correction}")
        logger.info(f"    Gain: {args.attitude_gain}")
        logger.info(f"    Max Pitch: {args.max_pitch}°")
        logger.info(f"    Max Roll: {args.max_roll}°")

        gps_follow = GPSSITLFollow(
            target_altitude=args.altitude,
            target_velocity=args.velocity,
            gps_port=args.gps_port,
            gps_baudrate=args.gps_baud,
            distance_threshold=args.distance_threshold,
            logger=logger  # Pass the configured logger
        )

        # Set PD control parameters
        gps_follow.max_accel = args.max_accel
        gps_follow.Kp_position = args.kp_position
        gps_follow.Kd_position = args.kd_position
        gps_follow.Kp_velocity = args.kp_velocity
        gps_follow.Kd_velocity = args.kd_velocity
        gps_follow.Ki_position = args.ki_position
        gps_follow.Ki_velocity = args.ki_velocity

        # Set attitude correction parameters
        gps_follow.enable_attitude_correction = args.attitude_correction
        gps_follow.attitude_correction_gain = args.attitude_gain
        gps_follow.max_pitch = args.max_pitch
        gps_follow.max_roll = args.max_roll

        if gps_follow.start_mission():
            logger.info("Mission started successfully - using PD control with attitude correction")
            try:
                # Main loop to monitor and allow for parameter tuning while running
                while True:
                    time.sleep(1)
                    # Report current attitude if significant
                    if hasattr(gps_follow, 'current_pitch') and hasattr(gps_follow, 'current_roll'):
                        if abs(gps_follow.current_pitch) > 10 or abs(gps_follow.current_roll) > 10:
                            logger.info(f"Current attitude - Pitch: {gps_follow.current_pitch:.1f}°, "
                                        f"Roll: {gps_follow.current_roll:.1f}°")
            except KeyboardInterrupt:
                logger.info("Mission interrupted by user")
        else:
            logger.error("Failed to start mission")

    except Exception as e:
        if 'logger' in locals():
            logger.error(f"Mission failed: {e}")
            logger.exception(e)
        else:
            # Fallback logging if logger creation failed
            print(f"ERROR: Mission failed: {e}")
            import traceback
            traceback.print_exc()
    finally:
        if 'gps_follow' in locals():
            gps_follow.cleanup()
        if 'logger' in locals():
            logger.info("Cleanup completed")
        else:
            print("Cleanup completed")


if __name__ == "__main__":
    main()
