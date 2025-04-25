import time
import logging
import threading
import serial
import pynmea2
from pymavlink import mavutil
import argparse
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
    def __init__(self, target_altitude=10, target_velocity=5, gps_port='/dev/ttyUSB0', gps_baudrate=9600):
        # SITL settings
        self.vehicle = None
        self._target_altitude = target_altitude
        self._target_velocity = target_velocity
        self.follow_enabled = False
        self.has_taken_off = False
        self.is_armed = False
        self.is_guided = False

        # Initialize serial port attributes
        self.serial_port = None
        self.gps_port = gps_port
        self.gps_baudrate = gps_baudrate

        self.last_command_time = 0
        self.command_frequency = 0.05  # 20Hz command rate
        self.position_mask = (
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )

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

    # def takeoff(self):
    #     """Takeoff to target altitude with state checking"""
    #     if not self.vehicle:
    #         return False
    #
    #     # Check if already at target altitude
    #     current_alt = self.get_current_altitude()
    #     if current_alt is not None and abs(current_alt - self.target_altitude) < 1.0:
    #         logger.info(f"Vehicle already at target altitude: {current_alt:.1f}m")
    #         self.has_taken_off = True
    #         return True
    #
    #     logger.info(f"Initiating takeoff to {self.target_altitude}m")
    #     self.vehicle.mav.command_long_send(
    #         self.vehicle.target_system,
    #         self.vehicle.target_component,
    #         mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    #         0,
    #         0, 0, 0, 0, 0, 0, self.target_altitude
    #     )
    #
    #     start_time = time.time()
    #     while time.time() - start_time < 30:
    #         current_alt = self.get_current_altitude()
    #         if current_alt is not None:
    #             logger.info(f"Current altitude: {current_alt:.1f}m")
    #             if abs(current_alt - self.target_altitude) < 1.0:
    #                 self.has_taken_off = True
    #                 logger.info(f"Reached target altitude: {self.target_altitude}m")
    #                 return True
    #         time.sleep(0.1)
    #
    #     logger.error("Takeoff timeout")
    #     return False

    def follow_gps(self):
        if not self.serial_port or not self.serial_port.is_open:
            if not self.connect_gps():
                logger.error("Failed to connect to GPS")
                return

        self.follow_enabled = True
        logger.info("Starting GPS follow with Kalman filtering and smoothing")

        # Initialize Kalman filter state
        # State: [lat, lon, v_lat, v_lon]
        x = np.zeros((4, 1))  # State vector
        P = np.eye(4)  # Covariance matrix

        # Filter parameters with more aggressive smoothing
        # Higher measurement noise values make the filter trust measurements less
        R = np.array([[0.05, 0], [0, 0.05]])  # Increased measurement noise (10x)

        # Lower process noise makes the filter more resistant to changes
        Q = np.array([  # Reduced process noise
            [0.0005, 0, 0, 0],
            [0, 0.0005, 0, 0],
            [0, 0, 0.005, 0],
            [0, 0, 0, 0.005]
        ])

        # Measurement matrix (we observe position, not velocity)
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])

        # Position history for additional smoothing - longer history for more aggressive smoothing
        position_history = []
        max_history_length = 10  # Increased history length

        # Track previous time for dt calculation
        prev_time = time.time()
        filter_initialized = False

        while self.follow_enabled:
            try:
                current_time = time.time()
                dt = current_time - prev_time  # Time delta for state transition

                if current_time - self.last_command_time >= self.command_frequency:
                    line = self.serial_port.readline().decode('ascii', errors='ignore')

                    if not line or not line.startswith('$'):
                        continue

                    if not self.verify_checksum(line):
                        continue

                    try:
                        msg = pynmea2.parse(line)

                        if msg.sentence_type == 'GGA':
                            if msg.latitude and msg.longitude:
                                # Convert GPS coordinates to float
                                raw_lat = float(msg.latitude)
                                raw_lon = float(msg.longitude)

                                # --- Kalman filter implementation ---
                                if not filter_initialized:
                                    # First measurement - initialize state
                                    x[0, 0] = raw_lat
                                    x[1, 0] = raw_lon
                                    filter_initialized = True
                                else:
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
                                    S = H @ P @ H.T + R  # Innovation covariance
                                    K = P @ H.T @ np.linalg.inv(S)  # Kalman gain

                                    x = x + K @ y  # Updated state estimate
                                    P = (np.eye(4) - K @ H) @ P  # Updated covariance

                                # Extract filtered position
                                filtered_lat = float(x[0, 0])
                                filtered_lon = float(x[1, 0])

                                # Apply more aggressive smoothing with position history
                                position_history.append((filtered_lat, filtered_lon))
                                if len(position_history) > max_history_length:
                                    position_history.pop(0)

                                if len(position_history) > 1:
                                    # Weighted moving average with more weight on older positions
                                    # This makes the smoothing more aggressive
                                    weights = np.linspace(0.5, 1.5, len(position_history))
                                    weights = weights / np.sum(weights)  # Normalize weights

                                    smoothed_lat = sum(w * p[0] for w, p in zip(weights, position_history))
                                    smoothed_lon = sum(w * p[1] for w, p in zip(weights, position_history))
                                else:
                                    smoothed_lat, smoothed_lon = filtered_lat, filtered_lon

                                # Send position target command with smoothed coordinates
                                self.vehicle.mav.set_position_target_global_int_send(
                                    0,  # timestamp
                                    self.vehicle.target_system,
                                    self.vehicle.target_component,
                                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                    self.position_mask,
                                    int(smoothed_lat * 1e7),
                                    int(smoothed_lon * 1e7),
                                    self.target_altitude,
                                    self._target_velocity, self._target_velocity, self._target_velocity,
                                    0, 0, 0,  # acceleration
                                    0, 0  # yaw, yaw_rate
                                )

                                self.last_command_time = current_time
                                prev_time = current_time  # Update previous time for next dt calculation

                                # Log current command info with filtering details
                                logger.info(
                                    f"Position command sent:\n"
                                    f"Raw Position: {raw_lat:.6f}, {raw_lon:.6f}\n"
                                    f"Filtered Position: {filtered_lat:.6f}, {filtered_lon:.6f}\n"
                                    f"Smoothed Position: {smoothed_lat:.6f}, {smoothed_lon:.6f}\n"
                                    f"Est. Velocity: {x[2, 0]:.3f}, {x[3, 0]:.3f}\n"
                                    f"Target Altitude: {self.target_altitude:.1f}m\n"
                                    f"Target Velocity: {self._target_velocity:.1f} m/s"
                                )

                    except pynmea2.ParseError:
                        continue
                    except ValueError as e:
                        logger.warning(f"Error parsing GPS data: {e}")
                        continue
                    except np.linalg.LinAlgError as e:
                        logger.warning(f"Matrix calculation error: {e}")
                        continue

                time.sleep(0.1)  # Small sleep to prevent CPU overload

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
        self.follow_enabled = False

        # Get and store current position
        current_position = None
        if self.vehicle:
            msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if msg:
                current_position = {
                    'lat': msg.lat,
                    'lon': msg.lon,
                    'alt': self.target_altitude  # Keep the target altitude
                }
                logger.warning(f"Current position captured - lat: {current_position['lat'] / 1e7:.6f}, "
                               f"lon: {current_position['lon'] / 1e7:.6f}, alt: {current_position['alt']:.1f}m")

                # Send position hold command using captured position
                self.vehicle.mav.set_position_target_global_int_send(
                    0,  # timestamp
                    self.vehicle.target_system,
                    self.vehicle.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    0b110111111000,  # Only positions enabled
                    current_position['lat'],  # stored latitude
                    current_position['lon'],  # stored longitude
                    current_position['alt'],  # stored altitude
                    0, 0, 0,  # velocity
                    0, 0, 0,  # acceleration
                    0, 0  # yaw, yaw_rate
                )
                logger.info("Vehicle commanded to hold at captured position")
            else:
                logger.warning("Could not get current position for hold")

        # Close GPS connection
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            logger.info("GPS connection closed")


def main():
    try:
        parser = argparse.ArgumentParser(description='GPS Follow with speed monitoring')
        parser.add_argument('--altitude', type=float, default=10.0,
                            help='Target altitude in meters (1-100)')
        parser.add_argument('--velocity', type=float, default=5.0,
                            help='Target velocity in m/s (1-20)')
        parser.add_argument('--gps-port', type=str, default='/dev/ttyUSB0',
                            help='GPS device port')
        parser.add_argument('--gps-baud', type=int, default=9600,
                            help='GPS baud rate')
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

        gps_follow = GPSSITLFollow(
            target_altitude=args.altitude,
            target_velocity=args.velocity,
            gps_port=args.gps_port,
            gps_baudrate=args.gps_baud
        )

        if gps_follow.start_mission():
            logger.info("Mission started successfully")
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