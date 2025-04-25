import zmq
import json
import time
from siyi_sdk import SIYISDK
import logging
from collections import deque
from datetime import datetime
import statistics


class PIDController:
    def __init__(self, kp, ki, kd, output_limits=None, sample_time=0.02):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        self.sample_time = sample_time

        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        self.integral_limit = 10.0

    def compute(self, setpoint, measured_value):
        current_time = time.time()
        dt = current_time - self.last_time

        if dt < self.sample_time:
            return None

        error = setpoint - measured_value
        p_term = self.kp * error

        self.integral += error * dt
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        i_term = self.ki * self.integral

        d_term = self.kd * (measured_value - self.last_error) / dt if dt > 0 else 0

        output = p_term + i_term - d_term

        if self.output_limits is not None:
            output = max(min(output, self.output_limits[1]), self.output_limits[0])

        self.last_error = error
        self.last_time = current_time

        return output

    def reset(self):
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()


class GimbalController:
    def __init__(self,
                 camera_ip="192.168.144.25",
                 camera_port=37260,
                 zmq_port=5555):
        # Setup logging
        self.setup_logger()

        # Initialize SIYI camera
        self.camera = SIYISDK(
            server_ip=camera_ip,
            port=camera_port
        )

        # Frame parameters
        self.frame_width = 640
        self.frame_height = 480
        self.frame_center_x = 320
        self.frame_center_y = 240

        # Control parameters
        self.min_pixel_error = 0.5
        self.target_zone = 3
        self.base_angle_scale = 1.0
        self.max_angle_change = 25.0
        self.smoothing_factor = 0.1
        self.track_history = deque(maxlen=10)  # Keep 10 points of history

        # PID Controllers
        self.yaw_pid = PIDController(
            kp=1.0,
            ki=0.12,
            kd=0.02,
            output_limits=(-25, 25),
            sample_time=0.02
        )

        self.pitch_pid = PIDController(
            kp=0.8,
            ki=0.12,
            kd=0.02,
            output_limits=(-25, 25),
            sample_time=0.02
        )

        # Gimbal limits
        self.YAW_MIN = -45
        self.YAW_MAX = 45
        self.PITCH_MIN = -90
        self.PITCH_MAX = 25

        # Movement history
        self.yaw_history = deque(maxlen=5)  # Increased history size
        self.pitch_history = deque(maxlen=5)
        self.last_yaw = 0
        self.last_pitch = 0

        # Enhanced ZMQ setup with reliability features
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)

        # Socket options for reliability
        self.socket.setsockopt(zmq.RCVHWM, 1000)  # Receive high water mark
        self.socket.setsockopt(zmq.LINGER, 100)  # Linger period
        self.socket.setsockopt(zmq.RCVTIMEO, 1000)  # Receive timeout (1 second)
        self.socket.setsockopt(zmq.TCP_KEEPALIVE, 1)
        self.socket.setsockopt(zmq.TCP_KEEPALIVE_IDLE, 300)
        self.socket.setsockopt(zmq.TCP_KEEPALIVE_INTVL, 60)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")

        # Enhanced connection handling
        self.zmq_port = zmq_port
        self.reconnect_delay = 1.0  # Initial reconnect delay
        self.max_reconnect_delay = 30.0
        self.connection_attempts = 0
        self.last_connection_time = None

        # Message tracking
        self.last_message_time = None
        self.message_timeouts = 0
        self.max_timeouts = 5
        self.message_intervals = deque(maxlen=100)
        self.processing_times = deque(maxlen=100)

        # Performance monitoring
        self.message_count = 0
        self.error_count = 0
        self.start_time = time.time()
        self.last_stats_time = time.time()
        self.performance_stats = {
            'processed_messages': 0,
            'missed_messages': 0,
            'reconnections': 0,
            'processing_errors': 0
        }

        self.previous_center_x = None
        self.previous_center_y = None
        self.last_calculation_time = time.time()
        self.position_change_threshold = 15  # More sensitive to movement
        self.calculation_count = 0
        self.significant_moves = 0
        self.total_moves = 0

    def setup_logger(self):
        """Setup logging configuration"""
        log_format = '[%(levelname)s] %(asctime)s [SIYISDK::%(funcName)s] :\t%(message)s'
        logging.basicConfig(
            format=log_format,
            level=logging.INFO,
            handlers=[
                logging.StreamHandler(),
                logging.FileHandler('gimbal_controller.log')
            ]
        )
        self.logger = logging.getLogger('GimbalController')

    def connect(self):
        """Connect to the SIYI camera"""
        self.logger.info("Connecting to SIYI camera...")
        if not self.camera.connect():
            self.logger.error("Failed to connect to camera")
            return False
        try:
            self.last_yaw, self.last_pitch, _ = self.camera.getAttitude()
            self.yaw_history.append(self.last_yaw)
            self.pitch_history.append(self.last_pitch)
            self.logger.info(
                f"Camera connected! Initial position: Yaw={self.last_yaw:.1f}, Pitch={self.last_pitch:.1f}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to get initial attitude: {e}")
            return False

    def connect_zmq(self):
        """Enhanced ZMQ connection with retry logic"""
        while True:
            try:
                self.logger.info(f"Attempting to connect to tracker on port {self.zmq_port}")
                self.socket.connect(f"tcp://localhost:{self.zmq_port}")

                # Wait for initial message to confirm connection
                if self.socket.poll(timeout=5000):  # 5 second initial timeout
                    _ = self.socket.recv_string()  # Test receive
                    self.last_connection_time = time.time()
                    self.connection_attempts = 0
                    self.reconnect_delay = 1.0  # Reset delay
                    self.logger.info("Successfully connected to tracker")
                    return True
                else:
                    raise zmq.Again("No initial message received")

            except Exception as e:
                self.logger.error(f"Connection failed: {e}")
                self.connection_attempts += 1

                # Exponential backoff with maximum delay
                self.reconnect_delay = min(self.reconnect_delay * 2, self.max_reconnect_delay)

                self.logger.info(f"Retrying connection in {self.reconnect_delay:.1f} seconds...")
                time.sleep(self.reconnect_delay)

                # Recreate socket after failed attempt
                self.socket.close()
                self.socket = self.context.socket(zmq.SUB)
                self.socket.setsockopt(zmq.RCVHWM, 1000)
                self.socket.setsockopt(zmq.LINGER, 100)
                self.socket.setsockopt(zmq.RCVTIMEO, 1000)
                self.socket.setsockopt(zmq.TCP_KEEPALIVE, 1)
                self.socket.setsockopt_string(zmq.SUBSCRIBE, "")

    def handle_connection_loss(self):
        """Handle connection loss and attempt reconnection"""
        self.logger.warning("Connection to tracker appears to be lost")
        self.performance_stats['reconnections'] += 1

        try:
            self.socket.close()
            self.socket = self.context.socket(zmq.SUB)
            self.socket.setsockopt(zmq.RCVHWM, 1000)
            self.socket.setsockopt(zmq.LINGER, 100)
            self.socket.setsockopt(zmq.RCVTIMEO, 1000)
            self.socket.setsockopt(zmq.TCP_KEEPALIVE, 1)
            self.socket.setsockopt_string(zmq.SUBSCRIBE, "")

            return self.connect_zmq()

        except Exception as e:
            self.logger.error(f"Reconnection failed: {e}")
            return False

    def calculate_gimbal_angles(self, tracking_data):
        """Calculate gimbal angles using tracking camera logic from reference"""
        try:
            # Extract messages from the batch
            if 'type' in tracking_data and tracking_data['type'] == 'batch':
                messages = tracking_data.get('messages', [])
                if not messages:
                    return None, None
            else:
                messages = [tracking_data]

            # Get the latest valid message
            valid_message = None
            for msg in reversed(messages):
                if all(key in msg for key in ['center_x', 'center_y']):
                    valid_message = msg
                    break

            if not valid_message:
                return None, None

            # Extract current position
            current_x = float(valid_message['center_x'])
            current_y = float(valid_message['center_y'])

            # Add point to track history
            self.track_history.append((int(current_x), int(current_y)))

            # Calculate offsets from center (similar to reference tracking_camera function)
            x_offset = self.frame_center_x - current_x
            y_offset = self.frame_center_y - current_y

            try:
                current_yaw, current_pitch, _ = self.camera.getAttitude()
                self.last_yaw, self.last_pitch = current_yaw, current_pitch
            except Exception as e:
                self.logger.warning(f"Using last known attitude: {e}")
                current_yaw, current_pitch = self.last_yaw, self.last_pitch

            # Apply PID control using offsets
            yaw_adjustment = self.yaw_pid.compute(0, x_offset)
            pitch_adjustment = self.pitch_pid.compute(0, y_offset)

            if yaw_adjustment is None or pitch_adjustment is None:
                return None, None

            # Calculate target angles
            target_yaw = current_yaw + yaw_adjustment
            target_pitch = current_pitch + pitch_adjustment

            # Basic smoothing with history
            self.yaw_history.append(target_yaw)
            self.pitch_history.append(target_pitch)

            if len(self.yaw_history) >= 3:
                weights = [0.6, 0.3, 0.1]
                smoothed_yaw = sum(y * w for y, w in zip(list(self.yaw_history)[-3:], weights))
                smoothed_pitch = sum(p * w for p, w in zip(list(self.pitch_history)[-3:], weights))
            else:
                smoothed_yaw = target_yaw
                smoothed_pitch = target_pitch

            # Apply gimbal limits
            final_yaw = max(min(smoothed_yaw, self.YAW_MAX), self.YAW_MIN)
            final_pitch = max(min(smoothed_pitch, self.PITCH_MAX), self.PITCH_MIN)

            self.calculation_count += 1

            # Enhanced logging
            self.logger.info(
                f"Track #{self.calculation_count} - "
                f"Position: ({current_x:.1f}, {current_y:.1f}) "
                f"Offset: (X={x_offset:.1f}, Y={y_offset:.1f}) "
                f"Angles: Y={final_yaw:.1f}°, P={final_pitch:.1f}°"
            )

            return final_yaw, final_pitch

        except Exception as e:
            self.logger.error(f"Error in angle calculation: {e}")
            import traceback
            self.logger.error(traceback.format_exc())
            return None, None

    def control_gimbal(self):
        """Enhanced main control loop with improved error handling"""
        self.logger.info("Starting gimbal control loop")

        if not self.connect_zmq():
            self.logger.error("Failed to establish initial connection")
            return

        try:
            while True:
                try:
                    process_start_time = time.time()

                    # Check for messages with timeout
                    if self.socket.poll(timeout=1000):  # 1 second timeout
                        message = self.socket.recv_string()
                        current_time = time.time()

                        # Update timing statistics
                        if self.last_message_time is not None:
                            interval = current_time - self.last_message_time
                            self.message_intervals.append(interval)

                            # Check for unusually large gaps
                            if interval > 1.0:  # Gap larger than 1 second
                                self.logger.warning(f"Large message interval detected: {interval:.2f}s")
                                self.performance_stats['missed_messages'] += 1

                        self.last_message_time = current_time
                        self.message_count += 1
                        self.message_timeouts = 0  # Reset timeout counter

                        # Process message
                        try:
                            tracking_data = json.loads(message)

                            # Calculate new angles
                            new_yaw, new_pitch = self.calculate_gimbal_angles(tracking_data)

                            if new_yaw is not None and new_pitch is not None:
                                # Send command to gimbal with error handling
                                try:
                                    self.camera.setGimbalRotation(
                                        yaw=new_yaw,
                                        pitch=new_pitch,
                                        err_thresh=0.08
                                    )
                                except Exception as e:
                                    self.logger.error(f"Gimbal control error: {e}")
                                    self.performance_stats['processing_errors'] += 1

                            # Update performance stats
                            self.performance_stats['processed_messages'] += 1

                            # Calculate and store processing time
                            process_duration = time.time() - process_start_time
                            self.processing_times.append(process_duration)

                        except json.JSONDecodeError as e:
                            self.logger.error(f"JSON parsing error: {e}")
                            self.logger.error(f"Problem message: {message[:200]}")
                            self.performance_stats['processing_errors'] += 1
                            continue

                        except Exception as e:
                            self.logger.error(f"Processing error: {e}")
                            self.performance_stats['processing_errors'] += 1
                            continue

                    else:
                        # Handle message timeout
                        self.message_timeouts += 1
                        self.logger.warning(f"No message received (timeout #{self.message_timeouts})")

                        if self.message_timeouts >= self.max_timeouts:
                            self.logger.error("Maximum timeouts reached, attempting reconnection")
                            if not self.handle_connection_loss():
                                self.logger.error("Reconnection failed, exiting")
                                break
                            self.message_timeouts = 0

                    # Print periodic statistics
                    if time.time() - self.last_stats_time >= 5.0:  # Every 5 seconds
                        self.print_statistics()
                        self.last_stats_time = time.time()

                except zmq.Again:
                    continue
                except KeyboardInterrupt:
                    self.logger.info("Shutting down...")
                    break
                except Exception as e:
                    self.logger.error(f"Main loop error: {type(e).__name__}: {e}")
                    self.error_count += 1
                    time.sleep(0.1)
                    continue

        finally:
            self.print_final_statistics()
            self.cleanup()

    def print_statistics(self):
        """Enhanced statistics printing"""
        elapsed_time = time.time() - self.start_time
        current_rate = self.message_count / elapsed_time if elapsed_time > 0 else 0

        stats = {
            "Runtime": f"{elapsed_time:.1f}s",
            "Total Messages": self.message_count,
            "Messages/second": f"{current_rate:.1f}",
            "Processed Messages": self.performance_stats['processed_messages'],
            "Missed Messages": self.performance_stats['missed_messages'],
            "Processing Errors": self.performance_stats['processing_errors'],
            "Reconnections": self.performance_stats['reconnections'],
            "Average Process Time": f"{statistics.mean(self.processing_times) * 1000:.1f}ms" if self.processing_times else "N/A",
            "Average Message Interval": f"{statistics.mean(self.message_intervals) * 1000:.1f}ms" if self.message_intervals else "N/A"
        }

        self.logger.info("\nPerformance Statistics:")
        for key, value in stats.items():
            self.logger.info(f"{key}: {value}")

    def print_final_statistics(self):
        """Print final statistics before shutdown"""
        self.logger.info(f"\nFinal Statistics:")
        self.logger.info(f"Total messages processed: {self.message_count}")
        self.logger.info(f"Total calculations performed: {self.calculation_count}")
        if self.processing_times:
            self.logger.info(f"Average processing time: {statistics.mean(self.processing_times) * 1000:.1f}ms")
        if self.message_intervals:
            self.logger.info(f"Average message interval: {statistics.mean(self.message_intervals) * 1000:.1f}ms")

    def cleanup(self):
        """Enhanced cleanup"""
        self.logger.info("Cleaning up...")
        try:
            self.socket.close()
            self.context.term()
            self.camera.disconnect()
        except Exception as e:
            self.logger.error(f"Cleanup error: {e}")
        self.logger.info("Cleanup complete")


def main():
    controller = GimbalController(
        camera_ip="192.168.144.25",
        camera_port=37260,
        zmq_port=5555
    )

    if controller.connect():
        controller.control_gimbal()
    else:
        print("Failed to connect to gimbal")


if __name__ == "__main__":
    main()



