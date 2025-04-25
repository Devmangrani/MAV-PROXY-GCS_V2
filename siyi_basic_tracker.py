import zmq
import json
import time
from siyi_sdk import SIYISDK
import logging
from collections import deque
from datetime import datetime


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

        # Control parameters [rest of the parameters remain the same]
        self.min_pixel_error = 0.5
        self.target_zone = 3
        self.base_angle_scale = 1.0
        self.max_angle_change = 25.0
        self.smoothing_factor = 0.1

        # PID Controllers [remain the same]
        self.yaw_pid = PIDController(
            kp=1.2,
            ki=0.15,
            kd=0.02,
            output_limits=(-25, 25),
            sample_time=0.001
        )

        self.pitch_pid = PIDController(
            kp=1.0,
            ki=0.15,
            kd=0.02,
            output_limits=(-20, 20),
            sample_time=0.001
        )

        # Gimbal limits [remain the same]
        self.YAW_MIN = -45
        self.YAW_MAX = 45
        self.PITCH_MIN = -90
        self.PITCH_MAX = 25

        # Movement history [remain the same]
        self.yaw_history = deque(maxlen=1)
        self.pitch_history = deque(maxlen=1)
        self.last_yaw = 0
        self.last_pitch = 0

        # ZMQ setup - Enhanced with proper configuration
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.RCVHWM, 1000)  # Set receive high water mark
        self.socket.setsockopt(zmq.LINGER, 100)   # Set linger period
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.socket.connect(f"tcp://localhost:{zmq_port}")

        # Timing and statistics
        self.message_count = 0
        self.calculation_count = 0
        self.last_debug_time = time.time()
        self.start_time = time.time()
        self.last_message_time = None
        self.message_intervals = deque(maxlen=100)
        self.processing_times = deque(maxlen=100)

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

    def calculate_gimbal_angles(self, tracking_data):
        """Calculate gimbal angles with detailed logging"""
        try:
            # Log incoming data
            self.logger.info(f"\nCalculating angles for tracking data:")
            self.logger.info(f"Input: {json.dumps(tracking_data, indent=2)}")

            # Get ROI center
            roi_center_x = tracking_data['center_x']
            roi_center_y = tracking_data['center_y']

            # Calculate pixel errors
            x_error = roi_center_x - self.frame_center_x
            y_error = roi_center_y - self.frame_center_y

            self.logger.info(f"Position errors: dx={x_error:.1f}, dy={y_error:.1f}")

            # Get current gimbal position
            try:
                current_yaw, current_pitch, _ = self.camera.getAttitude()
                self.last_yaw, self.last_pitch = current_yaw, current_pitch
            except Exception as e:
                self.logger.error(f"Failed to get gimbal attitude: {e}")
                current_yaw, current_pitch = self.last_yaw, self.last_pitch

            # Convert to angle errors
            yaw_error = x_error * (90.0 / self.frame_width)
            pitch_error = y_error * (60.0 / self.frame_height)

            self.logger.info(f"Angle errors: yaw={yaw_error:.1f}°, pitch={pitch_error:.1f}°")

            # Calculate PID adjustments
            yaw_adjustment = self.yaw_pid.compute(0, yaw_error)
            pitch_adjustment = self.pitch_pid.compute(0, pitch_error)

            if yaw_adjustment is None or pitch_adjustment is None:
                self.logger.warning("PID computation returned None")
                return None, None

            # Calculate target angles
            target_yaw = current_yaw + yaw_adjustment
            target_pitch = current_pitch + pitch_adjustment

            # Apply smoothing
            self.yaw_history.append(target_yaw)
            self.pitch_history.append(target_pitch)

            if len(self.yaw_history) >= 2:
                smoothed_yaw = self.yaw_history[-1] * 0.8 + self.yaw_history[-2] * 0.2
                smoothed_pitch = self.pitch_history[-1] * 0.8 + self.pitch_history[-2] * 0.2
            else:
                smoothed_yaw = target_yaw
                smoothed_pitch = target_pitch

            # Clamp to limits
            final_yaw = max(min(smoothed_yaw, self.YAW_MAX), self.YAW_MIN)
            final_pitch = max(min(smoothed_pitch, self.PITCH_MAX), self.PITCH_MIN)

            self.logger.info(
                f"Angle Calculation:\n"
                f"Current: (Y={current_yaw:.1f}°, P={current_pitch:.1f}°)\n"
                f"Adjustment: (Y={yaw_adjustment:.1f}°, P={pitch_adjustment:.1f}°)\n"
                f"Final: (Y={final_yaw:.1f}°, P={final_pitch:.1f}°)"
            )

            self.calculation_count += 1
            return final_yaw, final_pitch

        except Exception as e:
            self.logger.error(f"Error calculating angles: {e}")
            import traceback
            self.logger.error(traceback.format_exc())
            return None, None

    def control_gimbal(self):
        """Main control loop with enhanced message handling"""
        self.logger.info("Starting gimbal control loop")
        last_stats_time = time.time()

        try:
            while True:
                try:
                    # Use poll with timeout to check for messages
                    if self.socket.poll(timeout=100):  # 100ms timeout
                        # Start timing the message processing
                        process_start_time = time.time()

                        # Get the message
                        message = self.socket.recv_string()
                        print(f"Received message #{self.message_count}")
                        current_time = time.time()

                        # Update timing statistics
                        if self.last_message_time is not None:
                            interval = current_time - self.last_message_time
                            self.message_intervals.append(interval)

                        self.last_message_time = current_time
                        self.message_count += 1

                        # Parse and process message
                        try:
                            tracking_data = json.loads(message)

                            # Calculate new angles
                            new_yaw, new_pitch = self.calculate_gimbal_angles(tracking_data)

                            if new_yaw is not None and new_pitch is not None:
                                # Send command to gimbal
                                self.camera.setGimbalRotation(
                                    yaw=new_yaw,
                                    pitch=new_pitch,
                                    err_thresh=0.05
                                )

                            # Calculate processing time
                            process_end_time = time.time()
                            process_duration = process_end_time - process_start_time
                            self.processing_times.append(process_duration)

                            # Print periodic statistics
                            if current_time - last_stats_time >= 1.0:
                                self.print_statistics()
                                last_stats_time = current_time

                        except json.JSONDecodeError as e:
                            self.logger.error(f"JSON parsing error: {e}")
                            self.logger.error(f"Problem message: {message[:200]}")
                            continue

                        except Exception as e:
                            self.logger.error(f"Processing error: {type(e).__name__}: {e}")
                            continue
                    else:
                        # No message received in timeout period
                        self.logger.debug("No message received in last 100ms")

                except zmq.Again:
                    continue
                except KeyboardInterrupt:
                    self.logger.info("Shutting down...")
                    break
                except Exception as e:
                    self.logger.error(f"Main loop error: {type(e).__name__}: {e}")
                    time.sleep(0.1)
                    continue

        finally:
            self.print_final_statistics()
            self.cleanup()

    def print_statistics(self):
        """Print periodic statistics"""
        elapsed_time = time.time() - self.start_time
        current_rate = self.message_count / elapsed_time if elapsed_time > 0 else 0

        stats = {
            "Total Messages": self.message_count,
            "Total Calculations": self.calculation_count,
            "Messages/second": f"{current_rate:.1f}",
            "Average Process Time": f"{statistics.mean(self.processing_times) * 1000:.1f}ms" if self.processing_times else "N/A",
            "Average Message Interval": f"{statistics.mean(self.message_intervals) * 1000:.1f}ms" if self.message_intervals else "N/A"
        }

        self.logger.info("\nStatus Update:")
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
        """Cleanup resources"""
        self.logger.info("Cleaning up...")
        self.socket.close()
        self.context.term()
        self.camera.disconnect()
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