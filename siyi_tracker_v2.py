
import zmq
import json
import time
from siyi_sdk import SIYISDK
import logging
import math
from collections import deque


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

        # ZMQ setup
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(f"tcp://localhost:{zmq_port}")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")

        # Frame parameters - explicitly define center
        self.frame_width = 640
        self.frame_height = 480
        self.frame_center_x = 320
        self.frame_center_y = 240

        # Control parameters
        self.smoothing_factor = 0.3
        self.max_rate_change = 5.0
        self.deadzone = 1.0  # Add this line to define the deadzone

        # Ultra-aggressive control parameters
        self.min_pixel_error = 0.5  # Minimal threshold for instant response
        self.target_zone = 3  # Very tight target zone
        self.base_angle_scale = 1.0  # Maximum scaling
        self.max_angle_change = 25.0  # Very large movements allowed
        self.smoothing_factor = 0.1  # Almost no smoothing

        # Maximum aggressive PID for fastest movement
        self.yaw_pid = PIDController(
            kp=1.2,  # Very high proportional gain
            ki=0.15,  # Aggressive integral
            kd=0.02,  # Minimal damping
            output_limits=(-25, 25),  # Maximum limits
            sample_time=0.001  # Ultra-fast updates (1000Hz)
        )

        self.pitch_pid = PIDController(
            kp=1.0,  # Very high pitch gain
            ki=0.15,  # Match yaw integral
            kd=0.02,  # Minimal damping
            output_limits=(-20, 20),  # High limits
            sample_time=0.001  # Ultra-fast updates
        )

        # Gimbal limits - unchanged as these are hardware limits
        self.YAW_MIN = -45
        self.YAW_MAX = 45
        self.PITCH_MIN = -90
        self.PITCH_MAX = 25

        # Minimal smoothing history
        self.yaw_history = deque(maxlen=1)  # Only keep current value
        self.pitch_history = deque(maxlen=1)  # Only keep current value
        self.last_yaw = 0
        self.last_pitch = 0

        # State tracking
        self.is_tracking_active = False
        self.last_tracking_time = time.time()
        self.last_position = (self.frame_center_x, self.frame_center_y)

    def setup_logger(self):
        log_format = '[%(levelname)s] %(asctime)s [SIYISDK::%(funcName)s] :\t%(message)s'
        logging.basicConfig(format=log_format, level=logging.INFO)
        self.logger = logging.getLogger('GimbalController')

    def connect(self):
        self.logger.info("Connecting to SIYI camera...")
        if not self.camera.connect():
            self.logger.error("Failed to connect to camera")
            return False
        try:
            self.last_yaw, self.last_pitch, _ = self.camera.getAttitude()
            self.yaw_history.extend([self.last_yaw] * 3)
            self.pitch_history.extend([self.last_pitch] * 3)
            self.logger.info(f"Camera connected! Initial position: Yaw={self.last_yaw:.1f}, Pitch={self.last_pitch:.1f}")
            return True
        except:
            self.logger.error("Failed to get initial attitude")
            return False

    def smooth_command(self, new_value, history):
        if not history:
            return new_value

        # Heavy weight on newest value
        weights = [0.2, 0.3, 0.5]  # 50% newest, 30% middle, 20% oldest
        while len(weights) > len(history):
            weights.pop(0)
        weights = [w/sum(weights) for w in weights]

        smoothed = sum(h * w for h, w in zip(history, weights))
        return smoothed * self.smoothing_factor + new_value * (1 - self.smoothing_factor)

    def calculate_gimbal_angles(self, tracking_data):
        try:
            roi_center_x = tracking_data['center_x']
            roi_center_y = tracking_data['center_y']

            x_error = roi_center_x - self.frame_center_x
            y_error = roi_center_y - self.frame_center_y

            # Apply deadzone
            if abs(x_error) < self.deadzone:
                x_error = 0
            if abs(y_error) < self.deadzone:
                y_error = 0

            try:
                current_yaw, current_pitch, _ = self.camera.getAttitude()
                self.last_yaw, self.last_pitch = current_yaw, current_pitch
            except:
                current_yaw, current_pitch = self.last_yaw, self.last_pitch

            yaw_error = x_error * (90 / self.frame_width)
            pitch_error = y_error * (60 / self.frame_height)

            yaw_adjustment = self.yaw_pid.compute(0, yaw_error)
            pitch_adjustment = self.pitch_pid.compute(0, pitch_error)

            if yaw_adjustment is None or pitch_adjustment is None:
                return None, None

            # Apply rate limiting
            yaw_adjustment = max(min(yaw_adjustment, self.max_rate_change), -self.max_rate_change)
            pitch_adjustment = max(min(pitch_adjustment, self.max_rate_change), -self.max_rate_change)

            target_yaw = current_yaw + yaw_adjustment
            target_pitch = current_pitch + pitch_adjustment

            # Apply smoothing
            smoothed_yaw = self.smooth_command(target_yaw, self.yaw_history)
            smoothed_pitch = self.smooth_command(target_pitch, self.pitch_history)

            # Clamp to limits
            final_yaw = max(min(smoothed_yaw, self.YAW_MAX), self.YAW_MIN)
            final_pitch = max(min(smoothed_pitch, self.PITCH_MAX), self.PITCH_MIN)

            self.logger.debug(
                f"Smoothed Movement:\n"
                f"Error (pixels): ({x_error:.1f}, {y_error:.1f})\n"
                f"Adjustment: ({yaw_adjustment:.2f}°, {pitch_adjustment:.2f}°)\n"
                f"Final Position: ({final_yaw:.2f}°, {final_pitch:.2f}°)"
            )

            return final_yaw, final_pitch

        except KeyError as e:
            self.logger.error(f"Missing key in tracking data: {e}")
            return None, None

    def control_gimbal(self):
        """Main control loop with direct movement"""
        self.logger.info("Starting gimbal control...")
        last_movement_time = time.time()

        try:
            while True:
                try:
                    message = self.socket.recv_string(flags=zmq.NOBLOCK)
                    tracking_data = json.loads(message)
                    current_time = time.time()

                    # Process ROI position
                    roi_x = tracking_data.get('center_x', self.frame_center_x)
                    roi_y = tracking_data.get('center_y', self.frame_center_y)

                    # Log position periodically
                    if current_time - last_movement_time > 0.5:
                        x_off = roi_x - self.frame_center_x
                        y_off = roi_y - self.frame_center_y
                        self.logger.info(
                            f"ROI Position: ({roi_x:.1f}, {roi_y:.1f})\n"
                            f"Center Offset: ({x_off:+.1f}, {y_off:+.1f}) pixels"
                        )
                        last_movement_time = current_time

                    new_yaw, new_pitch = self.calculate_gimbal_angles(tracking_data)

                    if new_yaw is not None and new_pitch is not None:
                        # Use minimal error threshold for faster updates
                        self.camera.setGimbalRotation(
                            yaw=new_yaw,
                            pitch=new_pitch,
                            err_thresh=0.05  # Tighter threshold
                        )

                    time.sleep(0.005)  # 200Hz update rate

                except zmq.Again:
                    time.sleep(0.001)
                    continue
                except json.JSONDecodeError as e:
                    self.logger.error(f"Invalid JSON data: {e}")
                    continue
                except KeyError as e:
                    self.logger.error(f"Missing key in tracking data: {e}")
                    continue

        except KeyboardInterrupt:
            self.logger.info("Control stopped by user")
        except Exception as e:
            self.logger.error(f"Unexpected error: {e}")
        finally:
            self.cleanup()

    def cleanup(self):
        """Cleanup resources"""
        self.logger.info("Cleaning up...")
        self.camera.disconnect()
        self.socket.close()
        self.context.term()
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
