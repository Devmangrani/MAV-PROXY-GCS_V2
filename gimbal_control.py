import zmq
import json
import time
import math
import numpy as np
from pymavlink import mavutil
from threading import Thread, Lock
from typing import Dict, Optional, Tuple


class GimbalController:
    def __init__(self,
                 connection_string: str = 'udp:127.0.0.1:14550',
                 camera_fov_h: float = 60.0,  # Horizontal FOV in degrees
                 camera_fov_v: float = 45.0,  # Vertical FOV in degrees
                 frame_width: int = 640,
                 frame_height: int = 480,
                 debug: bool = False):
        """
        Initialize MAVLink gimbal controller

        Args:
            connection_string: MAVLink connection string
            camera_fov_h: Horizontal field of view in degrees
            camera_fov_v: Vertical field of view in degrees
            frame_width: Camera frame width
            frame_height: Camera frame height
            debug: Enable debug output
        """
        # MAVLink setup
        self.vehicle = mavutil.mavlink_connection(connection_string)
        self.vehicle.wait_heartbeat()
        print(f"Connected to vehicle on {connection_string}")

        # Camera parameters
        self.camera_fov_h = camera_fov_h
        self.camera_fov_v = camera_fov_v
        self.frame_width = frame_width
        self.frame_height = frame_height

        # ZMQ setup for receiving tracker data
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect("tcp://localhost:5555")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")

        # Threading setup
        self.running = True
        self.lock = Lock()
        self.last_tracking_data = None
        self.debug = debug

        # Gimbal parameters
        self.max_rot_speed = 90.0  # degrees per second
        self.min_confidence = 0.5

        # Start processing thread
        self.process_thread = Thread(target=self._process_tracking_data)
        self.process_thread.daemon = True
        self.process_thread.start()

    def _calculate_gimbal_angles(self,
                                 target_center: Tuple[float, float],
                                 target_size: Tuple[float, float],
                                 velocity: Tuple[float, float]) -> Tuple[float, float, float]:
        """
        Calculate gimbal angles from tracking data

        Args:
            target_center: (x, y) center of target in pixels
            target_size: (width, height) of target in pixels
            velocity: (vx, vy) target velocity in pixels/sec

        Returns:
            Tuple of (pitch, yaw, roll) in degrees
        """
        # Normalize coordinates to [-1, 1]
        norm_x = (target_center[0] / self.frame_width - 0.5) * 2.0
        norm_y = (target_center[1] / self.frame_height - 0.5) * 2.0

        # Calculate basic angles
        yaw = norm_x * (self.camera_fov_h / 2.0)
        pitch = -norm_y * (self.camera_fov_v / 2.0)  # Negative because positive pitch is down

        # Apply velocity compensation
        if abs(velocity[0]) > 0.1 or abs(velocity[1]) > 0.1:
            vel_magnitude = math.sqrt(velocity[0] ** 2 + velocity[1] ** 2)
            lead_factor = min(0.1, vel_magnitude / 1000.0)  # Limit lead angle

            yaw += math.atan2(velocity[0], self.frame_width) * lead_factor
            pitch -= math.atan2(velocity[1], self.frame_height) * lead_factor

        return pitch, yaw, 0.0  # Roll is usually 0 for normal tracking

    def _process_tracking_data(self):
        """Process incoming tracking data and send gimbal commands"""
        last_command_time = time.time()
        min_command_interval = 0.05  # 20Hz maximum command rate

        while self.running:
            try:
                # Receive tracking data
                message = self.socket.recv_string(flags=zmq.NOBLOCK)
                data = json.loads(message)

                current_time = time.time()
                if current_time - last_command_time < min_command_interval:
                    continue

                # Extract tracking data
                target_center = (
                    data['x'] + data['width'] / 2.0,
                    data['y'] + data['height'] / 2.0
                )
                target_size = (data['width'], data['height'])
                velocity = (data['velocity']['x'], data['velocity']['y'])
                confidence = data['confidence']
                is_active = data['is_active']

                if is_active and confidence > self.min_confidence:
                    # Calculate gimbal angles
                    pitch, yaw, roll = self._calculate_gimbal_angles(
                        target_center, target_size, velocity
                    )

                    # Send MAVLink command
                    self.vehicle.mav.mount_control_send(
                        self.vehicle.target_system,
                        self.vehicle.target_component,
                        int(pitch * 100),  # centidegrees
                        int(roll * 100),
                        int(yaw * 100),
                        0  # MAV_MOUNT_MODE_MAVLINK_TARGETING
                    )

                    # Send ROI location command
                    self.vehicle.mav.command_long_send(
                        self.vehicle.target_system,
                        self.vehicle.target_component,
                        mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
                        0,  # confirmation
                        0,  # param1
                        0,  # param2
                        0,  # param3
                        0,  # param4
                        target_center[0],  # param5 - lat
                        target_center[1],  # param6 - lon
                        0  # param7 - alt
                    )

                    if self.debug:
                        print(f"Sent gimbal command - Pitch: {pitch:.1f}, Yaw: {yaw:.1f}")
                        print(f"Target center: {target_center}, Velocity: {velocity}")
                        print(f"Confidence: {confidence:.2f}")

                    last_command_time = current_time

            except zmq.Again:
                time.sleep(0.001)  # Prevent busy waiting
                continue
            except Exception as e:
                print(f"Error processing tracking data: {e}")
                time.sleep(0.1)

    def stop(self):
        """Stop the controller and cleanup"""
        self.running = False
        self.process_thread.join()
        self.socket.close()
        self.context.term()


def main():
    # Create and start controller
    controller = GimbalController(
        connection_string='udp:127.0.0.1:14550',
        camera_fov_h=60.0,
        camera_fov_v=45.0,
        frame_width=640,
        frame_height=480,
        debug=True
    )

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        controller.stop()
        print("Controller stopped")


if __name__ == "__main__":
    main()
