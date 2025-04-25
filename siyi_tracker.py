import zmq
import json
import time
import logging
import numpy as np
from collections import deque
from siyi_control import SIYIControl


class GimbalController:
    def __init__(self, zmq_address="tcp://localhost:5555",
                 siyi_ip="192.168.144.25",
                 siyi_port=37260):
        # Initialize logging
        self.setup_logging()

        # ZMQ setup
        self.context = zmq.Context()
        self.subscriber = self.context.socket(zmq.SUB)

        # Tracking and control parameters
        self.frame_center = (320, 240)
        self.last_update = time.time()
        self.running = True
        self.power = 0.03  # Default power factor

        # Motion smoothing
        self.position_history = deque(maxlen=2)
        self.velocity_history = deque(maxlen=2)
        self.last_position = None

        # Initialize connections
        self.subscriber.connect(zmq_address)
        self.subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
        self.logger.info(f"Connected to tracking publisher at {zmq_address}")

        self.siyi_control = SIYIControl(_server_ip=siyi_ip, _port=siyi_port)
        self.logger.info("Gimbal controller initialized")

    def setup_logging(self):
        """Set up logging configuration."""
        self.logger = logging.getLogger('GimbalController')
        self.logger.setLevel(logging.INFO)

        # Create handlers
        console_handler = logging.StreamHandler()
        file_handler = logging.FileHandler('gimbal_control.log')

        # Create formatters and add it to handlers
        log_format = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        console_handler.setFormatter(log_format)
        file_handler.setFormatter(log_format)

        # Add handlers to the logger
        self.logger.addHandler(console_handler)
        self.logger.addHandler(file_handler)

    def calculate_gimbal_offset(self, tracking_data):
        """Calculate gimbal offset from tracking data with smoothing."""
        try:
            # Extract position data
            center_x = tracking_data.get('center_x', 0)
            center_y = tracking_data.get('center_y', 0)
            current_pos = (center_x, center_y)

            # Log received position
            self.logger.debug(f"Received position: ({center_x:.2f}, {center_y:.2f})")

            # Update position history
            self.position_history.append(current_pos)

            # Calculate smoothed position
            if len(self.position_history) >= 3:
                smoothed_x = np.median([p[0] for p in self.position_history])
                smoothed_y = np.median([p[1] for p in self.position_history])

                # Calculate offset from center
                offset_x = self.frame_center[0] - smoothed_x
                offset_y = self.frame_center[1] - smoothed_y

                # Calculate velocity if we have previous position
                if self.last_position:
                    dt = time.time() - self.last_update
                    if dt > 0:
                        velocity_x = (smoothed_x - self.last_position[0]) / dt
                        velocity_y = (smoothed_y - self.last_position[1]) / dt
                        self.velocity_history.append((velocity_x, velocity_y))

                # Apply dynamic power adjustment based on distance
                distance = np.sqrt(offset_x ** 2 + offset_y ** 2)
                base_power = self.power

                if distance > 200:  # Far from center
                    power = base_power * 1.5
                elif distance < 50:  # Close to center
                    power = base_power * 0.5
                else:
                    power = base_power

                # Calculate final offsets
                yaw_offset = offset_x * power
                pitch_offset = offset_y * power

                # Log calculations
                self.logger.info(f"Raw Offset: ({offset_x:.2f}, {offset_y:.2f})")
                self.logger.info(f"Adjusted Offset: ({yaw_offset:.2f}, {pitch_offset:.2f})")

                # Update gimbal
                self.siyi_control.set_offset(yaw_off=yaw_offset,
                                             pitch_offset=pitch_offset,
                                             power=power)

                self.last_position = (smoothed_x, smoothed_y)
                self.last_update = time.time()

                return True

            return False

        except Exception as e:
            self.logger.error(f"Error calculating gimbal offset: {e}")
            return False

    def process_message(self, message_data):
        """Process received ZMQ message."""
        try:
            data = json.loads(message_data)
            message_type = data.get('type', '')

            if message_type == 'batch':
                messages = data.get('messages', [])
                self.logger.debug(f"Received batch of {len(messages)} messages")

                for msg in messages:
                    if msg.get('is_active', False):
                        self.logger.info(f"Processing tracking data: ID={msg.get('id')}, "
                                         f"Sequence={msg.get('sequence')}, "
                                         f"Position=({msg.get('center_x', 0):.2f}, "
                                         f"{msg.get('center_y', 0):.2f})")
                        self.calculate_gimbal_offset(msg)

            elif message_type == 'heartbeat':
                self.logger.debug("Received heartbeat")
            else:
                self.logger.warning(f"Unknown message type: {message_type}")

        except json.JSONDecodeError as e:
            self.logger.error(f"Failed to decode message: {e}")
        except Exception as e:
            self.logger.error(f"Error processing message: {e}")

    def run(self):
        """Main processing loop."""
        self.logger.info("Starting gimbal control loop")

        while self.running:
            try:
                if self.subscriber.poll(100, zmq.POLLIN):
                    message = self.subscriber.recv_string()
                    self.process_message(message)

            except KeyboardInterrupt:
                self.logger.info("Shutting down...")
                break
            except Exception as e:
                self.logger.error(f"Error in main loop: {e}")
                continue

    def stop(self):
        """Clean shutdown."""
        self.running = False
        self.subscriber.close()
        self.context.term()
        self.logger.info("Gimbal controller shut down")


def main():
    controller = GimbalController()
    try:
        controller.run()
    except KeyboardInterrupt:
        print("\nShutting down controller...")
    finally:
        controller.stop()


if __name__ == "__main__":
    main()