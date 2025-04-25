import zmq
import json
from time import sleep, time
import threading
from siyi_control import SIYIControl


class ZMQSubscriber:
    def __init__(self, pub_address="tcp://localhost:5555", sync_address="tcp://localhost:5556"):
        self.context = zmq.Context()
        self.subscriber = self.context.socket(zmq.SUB)
        self.subscriber.setsockopt_string(zmq.SUBSCRIBE, "")

        # Connect to publisher
        self.subscriber.connect(pub_address)

        # Synchronization socket
        self.sync_socket = self.context.socket(zmq.REQ)
        self.sync_socket.connect(sync_address)

        # Perform synchronization
        print("Sending sync request...")
        self.sync_socket.send(b"SYNC")
        response = self.sync_socket.recv()
        print(f"Received sync response: {response}")

        self.last_sequence = 0
        self.last_heartbeat = time()

    def receive_message(self, timeout=1000):
        try:
            if self.subscriber.poll(timeout):
                message = self.subscriber.recv_string()
                data = json.loads(message)

                if data.get("type") == "heartbeat":
                    self.last_heartbeat = time()
                    return None

                if data.get("type") == "batch":
                    messages = data.get("messages", [])
                    if messages:
                        # Return the latest message from the batch
                        latest_msg = messages[-1]
                        # Print tracking info for debugging
                        print(
                            f"Received tracking - Center: ({latest_msg.get('center_x', 0)}, {latest_msg.get('center_y', 0)}), "
                            f"Confidence: {latest_msg.get('confidence', 0):.2f}")
                        return latest_msg

            current_time = time()
            if current_time - self.last_heartbeat > 2.0:  # 2 seconds timeout
                print("Warning: No heartbeat received for 2 seconds")

            return None

        except zmq.error.Again:
            return None
        except Exception as e:
            print(f"Error receiving message: {e}")
            return None


def calculate_gimbal_offset(tracking_data, frame_width=1920, frame_height=1080, power=0.03):
    """Calculate gimbal offset based on tracking data"""
    if not tracking_data or not tracking_data.get("is_active", False):
        return None, None

    # Get center point from tracking data
    center_x = tracking_data.get("center_x", 0)
    center_y = tracking_data.get("center_y", 0)

    # Calculate frame center
    frame_center_x = frame_width / 2
    frame_center_y = frame_height / 2

    # Calculate offset from center
    x_offset = frame_center_x - center_x
    y_offset = frame_center_y - center_y

    return x_offset, y_offset


def main():
    # Initialize SIYI control
    siyi_control = SIYIControl()

    # Initialize ZMQ subscriber
    zmq_sub = ZMQSubscriber()

    print("Starting gimbal control loop...")

    try:
        while True:
            # Get tracking data from ZMQ
            tracking_data = zmq_sub.receive_message()

            if tracking_data:
                # Calculate gimbal offset
                x_offset, y_offset = calculate_gimbal_offset(tracking_data)

                if x_offset is not None and y_offset is not None:
                    # Control gimbal
                    siyi_control.set_offset(yaw_off=x_offset, pitch_offset=y_offset, power=0.03)

            # Small sleep to prevent CPU overuse
            sleep(0.01)

    except KeyboardInterrupt:
        print("\nShutting down gimbal control...")
    except Exception as e:
        print(f"Error in main loop: {e}")
    finally:
        print("Gimbal control terminated")


if __name__ == "__main__":
    main()