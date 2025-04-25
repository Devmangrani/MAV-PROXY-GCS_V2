import zmq
import json
import time
from datetime import datetime
import logging
from collections import deque
import statistics


class ZMQMonitor:
    def __init__(self, zmq_port=5555):
        # Setup logging
        self.setup_logger()

        # ZMQ setup
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.RCVHWM, 1000)
        self.socket.setsockopt(zmq.LINGER, 100)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")

        # Statistics tracking
        self.message_count = 0
        self.start_time = time.time()
        self.last_message_time = None
        self.message_intervals = deque(maxlen=100)  # Store last 100 intervals
        self.rates = deque(maxlen=10)  # Store last 10 rate calculations

        # Connect to publisher
        self.connect_to_publisher(zmq_port)

    def setup_logger(self):
        logging.basicConfig(
            format='%(asctime)s - %(levelname)s - %(message)s',
            level=logging.INFO
        )
        self.logger = logging.getLogger('ZMQMonitor')

    def connect_to_publisher(self, port):
        url = f"tcp://localhost:{port}"
        try:
            self.logger.info(f"Connecting to publisher at {url}")
            self.socket.connect(url)

            # Optional: Setup sync socket
            sync_socket = self.context.socket(zmq.REQ)
            sync_socket.connect(f"tcp://localhost:{port + 1}")

            # Send sync request
            sync_socket.send_string("READY")

            # Wait for acknowledgment with timeout
            if sync_socket.poll(timeout=1000):  # 1 second timeout
                sync_socket.recv_string()
                self.logger.info("Successfully synchronized with publisher")
            else:
                self.logger.warning("No sync response from publisher, continuing anyway")

            sync_socket.close()

        except Exception as e:
            self.logger.error(f"Failed to connect: {e}")
            raise

    def calculate_statistics(self):
        elapsed_time = time.time() - self.start_time
        current_rate = self.message_count / elapsed_time if elapsed_time > 0 else 0
        self.rates.append(current_rate)

        stats = {
            "Total Messages": self.message_count,
            "Runtime": f"{elapsed_time:.1f}s",
            "Current Rate": f"{current_rate:.1f} msg/s",
            "Average Rate": f"{statistics.mean(self.rates):.1f} msg/s" if self.rates else "N/A",
        }

        if self.message_intervals:
            stats.update({
                "Avg Interval": f"{statistics.mean(self.message_intervals) * 1000:.1f}ms",
                "Min Interval": f"{min(self.message_intervals) * 1000:.1f}ms",
                "Max Interval": f"{max(self.message_intervals) * 1000:.1f}ms"
            })

        return stats

    def monitor_messages(self):
        self.logger.info("Starting message monitoring...")
        last_stats_time = time.time()

        try:
            while True:
                try:
                    # Try to receive message with timeout
                    if self.socket.poll(timeout=1000):  # 1 second timeout
                        message = self.socket.recv_string()
                        current_time = time.time()

                        # Update timing statistics
                        if self.last_message_time is not None:
                            interval = current_time - self.last_message_time
                            self.message_intervals.append(interval)

                        self.last_message_time = current_time
                        self.message_count += 1

                        # Parse and display message
                        try:
                            data = json.loads(message)

                            # Print message details
                            if self.message_count <= 5 or self.message_count % 100 == 0:
                                self.logger.info("\nMessage Details:")
                                self.logger.info(json.dumps(data, indent=2))

                        except json.JSONDecodeError as e:
                            self.logger.error(f"Failed to parse message: {e}")
                            self.logger.error(f"Raw message: {message[:100]}...")
                            continue

                        # Print statistics every second
                        if current_time - last_stats_time >= 1.0:
                            stats = self.calculate_statistics()
                            self.logger.info("\nStatistics:")
                            for key, value in stats.items():
                                self.logger.info(f"{key}: {value}")
                            last_stats_time = current_time

                    else:
                        self.logger.warning("No messages received in the last second")

                except zmq.Again:
                    continue
                except KeyboardInterrupt:
                    break
                except Exception as e:
                    self.logger.error(f"Error processing message: {e}")
                    continue

        finally:
            self.cleanup()

    def cleanup(self):
        self.logger.info("Cleaning up...")
        self.socket.close()
        self.context.term()
        self.logger.info("Monitor stopped")


def main():
    monitor = ZMQMonitor(zmq_port=5555)
    monitor.monitor_messages()


if __name__ == "__main__":
    main()