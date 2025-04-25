import time
import math
import logging
import threading
from queue import Queue
from datetime import datetime
import random

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('sitl_gps_sim.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


class SITLGPSSimulator:
    def __init__(self, start_lat=23.193224, start_lon=77.365835, start_alt=0):
        """
        Initialize SITL GPS Simulator
        Args:
            start_lat: Starting latitude (default: PX4 SITL default location)
            start_lon: Starting longitude (default: PX4 SITL default location)
            start_alt: Starting altitude in meters
        """
        self.current_lat = start_lat
        self.current_lon = start_lon
        self.current_alt = start_alt
        self.gps_queue = Queue()
        self.running = False
        self.movement_thread = None

        # Movement parameters
        self.speed = 10.0  # meters per second
        self.update_rate = 10  # Hz
        self.radius = 50  # meters (for circular patterns)
        self.square_size = 100  # meters (for square pattern)

        # Movement patterns
        self.movement_patterns = {
            'circle': self._move_in_circle,
            'square': self._move_in_square,
            'figure_eight': self._move_figure_eight,
            'random_walk': self._move_random_walk,
            'straight_line': self._move_straight_line,
            'hover': self._hover
        }
        self.current_pattern = None

    def _meters_to_latlon(self, dx, dy):
        """Convert meters offset to lat/lon difference"""
        earth_radius = 6371000  # Earth radius in meters
        lat_diff = (dy / earth_radius) * (180 / math.pi)
        lon_diff = (dx / (earth_radius * math.cos(math.radians(self.current_lat)))) * (180 / math.pi)
        return lat_diff, lon_diff

    def _update_position(self, new_lat, new_lon, new_alt=None):
        """Update current position and put it in the queue"""
        self.current_lat = new_lat
        self.current_lon = new_lon
        if new_alt is not None:
            self.current_alt = new_alt

        position = {
            'lat': self.current_lat,
            'lon': self.current_lon,
            'alt': self.current_alt,
            'timestamp': datetime.now(),
            'fix_type': 3,  # Simulating 3D fix
            'num_satellites': 14  # Simulating good satellite coverage
        }

        self.gps_queue.put(position)
        logger.debug(f"Updated position: Lat {new_lat:.6f}, Lon {new_lon:.6f}, Alt {self.current_alt:.1f}m")

    def _hover(self):
        """Stay at the current position"""
        while self.running and self.current_pattern == 'hover':
            self._update_position(self.current_lat, self.current_lon)
            time.sleep(1 / self.update_rate)

    def _move_in_circle(self):
        """Generate circular movement pattern"""
        center_lat = self.current_lat
        center_lon = self.current_lon
        angle = 0

        while self.running and self.current_pattern == 'circle':
            # Calculate offset in meters
            dx = self.radius * math.cos(angle)
            dy = self.radius * math.sin(angle)

            # Convert to lat/lon
            lat_diff, lon_diff = self._meters_to_latlon(dx, dy)

            # Update position
            new_lat = center_lat + lat_diff
            new_lon = center_lon + lon_diff

            self._update_position(new_lat, new_lon)

            # Update angle based on speed and radius
            angle += (self.speed / self.radius) * (1 / self.update_rate)
            if angle > 2 * math.pi:
                angle -= 2 * math.pi

            time.sleep(1 / self.update_rate)

    def _move_in_square(self):
        """Generate square movement pattern"""
        # Define the four corners of the square relative to the starting position
        # Using half the square size as the distance from center to each corner
        half_size = self.square_size / 2
        corners = [
            (half_size, half_size),  # Top right
            (-half_size, half_size),  # Top left
            (-half_size, -half_size),  # Bottom left
            (half_size, -half_size)  # Bottom right
        ]

        # Store the center position
        center_lat = self.current_lat
        center_lon = self.current_lon

        # Initialize corner tracking
        current_corner_idx = 0
        next_corner_idx = 1

        # Calculate initial position (first corner)
        start_dx, start_dy = corners[current_corner_idx]
        start_lat_diff, start_lon_diff = self._meters_to_latlon(start_dx, start_dy)
        self.current_lat = center_lat + start_lat_diff
        self.current_lon = center_lon + start_lon_diff

        # Log the corners for debugging
        logger.info(f"Square pattern with size {self.square_size}m and speed {self.speed}m/s")
        for i, (dx, dy) in enumerate(corners):
            lat_diff, lon_diff = self._meters_to_latlon(dx, dy)
            logger.info(f"Corner {i}: dx={dx:.1f}m, dy={dy:.1f}m → lat_diff={lat_diff:.6f}, lon_diff={lon_diff:.6f}")

        while self.running and self.current_pattern == 'square':
            # Get current and next corner
            current_corner = corners[current_corner_idx]
            next_corner = corners[next_corner_idx]

            # Calculate vector from current to next corner
            dx = next_corner[0] - current_corner[0]
            dy = next_corner[1] - current_corner[1]

            # Calculate straight-line distance between corners
            distance = math.sqrt(dx * dx + dy * dy)

            # How many steps we need based on speed and update rate
            steps_needed = distance / (self.speed / self.update_rate)
            if steps_needed <= 0:
                steps_needed = 1  # Prevent division by zero

            # Calculate increments per step
            dx_step = dx / steps_needed
            dy_step = dy / steps_needed

            # Move in steps from current corner to next corner
            for _ in range(int(steps_needed)):
                if not (self.running and self.current_pattern == 'square'):
                    break

                # Calculate lat/lon difference for this step
                lat_diff, lon_diff = self._meters_to_latlon(dx_step, dy_step)

                # Update position
                self._update_position(
                    self.current_lat + lat_diff,
                    self.current_lon + lon_diff
                )

                # Sleep for precise timing
                time.sleep(1 / self.update_rate)

            # Move to next corner
            current_corner_idx = next_corner_idx
            next_corner_idx = (next_corner_idx + 1) % 4

            # Log when reaching a corner
            logger.info(f"Reached corner {current_corner_idx} of square pattern")

    def _move_figure_eight(self):
        """Generate figure-8 movement pattern"""
        angle = 0
        while self.running and self.current_pattern == 'figure_eight':
            # Calculate position using lemniscate formula
            dx = self.radius * math.cos(angle) / (1 + math.sin(angle) ** 2)
            dy = self.radius * math.sin(angle) * math.cos(angle) / (1 + math.sin(angle) ** 2)

            # Convert to lat/lon
            lat_diff, lon_diff = self._meters_to_latlon(dx, dy)

            # Update position
            self._update_position(
                self.current_lat + lat_diff,
                self.current_lon + lon_diff
            )

            angle += self.speed / (self.radius * self.update_rate)
            if angle > 2 * math.pi:
                angle -= 2 * math.pi

            time.sleep(1 / self.update_rate)

    def _move_random_walk(self):
        """Generate random walk movement pattern"""
        step_size = 2  # meters
        while self.running and self.current_pattern == 'random_walk':
            # Generate random direction
            angle = random.uniform(0, 2 * math.pi)

            # Calculate step in meters
            dx = step_size * math.cos(angle)
            dy = step_size * math.sin(angle)

            # Convert to lat/lon
            lat_diff, lon_diff = self._meters_to_latlon(dx, dy)

            # Update position
            self._update_position(
                self.current_lat + lat_diff,
                self.current_lon + lon_diff
            )

            time.sleep(1 / self.update_rate)

    def _move_straight_line(self, heading=0, distance=50):
        """
        Generate straight line movement pattern
        Args:
            heading: Heading in degrees (0 = North, 90 = East)
            distance: Distance to travel in meters
        """
        start_lat = self.current_lat
        start_lon = self.current_lon
        progress = 0

        while self.running and self.current_pattern == 'straight_line' and progress < distance:
            # Calculate step based on speed
            step = self.speed / self.update_rate
            progress += step

            # Calculate offset in meters
            dx = step * math.sin(math.radians(heading))
            dy = step * math.cos(math.radians(heading))

            # Convert to lat/lon
            lat_diff, lon_diff = self._meters_to_latlon(dx, dy)

            # Update position
            self._update_position(
                self.current_lat + lat_diff,
                self.current_lon + lon_diff
            )

            time.sleep(1 / self.update_rate)

        # After reaching distance, switch to hover
        self.start_movement('hover')

    def start_movement(self, pattern='circle', **kwargs):
        """
        Start GPS movement simulation
        Args:
            pattern: Movement pattern ('circle', 'square', 'figure_eight', 'random_walk', 'straight_line', 'hover')
            **kwargs: Additional parameters for specific patterns
        """
        if pattern not in self.movement_patterns:
            raise ValueError(f"Invalid movement pattern. Choose from: {list(self.movement_patterns.keys())}")

        # Stop any existing movement
        self.stop_movement()

        self.running = True
        self.current_pattern = pattern

        # Start movement thread
        target_func = self.movement_patterns[pattern]
        self.movement_thread = threading.Thread(
            target=target_func,
            kwargs=kwargs,
            daemon=True
        )
        self.movement_thread.start()
        logger.info(f"Started GPS simulation with {pattern} pattern")

    def stop_movement(self):
        """Stop GPS movement simulation"""
        self.running = False
        if self.movement_thread and self.movement_thread.is_alive():
            self.movement_thread.join(timeout=2)
        self.current_pattern = None
        logger.info("Stopped GPS simulation")

    def set_speed(self, speed):
        """Set movement speed in meters per second"""
        self.speed = float(speed)
        logger.info(f"Set movement speed to {self.speed} m/s")

    def set_update_rate(self, rate):
        """Set position update rate in Hz"""
        self.update_rate = float(rate)
        logger.info(f"Set update rate to {self.update_rate} Hz")

    def get_position(self):
        """Get the latest position from the queue"""
        try:
            return self.gps_queue.get_nowait()
        except Queue.Empty:
            return None

    def cleanup(self):
        """Clean up resources"""
        self.stop_movement()
        while not self.gps_queue.empty():
            self.gps_queue.get()


if __name__ == "__main__":
    # Example usage
    gps_sim = SITLGPSSimulator()

    try:
        # Set speed to 10 m/s
        gps_sim.set_speed(10.0)

        # Start with square pattern
        gps_sim.start_movement('square')

        # Monitor positions for 30 seconds
        start_time = time.time()
        while time.time() - start_time < 30:
            position = gps_sim.get_position()
            if position:
                print(f"Position: Lat {position['lat']:.6f}, Lon {position['lon']:.6f}, Alt {position['alt']:.1f}m")
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nStopping simulation...")
    finally:
        gps_sim.cleanup()
