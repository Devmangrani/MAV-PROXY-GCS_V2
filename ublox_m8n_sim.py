import argparse
import math
import random
import time
import serial
import pynmea2
from datetime import datetime, timezone
import threading
import logging
import numpy as np  # For advanced calculations

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('gps_simulator.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


class GPSCarSimulator:
    """
    Simulates a car moving around with a U-blox GPS M8N, generating NMEA sentences.
    """

    def __init__(self, serial_port, baudrate=9600, initial_lat=37.7749, initial_lon=-122.4194,
                 initial_speed=0, heading=0, update_rate=1.0, noise_level=0.000001,
                 pattern='circle', circle_radius=100, constant_speed=False):
        """
        Initialize the GPS simulator.

        Args:
            serial_port: The serial port to write NMEA sentences to
            baudrate: Baud rate for the serial connection
            initial_lat: Starting latitude in decimal degrees
            initial_lon: Starting longitude in decimal degrees
            initial_speed: Starting speed in m/s
            heading: Initial heading in degrees (0-359)
            update_rate: Rate to generate GPS updates in Hz
            noise_level: GPS position noise level in decimal degrees
            pattern: Movement pattern ('circle', 'figure8', 'random', 'line')
            circle_radius: Radius for circular patterns in meters
            constant_speed: Whether to maintain constant speed (no random changes)
        """
        self.serial_port_name = serial_port
        self.baudrate = baudrate
        self.serial_port = None

        # Position and movement
        self.latitude = initial_lat
        self.longitude = initial_lon
        self.altitude = 10.0  # meters above sea level

        # Speed settings
        self.speed = initial_speed
        self.target_speed = initial_speed
        self.min_speed = 0.5
        self.max_speed = 5.0
        self.acceleration = 0.5  # m/s²
        self.constant_speed = constant_speed

        # If constant speed is enabled, make sure we start at the target speed
        if self.constant_speed:
            self.speed = self.target_speed

        # Heading in degrees (0-359)
        self.heading = heading
        self.turn_rate = 5.0  # degrees per second for standard turns

        # Update frequency
        self.update_rate = update_rate
        self.update_interval = 1.0 / update_rate

        # GPS quality
        self.hdop = 0.8  # horizontal dilution of precision
        self.num_sats = 14  # number of satellites
        self.fix_quality = 1  # GPS fix quality (0=no fix, 1=GPS fix, 2=DGPS)
        self.noise_level = noise_level

        # Movement pattern
        self.pattern = pattern
        self.circle_radius = circle_radius  # meters
        self.circle_center = (initial_lat, initial_lon)
        self.circle_angle = 0  # degrees

        # Figure-8 pattern parameters
        self.figure8_a = circle_radius  # semi-major axis for figure-8
        self.figure8_b = circle_radius / 2  # semi-minor axis for figure-8
        self.figure8_angle = 0  # degrees

        # Line pattern parameters
        self.line_start = (initial_lat, initial_lon)
        # Calculate end point 1km away at initial heading
        self.line_distance = 1000  # meters
        self.line_end = self._compute_destination(
            initial_lat, initial_lon, self.line_distance, heading)
        self.line_progress = 0.0  # 0.0 to 1.0
        self.line_direction = 1  # 1 for forward, -1 for backward

        # Control
        self.running = False
        self.thread = None
        self.generate_gga = True
        self.generate_rmc = True
        self.generate_vtg = True

        # For NMEA sentence generation
        self.last_time = time.time()

    def _earth_radius(self, lat):
        """
        Calculate Earth radius at a given latitude using WGS84 ellipsoid.
        """
        # WGS84 parameters
        a = 6378137.0  # semi-major axis (m)
        b = 6356752.314245  # semi-minor axis (m)

        lat_rad = math.radians(lat)
        cos_lat = math.cos(lat_rad)
        sin_lat = math.sin(lat_rad)

        # Calculate radius
        numerator = (a * a * cos_lat) ** 2 + (b * b * sin_lat) ** 2
        denominator = (a * cos_lat) ** 2 + (b * sin_lat) ** 2
        radius = math.sqrt(numerator / denominator)

        return radius

    def _compute_destination(self, lat, lon, distance, bearing):
        """
        Compute destination point given starting point, distance, and bearing.

        Args:
            lat: Starting latitude in decimal degrees
            lon: Starting longitude in decimal degrees
            distance: Distance to travel in meters
            bearing: Bearing in degrees (0=North, 90=East, etc.)

        Returns:
            (lat, lon) tuple with destination coordinates
        """
        # Convert to radians
        lat1 = math.radians(lat)
        lon1 = math.radians(lon)
        bearing_rad = math.radians(bearing)

        # Get Earth radius at current latitude
        radius = self._earth_radius(lat)

        # Calculate angular distance
        angular_distance = distance / radius

        # Calculate new latitude
        sin_lat1 = math.sin(lat1)
        cos_lat1 = math.cos(lat1)
        sin_d = math.sin(angular_distance)
        cos_d = math.cos(angular_distance)
        sin_bearing = math.sin(bearing_rad)
        cos_bearing = math.cos(bearing_rad)

        lat2_rad = math.asin(sin_lat1 * cos_d + cos_lat1 * sin_d * cos_bearing)
        lon2_rad = lon1 + math.atan2(sin_bearing * sin_d * cos_lat1, cos_d - sin_lat1 * math.sin(lat2_rad))

        # Convert back to degrees
        lat2 = math.degrees(lat2_rad)
        lon2 = math.degrees(lon2_rad)

        return (lat2, lon2)

    def _calculate_distance(self, lat1, lon1, lat2, lon2):
        """
        Calculate distance between two GPS coordinates using Haversine formula.
        Returns distance in meters.
        """
        # Calculate earth radius at midpoint latitude
        mid_lat = (lat1 + lat2) / 2
        radius = self._earth_radius(mid_lat)

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
        distance = radius * c

        return distance

    def _get_current_time(self):
        """
        Get current UTC time formatted for NMEA sentences.
        Returns (time_str, date_str) tuple.
        """
        now = datetime.now(timezone.utc)
        time_str = now.strftime("%H%M%S.%f")[:-4]  # HHMMSS.ss format
        date_str = now.strftime("%d%m%y")  # DDMMYY format
        return time_str, date_str

    def _format_degrees(self, decimal_degrees, is_latitude=True):
        """
        Convert decimal degrees to NMEA format (DDMM.MMMM or DDDMM.MMMM).
        """
        # Handle negative values
        sign = 1
        if decimal_degrees < 0:
            sign = -1
            decimal_degrees = abs(decimal_degrees)

        # For longitude (DDDMM.MMMM)
        if not is_latitude:
            degrees = int(decimal_degrees)
            minutes = (decimal_degrees - degrees) * 60.0
            formatted = f"{degrees:03d}{minutes:09.6f}".replace(".", "")
            formatted = formatted[:3] + formatted[3:5] + "." + formatted[5:]
            return formatted

        # For latitude (DDMM.MMMM)
        degrees = int(decimal_degrees)
        minutes = (decimal_degrees - degrees) * 60.0
        formatted = f"{degrees:02d}{minutes:09.6f}".replace(".", "")
        formatted = formatted[:2] + formatted[2:4] + "." + formatted[4:]
        return formatted

    def _add_nmea_checksum(self, nmea_str):
        """
        Calculate and append the checksum for an NMEA sentence.
        """
        checksum = 0
        for char in nmea_str[1:]:  # Skip the $ at the beginning
            checksum ^= ord(char)
        return f"{nmea_str}*{checksum:02X}\r\n"

    def _create_gga_sentence(self):
        """
        Create a GGA (Global Positioning System Fix Data) sentence.
        """
        time_str, _ = self._get_current_time()

        # Format latitude and longitude for NMEA
        lat_nmea = self._format_degrees(abs(self.latitude), is_latitude=True)
        lat_dir = 'N' if self.latitude >= 0 else 'S'

        lon_nmea = self._format_degrees(abs(self.longitude), is_latitude=False)
        lon_dir = 'E' if self.longitude >= 0 else 'W'

        # Create GGA sentence components
        gga_parts = [
            "$GPGGA",
            time_str,
            lat_nmea,
            lat_dir,
            lon_nmea,
            lon_dir,
            str(self.fix_quality),
            f"{self.num_sats:02d}",
            f"{self.hdop:.1f}",
            f"{self.altitude:.1f}",
            "M",  # Altitude units (meters)
            "0.0",  # Height of geoid above WGS84 ellipsoid
            "M",  # Units (meters)
            "",  # Time since DGPS update (empty)
            ""  # DGPS reference station ID (empty)
        ]

        # Join and add checksum
        gga_sentence = ",".join(gga_parts)
        return self._add_nmea_checksum(gga_sentence)

    def _create_rmc_sentence(self):
        """
        Create an RMC (Recommended Minimum Navigation Information) sentence.
        """
        time_str, date_str = self._get_current_time()

        # Format latitude and longitude for NMEA
        lat_nmea = self._format_degrees(abs(self.latitude), is_latitude=True)
        lat_dir = 'N' if self.latitude >= 0 else 'S'

        lon_nmea = self._format_degrees(abs(self.longitude), is_latitude=False)
        lon_dir = 'E' if self.longitude >= 0 else 'W'

        # Convert speed from m/s to knots (1 m/s = 1.94384 knots)
        speed_knots = self.speed * 1.94384

        # Create RMC sentence components
        rmc_parts = [
            "$GPRMC",
            time_str,
            "A",  # Status (A=active, V=void)
            lat_nmea,
            lat_dir,
            lon_nmea,
            lon_dir,
            f"{speed_knots:.2f}",  # Speed in knots
            f"{self.heading:.2f}",  # Track angle in degrees
            date_str,  # Date (DDMMYY)
            "0.0",  # Magnetic variation (empty)
            "",  # Magnetic variation direction (empty)
            "A"  # Mode indicator (A=autonomous)
        ]

        # Join and add checksum
        rmc_sentence = ",".join(rmc_parts)
        return self._add_nmea_checksum(rmc_sentence)

    def _create_vtg_sentence(self):
        """
        Create a VTG (Track Made Good and Ground Speed) sentence.
        """
        # Convert speed from m/s to knots and km/h
        speed_knots = self.speed * 1.94384
        speed_kmh = self.speed * 3.6

        # Create VTG sentence components
        vtg_parts = [
            "$GPVTG",
            f"{self.heading:.2f}",  # True track in degrees
            "T",  # True track reference
            "",  # Magnetic track (empty)
            "M",  # Magnetic track reference
            f"{speed_knots:.2f}",  # Speed in knots
            "N",  # Knots unit
            f"{speed_kmh:.2f}",  # Speed in km/h
            "K",  # Km/h unit
            "A"  # Mode indicator (A=autonomous)
        ]

        # Join and add checksum
        vtg_sentence = ",".join(vtg_parts)
        return self._add_nmea_checksum(vtg_sentence)

    def _update_position_circle(self, dt):
        """
        Update position for circular movement pattern.
        """
        # Update circle angle based on current speed and radius
        # Angular velocity = linear velocity / radius
        if self.circle_radius > 0:
            angular_velocity = self.speed / self.circle_radius  # radians per second
            self.circle_angle += math.degrees(angular_velocity * dt)
            self.circle_angle %= 360  # Keep within 0-360 range

        # Calculate new position based on circle angle
        angle_rad = math.radians(self.circle_angle)

        # Convert radius from meters to degrees (approximate)
        radius_lat = self.circle_radius / 111000  # 1 degree latitude ≈ 111 km
        radius_lon = self.circle_radius / (111000 * math.cos(math.radians(self.circle_center[0])))

        # Calculate new position
        new_lat = self.circle_center[0] + radius_lat * math.cos(angle_rad)
        new_lon = self.circle_center[1] + radius_lon * math.sin(angle_rad)

        # Update heading to be tangent to the circle (90 degrees offset from radius)
        self.heading = (self.circle_angle + 90) % 360

        return new_lat, new_lon

    def _update_position_figure8(self, dt):
        """
        Update position for figure-8 movement pattern.
        """
        # Update angle based on current speed
        perimeter = 2 * math.pi * math.sqrt((self.figure8_a ** 2 + self.figure8_b ** 2) / 2)  # approximate
        angular_velocity = self.speed / (perimeter / (2 * math.pi))  # radians per second
        self.figure8_angle += math.degrees(angular_velocity * dt)
        self.figure8_angle %= 360

        # Calculate parametric figure-8 position
        angle_rad = math.radians(self.figure8_angle)

        # Convert from meters to degrees (approximate)
        a_lat = self.figure8_a / 111000
        b_lon = self.figure8_b / (111000 * math.cos(math.radians(self.circle_center[0])))

        # Figure-8 formula
        new_lat = self.circle_center[0] + a_lat * math.sin(angle_rad)
        new_lon = self.circle_center[1] + b_lon * math.sin(angle_rad) * math.cos(angle_rad)

        # Calculate heading as tangent to the curve
        dx = b_lon * (math.cos(angle_rad) ** 2 - math.sin(angle_rad) ** 2)
        dy = a_lat * math.cos(angle_rad)

        # Calculate heading angle (in degrees from North)
        self.heading = (math.degrees(math.atan2(dx, dy)) + 360) % 360

        return new_lat, new_lon

    def _update_position_line(self, dt):
        """
        Update position for linear movement pattern.
        """
        # Calculate distance for this update
        distance_this_update = self.speed * dt

        # Calculate full line length
        full_distance = self._calculate_distance(
            self.line_start[0], self.line_start[1],
            self.line_end[0], self.line_end[1]
        )

        # Update progress along the line
        if full_distance > 0:
            progress_change = (distance_this_update / full_distance) * self.line_direction
            self.line_progress += progress_change

            # Check if we've reached an endpoint
            if self.line_progress >= 1.0:
                self.line_progress = 1.0
                self.line_direction = -1  # Switch direction
                self.heading = (self.heading + 180) % 360
            elif self.line_progress <= 0.0:
                self.line_progress = 0.0
                self.line_direction = 1  # Switch direction
                self.heading = (self.heading + 180) % 360

        # Interpolate position along the line
        new_lat = self.line_start[0] + (self.line_end[0] - self.line_start[0]) * self.line_progress
        new_lon = self.line_start[1] + (self.line_end[1] - self.line_start[1]) * self.line_progress

        return new_lat, new_lon

    def _update_position_random(self, dt):
        """
        Update position with random walk.
        """
        # Randomly change heading occasionally
        if random.random() < 0.05:
            heading_change = random.uniform(-45, 45)
            self.heading = (self.heading + heading_change) % 360

        # Calculate distance for this update
        distance = self.speed * dt

        # Calculate new position using the bearing formula
        new_lat, new_lon = self._compute_destination(
            self.latitude, self.longitude, distance, self.heading)

        return new_lat, new_lon

    def _update_speed(self, dt):
        """
        Update the current speed, gradually moving toward target speed.
        """
        # Calculate speed difference
        speed_diff = self.target_speed - self.speed

        # Apply acceleration
        if abs(speed_diff) < self.acceleration * dt:
            self.speed = self.target_speed
        else:
            self.speed += math.copysign(self.acceleration * dt, speed_diff)

    def _update_target_speed(self):
        """
        Occasionally update the target speed.
        Only applies if constant_speed is False.
        """
        # Skip if constant_speed is enabled
        if self.constant_speed:
            return

        # Change target speed with 5% probability per second
        # Adjusted by update_rate to be time-consistent
        if random.random() < 0.05 / self.update_rate:
            self.target_speed = random.uniform(self.min_speed, self.max_speed)
            logger.info(f"New target speed: {self.target_speed:.2f} m/s")

    def _add_gps_noise(self, lat, lon):
        """
        Add realistic GPS noise to latitude and longitude.
        """
        # Base noise level (more in longitude at higher latitudes)
        lat_noise = random.gauss(0, self.noise_level)
        lon_factor = 1.0 / math.cos(math.radians(abs(lat)))
        lon_noise = random.gauss(0, self.noise_level * lon_factor)

        # HDOP affects noise amplitude
        lat_noise *= self.hdop
        lon_noise *= self.hdop

        return lat + lat_noise, lon + lon_noise

    def update(self):
        """
        Update GPS position, speed, and heading based on movement pattern.
        """
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Update target speed occasionally
        self._update_target_speed()

        # Gradually update speed to match target
        self._update_speed(dt)

        # Update position based on the selected pattern
        if self.pattern == 'circle':
            new_lat, new_lon = self._update_position_circle(dt)
        elif self.pattern == 'figure8':
            new_lat, new_lon = self._update_position_figure8(dt)
        elif self.pattern == 'line':
            new_lat, new_lon = self._update_position_line(dt)
        else:  # 'random' or default
            new_lat, new_lon = self._update_position_random(dt)

        # Add GPS noise
        self.latitude, self.longitude = self._add_gps_noise(new_lat, new_lon)

        # Write NMEA sentences to the serial port
        if self.serial_port and self.serial_port.is_open:
            if self.generate_gga:
                gga = self._create_gga_sentence()
                self.serial_port.write(gga.encode('ascii'))

            if self.generate_rmc:
                rmc = self._create_rmc_sentence()
                self.serial_port.write(rmc.encode('ascii'))

            if self.generate_vtg:
                vtg = self._create_vtg_sentence()
                self.serial_port.write(vtg.encode('ascii'))

    def start(self):
        """
        Start the GPS simulation.
        """
        if self.running:
            logger.warning("Simulator already running")
            return False

        try:
            # Open serial port
            self.serial_port = serial.Serial(
                port=self.serial_port_name,
                baudrate=self.baudrate,
                timeout=1
            )
            logger.info(f"Opened serial port: {self.serial_port_name}")

            # Start simulation thread
            self.running = True
            self.thread = threading.Thread(target=self._run_simulation)
            self.thread.daemon = True
            self.thread.start()

            logger.info(f"GPS simulation started with pattern: {self.pattern}")
            logger.info(f"Initial position: {self.latitude:.6f}, {self.longitude:.6f}")
            logger.info(f"Initial speed: {self.speed:.2f} m/s, heading: {self.heading:.1f}°")

            return True

        except serial.SerialException as e:
            logger.error(f"Failed to open serial port: {e}")
            return False

    def _run_simulation(self):
        """
        Run the simulation loop (called in a separate thread).
        """
        while self.running:
            start_time = time.time()
            self.update()

            # Calculate sleep time to maintain update rate
            elapsed = time.time() - start_time
            sleep_time = max(0, self.update_interval - elapsed)

            time.sleep(sleep_time)

    def stop(self):
        """
        Stop the GPS simulation.
        """
        self.running = False

        if self.thread:
            self.thread.join(timeout=2.0)
            self.thread = None

        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            logger.info("Closed serial port")

        logger.info("GPS simulation stopped")


def create_virtual_serial_ports():
    """
    Create a pair of virtual serial ports for testing using socat.
    Returns (port1, port2) names.
    """
    import subprocess
    import time
    import os

    # Check if socat is installed
    try:
        subprocess.check_call(["which", "socat"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except subprocess.CalledProcessError:
        logger.error("socat is not installed. Please install it with: sudo apt-get install socat")
        return None, None

    # Create a unique pair of virtual ports
    port1 = "/tmp/vgps1"
    port2 = "/tmp/vgps2"

    # Remove if they already exist
    if os.path.exists(port1):
        os.unlink(port1)
    if os.path.exists(port2):
        os.unlink(port2)

    # Start socat in the background
    cmd = f"socat -d -d pty,link={port1},raw,echo=0 pty,link={port2},raw,echo=0"
    try:
        process = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        # Give socat time to set up the ports
        time.sleep(1)

        # Check if ports were created
        if os.path.exists(port1) and os.path.exists(port2):
            logger.info(f"Created virtual serial ports: {port1} and {port2}")
            return port1, port2
        else:
            logger.error("Failed to create virtual serial ports")
            return None, None
    except Exception as e:
        logger.error(f"Error creating virtual serial ports: {e}")
        return None, None


def main():
    parser = argparse.ArgumentParser(description='GPS Car Movement Simulator')
    parser.add_argument('--port', type=str, default='/dev/ttyUSB0',
                        help='Serial port for GPS output')
    parser.add_argument('--baud', type=int, default=9600,
                        help='Baud rate for serial connection')
    parser.add_argument('--virtual', action='store_true',
                        help='Create virtual serial port pair using socat')
    parser.add_argument('--lat', type=float, default=37.7749,
                        help='Initial latitude')
    parser.add_argument('--lon', type=float, default=-122.4194,
                        help='Initial longitude')
    parser.add_argument('--speed', type=float, default=5.0,
                        help='Initial speed in m/s')
    parser.add_argument('--heading', type=float, default=0.0,
                        help='Initial heading in degrees')
    parser.add_argument('--pattern', type=str, default='line',
                        choices=['circle', 'figure8', 'line', 'random'],
                        help='Movement pattern')
    parser.add_argument('--radius', type=float, default=100.0,
                        help='Radius for circular patterns in meters')
    parser.add_argument('--rate', type=float, default=1.0,
                        help='Update rate in Hz')
    parser.add_argument('--noise', type=float, default=0.000001,
                        help='GPS noise level (degrees)')
    parser.add_argument('--debug', action='store_true',
                        help='Enable debug logging')
    parser.add_argument('--constant-speed', action='store_true',
                        help='Maintain constant speed (no random speed changes)')

    args = parser.parse_args()

    if args.debug:
        logger.setLevel(logging.DEBUG)

    port = args.port
    other_port = None

    if args.virtual:
        try:
            port1, port2 = create_virtual_serial_ports()
            if port1 is None or port2 is None:
                logger.error("Failed to create virtual serial ports. Make sure socat is installed.")
                return

            port = port1
            other_port = port2
            logger.info(f"Using virtual serial port: {port}")
            logger.info(f"Connect your GPS follower to: {other_port}")

            # Print instructions for connecting
            print("\n==== IMPORTANT CONNECTION INSTRUCTIONS ====")
            print(f"1. Run your GPS follower with: python sitl_follow_test_v5.py --gps-port={other_port}")
            print("2. This creates a direct virtual connection between the simulator and your follower")
            print("3. The simulator will provide NMEA sentences directly to your follower code")
            print("=======================================\n")

        except Exception as e:
            logger.error(f"Failed to create virtual serial ports: {e}")
            return

    # Create and start simulator
    simulator = GPSCarSimulator(
        serial_port=port,
        baudrate=args.baud,
        initial_lat=args.lat,
        initial_lon=args.lon,
        initial_speed=args.speed,
        heading=args.heading,
        update_rate=args.rate,
        noise_level=args.noise,
        pattern=args.pattern,
        circle_radius=args.radius,
        constant_speed=args.constant_speed
    )

    if simulator.start():
        try:
            logger.info(f"GPS simulator running. Press Ctrl+C to exit.")
            logger.info(f"Movement pattern: {args.pattern}")
            logger.info(f"Output port: {port} at {args.baud} baud")

            # Keep main thread alive
            while True:
                time.sleep(1)

        except KeyboardInterrupt:
            logger.info("Simulation interrupted by user")
        finally:
            simulator.stop()


if __name__ == "__main__":
    main()