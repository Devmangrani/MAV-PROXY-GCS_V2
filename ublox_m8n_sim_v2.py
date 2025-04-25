import argparse
import math
import random
import time
import serial
import subprocess
import os
import threading
import logging
from datetime import datetime, timezone

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
    """Simulates GPS movement through predefined waypoints with virtual port support"""

    def __init__(self, serial_port, baudrate=9600, speed=15.0, noise=0.000001, loop=False):
        self.serial_port = None
        self.baudrate = baudrate
        self.port_name = serial_port

        # Movement parameters
        self.speed = speed  # m/s
        self.noise_level = noise
        self.loop = loop

        # Predefined Bhopal waypoints
        self.waypoints = [[lat, lon] for lon, lat in [
            [77.371005, 23.192414],
            [77.371092, 23.192337],
            [77.371152, 23.192313],
            [77.372131, 23.192131],
            [77.373012, 23.19201],
            [77.37544, 23.191633],
            [77.376744, 23.191436],
            [77.37818, 23.191206],
            [77.378853, 23.191082],
            [77.380101, 23.191002],
            [77.381232, 23.191047],
            [77.381648, 23.191008],
            [77.381801, 23.190919],
            [77.38193, 23.190264],
            [77.382307, 23.18847],
            [77.382467, 23.188266],
            [77.383105, 23.187718],
            [77.383723, 23.18768],
            [77.384647, 23.187794],
            [77.385333, 23.187869],
            [77.385527, 23.187952],
            [77.385637, 23.187979],
            [77.38613, 23.188247],
            [77.38769, 23.189572],
            [77.388392, 23.190138],
            [77.388689, 23.190458],
            [77.388866, 23.190689],
            [77.388953, 23.190715],
            [77.389107, 23.190718],
            [77.390367, 23.190275],
            [77.391566, 23.189876],
            [77.391971, 23.189792],
            [77.392297, 23.189816],
            [77.392488, 23.189855],
            [77.392632, 23.189874],
            [77.39498, 23.189409],
            [77.395075, 23.189404],
            [77.395135, 23.189411],
            [77.395184, 23.189426],
            [77.395269, 23.189467],
            [77.395352, 23.189546],
            [77.395873, 23.19029],
            [77.39622, 23.190653],
            [77.397321, 23.192089],
            [77.398013, 23.192613],
            [77.398139, 23.192734],
            [77.398661, 23.192962],
            [77.399063, 23.193128],
            [77.399472, 23.193274],
            [77.399648, 23.193311],
            [77.400085, 23.193266],
            [77.400151, 23.193274],
            [77.401004, 23.193681],
            [77.40113, 23.193723],
            [77.401703, 23.193812],
            [77.402159, 23.193848],
            [77.402814, 23.193956],
            [77.403256, 23.194053],
            [77.403585, 23.194092],
            [77.403898, 23.194009],
            [77.405262, 23.193373],
            [77.405309, 23.193345],
            [77.405593, 23.193075],
            [77.405633, 23.193047],
            [77.4057, 23.19302],
            [77.405794, 23.193007],
            [77.406007, 23.193009],
            [77.406113, 23.193022],
            [77.406289, 23.193012],
            [77.406729, 23.19274],
            [77.407024, 23.192632],
            [77.407212, 23.19271],
            [77.407316, 23.192659],
            [77.407391, 23.19243],
            [77.407403, 23.192273],
            [77.407336, 23.192051],
            [77.406879, 23.191392],
            [77.40692, 23.1913],
            [77.407042, 23.191274],
            [77.407174, 23.191298],
            [77.407932, 23.191951],
            [77.408203, 23.192212],
            [77.408263, 23.192379],
            [77.408221, 23.192522],
            [77.408093, 23.192868],
            [77.408006, 23.193023],
            [77.40801, 23.193073],
            [77.40825, 23.193422],
            [77.408968, 23.194357],
            [77.409349, 23.194897],
            [77.409463, 23.195109],
            [77.409504, 23.195458],
            [77.409548, 23.195736],
            [77.409643, 23.195913],
            [77.410319, 23.196974],
            [77.410289, 23.197087],
            [77.409889, 23.197788],
            [77.409781, 23.198008],
            [77.409845, 23.198399],
            [77.410123, 23.199517],
            [77.410038, 23.199834],
            [77.409892, 23.200293],
            [77.409895, 23.200407],
            [77.411824, 23.199866],
            [77.412661, 23.199666],
            [77.413212, 23.199418],
            [77.413633, 23.199116],
            [77.413757, 23.199012],
            [77.413937, 23.199215],
            [77.414498, 23.198809],
            [77.414922, 23.198533],
            [77.414915, 23.19876],
            [77.415315, 23.198755],
            [77.415307, 23.199146],
            [77.415693, 23.199159],
            [77.41674, 23.199213]
        ]]

        # Initialize position and waypoints
        self.current_wp_idx = 0
        self.position = list(self.waypoints[0])
        self.heading = 0.0
        self.waypoint_reached_distance = 5.0  # meters

        # GPS parameters
        self.hdop = 0.9
        self.altitude = 180.0  # Average Bhopal elevation
        self.fix_quality = 1
        self.sats = 12

        # Control
        self.running = False
        self.thread = None

    def _earth_radius(self, lat):
        """Calculate Earth radius at a given latitude"""
        a = 6378137.0  # WGS84 semi-major axis
        b = 6356752.314245  # WGS84 semi-minor axis
        lat_rad = math.radians(lat)
        return math.sqrt(((a ** 2 * math.cos(lat_rad)) ** 2 + (b ** 2 * math.sin(lat_rad)) ** 2) /
                         ((a * math.cos(lat_rad)) ** 2 + (b * math.sin(lat_rad)) ** 2))

    def _calculate_bearing(self, lat1, lon1, lat2, lon2):
        """Calculate initial bearing between two points in degrees"""
        lat1 = math.radians(lat1)
        lon1 = math.radians(lon1)
        lat2 = math.radians(lat2)
        lon2 = math.radians(lon2)

        dlon = lon2 - lon1
        x = math.cos(lat2) * math.sin(dlon)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        bearing = (math.degrees(math.atan2(x, y)) + 360) % 360
        return bearing

    def _calculate_distance(self, lat1, lon1, lat2, lon2):
        """Haversine distance calculation between two points"""
        R = self._earth_radius((lat1 + lat2) / 2)
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(
            dlon / 2) ** 2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    def _calculate_new_position(self, distance, bearing):
        """Calculate new position given distance and bearing"""
        R = self._earth_radius(self.position[0])
        lat1 = math.radians(self.position[0])
        lon1 = math.radians(self.position[1])
        brng = math.radians(bearing)

        lat2 = math.asin(math.sin(lat1) * math.cos(distance / R) +
                         math.cos(lat1) * math.sin(distance / R) * math.cos(brng))
        lon2 = lon1 + math.atan2(math.sin(brng) * math.sin(distance / R) * math.cos(lat1),
                                 math.cos(distance / R) - math.sin(lat1) * math.sin(lat2))
        return [math.degrees(lat2), math.degrees(lon2)]

    def _move_along_path(self, dt):
        """Move along the path towards the current waypoint"""
        # If we've reached the end of waypoints
        if self.current_wp_idx >= len(self.waypoints):
            if self.loop:
                logger.info("Reached end of waypoints, looping back to start")
                self.current_wp_idx = 0
            else:
                logger.info("Reached end of waypoints, stopping")
                return self.position

        # Get current target waypoint
        target_waypoint = self.waypoints[self.current_wp_idx]

        # Calculate distance to current waypoint
        distance_to_target = self._calculate_distance(*self.position, *target_waypoint)

        # If we've reached the current waypoint, move to the next one
        if distance_to_target < self.waypoint_reached_distance:
            self.current_wp_idx += 1
            logger.info(f"Waypoint {self.current_wp_idx} reached")

            # If we've reached the end of waypoints and looping
            if self.current_wp_idx >= len(self.waypoints):
                if self.loop:
                    self.current_wp_idx = 0
                    logger.info("Looping back to first waypoint")
                    target_waypoint = self.waypoints[0]
                else:
                    logger.info("End of waypoints reached")
                    return self.position
            else:
                target_waypoint = self.waypoints[self.current_wp_idx]

        # Calculate bearing to target
        self.heading = self._calculate_bearing(*self.position, *target_waypoint)

        # Calculate movement step based on speed and time delta
        step_distance = self.speed * dt

        # Make sure we don't overshoot the waypoint
        if step_distance > distance_to_target:
            step_distance = distance_to_target

        # Calculate new position
        new_position = self._calculate_new_position(step_distance, self.heading)

        # Add small random noise for realism
        noisy_position = [
            new_position[0] + random.gauss(0, self.noise_level),
            new_position[1] + random.gauss(0, self.noise_level)
        ]

        return noisy_position

    def _create_nmea_sentence(self):
        """Generate GGA and RMC NMEA sentences"""
        timestamp = datetime.now(timezone.utc).strftime("%H%M%S.%f")[:-4]
        date = datetime.now(timezone.utc).strftime("%d%m%y")

        lat = abs(self.position[0])
        lat_deg = int(lat)
        lat_min = (lat - lat_deg) * 60
        lat_dir = 'N' if self.position[0] >= 0 else 'S'

        lon = abs(self.position[1])
        lon_deg = int(lon)
        lon_min = (lon - lon_deg) * 60
        lon_dir = 'E' if self.position[1] >= 0 else 'W'

        gga = (
            f"$GPGGA,{timestamp},{lat_deg:02d}{lat_min:09.6f},{lat_dir},"
            f"{lon_deg:03d}{lon_min:09.6f},{lon_dir},{self.fix_quality},{self.sats:02d},"
            f"{self.hdop:.1f},{self.altitude:.1f},M,,M,,*"
        )

        speed_knots = self.speed * 1.94384
        rmc = (
            f"$GPRMC,{timestamp},A,{lat_deg:02d}{lat_min:09.6f},{lat_dir},"
            f"{lon_deg:03d}{lon_min:09.6f},{lon_dir},{speed_knots:.2f},{self.heading:.2f},"
            f"{date},,,A*"
        )

        for sentence in [gga, rmc]:
            checksum = 0
            for c in sentence[1:-1]:
                checksum ^= ord(c)
            yield f"{sentence}{checksum:02X}\r\n"

    def start(self):
        """Start simulation and virtual port"""
        try:
            self.serial_port = serial.Serial(
                port=self.port_name,
                baudrate=self.baudrate,
                timeout=1
            )
            logger.info(f"Connected to {self.port_name}")

            self.running = True
            self.thread = threading.Thread(target=self._run)
            self.thread.start()

            logger.info(f"Following {len(self.waypoints)} waypoints at {self.speed} m/s")
            logger.info(f"Starting from waypoint 1: {self.waypoints[0][0]:.6f}, {self.waypoints[0][1]:.6f}")
            return True
        except Exception as e:
            logger.error(f"Failed to start: {e}")
            return False

    def _run(self):
        """Main simulation loop"""
        last_update = time.time()
        while self.running:
            now = time.time()
            dt = now - last_update
            last_update = now

            # Move along the path
            self.position = self._move_along_path(dt)

            # Generate and send NMEA sentences
            for sentence in self._create_nmea_sentence():
                self.serial_port.write(sentence.encode('utf-8'))

            # Sleep to maintain reasonable update rate
            sleep_time = max(0, 1.0 - (time.time() - last_update))
            time.sleep(sleep_time)

    def stop(self):
        """Stop simulation"""
        self.running = False
        if self.thread:
            self.thread.join()
        if self.serial_port:
            self.serial_port.close()
        logger.info("Simulation stopped")


def create_virtual_ports():
    """Create virtual serial ports using socat"""
    try:
        ports = ["/tmp/vgps1", "/tmp/vgps2"]
        for p in ports:
            if os.path.exists(p):
                os.remove(p)

        process = subprocess.Popen(
            ["socat", "-d", "-d",
             f"pty,link={ports[0]},raw,echo=0",
             f"pty,link={ports[1]},raw,echo=0"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        time.sleep(1)  # Allow time for port creation
        return ports
    except Exception as e:
        logger.error(f"Failed to create virtual ports: {e}")
        return None


def main():
    parser = argparse.ArgumentParser(description="GPS Waypoint Simulator")
    parser.add_argument("--virtual", action="store_true", help="Use virtual ports")
    parser.add_argument("--port", help="Physical serial port name")
    parser.add_argument("--baud", type=int, default=9600, help="Baud rate")
    parser.add_argument("--speed", type=float, default=15.0, help="Speed in m/s")
    parser.add_argument("--loop", action="store_true", help="Loop waypoints continuously")
    parser.add_argument("--waypoint-distance", type=float, default=5.0,
                        help="Distance in meters to consider a waypoint reached")
    args = parser.parse_args()

    if args.virtual:
        ports = create_virtual_ports()
        if not ports:
            return
        sim_port, recv_port = ports
        logger.info(f"\nVirtual ports created!\n"
                    f"Simulator using: {sim_port}\n"
                    f"Receiver should use: {recv_port}\n")
    else:
        if not args.port:
            logger.error("Physical port required when not using --virtual")
            return
        sim_port = args.port
        recv_port = None

    simulator = GPSCarSimulator(
        serial_port=sim_port,
        baudrate=args.baud,
        speed=args.speed,
        loop=args.loop
    )

    # Set waypoint reached distance from command line if provided
    if args.waypoint_distance:
        simulator.waypoint_reached_distance = args.waypoint_distance

    try:
        if simulator.start():
            logger.info("GPS simulation started. Press Ctrl+C to stop.")
            while True:
                time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Stopping simulation...")
    finally:
        simulator.stop()


if __name__ == "__main__":
    main()