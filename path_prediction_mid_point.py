import math
import json
import time
import logging
import threading
from pymavlink import mavutil
from collections import deque

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('midpoint_mission.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


class GeoPoint:
    """Class to represent a geographical point with latitude, longitude, and optional elevation."""

    def __init__(self, lat, lon, elevation=0, name=""):
        self.lat = lat
        self.lon = lon
        self.elevation = elevation
        self.name = name

    def distance_to(self, other_point):
        """Calculate the distance between this point and another using the Haversine formula."""
        R = 6371000  # Earth radius in meters

        phi1 = math.radians(self.lat)
        phi2 = math.radians(other_point.lat)
        delta_phi = math.radians(other_point.lat - self.lat)
        delta_lambda = math.radians(other_point.lon - self.lon)

        a = (math.sin(delta_phi / 2) * math.sin(delta_phi / 2) +
             math.cos(phi1) * math.cos(phi2) *
             math.sin(delta_lambda / 2) * math.sin(delta_lambda / 2))
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        distance = R * c
        return distance

    def midpoint_to(self, other_point):
        """Find the midpoint between this point and another point."""
        phi1 = math.radians(self.lat)
        lambda1 = math.radians(self.lon)
        phi2 = math.radians(other_point.lat)
        lambda2 = math.radians(other_point.lon)

        Bx = math.cos(phi2) * math.cos(lambda2 - lambda1)
        By = math.cos(phi2) * math.sin(lambda2 - lambda1)

        phi3 = math.atan2(math.sin(phi1) + math.sin(phi2),
                          math.sqrt((math.cos(phi1) + Bx) * (math.cos(phi1) + Bx) + By * By))
        lambda3 = lambda1 + math.atan2(By, math.cos(phi1) + Bx)

        mid_lat = math.degrees(phi3)
        mid_lon = math.degrees(lambda3)

        # For elevation, take the average
        mid_elev = (self.elevation + other_point.elevation) / 2

        name = f"Midpoint {self.name}-{other_point.name}"
        return GeoPoint(mid_lat, mid_lon, mid_elev, name)

    def bearing_to(self, other_point):
        """Calculate bearing from this point to another point"""
        lat1 = math.radians(self.lat)
        lon1 = math.radians(self.lon)
        lat2 = math.radians(other_point.lat)
        lon2 = math.radians(other_point.lon)

        y = math.sin(lon2 - lon1) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(lon2 - lon1)
        bearing = math.atan2(y, x)

        # Convert to degrees
        bearing = math.degrees(bearing)
        # Normalize to 0-360
        bearing = (bearing + 360) % 360

        return bearing

    def __str__(self):
        return f"{self.name} ({self.lat:.6f}, {self.lon:.6f}, {self.elevation:.1f}m)"

    def to_dict(self):
        """Convert the point to a dictionary for JSON serialization."""
        return {
            "name": self.name,
            "lat": self.lat,
            "lon": self.lon,
            "elevation": self.elevation
        }


class DronePathPlanner:
    """Class to plan drone paths between waypoints and calculate midpoints."""

    def __init__(self, max_points=10):
        self.waypoints = []
        self.paths = []
        self.midpoints = []
        self.max_points = max_points
        self.current_target_index = -1
        self.mission_complete = False
        self.distance_threshold = 5.0  # Meters
        self.vehicle = None

    def add_waypoint(self, lat, lon, elevation=0, name=None):
        """Add a waypoint to the planner."""
        if len(self.waypoints) >= self.max_points:
            raise ValueError(f"Maximum number of waypoints ({self.max_points}) reached")

        if name is None:
            name = f"Point {len(self.waypoints) + 1}"

        point = GeoPoint(lat, lon, elevation, name)
        self.waypoints.append(point)

        # Reset calculated paths and midpoints when adding a new waypoint
        self.paths = []
        self.midpoints = []

        return point

    def calculate_paths(self):
        """Calculate shortest paths between consecutive waypoints."""
        self.paths = []

        if len(self.waypoints) < 2:
            return self.paths

        for i in range(len(self.waypoints) - 1):
            start = self.waypoints[i]
            end = self.waypoints[i + 1]

            # Calculate straight-line distance
            distance = start.distance_to(end)
            midpoint = start.midpoint_to(end)

            self.paths.append({
                "start": start,
                "end": end,
                "distance": distance,
                "midpoint": midpoint
            })

        return self.paths

    def calculate_midpoints(self):
        """Calculate midpoints for all paths."""
        if not self.paths:
            self.calculate_paths()

        self.midpoints = [path["midpoint"] for path in self.paths]
        return self.midpoints

    def connect_vehicle(self, connection_string='udp:127.0.0.1:14551'):
        """Connect to the vehicle using MAVLink."""
        logger.info(f"Connecting to vehicle on {connection_string}")
        try:
            self.vehicle = mavutil.mavlink_connection(connection_string)
            self.vehicle.wait_heartbeat()
            logger.info("Connected to vehicle")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to vehicle: {e}")
            return False

    def print_mission_summary(self):
        """Print a summary of the mission."""
        total_distance = sum(path["distance"] for path in self.paths)

        summary = f"Mission with {len(self.waypoints)} waypoints and {len(self.midpoints)} midpoints\n"
        summary += f"Total path distance: {total_distance / 1000:.2f} km\n\n"

        summary += "Waypoints:\n"
        for wp in self.waypoints:
            summary += f"- {wp}\n"

        summary += "\nPath Segments:\n"
        for i, path in enumerate(self.paths):
            summary += f"- Path {i + 1}: {path['start'].name} to {path['end'].name} ({path['distance'] / 1000:.2f} km)\n"
            summary += f"  Midpoint: {path['midpoint']}\n"

        return summary

    def save_mission_to_json(self, filename):
        """Save the mission plan to a JSON file."""
        # Convert to serializable format
        serializable_waypoints = [wp.to_dict() for wp in self.waypoints]
        serializable_paths = [{
            "start": path["start"].to_dict(),
            "end": path["end"].to_dict(),
            "distance": path["distance"],
            "midpoint": path["midpoint"].to_dict()
        } for path in self.paths]

        mission_data = {
            "waypoints": serializable_waypoints,
            "paths": serializable_paths,
            "midpoints": [mp.to_dict() for mp in self.midpoints],
        }

        with open(filename, 'w') as f:
            json.dump(mission_data, f, indent=2)

        logger.info(f"Mission saved to {filename}")

    def load_mission_from_json(self, filename):
        """Load mission data from a JSON file."""
        with open(filename, 'r') as f:
            mission_data = json.load(f)

        # Clear existing data
        self.waypoints = []
        self.paths = []
        self.midpoints = []

        # Load waypoints
        for wp_data in mission_data["waypoints"]:
            self.add_waypoint(
                wp_data["lat"],
                wp_data["lon"],
                wp_data["elevation"],
                wp_data["name"]
            )

        # Recalculate paths and midpoints
        self.calculate_paths()
        self.calculate_midpoints()

        logger.info(f"Mission loaded from {filename}")

    def get_drone_position(self):
        """Get current drone position from MAVLink."""
        if not self.vehicle:
            logger.error("No vehicle connection")
            return None

        msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            # Convert from 1e7 to decimal degrees
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000.0  # mm to meters
            return GeoPoint(lat, lon, alt, "Drone")
        return None

    def set_guided_mode(self):
        """Set vehicle mode to GUIDED."""
        if not self.vehicle:
            logger.error("No vehicle connection")
            return False

        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4,  # GUIDED mode
            0, 0, 0, 0, 0
        )

        # Wait for mode change
        for _ in range(30):
            msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg and msg.custom_mode == 4:  # GUIDED mode
                logger.info("Vehicle mode set to GUIDED")
                return True
            time.sleep(0.1)

        logger.error("Failed to set GUIDED mode")
        return False

    def arm_vehicle(self):
        """Arm the vehicle."""
        if not self.vehicle:
            logger.error("No vehicle connection")
            return False

        # Check if already armed
        msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            logger.info("Vehicle already armed")
            return True

        # Arm the vehicle
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )

        # Wait for arming
        for _ in range(30):
            msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                logger.info("Vehicle armed")
                return True
            time.sleep(0.1)

        logger.error("Failed to arm vehicle")
        return False

    def takeoff(self, target_altitude):
        """Takeoff to the specified altitude."""
        if not self.vehicle:
            logger.error("No vehicle connection")
            return False

        # Check if already flying
        current_alt = self.get_drone_position().elevation if self.get_drone_position() else 0
        if current_alt > 1.0:
            logger.info(f"Already flying at {current_alt:.1f}m")
            return True

        # Command takeoff
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0, target_altitude
        )

        # Wait for takeoff
        start_time = time.time()
        while time.time() - start_time < 30:
            position = self.get_drone_position()
            if position and position.elevation >= target_altitude - 1.0:
                logger.info(f"Takeoff complete - altitude: {position.elevation:.1f}m")
                return True
            time.sleep(0.5)

        logger.error("Takeoff timed out")
        return False

    def goto_position(self, target_point, target_velocity=10.0):
        """Command the drone to go to a specific position with velocity control."""
        if not self.vehicle:
            logger.error("No vehicle connection")
            return

        # Calculate position
        current_pos = self.get_drone_position()
        if not current_pos:
            logger.error("Failed to get current position")
            return

        # Calculate distance and bearing to target
        distance = current_pos.distance_to(target_point)
        self.mission_complete = False
        self.current_target_index = 0
        current_target = self.midpoints[self.current_target_index]

        logger.info(f"Starting midpoint mission with {len(self.midpoints)} targets")
        logger.info(f"First target: {current_target}")

        # Position tracking
        last_command_time = 0
        command_frequency = 0.2  # 5 Hz

        while not self.mission_complete:
            current_time = time.time()

            # Update commands at specified frequency
            if current_time - last_command_time >= command_frequency:
                # Get current position
                current_pos = self.get_drone_position()
                if not current_pos:
                    logger.warning("Could not get current position")
                    time.sleep(0.5)
                    continue

                # Get current target
                current_target = self.midpoints[self.current_target_index]

                # Calculate distance to current target
                distance = current_pos.distance_to(current_target)

                # Log progress
                logger.info(
                    f"Target {self.current_target_index + 1}/{len(self.midpoints)}: "
                    f"Distance: {distance:.2f}m to {current_target.name}"
                )

                # Check if we've reached the target
                if distance <= self.distance_threshold:
                    logger.info(f"Reached midpoint {self.current_target_index + 1}: {current_target}")

                    # Move to next waypoint
                    self.current_target_index += 1

                    # Check if mission complete
                    if self.current_target_index >= len(self.midpoints):
                        logger.info("Mission complete! All midpoints visited.")
                        self.mission_complete = True
                        break

                    # Get next target
                    current_target = self.midpoints[self.current_target_index]
                    logger.info(f"Moving to next target: {current_target}")

                # Send command to go to current target
                self.goto_position(current_target, target_velocity)

                last_command_time = current_time

            # Small delay to prevent CPU overuse
            time.sleep(0.05)

        return True

    def execute_midpoint_mission(self, target_altitude=None, target_velocity=10.0):
        """Execute a mission following midpoints."""
        if not self.midpoints:
            logger.error("No midpoints available for mission")
            return False

        self.current_target_index = 0
        self.mission_complete = False

        # Set altitude if provided
        if target_altitude is not None:
            for midpoint in self.midpoints:
                midpoint.elevation = target_altitude

        current_target = self.midpoints[0]
        logger.info(f"Starting midpoint mission with {len(self.midpoints)} targets")
        logger.info(f"First target: {current_target}")

        # Position tracking
        last_command_time = 0
        command_frequency = 0.5  # 2 Hz

        while not self.mission_complete:
            current_time = time.time()

            # Update commands at specified frequency
            if current_time - last_command_time >= command_frequency:
                # Get current position
                current_pos = self.get_drone_position()
                if not current_pos:
                    logger.warning("Could not get current position")
                    time.sleep(0.5)
                    continue

                # Get current target
                current_target = self.midpoints[self.current_target_index]

                # Calculate distance to current target
                distance = current_pos.distance_to(current_target)

                # Log progress
                logger.info(
                    f"Target {self.current_target_index + 1}/{len(self.midpoints)}: "
                    f"Distance: {distance:.2f}m to {current_target.name}"
                )

                # Check if we've reached the target
                if distance <= self.distance_threshold:
                    logger.info(f"Reached midpoint {self.current_target_index + 1}: {current_target}")

                    # Move to next waypoint
                    self.current_target_index += 1

                    # Check if mission complete
                    if self.current_target_index >= len(self.midpoints):
                        logger.info("Mission complete! All midpoints visited.")

                        # Proceed to final waypoint
                        if self.waypoints:
                            final_waypoint = self.waypoints[-1]
                            logger.info(f"Moving to final waypoint: {final_waypoint}")
                            self.send_goto_command(final_waypoint, target_velocity)

                        self.mission_complete = True
                        break

                    # Get next target
                    current_target = self.midpoints[self.current_target_index]
                    logger.info(f"Moving to next target: {current_target}")

                # Send command to go to current target
                self.send_goto_command(current_target, target_velocity)

                last_command_time = current_time

            # Small delay to prevent CPU overuse
            time.sleep(0.05)

        return self.mission_complete

    def send_goto_command(self, target_point, target_velocity):
        """
        Send a goto command to the vehicle using SET_POSITION_TARGET_GLOBAL_INT
        which is more reliable than mission_item commands.
        """
        if not self.vehicle:
            logger.error("No vehicle connection")
            return

        try:
            # Convert lat/lon to int * 1e7 format
            lat_int = int(target_point.lat * 1e7)
            lon_int = int(target_point.lon * 1e7)
            alt = float(target_point.elevation)

            # Create SET_POSITION_TARGET_GLOBAL_INT message
            # Mask: specify which fields to ignore (set all to ignore except lat/lon/alt)
            mask = (
                    0b0000111111111000 |  # ignore velocity
                    0b0001000000000000 |  # ignore acceleration
                    0b0010000000000000  # ignore yaw
            )

            # Send position target message
            self.vehicle.mav.set_position_target_global_int_send(
                0,  # time_boot_ms
                self.vehicle.target_system,
                self.vehicle.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mask,  # type_mask
                lat_int,  # lat_int
                lon_int,  # lon_int
                alt,  # alt
                0, 0, 0,  # vx, vy, vz
                0, 0, 0,  # afx, afy, afz
                0, 0  # yaw, yaw_rate
            )

            # Also set speed
            self.vehicle.mav.command_long_send(
                self.vehicle.target_system,
                self.vehicle.target_component,
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                0,
                1,  # Speed type (1=Ground speed)
                target_velocity,  # Speed in m/s
                -1,  # Throttle - no change
                0, 0, 0, 0  # Unused parameters
            )

            logger.info(f"Sent position target: lat={target_point.lat}, lon={target_point.lon}, alt={alt}")

        except Exception as e:
            logger.error(f"Error sending goto command: {str(e)}")

    def cleanup(self):
        """Clean up resources and stop mission execution"""
        logger.warning("DronePathPlanner cleanup called - forcing mission to stop")

        # Set flag to ensure the mission stops
        self.mission_complete = True

        # Additionally, signal to external systems (mission_mgr) that we want to stop
        # This line forces the run_mission_thread to terminate
        if 'mission_mgr' in globals():
            globals()['mission_mgr'].is_running = False
            logger.warning("Set mission_mgr.is_running to False")

        # Send a position hold command at the current position (not using LOITER)
        if self.vehicle:
            try:
                # Get current position for position hold
                msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
                if msg:
                    current_lat = msg.lat  # Keep in int format (1e7)
                    current_lon = msg.lon
                    current_alt = msg.relative_alt / 1000.0  # mm to meters

                    # Log position
                    logger.warning(
                        f"Vehicle position when stopped: lat={current_lat / 1e7:.6f}, lon={current_lon / 1e7:.6f}, alt={current_alt:.1f}m")

                    mask = (0b0000111111111000 |  # ignore velocity
                            0b0001000000000000 |  # ignore acceleration
                            0b0010000000000000)  # ignore yaw

                    # Send position target command to hold at current position
                    self.vehicle.mav.set_position_target_global_int_send(
                        0,  # time_boot_ms
                        self.vehicle.target_system,  # target_system
                        self.vehicle.target_component,  # target_component
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
                        mask,  # type_mask
                        current_lat,  # lat_int (already in 1e7 format)
                        current_lon,  # lon_int (already in 1e7 format)
                        current_alt,  # alt in meters
                        0, 0, 0,  # vx, vy, vz
                        0, 0, 0,  # afx, afy, afz
                        0, 0  # yaw, yaw_rate
                    )
                    logger.warning("Sent position hold command at current position")
                else:
                    logger.warning("Could not get current position for position hold command")

            except Exception as e:
                logger.error(f"Error sending position hold command: {str(e)}")

        logger.warning("DronePathPlanner cleanup completed")
        return True


def main():
    # Create the path planner
    planner = DronePathPlanner(max_points=1000)

    try:
        # Add waypoints
        planner.add_waypoint(23.190611, 77.367541, 10, "Point A")
        planner.add_waypoint(23.191218, 77.369601, 10, "Point B")
        planner.add_waypoint(23.192184, 77.372053, 10, "Point C")
        planner.add_waypoint(23.193338, 77.370819, 10, "Point D")
        # Calculate midpoints
        planner.calculate_paths()
        planner.calculate_midpoints()

        # Print the mission summary
        print(planner.print_mission_summary())

        # Save the mission to a file
        planner.save_mission_to_json("midpoint_mission.json")

        # Connect to the vehicle
        if planner.connect_vehicle():
            # Setup flight
            if planner.set_guided_mode() and planner.arm_vehicle():
                # Takeoff
                if planner.takeoff(10.0):
                    # Execute the mission
                    planner.execute_midpoint_mission(target_altitude=10.0, target_velocity=10.0)
                else:
                    logger.error("Takeoff failed")
            else:
                logger.error("Failed to set up vehicle")
        else:
            logger.error("Failed to connect to vehicle")

    except KeyboardInterrupt:
        logger.info("Mission interrupted by user")
    except Exception as e:
        logger.error(f"Error in mission: {e}")
    finally:
        # Clean up
        planner.cleanup()
        logger.info("Mission ended")


if __name__ == "__main__":
    main()