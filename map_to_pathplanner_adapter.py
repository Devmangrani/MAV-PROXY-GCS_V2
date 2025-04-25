"""
Map to PathPlannerV2 Adapter with Car GPS Tracking

A complete adapter to bridge between your existing route handlers and the path_prediction_mid_point_v2 module.
This adapter enables car GPS tracking functionality using the DronePathPlanner class.
"""

import time
import logging
import threading
import os
from path_prediction_mid_point_v2 import DronePathPlanner, GeoPoint

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class MapToPathPlannerAdapter:
    """
    Adapter class to handle the parsing of map coordinates from your existing API
    to the DronePathPlanner class in path_prediction_mid_point_v2.py with car tracking
    """

    def __init__(self):
        self.planner = None
        self.mission_running = False
        self.mission_thread = None
        self.car_gps_port = None  # This will be set based on configuration

    def get_planner(self, car_gps_port=None):
        """
        Get or initialize the planner instance with car GPS support

        Args:
            car_gps_port: Serial port for car GPS (e.g., '/dev/ttyACM0')
        """
        if car_gps_port:
            self.car_gps_port = car_gps_port

        if self.planner is None:
            # Initialize with car GPS if port is provided
            self.planner = DronePathPlanner(
                max_points=100,
                car_gps_port=self.car_gps_port
            )

        return self.planner

    def discover_car_gps_port(self):
        """
        Try to discover an available car GPS port
        Returns the first available tty port or a default
        """
        # List of common serial port patterns to check
        potential_ports = [
            '/dev/ttyUSB0'
        ]

        # On Windows, check COM ports
        if os.name == 'nt':
            potential_ports = ['COM%s' % (i + 1) for i in range(12)]  # COM1-COM12

        # For simulation environments, use a designated testing port
        if os.environ.get('SIMULATION_ENV') == 'true':
            logger.info("Simulation environment detected, using virtual GPS port")
            return '/temp/vgps2'

        # Try to find an existing port
        for port in potential_ports:
            if os.path.exists(port):
                logger.info(f"Discovered potential GPS port: {port}")
                return port

        # Default fallback
        logger.warning("No GPS port found, using default: /temp/vgps2")
        return '/temp/vgps2'

    def process_waypoints(self, waypoints_data):
        """
        Process waypoints received from the map API to format compatible with DronePathPlanner

        Args:
            waypoints_data: List of waypoint dictionaries from API request

        Returns:
            DronePathPlanner instance with waypoints loaded
        """
        # Get car GPS port (discover if not set)
        if not self.car_gps_port:
            self.car_gps_port = self.discover_car_gps_port()

        # Get or initialize planner with car GPS
        planner = self.get_planner(self.car_gps_port)

        # Clear existing waypoints
        planner.waypoints = []

        # Add each waypoint
        for i, wp in enumerate(waypoints_data):
            try:
                lat = float(wp.get('lat'))
                lon = float(wp.get('lon'))
                alt = float(wp.get('alt', 10.0))  # Default altitude if not specified

                # Add to planner
                planner.add_waypoint(
                    lat=lat,
                    lon=lon,
                    elevation=alt,
                    name=f"Point {i+1}"
                )
                logger.info(f"Added waypoint {i+1}: ({lat}, {lon}, {alt}m)")
            except Exception as e:
                logger.error(f"Error processing waypoint {i+1}: {str(e)}")
                raise

        # Calculate paths and midpoints
        planner.calculate_paths()
        planner.calculate_midpoints()

        logger.info(f"Processed {len(planner.waypoints)} waypoints with {len(planner.midpoints)} midpoints")
        return planner

    def run_convoy_mission(self, waypoints_data, altitude=10.0, velocity=10.0, distance_threshold=10.0):
        """
        Run convoy mission using waypoints from map with car GPS tracking

        Args:
            waypoints_data: List of waypoint dictionaries
            altitude: Target flight altitude in meters
            velocity: Target flight velocity in m/s
            distance_threshold: Distance threshold for car detection in meters

        Returns:
            Dictionary with mission status
        """
        try:
            # Stop any existing mission
            if self.mission_running:
                self.stop_mission()

            # Process waypoints
            planner = self.process_waypoints(waypoints_data)

            # Set distance threshold for car detection
            planner.distance_threshold = distance_threshold

            # Connect to vehicle
            if not planner.connect_vehicle():
                return {
                    'success': False,
                    'message': 'Failed to connect to vehicle'
                }

            # Connect to car GPS
            if not planner.connect_car_gps():
                return {
                    'success': False,
                    'message': 'Failed to connect to car GPS'
                }

            # Mark mission as running and store planner instance
            self.planner = planner
            self.mission_running = True

            # Start mission in background thread
            self.mission_thread = threading.Thread(
                target=self._run_car_tracking_mission_thread,
                args=(altitude, velocity)
            )
            self.mission_thread.daemon = True
            self.mission_thread.start()

            return {
                'success': True,
                'message': f'Car tracking mission started with {len(planner.waypoints)} waypoints',
                'waypoints': len(planner.waypoints),
                'midpoints': len(planner.midpoints),
                'car_gps_port': self.car_gps_port
            }

        except Exception as e:
            logger.error(f"Error starting mission: {str(e)}")
            return {
                'success': False,
                'message': f'Error: {str(e)}'
            }

    def _run_car_tracking_mission_thread(self, altitude, velocity):
        """Execute car tracking mission in background thread"""
        try:
            planner = self.planner

            # Setup flight mode
            if not planner.set_guided_mode():
                logger.error("Failed to set GUIDED mode")
                self.mission_running = False
                return

            # Arm the vehicle
            if not planner.arm_vehicle():
                logger.error("Failed to arm vehicle")
                self.mission_running = False
                return

            # Check if takeoff needed
            current_pos = planner.get_drone_position()
            current_alt = current_pos.elevation if current_pos else 0

            if current_alt < 1.0:  # Less than 1 meter - needs takeoff
                logger.info(f"Taking off to {altitude}m")
                if not planner.takeoff(altitude):
                    logger.error("Takeoff failed")
                    self.mission_running = False
                    return
            else:
                logger.info(f"Already at altitude {current_alt}m, skipping takeoff")

            # Execute the car tracking mission
            logger.info("Starting car tracking mission")
            result = planner.execute_car_tracking_mission(
                target_altitude=altitude,
                target_velocity=velocity
            )

            if result:
                logger.info("Car tracking mission completed successfully")
            else:
                logger.warning("Car tracking mission did not complete successfully")

            self.mission_running = False

        except Exception as e:
            logger.error(f"Error in car tracking mission execution: {str(e)}")
            self.mission_running = False
        finally:
            # Ensure mission gets marked as complete even if there's an error
            self.mission_running = False

    def stop_mission(self):
        """Stop the current mission"""
        if not self.mission_running:
            return {
                'success': True,
                'message': 'No mission running'
            }

        # Set flag to stop mission
        self.mission_running = False

        # Clean up planner
        if self.planner:
            try:
                self.planner.cleanup()
            except Exception as e:
                logger.error(f"Error cleaning up planner: {str(e)}")

        # Wait for thread to finish
        if self.mission_thread and self.mission_thread.is_alive():
            self.mission_thread.join(timeout=2)

        return {
            'success': True,
            'message': 'Mission stopped successfully'
        }

    def get_status(self):
        """Get current mission status"""
        if not self.planner:
            return {
                'active': False,
                'message': 'No mission initialized'
            }

        if not self.mission_running:
            return {
                'active': False,
                'message': 'No mission running'
            }

        # Get car position if available
        car_position = self.planner.get_car_position()
        car_pos_data = None
        if car_position:
            car_pos_data = {
                'lat': car_position.lat,
                'lon': car_position.lon,
                'alt': car_position.elevation
            }

        # Get drone position if available
        drone_position = self.planner.get_drone_position()
        drone_pos_data = None
        if drone_position:
            drone_pos_data = {
                'lat': drone_position.lat,
                'lon': drone_position.lon,
                'alt': drone_position.elevation
            }

        # Get current progress
        current_index = self.planner.current_target_index
        total_points = len(self.planner.midpoints) if self.planner.midpoints else len(self.planner.waypoints)

        # Check if drone is at midpoint
        at_midpoint = getattr(self.planner, 'at_midpoint', False)

        # Get min distance recorded if available
        min_distance = getattr(self.planner, 'min_distance_recorded', float('inf'))
        if min_distance == float('inf'):
            min_distance = None

        return {
            'active': True,
            'current_index': current_index,
            'total_points': total_points,
            'progress': f"{current_index + 1}/{total_points}" if total_points > 0 and current_index >= 0 else "N/A",
            'car_position': car_pos_data,
            'drone_position': drone_pos_data,
            'at_midpoint': at_midpoint,
            'min_distance': min_distance,
            'car_gps_port': self.car_gps_port
        }

    def set_car_gps_port(self, port):
        """Manually set the car GPS port"""
        self.car_gps_port = port
        logger.info(f"Car GPS port manually set to: {port}")
        return {
            'success': True,
            'message': f'Car GPS port set to {port}'
        }

# Create a singleton instance
adapter = MapToPathPlannerAdapter()