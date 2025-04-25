"""
Simple adapter to integrate map coordinates with path_prediction_mid_point_v2.py

This adapter focuses only on parsing map coordinates and passing them to the path_prediction_mid_point_v2 module
without changing any other functionality.
"""

import logging
from path_prediction_mid_point_v2 import DronePathPlanner

# Configure logging
logger = logging.getLogger(__name__)


def parse_coordinates_to_v2(waypoints_data):
    """
    Parse coordinates from the map and prepare them for path_prediction_mid_point_v2.py

    Args:
        waypoints_data: List of waypoint dictionaries with lat, lon, alt

    Returns:
        DronePathPlanner instance with waypoints loaded
    """
    try:
        # Create a DronePathPlanner instance
        planner = DronePathPlanner(max_points=len(waypoints_data) + 5)

        # Add each waypoint
        for i, wp in enumerate(waypoints_data):
            lat = float(wp.get('lat'))
            lon = float(wp.get('lon'))
            alt = float(wp.get('alt', 10.0))  # Default to 10m if not specified

            # Add to planner
            planner.add_waypoint(
                lat=lat,
                lon=lon,
                elevation=alt,
                name=f"Point {i + 1}"
            )
            logger.info(f"Added waypoint {i + 1}: ({lat}, {lon}, {alt}m)")

        # Calculate paths and midpoints
        planner.calculate_paths()
        planner.calculate_midpoints()

        logger.info(f"Processed {len(planner.waypoints)} waypoints with {len(planner.midpoints)} midpoints")

        return planner
    except Exception as e:
        logger.error(f"Error parsing coordinates: {str(e)}")
        raise