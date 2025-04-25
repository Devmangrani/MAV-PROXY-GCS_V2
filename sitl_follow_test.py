import time
import logging
from pymavlink import mavutil
import threading
from queue import Queue, Empty
from sitl_gps_simulator import SITLGPSSimulator

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('sitl_test.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# Global variables
gps_data_queue = Queue()
follow_enabled = False
target_altitude = 10  # 10 meters default altitude
vehicle = None


def connect_vehicle():
    """Connect to SITL vehicle"""
    global vehicle
    connection_string = 'udp:127.0.0.1:14551'
    logger.info(f"Connecting to SITL on {connection_string}")

    try:
        vehicle = mavutil.mavlink_connection(connection_string)
        vehicle.wait_heartbeat()
        logger.info("Connected to vehicle")
        return True
    except Exception as e:
        logger.error(f"Failed to connect to vehicle: {e}")
        return False


def set_guided_mode():
    """Set vehicle mode to GUIDED"""
    if vehicle:
        # Request GUIDED mode
        vehicle.mav.command_long_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4,  # GUIDED mode
            0, 0, 0, 0, 0
        )

        # Wait for mode change
        for _ in range(30):  # 3-second timeout
            msg = vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=0.1)
            if msg and msg.custom_mode == 4:  # GUIDED mode
                logger.info("Vehicle mode set to GUIDED")
                return True

        logger.error("Failed to set GUIDED mode")
        return False
    return False


def arm_vehicle():
    """Arm the vehicle"""
    if vehicle:
        # Send arm command
        vehicle.mav.command_long_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )

        # Wait for arming
        for _ in range(30):  # 3-second timeout
            msg = vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=0.1)
            if msg and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                logger.info("Vehicle armed")
                return True

        logger.error("Failed to arm vehicle")
        return False
    return False


def takeoff(altitude):
    """Takeoff to specified altitude"""
    if vehicle:
        # Send takeoff command
        vehicle.mav.command_long_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0, altitude
        )

        # Wait for altitude
        start_time = time.time()
        while time.time() - start_time < 30:  # 30-second timeout
            msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=0.1)
            if msg:
                current_alt = msg.relative_alt / 1000.0  # Convert to meters
                logger.info(f"Current altitude: {current_alt:.1f}m")
                if abs(current_alt - altitude) < 1.0:  # Within 1 meter
                    logger.info(f"Reached target altitude: {altitude}m")
                    return True
            time.sleep(0.1)

        logger.error("Takeoff timeout")
        return False
    return False


def follow_gps():
    """GPS follow function"""
    global follow_enabled, vehicle

    logger.info("Starting GPS follow")
    follow_enabled = True

    while follow_enabled:
        try:
            if not gps_data_queue.empty():
                gps_data = gps_data_queue.get()

                # Send position target command
                vehicle.mav.set_position_target_global_int_send(
                    0,  # timestamp
                    vehicle.target_system,
                    vehicle.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    0b110111111000,  # Only positions enabled
                    int(gps_data['lat'] * 1e7),  # lat
                    int(gps_data['lon'] * 1e7),  # lon
                    target_altitude,  # alt
                    0, 0, 0,  # velocity
                    0, 0, 0,  # acceleration
                    0, 0  # yaw, yaw_rate
                )

                # Log current position and target
                msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
                if msg:
                    current_lat = msg.lat / 1e7
                    current_lon = msg.lon / 1e7
                    current_alt = msg.relative_alt / 1000
                    logger.info(f"Current: Lat={current_lat:.6f}, Lon={current_lon:.6f}, Alt={current_alt:.1f}m")
                    logger.info(
                        f"Target:  Lat={gps_data['lat']:.6f}, Lon={gps_data['lon']:.6f}, Alt={target_altitude:.1f}m")

        except Exception as e:
            logger.error(f"Error in follow thread: {e}")

        time.sleep(0.1)


def start_follow_thread():
    """Start GPS following in a separate thread"""
    follow_thread = threading.Thread(target=follow_gps)
    follow_thread.daemon = True
    follow_thread.start()
    return follow_thread


def main():
    """Main test function"""
    global follow_enabled, target_altitude

    try:
        # Connect to vehicle
        if not connect_vehicle():
            return

        # Create GPS simulator
        gps_sim = SITLGPSSimulator()

        # Set speed to 10 m/s
        logger.info("Setting GPS simulation speed to 10 m/s")
        gps_sim.set_speed(10.0)  # 10 m/s

        # Start square pattern directly
        logger.info("Starting GPS simulation with square pattern")
        gps_sim.start_movement('square')

        # Set up vehicle
        if not set_guided_mode():
            return

        if not arm_vehicle():
            return

        # Takeoff
        target_altitude = 10  # meters
        if not takeoff(target_altitude):
            return

        # Start GPS reader thread to transfer positions from simulator to follow queue
        def gps_reader():
            while True:
                position = gps_sim.get_position()
                if position:
                    gps_data_queue.put(position)
                time.sleep(0.1)

        gps_thread = threading.Thread(target=gps_reader, daemon=True)
        gps_thread.start()

        # Start following
        follow_thread = start_follow_thread()

        # Run for a set duration
        pattern_time = 1200  # seconds
        logger.info(f"Following square pattern at 10 m/s for {pattern_time} seconds...")
        try:
            time.sleep(pattern_time)
        except KeyboardInterrupt:
            logger.info("Test interrupted by user")

        # Clean up
        follow_enabled = False
        if follow_thread.is_alive():
            follow_thread.join(timeout=2)

        gps_sim.cleanup()
        logger.info("Test completed")

    except Exception as e:
        logger.error(f"Test failed: {e}")
    finally:
        # Land the vehicle
        if vehicle:
            vehicle.mav.command_long_send(
                vehicle.target_system,
                vehicle.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            logger.info("Landing command sent")


if __name__ == "__main__":
    main()
