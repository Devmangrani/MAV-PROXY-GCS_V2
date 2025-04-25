from pymavlink import mavutil
import time


def connect_vehicle():
    connection_string = '/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_1D0048000D51393239383638-if00'
    print(f"Attempting to connect to {connection_string}")
    vehicle = mavutil.mavlink_connection(connection_string, autoreconnect=True, timeout=60)
    print("Waiting for heartbeat")
    vehicle.wait_heartbeat(timeout=30)
    print("Connected successfully")
    return vehicle


try:
    vehicle = connect_vehicle()

    # Basic communication test
    for i in range(10):
        msg = vehicle.recv_match(blocking=True, timeout=5)
        if msg:
            print(f"Received message: {msg.get_type()}")
        else:
            print("No message received")
        time.sleep(1)

except Exception as e:
    print(f"Error: {str(e)}")
finally:
    if 'vehicle' in locals() and vehicle:
        vehicle.close()
    print("Script finished")