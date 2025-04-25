"""
@file: simple_gimbal_rotation.py
@Description: Simple script to control SIYI gimbal rotation using the SDK
@Author: Assistant
Based on work by Mohamed Abdelkader
"""

import time
from siyi_sdk import SIYISDK


def test_gimbal():
    # Initialize camera with default SIYI camera IP and port
    camera = SIYISDK(
        server_ip="192.168.144.25",  # Default SIYI IP
        port=37260  # Default UDP port
    )

    try:
        # Attempt to connect
        print("Connecting to camera...")
        if not camera.connect():
            print("Failed to connect to camera")
            return

        print("Connected successfully!")

        # Get initial attitude
        initial_yaw, initial_pitch, initial_roll = camera.getAttitude()
        print(f"Initial attitude - Yaw: {initial_yaw:.1f}, Pitch: {initial_pitch:.1f}, Roll: {initial_roll:.1f}")

        # Move gimbal
        print("\nMoving gimbal...")
        camera.setGimbalRotation(yaw=-45, pitch=5)  # Example angles

        # Small delay to allow movement
        time.sleep(2)

        # Get final attitude
        final_yaw, final_pitch, final_roll = camera.getAttitude()
        print(f"Final attitude - Yaw: {final_yaw:.1f}, Pitch: {final_pitch:.1f}, Roll: {final_roll:.1f}")

    except Exception as e:
        print(f"Error occurred: {e}")

    finally:
        # Always disconnect properly
        print("\nDisconnecting from camera...")
        camera.disconnect()
        print("Test complete")


if __name__ == "__main__":
    test_gimbal()