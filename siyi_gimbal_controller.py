import socket
import struct
import time
import threading
from typing import Tuple, Optional


class SIYIGimbalController:

    # SIYI Protocol Constants
    HEADER = 0x55
    VERSION = 0x04
    ENCRYPT = 0x00

    # Command IDs
    CMD_GIMBAL_ROT_CTRL = 0x0A
    CMD_CAMERA_PHOTO_CTRL = 0x02
    CMD_CAMERA_RECORD_CTRL = 0x03
    CMD_CAMERA_FOCUS_CTRL = 0x04
    CMD_CENTER_GIMBAL = 0x07

    def __init__(self,
                 ip: str = "192.168.144.25",
                 port: int = 37260,
                 debug: bool = False):
        """
        Initialize SIYI Gimbal Controller

        Args:
            ip: Camera IP address (default for SIYI cameras)
            port: UDP port (default for SIYI protocol)
            debug: Enable debug printing
        """
        self.ip = ip
        self.port = port
        self.debug = debug

        # Create UDP socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.settimeout(1.0)  # 1 second timeout

        # Initialize sequence number
        self.seq_num = 0

        # Threading lock for packet sending
        self.lock = threading.Lock()

        if self.debug:
            print(f"Initialized SIYI controller at {ip}:{port}")

    def _calculate_checksum(self, data: bytes) -> int:
        """Calculate SIYI protocol checksum"""
        return sum(data) & 0xFF

    def _create_packet(self, cmd_id: int, data: bytes = b'') -> bytes:
        """
        Create a SIYI protocol packet

        Format:
        | Header | Version | Length | Encrypt | Seq | CMD ID | Data | Checksum |
        |   1B   |   1B    |   2B   |   1B    |  2B |   1B   |  nB  |    1B    |
        """
        length = len(data) + 8  # Header size (7) + Checksum (1)

        # Create packet without checksum
        packet = struct.pack('<BBBHH',
                             self.HEADER,  # Header
                             self.VERSION,  # Version
                             self.ENCRYPT,  # Encryption
                             length,  # Length
                             self.seq_num  # Sequence number
                             )

        # Add command ID and data
        packet += bytes([cmd_id]) + data

        # Calculate and add checksum
        checksum = self._calculate_checksum(packet)
        packet += bytes([checksum])

        # Increment sequence number
        self.seq_num = (self.seq_num + 1) & 0xFFFF

        return packet

    def _send_packet(self, packet: bytes) -> Optional[bytes]:
        """Send packet and receive response"""
        with self.lock:
            try:
                self.socket.sendto(packet, (self.ip, self.port))
                if self.debug:
                    print(f"Sent packet: {packet.hex()}")

                # Wait for response
                try:
                    response, _ = self.socket.recvfrom(1024)
                    if self.debug:
                        print(f"Received response: {response.hex()}")
                    return response
                except socket.timeout:
                    if self.debug:
                        print("No response received")
                    return None

            except Exception as e:
                print(f"Error sending packet: {e}")
                return None

    def set_gimbal_rotation(self,
                            yaw: float = 0.0,
                            pitch: float = 0.0,
                            roll: float = 0.0,
                            speed: int = 50) -> bool:
        """
        Set gimbal rotation angles

        Args:
            yaw: Yaw angle in degrees (-180 to 180)
            pitch: Pitch angle in degrees (-90 to 90)
            roll: Roll angle in degrees (-45 to 45)
            speed: Movement speed (1-100)

        Returns:
            bool: True if command was sent successfully
        """
        # Clamp values to valid ranges
        yaw = max(-180, min(180, yaw))
        pitch = max(-90, min(90, pitch))
        roll = max(-45, min(45, roll))
        speed = max(1, min(100, speed))

        # Convert angles to SIYI protocol format (0.1 degree units)
        yaw_val = int(yaw * 10)
        pitch_val = int(pitch * 10)
        roll_val = int(roll * 10)

        # Create rotation control packet
        # Format: [control_byte, yaw, pitch, roll, speed]
        data = struct.pack('<Bhhhb',
                           0x01,  # Control byte (absolute position)
                           yaw_val,  # Yaw in 0.1°
                           pitch_val,  # Pitch in 0.1°
                           roll_val,  # Roll in 0.1°
                           speed  # Speed
                           )

        packet = self._create_packet(self.CMD_GIMBAL_ROT_CTRL, data)
        response = self._send_packet(packet)

        return response is not None

    def center_gimbal(self) -> bool:
        """Center the gimbal (return to neutral position)"""
        packet = self._create_packet(self.CMD_CENTER_GIMBAL)
        response = self._send_packet(packet)
        return response is not None

    def take_photo(self) -> bool:
        """Trigger camera to take a photo"""
        packet = self._create_packet(self.CMD_CAMERA_PHOTO_CTRL, bytes([0x01]))
        response = self._send_packet(packet)
        return response is not None

    def start_recording(self) -> bool:
        """Start video recording"""
        packet = self._create_packet(self.CMD_CAMERA_RECORD_CTRL, bytes([0x01]))
        response = self._send_packet(packet)
        return response is not None

    def stop_recording(self) -> bool:
        """Stop video recording"""
        packet = self._create_packet(self.CMD_CAMERA_RECORD_CTRL, bytes([0x00]))
        response = self._send_packet(packet)
        return response is not None

    def set_focus(self, focus_value: int) -> bool:
        """
        Set camera focus

        Args:
            focus_value: Focus value (0-100)
        """
        focus_value = max(0, min(100, focus_value))
        packet = self._create_packet(self.CMD_CAMERA_FOCUS_CTRL, bytes([focus_value]))
        response = self._send_packet(packet)
        return response is not None

    def close(self):
        """Close the UDP socket"""
        self.socket.close()


def main():
    """Example usage of SIYI gimbal controller"""
    # Create controller
    controller = SIYIGimbalController(debug=True)

    try:
        # Test sequence
        print("Centering gimbal...")
        controller.center_gimbal()
        time.sleep(2)

        print("Moving gimbal...")
        controller.set_gimbal_rotation(yaw=45, pitch=30, speed=30)
        time.sleep(3)

        print("Taking photo...")
        controller.take_photo()
        time.sleep(1)

        print("Starting recording...")
        controller.start_recording()
        time.sleep(5)

        print("Stopping recording...")
        controller.stop_recording()
        time.sleep(1)

        print("Returning to center...")
        controller.center_gimbal()

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        controller.close()
        print("Controller closed")


if __name__ == "__main__":
    main()