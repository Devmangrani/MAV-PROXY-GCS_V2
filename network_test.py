import socket
import time
import platform
import subprocess


def ping_test(host):
    """
    Returns True if host responds to a ping request
    """
    # Option for the number of packets as a function of OS
    param = '-n' if platform.system().lower() == 'windows' else '-c'

    command = ['ping', param, '1', host]
    return subprocess.call(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0


def test_udp_connection(ip="192.168.144.25", port=37260):
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)  # 1 second timeout

    # Simple heartbeat packet for SIYI (you can modify based on your camera model)
    heartbeat = bytes([0x55, 0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x62])

    try:
        print(f"\nTesting connection to {ip}:{port}")
        print("1. Network Connectivity Test:")
        if ping_test(ip):
            print("   ✓ Can ping camera IP")
        else:
            print("   ✗ Cannot ping camera IP")
            print("   → Check WiFi connection to camera")
            print("   → Verify camera IP address")
            return

        print("\n2. UDP Port Test:")
        for i in range(3):
            print(f"\nAttempt {i + 1}/3:")
            print(f"   Sending heartbeat packet: {heartbeat.hex()}")
            sock.sendto(heartbeat, (ip, port))

            try:
                data, addr = sock.recvfrom(1024)
                print(f"   Received response: {data.hex()}")
                print(f"   From: {addr}")
                break
            except socket.timeout:
                print("   ✗ No response received")
                if i < 2:
                    print("   Retrying...")
                else:
                    print("\nTroubleshooting steps:")
                    print("1. Verify camera is powered on")
                    print("2. Check if you're connected to the correct WiFi network")
                    print("3. Confirm camera IP and port")
                    print("4. Ensure no firewall is blocking UDP traffic")
                    print("5. Try rebooting the camera")
                    print("\nCamera WiFi Settings:")
                    print("- Look for network starting with 'SIYI'")
                    print("- Check camera manual for default password")
                    print("\nDefault Network Settings:")
                    print("- IP: 192.168.144.25")
                    print("- Port: 37260")

    except Exception as e:
        print(f"\nError: {e}")
    finally:
        sock.close()


if __name__ == "__main__":
    test_udp_connection()