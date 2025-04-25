from siyi_gimbal_controller import SIYIGimbalController
import time


def test_basic_movement():
    # Create controller
    gimbal = SIYIGimbalController(debug=True)

    try:
        print("Centering gimbal...")
        gimbal.center_gimbal()
        time.sleep(2)

        print("Moving right...")
        gimbal.set_gimbal_rotation(yaw=30, pitch=0, speed=20)
        time.sleep(3)

        print("Moving left...")
        gimbal.set_gimbal_rotation(yaw=-30, pitch=0, speed=20)
        time.sleep(3)

        print("Centering...")
        gimbal.center_gimbal()

    finally:
        gimbal.close()


if __name__ == "__main__":
    test_basic_movement()