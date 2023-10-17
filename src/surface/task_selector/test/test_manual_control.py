from task_selector.manual_control_node import ManualControlNode, ZERO_SPEED, RANGE_SPEED
import rclpy


def test_joystick_profiles():
    """Unit test for the joystick_profiles function"""

    rclpy.init()
    node = ManualControlNode()
    # Nice boundary values
    assert node.joystick_profiles(0) == ZERO_SPEED
    assert node.joystick_profiles(1) == (ZERO_SPEED + RANGE_SPEED)
    assert node.joystick_profiles(-1) == (ZERO_SPEED - RANGE_SPEED)

    # Not nice possible values
    assert node.joystick_profiles(0.34) == 1539
    assert node.joystick_profiles(-0.6) == 1378
    rclpy.shutdown()
