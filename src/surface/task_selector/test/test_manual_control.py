import task_selector
import rclpy


def test_manual_control_instantiation() -> None:
    """Unit test for the Manual Control instantiation."""
    rclpy.init()
    task_selector.manual_control_node.ManualControlNode()
    rclpy.shutdown()


def test_joystick_profiles() -> None:
    """Unit test for the joystick_profiles function."""
    rclpy.init()
    node = task_selector.manual_control_node.ManualControlNode()

    zero_speed = task_selector.manual_control_node.ZERO_SPEED
    range_speed = task_selector.manual_control_node.RANGE_SPEED
    # Nice boundary values
    assert node.joystick_profiles(0) == zero_speed
    assert node.joystick_profiles(1) == (zero_speed + range_speed)
    assert node.joystick_profiles(-1) == (zero_speed - range_speed)

    # Not nice possible values
    assert node.joystick_profiles(0.34) == 1539
    assert node.joystick_profiles(-0.6) == 1378
    rclpy.shutdown()
