import rclpy
from keyboard_driver.keyboard_driver_node import KeyboardListenerNode


def test_keyboard_driver_instantiation():
    """Unit test for the Keyboard Driver instantiation."""
    rclpy.init()
    KeyboardListenerNode()
    rclpy.shutdown()
