from keyboard_driver.keyboard_driver_node import KeyboardListenerNode
import rclpy
import pytest


@pytest.fixture
def test_keyboard_listener_instantiation() -> None:
    """Unit test for KeyboardListenerNode instantiation."""
    rclpy.init()
    KeyboardListenerNode()
    rclpy.shutdown()
