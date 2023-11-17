import keyboard_driver
import rclpy
import pytest


@pytest.fixture
def test_keyboard_listener_instantiation() -> None:
    """Unit test for KeyboardListenerNode instantiation."""
    rclpy.init()
    keyboard_driver.keyboard_driver.KeyboardListenerNode()
    rclpy.shutdown()
