from task_selector.task_selector import TaskSelector
import rclpy


def test_manual_control_instantiation():
    """Unit test for the Task Selector instantiation."""
    rclpy.init()
    TaskSelector()
    rclpy.shutdown()
