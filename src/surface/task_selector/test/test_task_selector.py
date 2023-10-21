from task_selector.task_selector import TaskSelector
from task_selector.manual_control_node import main
import rclpy


def test_manual_control_instantiation():
    """Unit test for the Task Selector instantiation."""
    # TaskSelector blocks waiting for Manual Control Client
    main()
    TaskSelector()
    rclpy.shutdown()
