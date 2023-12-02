from rov_msgs.srv import TaskControl

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class AutoDocker(Node):

    def __init__(self):
        super().__init__('auto_docker',
                         parameter_overrides=[])

        self.control_server = self.create_service(
            TaskControl, '/auto_docker_control', self.task_control_callback)

        self.running = False

        # Add cam frame subscriber here to act as control loop for auto docking

    def task_control_callback(self, request: TaskControl.Request,
                              response: TaskControl.Response):
        self.running = request.start
        response.is_running = self.running

        return response


def main():
    rclpy.init()
    auto_docker = AutoDocker()
    executor = MultiThreadedExecutor()
    rclpy.spin(auto_docker, executor=executor)
