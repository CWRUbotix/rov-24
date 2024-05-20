import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_system_default

import cv2
from cv2.typing import MatLike
from cv_bridge import CvBridge

from rov_msgs.msg import PixhawkInstruction
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Image

from flight_control.control_inverter_node import ControlInverterNode


class AutonomousTransplanting(Node):

    def __init__(self) -> None:
        super().__init__('auto_transplanting',
                         parameter_overrides=[])

        self.front_cam_subscriber: Subscription = self.create_subscription(
            Image,
            'front_cam/image_raw',
            lambda m: self.handle_frame(m, 'bottom'),
            qos_profile_system_default
        )

        self.bottom_cam_subscriber: Subscription = self.create_subscription(
            Image,
            'bottom_cam/image_raw',
            lambda m: self.handle_frame(m, 'front'),
            qos_profile_system_default
        )

        self.rc_pub: Publisher = self.create_publisher(
            OverrideRCIn,
            'mavros/rc/override',
            qos_profile_system_default
        )

        self.cv_bridge: CvBridge = CvBridge()

    def handle_frame(self, frame: Image, cam_id: str):
        # Gets called once for each frame from either camera
        cv_image: MatLike = self.cv_bridge.imgmsg_to_cv2(
            frame, desired_encoding='passthrough')

        # TODO Transplanting algorithm
        if cam_id == "front":
            # Handle front cam
            pass
        elif cam_id == "bottom":
            # Handle bottom cam
            pass
        else:
            raise ValueError(f"Unknown camera id: {cam_id}")

        # TODO Adjust the instruction based on the location of the red square
        command = PixhawkInstruction(
            forward=0,
            vertical=0.1,
            lateral=0,
            pitch=0,
            yaw=0,
            roll=0,
        )

        self.rc_pub.publish(msg=ControlInverterNode.to_override_rc_in(command))


def main() -> None:
    rclpy.init()
    auto_transplanting = AutonomousTransplanting()
    executor = MultiThreadedExecutor()
    rclpy.spin(auto_transplanting, executor=executor)
