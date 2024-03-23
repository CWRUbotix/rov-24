from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_system_default
from mavros_msgs.msg import OverrideRCIn

from rov_msgs.msg import CameraControllerSwitch

FORWARD_CHANNEL: int = 4  # X
THROTTLE_CHANNEL: int = 2  # Z (vertical)
LATERAL_CHANNEL: int = 3  # Y (left & right)
PITCH_CHANNEL: int = 0  # Pitch
YAW_CHANNEL: int = 5  # Yaw
ROLL_CHANNEL: int = 1  # Roll


class ControlInverterNode(Node):
    def __init__(self) -> None:
        super().__init__('control_inverter_node',
                         parameter_overrides=[])

        self.inverted = False

        self.inversion_subscription: Subscription = self.create_subscription(
            CameraControllerSwitch,
            'camera_switch',
            self.invert_callback,
            qos_profile_system_default
        )

        self.control_subscription: Subscription = self.create_subscription(
            OverrideRCIn,
            'pixhawk_control',
            self.control_callback,
            qos_profile_system_default
        )

        self.rc_pub: Publisher = self.create_publisher(
            OverrideRCIn,
            'mavros/rc/override',
            qos_profile_system_default
        )

    def invert_callback(self, msg: CameraControllerSwitch) -> None:
        self.inverted = not self.inverted

    def control_callback(self, msg: OverrideRCIn) -> None:
        if self.inverted:
            msg.channels[FORWARD_CHANNEL] *= -1
            msg.channels[LATERAL_CHANNEL] *= -1
            msg.channels[PITCH_CHANNEL] *= -1
            msg.channels[ROLL_CHANNEL] *= -1

        self.rc_pub.publish(msg=msg)
