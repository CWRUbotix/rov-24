import rclpy
from rclpy.executors import MultiThreadedExecutor

from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_system_default
from mavros_msgs.msg import OverrideRCIn
from typing import Callable

from rov_msgs.msg import CameraControllerSwitch, PixhawkInstruction

# Brown out protection
SPEED_THROTTLE: float = 0.85

# Joystick curve
JOYSTICK_EXPONENT = 3

# Range of values Pixhawk takes
# In microseconds
ZERO_SPEED: int = 1500
MAX_RANGE_SPEED: int = 400
RANGE_SPEED: float = MAX_RANGE_SPEED * SPEED_THROTTLE

# Channels for RC command
MAX_CHANNEL: int = 8
MIN_CHANNEL: int = 1

FORWARD_CHANNEL: int = 4  # X
THROTTLE_CHANNEL: int = 2  # Z (vertical)
LATERAL_CHANNEL: int = 5  # Y (left & right)
PITCH_CHANNEL: int = 0  # Pitch
YAW_CHANNEL: int = 3  # Yaw
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
            PixhawkInstruction,
            'pixhawk_control',
            self.control_callback,
            qos_profile_system_default
        )

        self.rc_pub: Publisher = self.create_publisher(
            OverrideRCIn,
            'mavros/rc/override',
            qos_profile_system_default
        )

    @staticmethod
    def apply(msg: PixhawkInstruction, function_to_apply: Callable[[float], float]) -> None:
        """Apply a function to each dimension of this PixhawkInstruction."""
        msg.forward = function_to_apply(msg.forward)
        msg.vertical = function_to_apply(msg.vertical)
        msg.lateral = function_to_apply(msg.lateral)
        msg.pitch = function_to_apply(msg.pitch)
        msg.yaw = function_to_apply(msg.yaw)
        msg.roll = function_to_apply(msg.roll)

    @staticmethod
    def to_override_rc_in(msg: PixhawkInstruction) -> OverrideRCIn:
        """Convert this PixhawkInstruction to an rc_msg with the appropriate channels array."""
        rc_msg = OverrideRCIn()

        ControlInverterNode.apply(msg, lambda value: int(RANGE_SPEED * value) + ZERO_SPEED)

        rc_msg.channels[FORWARD_CHANNEL] = msg.forward
        rc_msg.channels[THROTTLE_CHANNEL] = msg.vertical
        rc_msg.channels[LATERAL_CHANNEL] = msg.lateral
        rc_msg.channels[PITCH_CHANNEL] = msg.pitch
        rc_msg.channels[YAW_CHANNEL] = msg.yaw
        rc_msg.channels[ROLL_CHANNEL] = msg.roll

        return rc_msg

    def invert_callback(self, msg: CameraControllerSwitch) -> None:
        self.inverted = not self.inverted

    def control_callback(self, msg: PixhawkInstruction) -> None:
        # Smooth out adjustments

        def joystick_map(raw: float) -> float:
            val = abs(raw) ** JOYSTICK_EXPONENT
            if raw < 0:
                val *= -1
            return val

        ControlInverterNode.apply(msg, joystick_map)

        if self.inverted:
            msg.forward *= -1
            msg.lateral *= -1
            msg.pitch *= -1
            msg.roll *= -1

        self.rc_pub.publish(msg=self.to_override_rc_in(msg))


def main() -> None:
    rclpy.init()
    control_invert = ControlInverterNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(control_invert, executor=executor)
