from mavros_msgs.msg import OverrideRCIn
from rclpy.node import Node, Publisher
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data
from dataclasses import dataclass
from typing import Callable

# Brown out protection
SPEED_THROTTLE: float = 0.85

# Range of values Pixhawk takes
# In microseconds
ZERO_SPEED: int = 1500
MAX_RANGE_SPEED: int = 400
RANGE_SPEED: float = MAX_RANGE_SPEED*SPEED_THROTTLE

# Channels for RC command
MAX_CHANNEL: int = 8
MIN_CHANNEL: int = 1

PITCH_CHANNEL:    int = 0  # Pitch
ROLL_CHANNEL:     int = 1  # Roll
THROTTLE_CHANNEL: int = 2  # Z (vertical)
LATERAL_CHANNEL:  int = 3  # Y (left & right)
FORWARD_CHANNEL:  int = 4  # X
YAW_CHANNEL:      int = 5  # Yaw


@dataclass
class PixhawkInstruction:
    """Store movement instructions for the Pixhawk."""

    forward:  int = 0
    vertical: int = 0
    lateral:  int = 0
    pitch:    int = 0
    yaw:      int = 0
    roll:     int = 0

    def map(self, mapping_function: Callable[[int], int]) -> None:
        self.forward  = mapping_function(self.forward)
        self.vertical = mapping_function(self.vertical)
        self.lateral  = mapping_function(self.lateral)
        self.pitch    = mapping_function(self.pitch)
        self.yaw      = mapping_function(self.yaw)
        self.roll     = mapping_function(self.roll)


class PixhawkPublisher:
    def __init__(self, node: Node):
        """Create a Pixhawk movement instruction publisher on the provided node."""
        self.rc_pub: Publisher = node.create_publisher(
            OverrideRCIn,
            '/mavros/rc/override',
            qos_profile_system_default
        )

    def publish_instruction(self, instruction: PixhawkInstruction):
        """Publish the provided movement instruction to the Pixhawk."""
        rc_msg = OverrideRCIn()

        instruction.map(lambda value: int(RANGE_SPEED * value) + ZERO_SPEED)

        rc_msg.channels[PITCH_CHANNEL]    = instruction.pitch
        rc_msg.channels[ROLL_CHANNEL]     = instruction.roll
        rc_msg.channels[THROTTLE_CHANNEL] = instruction.vertical
        rc_msg.channels[FORWARD_CHANNEL]  = instruction.forward
        rc_msg.channels[LATERAL_CHANNEL]  = instruction.lateral
        rc_msg.channels[YAW_CHANNEL]      = instruction.yaw

        self.rc_pub.publish(rc_msg)
