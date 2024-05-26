from collections.abc import MutableSequence

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from sensor_msgs.msg import Joy

from rov_msgs.msg import (CameraControllerSwitch, Manip, PixhawkInstruction,
                          ValveManip)

UNPRESSED = 0
PRESSED = 1

# Button meanings for PS5 Control might be different for others
X_BUTTON = 0  # Manipulator 0
O_BUTTON = 1  # Manipulator 1
TRI_BUTTON = 2  # Manipulator 2
SQUARE_BUTTON = 3  # Manipulator 3
L1 = 4
R1 = 5
L2 = 6
R2 = 7
PAIRING_BUTTON = 8
MENU = 9
PS_BUTTON = 10
LJOYPRESS = 11
RJOYPRESS = 12
# Joystick Directions 1 is up/left -1 is down/right
# X is forward/backward Y is left/right
# L2 and R2 1 is not pressed and -1 is pressed
LJOYX = 0
LJOYY = 1
L2PRESS_PERCENT = 2
RJOYX = 3
RJOYY = 4
R2PRESS_PERCENT = 5
DPADHOR = 6
DPADVERT = 7


class ManualControlNode(Node):
    def __init__(self) -> None:
        super().__init__('manual_control_node')

        self.rc_pub = self.create_publisher(
            PixhawkInstruction,
            'uninverted_pixhawk_control',
            qos_profile_system_default
        )

        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.controller_callback,
            qos_profile_sensor_data
        )

        # Manipulators
        self.manip_publisher = self.create_publisher(
            Manip,
            'manipulator_control',
            qos_profile_system_default
        )

        # Valve Manip
        self.valve_manip = self.create_publisher(
            ValveManip,
            "valve_manipulator",
            qos_profile_system_default
        )

        # Cameras
        self.camera_toggle_publisher = self.create_publisher(
            CameraControllerSwitch,
            "camera_switch",
            qos_profile_system_default
        )

        self.manip_buttons: dict[int, ManipButton] = {
            X_BUTTON: ManipButton("left"),
            O_BUTTON: ManipButton("right")
        }

        self.seen_left_cam = False
        self.seen_right_cam = False
        self.valve_manip_state = False

    def controller_callback(self, msg: Joy) -> None:
        self.joystick_to_pixhawk(msg)
        self.valve_manip_callback(msg)
        self.manip_callback(msg)
        self.camera_toggle(msg)

    def joystick_to_pixhawk(self, msg: Joy) -> None:
        axes: MutableSequence[float] = msg.axes
        buttons: MutableSequence[int] = msg.buttons

        instruction = PixhawkInstruction(
            forward=float(axes[LJOYY]),  # Left Joystick Y
            lateral=-float(axes[LJOYX]),  # Left Joystick X
            vertical=float(axes[L2PRESS_PERCENT] - axes[R2PRESS_PERCENT]) / 2,  # L2/R2 triggers
            roll=float(buttons[L1] - buttons[R1]),  # L1/R1 buttons
            pitch=float(axes[RJOYY]),  # Right Joystick Y
            yaw=-float(axes[RJOYX]),  # Right Joystick X
            author=PixhawkInstruction.MANUAL_CONTROL
        )

        self.rc_pub.publish(instruction)

    def manip_callback(self, msg: Joy) -> None:
        buttons: MutableSequence[int] = msg.buttons

        for button_id, manip_button in self.manip_buttons.items():
            just_pressed = buttons[button_id] == PRESSED

            if manip_button.last_button_state is False and just_pressed:
                new_manip_state = not manip_button.is_active
                manip_button.is_active = new_manip_state

                manip_msg = Manip(manip_id=manip_button.claw,
                                  activated=manip_button.is_active)
                self.manip_publisher.publish(manip_msg)
            manip_button.last_button_state = just_pressed

    def valve_manip_callback(self, msg: Joy) -> None:
        tri_pressed = msg.buttons[TRI_BUTTON] == PRESSED
        square_pressed = msg.buttons[SQUARE_BUTTON] == PRESSED
        if tri_pressed and not self.valve_manip_state:
            self.valve_manip.publish(ValveManip(active=True, pwm=ValveManip.MAX_PWM))
            self.valve_manip_state = True
        elif square_pressed and not self.valve_manip_state:
            self.valve_manip.publish(ValveManip(active=True, pwm=ValveManip.MIN_PWM))
            self.valve_manip_state = True
        elif self.valve_manip_state and not tri_pressed and not square_pressed:
            self.valve_manip.publish(ValveManip(active=False))
            self.valve_manip_state = False

    def camera_toggle(self, msg: Joy) -> None:
        """Cycles through connected cameras on pilot GUI using menu and pairing buttons."""
        buttons: MutableSequence[int] = msg.buttons

        if buttons[MENU] == PRESSED:
            self.seen_right_cam = True
        elif buttons[PAIRING_BUTTON] == PRESSED:
            self.seen_left_cam = True
        elif buttons[MENU] == UNPRESSED and self.seen_right_cam:
            self.seen_right_cam = False
            self.camera_toggle_publisher.publish(CameraControllerSwitch(toggle_right=True))
        elif buttons[PAIRING_BUTTON] == UNPRESSED and self.seen_left_cam:
            self.seen_left_cam = False
            self.camera_toggle_publisher.publish(CameraControllerSwitch(toggle_right=False))


class ManipButton:
    def __init__(self, claw: str) -> None:
        self.claw = claw
        self.last_button_state: bool = False
        self.is_active: bool = False


def main() -> None:
    rclpy.init()
    manual_control = ManualControlNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(manual_control, executor=executor)
