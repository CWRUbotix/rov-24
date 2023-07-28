import rclpy
from mavros_msgs.msg import OverrideRCIn
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node, Publisher, Subscription
from sensor_msgs.msg import Joy

from interfaces.action import BasicTask
from interfaces.msg import CameraControllerSwitch, Manip

# Button meanings for PS5 Control might be different for others
X_BUTTON:        int = 0  # Manipulator 0
O_BUTTON:        int = 1  # Manipulator 1
TRI_BUTTON:      int = 2  # Manipulator 2
SQUARE_BUTTON:   int = 3  # Manipulator 3
L1:              int = 4
R1:              int = 5
L2:              int = 6
R2:              int = 7
PAIRING_BUTTON:  int = 8
MENU:            int = 9
PS_BUTTON:       int = 10
LJOYPRESS:       int = 11
RJOYPRESS:       int = 12
# Joystick Directions 1 is up/left -1 is down/right
# X is forward/backward Y is left/right
# L2 and R2 1 is not pressed and -1 is pressed
LJOYY:           int = 0
LJOYX:           int = 1
L2PRESS_PERCENT: int = 2
RJOYY:           int = 3
RJOYX:           int = 4
R2PRESS_PERCENT: int = 5
DPADHOR:         int = 6
DPADVERT:        int = 7

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
THROTTLE_CHANNEL: int = 2  # Z
LATERAL_CHANNEL:  int = 3  # Y
FORWARD_CHANNEL:  int = 4  # X
YAW_CHANNEL:      int = 5  # Yaw


class ManualControlNode(Node):
    _passing: bool = False

    def __init__(self):
        super().__init__('manual_control_node',
                         parameter_overrides=[],
                         namespace='surface')
        # TODO would Service make more sense then Actions?
        self._action_server: ActionServer = ActionServer(
            self,
            BasicTask,
            'manual_control',
            self.execute_callback
        )
        self.rc_pub: Publisher = self.create_publisher(
            OverrideRCIn,
            '/mavros/rc/override',
            10
        )
        self.subscription: Subscription = self.create_subscription(
            Joy,
            'joy',
            self.controller_callback,
            100
        )

        # Manipulators
        self.manip_publisher: Publisher = self.create_publisher(
            Manip,
            'manipulator_control',
            10
        )

        # Cameras
        self.camera_toggle_publisher = self.create_publisher(
            CameraControllerSwitch,
            "camera_switch",
            10
        )

        self.manip_buttons: dict[int, ManipButton] = {
            X_BUTTON: ManipButton("claw0"),
            O_BUTTON: ManipButton("claw1"),
            TRI_BUTTON: ManipButton("light")
        }

        self.seen_left_cam = False
        self.seen_right_cam = False

    def controller_callback(self, msg: Joy):
        if self._passing:
            self.joystick_to_pixhawk(msg)
            self.manip_callback(msg)
            self.camera_toggle(msg)

    def joystick_to_pixhawk(self, msg: Joy):

        axes = msg.axes
        buttons = msg.buttons
        rc_msg = OverrideRCIn()

        # DPad Pitch
        rc_msg.channels[PITCH_CHANNEL] = self.joystick_profiles(axes[DPADVERT])
        # L1/R1 Buttons for Roll
        rc_msg.channels[ROLL_CHANNEL] = self.joystick_profiles(buttons[R1] - buttons[L1])
        # Right Joystick Z
        rc_msg.channels[THROTTLE_CHANNEL] = self.joystick_profiles(axes[RJOYX])
        # Left Joystick XY
        rc_msg.channels[FORWARD_CHANNEL] = self.joystick_profiles(axes[LJOYX])
        rc_msg.channels[LATERAL_CHANNEL] = self.joystick_profiles(-axes[LJOYY])
        # L2/R2 Buttons for Yaw
        rc_msg.channels[YAW_CHANNEL] = self.joystick_profiles((axes[R2PRESS_PERCENT] -
                                                               axes[L2PRESS_PERCENT])/2)
        self.rc_pub.publish(rc_msg)

    # Used to create smoother adjustments
    def joystick_profiles(self, val: float) -> int:
        return ZERO_SPEED + int(RANGE_SPEED * val * abs(val))

    def execute_callback(self, goal_handle: ServerGoalHandle) -> BasicTask.Result:
        self.get_logger().info('Starting Manual Control')

        if goal_handle.is_cancel_requested:
            self._passing = False

            goal_handle.canceled()
            self.get_logger().info('Ending Manual Control')
            return BasicTask.Result()
        else:
            self._passing = True

            feedback_msg = BasicTask.Feedback()
            feedback_msg.feedback_message = "Task is executing"
            goal_handle.publish_feedback(feedback_msg)
            goal_handle.succeed()
            return BasicTask.Result()

    # TODO jank?
    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info('Received cancel request')
        self._passing = False
        return CancelResponse.ACCEPT

    def manip_callback(self, msg: Joy):
        buttons: list[int] = msg.buttons

        for button_id, manip_button in self.manip_buttons.items():

            just_pressed: bool = False

            if buttons[button_id] == 1:
                just_pressed = True

            if manip_button.last_button_state is False and just_pressed:
                new_manip_state: bool = not manip_button.is_active
                manip_button.is_active = new_manip_state

                log_msg: str = f"manip_id= {manip_button.claw}, manip_active= {new_manip_state}"
                self.get_logger().info(log_msg)

                manip_msg: Manip = Manip(manip_id=manip_button.claw,
                                         activated=manip_button.is_active)
                self.manip_publisher.publish(manip_msg)

            manip_button.last_button_state = just_pressed

    def camera_toggle(self, msg: Joy):
        """Cycles through connected cameras on pilot GUI using menu and pairing buttons."""
        buttons: list[int] = msg.buttons

        if buttons[MENU] == 1:
            self.seen_right_cam = True
        elif buttons[PAIRING_BUTTON] == 1:
            self.seen_left_cam = True
        elif buttons[MENU] == 0 and self.seen_right_cam:
            self.seen_right_cam = False
            self.camera_toggle_publisher.publish(CameraControllerSwitch(toggle_right=True))
        elif buttons[PAIRING_BUTTON] == 0 and self.seen_left_cam:
            self.seen_left_cam = False
            self.camera_toggle_publisher.publish(CameraControllerSwitch(toggle_right=False))


class ManipButton:
    def __init__(self, claw: str):
        self.claw: str = claw
        self.last_button_state: bool = False
        self.is_active: bool = False


def main():
    rclpy.init()
    manual_control = ManualControlNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(manual_control, executor=executor)
