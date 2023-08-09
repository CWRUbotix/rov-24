from typing import Optional

import rclpy
from mavros_msgs.msg import OverrideRCIn
from pynput import keyboard
from pynput.keyboard import Key, KeyCode
from rclpy.node import Node

# Channels for RC command
MAX_CHANNEL: int = 8
MIN_CHANNEL: int = 1

PITCH_CHANNEL:    int = 0  # Pitch
ROLL_CHANNEL:     int = 1  # Roll
THROTTLE_CHANNEL: int = 2  # Z
LATERAL_CHANNEL:  int = 3  # Y
FORWARD_CHANNEL:  int = 4  # X
YAW_CHANNEL:      int = 5  # Yaw


class Keys:
    # key bindings
    FORWARD = "w"
    BACKWARD = "s"
    LEFT = "a"
    RIGHT = "d"
    UP = "2"
    DOWN = "x"

    ROLL_LEFT = "j"
    ROLL_RIGHT = "l"
    PITCH_UP = "i"
    PITCH_DOWN = "k"
    YAW_LEFT = "h"
    YAW_RIGHT = ";"

    HELP = "p"

    HELP_MSG = """
    Use keyboard to control ROV

    Key Bindings:
    [2]
    [w]            [i]
    [a][s][d]   [h][j][k][l][;]
    [x]

    [w] = Forward
    [s] = Backward
    [a] = Left
    [d] = Right
    [2] = Up
    [x] = Down

    [j] = Roll Left
    [l] = Roll Right
    [i] = Pitch Up
    [k] = Pitch Down
    [h] = Yaw Left
    [;] = Yaw Right

    [p] = Show this help"""


# Range of values Pixhawk takes
# In microseconds
ZERO_SPEED: int = 1500
RANGE_SPEED: int = 400


class KeyboardListenerNode(Node):
    def __init__(self):
        super().__init__("keyboard_listener_node", parameter_overrides=[])

        self.rc_pub = self.create_publisher(
            OverrideRCIn, "/mavros/rc/override", qos_profile=10
        )
        self.logger.info(Keys.HELP_MSG)
        self.status = {
            "forward": False,
            "backward": False,
            "left": False,
            "right": False,
            "up": False,
            "down": False,
            "roll_left": False,
            "roll_right": False,
            "pitch_up": False,
            "pitch_down": False,
            "yaw_left": False,
            "yaw_right": False,
        }

    @property
    def logger(self):
        return self.get_logger()

    def on_press(self, key: Optional[Key | KeyCode]):
        try:
            key_name: Optional[str] = ''
            if isinstance(key, KeyCode):
                key_name = key.char
                if key_name is None:
                    return
            elif isinstance(key, Key):
                key_name = key.name
            else:
                return

            match key_name:
                case Keys.FORWARD:
                    self.status["forward"] = True
                case Keys.BACKWARD:
                    self.status["backward"] = True
                case Keys.LEFT:
                    self.status["left"] = True
                case Keys.RIGHT:
                    self.status["right"] = True
                case Keys.UP:
                    self.status["up"] = True
                case Keys.DOWN:
                    self.status["down"] = True
                case Keys.ROLL_LEFT:
                    self.status["roll_left"] = True
                case Keys.ROLL_RIGHT:
                    self.status["roll_right"] = True
                case Keys.PITCH_UP:
                    self.status["pitch_up"] = True
                case Keys.PITCH_DOWN:
                    self.status["pitch_down"] = True
                case Keys.YAW_LEFT:
                    self.status["yaw_left"] = True
                case Keys.YAW_RIGHT:
                    self.status["yaw_right"] = True
                case Keys.HELP:
                    self.logger.info(Keys.HELP_MSG)
                case _:
                    return

            self.pub_rov_control()

        except Exception as e:
            self.logger.error(str(e))
            raise e

    def on_release(self, key: Optional[Key | KeyCode]):
        try:
            key_name: Optional[str] = ''
            if isinstance(key, KeyCode):
                key_name = key.char
                if key_name is None:
                    return
            elif isinstance(key, Key):
                key_name = key.name
            else:
                return

            match key_name:
                case Keys.FORWARD:
                    self.status["forward"] = False
                case Keys.BACKWARD:
                    self.status["backward"] = False
                case Keys.LEFT:
                    self.status["left"] = False
                case Keys.RIGHT:
                    self.status["right"] = False
                case Keys.UP:
                    self.status["up"] = False
                case Keys.DOWN:
                    self.status["down"] = False
                case Keys.ROLL_LEFT:
                    self.status["roll_left"] = False
                case Keys.ROLL_RIGHT:
                    self.status["roll_right"] = False
                case Keys.PITCH_UP:
                    self.status["pitch_up"] = False
                case Keys.PITCH_DOWN:
                    self.status["pitch_down"] = False
                case Keys.YAW_LEFT:
                    self.status["yaw_left"] = False
                case Keys.YAW_RIGHT:
                    self.status["yaw_right"] = False
                case Keys.HELP:
                    self.logger.info(Keys.HELP_MSG)
                case _:
                    return

            self.pub_rov_control()

        except Exception as e:
            self.logger.error(str(e))
            raise e

    def pub_rov_control(self):
        msg = OverrideRCIn()

        msg.channels[PITCH_CHANNEL] = (self.status["pitch_up"] - self.status["pitch_down"]) * 400
        msg.channels[ROLL_CHANNEL] = (self.status["roll_left"] - self.status["roll_right"]) * 400
        msg.channels[THROTTLE_CHANNEL] = (self.status["up"] - self.status["down"]) * 400
        msg.channels[LATERAL_CHANNEL] = (self.status["left"] - self.status["right"]) * 400
        msg.channels[FORWARD_CHANNEL] = (self.status["forward"] - self.status["backward"]) * 400
        msg.channels[YAW_CHANNEL] = (self.status["yaw_left"] - self.status["yaw_right"]) * 400

        for channel in range(YAW_CHANNEL):
            # 1500 is no movement
            msg.channels[channel] += 1500

        self.rc_pub.publish(msg)

    def spin(self):
        with keyboard.Listener(
            on_press=self.on_press, on_release=self.on_release
        ) as listener:
            while rclpy.ok() and listener.running:
                rclpy.spin_once(self, timeout_sec=0.1)


def main():
    rclpy.init()
    KeyboardListenerNode().spin()


if __name__ == "__main__":
    main()
