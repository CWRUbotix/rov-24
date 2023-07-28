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
        self.logger.info(HELP_MSG)
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
            if isinstance(key, KeyCode):
                key = key.char
            elif isinstance(key, Key):
                key = key.name

            if key == FORWARD:
                self.status["forward"] = True
            if key == BACKWARD:
                self.status["backward"] = True
            if key == LEFT:
                self.status["left"] = True
            if key == RIGHT:
                self.status["right"] = True
            if key == UP:
                self.status["up"] = True
            if key == DOWN:
                self.status["down"] = True
            if key == ROLL_LEFT:
                self.status["roll_left"] = True
            if key == ROLL_RIGHT:
                self.status["roll_right"] = True
            if key == PITCH_UP:
                self.status["pitch_up"] = True
            if key == PITCH_DOWN:
                self.status["pitch_down"] = True
            if key == YAW_LEFT:
                self.status["yaw_left"] = True
            if key == YAW_RIGHT:
                self.status["yaw_right"] = True
            if key == HELP:
                self.logger.info(HELP_MSG)

            self.pub_rov_control()

        except Exception as e:
            self.logger.error(str(e))
            raise e

    def on_release(self, key: Optional[Key | KeyCode]):
        try:
            if isinstance(key, KeyCode):
                key = key.char
            elif isinstance(key, Key):
                key = key.name

            if key == FORWARD:
                self.status["forward"] = False
            if key == BACKWARD:
                self.status["backward"] = False
            if key == LEFT:
                self.status["left"] = False
            if key == RIGHT:
                self.status["right"] = False
            if key == UP:
                self.status["up"] = False
            if key == DOWN:
                self.status["down"] = False
            if key == ROLL_LEFT:
                self.status["roll_left"] = False
            if key == ROLL_RIGHT:
                self.status["roll_right"] = False
            if key == PITCH_UP:
                self.status["pitch_up"] = False
            if key == PITCH_DOWN:
                self.status["pitch_down"] = False
            if key == YAW_LEFT:
                self.status["yaw_left"] = False
            if key == YAW_RIGHT:
                self.status["yaw_right"] = False

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
