from typing import Optional

import rclpy
from pynput import keyboard
from pynput.keyboard import Key, KeyCode
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from interfaces.msg import ROVControl


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

        self.pub_status = self.create_publisher(
            ROVControl, "manual_control", qos_profile_system_default
        )
        self.get_logger().info(HELP_MSG)
        self.status = {
            FORWARD: False,
            BACKWARD: False,
            LEFT: False,
            RIGHT: False,
            UP: False,
            DOWN: False,
            ROLL_LEFT: False,
            ROLL_RIGHT: False,
            PITCH_UP: False,
            PITCH_DOWN: False,
            YAW_LEFT: False,
            YAW_RIGHT: False,
        }

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

            if key_name == HELP:
                self.get_logger().info(HELP_MSG)
            else:
                self.status[key_name] = True

            self.pub_rov_control()

        except Exception as e:
            self.get_logger().error(str(e))
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

            if key_name == HELP:
                pass
            else:
                self.status[key_name] = False

            self.pub_rov_control()

        except Exception as e:
            self.get_logger().error(str(e))
            raise e

    def pub_rov_control(self):
        msg = ROVControl()
        msg.x = (self.status[FORWARD] - self.status[BACKWARD]) * 400 + 1500
        msg.y = (self.status[LEFT] - self.status[RIGHT]) * 400 + 1500
        msg.z = (self.status[UP] - self.status[DOWN]) * 400 + 1500
        msg.roll = (self.status[ROLL_LEFT] - self.status[ROLL_RIGHT]) * 400 + 1500

        msg.pitch = (self.status[PITCH_UP] - self.status[PITCH_DOWN]) * 400 + 1500

        msg.yaw = (self.status[YAW_LEFT] - self.status[YAW_RIGHT]) * 400 + 1500

        self.pub_status.publish(msg)

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
