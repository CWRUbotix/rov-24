from launch.launch_description import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    manip_node = Node(
        package="manipulators",
        executable="manipulators",
        parameters=[
            {"left": 0},
            {"right": 1},
            # {"light": 2},
        ],
        remappings=[("/pi/manipulator_control", "/tether/manipulator_control")],
        emulate_tty=True,
        output="screen"
    )

    valve_manip_node = Node(
        package="manipulators",
        executable="test_gpio",
        remappings=[("/pi/manipulator_control", "/tether/manipulator_control")],
        emulate_tty=True,
        output="screen"
    )

    return LaunchDescription([
        manip_node,
        valve_manip_node
    ])
