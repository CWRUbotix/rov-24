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
        remappings=[("/pi/manipulator_control", "/tether/manipulator_control")]
    )

    valve_manip_node = Node(
        package="manipulators",
        executable="valve_manipulator"
    )

    return LaunchDescription([
        manip_node,
        valve_manip_node
    ])
