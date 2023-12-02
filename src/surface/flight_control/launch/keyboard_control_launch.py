from launch.launch_description import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='flight_control',
            executable='keyboard_control_node',
            remappings=[('/surface/mavros/rc/override', '/tether/mavros/rc/override')],
            emulate_tty=True,
            output='screen'
        )
    ])
