from launch.launch_description import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    heartbeat_node = Node(
        package="pi_diagnostics",
        executable="heartbeat_node",
        remappings=[("/pi/pi_heartbeat", "/tether/pi_heartbeat")],
        emulate_tty=True,
        output='screen'
    )

    ip_publisher = Node(
        package="pi_diagnostics",
        executable="ip_publisher",
        remappings=[('/pi/ip_address', '/tether/ip_address')],
        emulate_tty=True,
        output='screen'
    )

    return LaunchDescription([
        heartbeat_node,
        ip_publisher
    ])
