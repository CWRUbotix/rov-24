from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        output="screen",
        namespace="mavros",
        parameters=[
            # https://github.com/mavlink/mavros/issues/1632
            # Done so RC Override thinks mavros in a gcs.
            {"system_id": 255},
            # plugin_allowlist allows which mavros nodes get launched default is all of them.
            {"plugin_allowlist": ["setpoint_velocity", "command"]},
            {"fcu_url", "/dev/ttyPixhawk:57600"}
        ],
        remappings=[('/pi/mavros/setpoint_velocity/cmd_vel', '/mavros/setpoint_velocity/cmd_vel'),
                    ('/pi/mavros/cmd/arming', '/mavros/cmd/arming')]
    )

    return LaunchDescription([
        mavros_node
    ])
