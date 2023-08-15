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
            {"plugin_allowlist": ["sys_status", "rc_io", "command"]},
            {"fcu_url": "/dev/ttyPixhawk"}
        ],
        remappings=[('/pi/mavros/rc/override', '/mavros/rc/override'),
                    ('/pi/mavros/cmd/arming', '/mavros/cmd/arming')]
    )

    return LaunchDescription([
        mavros_node
    ])
