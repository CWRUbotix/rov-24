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
            # Set the system_id to 255 so mavros_node is treated as
            # Ground Control Station (GCS). To use RC control it needs to
            # receive a signal from a GCS.
            {"system_id": 255},
            # plugin_allowlist allows which mavros nodes get launched. The default is all of them.
            {"plugin_allowlist": ["sys_status", "rc_io", "command", "param"]},
            {"fcu_url": "/dev/ttyACM0"}
        ],
        remappings=[('/pi/mavros/rc/override', '/tether/mavros/rc/override'),
                    ('/pi/mavros/cmd/arming', '/tether/mavros/cmd/arming'),
                    ('/pi/mavros/cmd/cmd', '/tether/mavros/cmd/cmd')]
    )

    return LaunchDescription([
        mavros_node
    ])
