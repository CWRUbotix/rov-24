from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        output="screen",
        parameters=[
            # https://github.com/mavlink/mavros/issues/1632
            # Done so RC Override thinks mavros in a gcs.
            {"system_id": 255},
            # TODO check if needed
            {"component_id": 240},
            {"plugin_allowlist": ["rc_io", "command"]}

        ]
    )

    return LaunchDescription([
        mavros_node
    ])
