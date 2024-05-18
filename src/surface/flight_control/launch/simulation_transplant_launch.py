from launch.actions import GroupAction
from launch.launch_description import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    transplant_node = Node(
        package='flight_control',
        executable='auto_transplant_node',
        remappings=[('/surface/mavros/rc/override', '/tether/mavros/rc/override')],
        emulate_tty=True,
        output='screen'
    )

    namespace_launch: GroupAction = GroupAction(
        actions=[
            PushRosNamespace("surface"),
            transplant_node
        ]
    )

    return LaunchDescription([
        namespace_launch
    ])
