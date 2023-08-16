from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PORT = '/dev/ttyPixhawk'


def generate_launch_description():

    # launches PI to Pixhawk Communication
    pixhawk_com_node: Node = Node(
        package='pixhawk_communication',
        executable='pixhawk_com',
        parameters=[{'communication': LaunchConfiguration('communication', default=PORT)}],
        remappings=[("/pi/armed", "/armed"),
                    ("/pi/manual_control", "/manual_control")],
        emulate_tty=True,
        output='screen'
    )

    return LaunchDescription([pixhawk_com_node])