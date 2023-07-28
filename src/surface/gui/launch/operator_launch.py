from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Asynchronously launches operator's gui node."""
    gui_node: Node = Node(
        package='gui',
        executable='run_operator',
        parameters=[
                {'theme': LaunchConfiguration('theme', default='dark')}],
        remappings=[("/surface/gui/armed", "/armed")],
        emulate_tty=True
    )

    return LaunchDescription([gui_node])
