from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Asynchronously launches pilot's gui node."""
    gui_node: Node = Node(
        package='gui',
        executable='run_pilot',
        namespace='gui',
        parameters=[
                {'theme': LaunchConfiguration('theme', default='dark')}],
        remappings=[("/surface/gui/armed", "/armed"),
                    ("/surface/gui/camera_switch", "/surface/camera_switch")],
        emulate_tty=True,
        output='screen'
    )

    return LaunchDescription([gui_node])
