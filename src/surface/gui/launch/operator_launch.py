from launch.launch_description import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction


def generate_launch_description() -> LaunchDescription:
    """Asynchronously launches operator's gui node."""
    gui_node: Node = Node(
        package='gui',
        executable='run_operator',
        parameters=[{'theme': LaunchConfiguration('theme', default='dark')}],
        remappings=[("/surface/gui/task_request", "/surface/task_request"),
                    ("/surface/gui/task_feedback", "/surface/task_feedback"),
                    ("/surface/gui/bottom_cam/image_raw", "/surface/bottom_cam/image_raw"),
                    ("/surface/gui/mavros/cmd/command", "/tether/mavros/cmd/command"),
                    ("/surface/gui/mavros/param/set", "/tether/mavros/param/set"),
                    ("/surface/gui/mavros/param/pull", "/tether/mavros/param/pull"),
                    ("/surface/gui/vehicle_state_event", "/surface/vehicle_state_event"),
                    ("/surface/gui/mavros/cmd/arming", "/tether/mavros/cmd/arming")],
        emulate_tty=True,
        output='screen'
    )

    namespace_launch = GroupAction(
        actions=[
            PushRosNamespace('gui'),
            gui_node
        ]
    )

    return LaunchDescription([namespace_launch])
