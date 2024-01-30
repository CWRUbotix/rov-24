"""pi_launch launch file."""
import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace, Node


def generate_launch_description() -> LaunchDescription:
    """
    Generate LaunchDescription for pi_main.

    Returns
    -------
    LaunchDescription
        Launches camera_streamer package and pixhawk_communication package.

    """
    NAMESPACE = 'pi'
    # Manipulator Controller
    # manip_path: str = get_package_share_directory('manipulators')
    #
    # manip_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(
    #             manip_path, 'launch', 'manip_launch.py'
    #         )
    #     ])
    # )

    # Camera Streamer
    cam_path: str = get_package_share_directory('camera_streamer')

    cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                cam_path, 'launch', 'camera_launch.py'
            )
        ])
    )

    # Pixhawk Communication
    pixhawk_path: str = get_package_share_directory('pixhawk_communication')

    pixhawk_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                pixhawk_path, 'launch', 'mavros_launch.py'
            )
        ])
    )

    # realsense_path: str = get_package_share_directory('realsense')

    # # Launches Realsense
    # realsense_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(
    #             realsense_path, 'launch', 'realsense_launch.py'
    #         )
    #     ]),
    #     launch_arguments={'align_depth.enable': 'true'}.items()
    # )

    # Launches pi_publisher node.
    pi_publisher_node = Node(
        package='pi_main',
        executable='ip_publisher',
        emulate_tty=True,
        output='screen'
    )

    namespace_launch = GroupAction(
        actions=[
            PushRosNamespace(NAMESPACE),
            # manip_launch,
            pixhawk_launch,
            cam_launch,
            # realsense_launch,
            pi_publisher_node
        ]
    )

    return LaunchDescription([
        namespace_launch,
    ])
