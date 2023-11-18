import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace


def generate_launch_description():

    gui_path: str = get_package_share_directory('gui')
    task_selector_path: str = get_package_share_directory('task_selector')
    transceiver_path: str = get_package_share_directory('transceiver')

    # Launches Gui
    gui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                gui_path, 'launch', 'operator_launch.py'
            )
        ]),
    )

    # Launches Task Selector
    task_selector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                task_selector_path, 'launch', 'task_scheduler_launch.py'
            )
        ]),
    )

    namespace_launch = GroupAction(
        actions=[
            PushRosNamespace("/surface"),
            gui_launch,
            task_selector_launch,
        ]
    )

    # Launches Transceiver
    transceiver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                transceiver_path, 'launch', 'serial_reader_launch.py'
            )
        ]),
    )

    return LaunchDescription([
        namespace_launch,
        transceiver_launch,
    ])
