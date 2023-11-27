import os

import pytest
from ament_index_python.packages import get_package_share_directory
from launch.launch_description import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest


@pytest.mark.launch_test
def generate_test_description() -> LaunchDescription:

    task_selector_path: str = get_package_share_directory('task_selector')
    return LaunchDescription([
        # Launches Task Selector
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    task_selector_path, 'launch', 'task_scheduler_launch.py'
                )
            ]),
        ),
        ReadyToTest()
    ])
