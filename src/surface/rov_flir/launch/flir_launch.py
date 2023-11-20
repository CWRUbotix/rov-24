from launch.launch_description import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    # launches node to run front flir camera
    front_cam: Node = Node(
        package='spinnaker_camera_driver',
        executable='camera_driver_node',
        emulate_tty=True,
        name='front_cam',
        output='screen',
        parameters={'serial_number': 33}
    )

    # launches node to run bottom flir camera
    bottom_cam: Node = Node(
        package='spinnaker_camera_driver',
        executable='camera_driver_node',
        emulate_tty=True,
        name='bottom_Cam',
        output='screen',
        parameters={'serial_number': 34}
    )

    return LaunchDescription([
        front_cam,
        bottom_cam
    ])
