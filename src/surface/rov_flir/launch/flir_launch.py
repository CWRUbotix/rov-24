from launch.launch_description import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import Parameter


def generate_launch_description() -> LaunchDescription:

    # launches node to run front flir camera
    front_cam: Node = Node(
        package='spinnaker_camera_driver',
        executable='camera_driver_node',
        emulate_tty=True,
        name='front_cam',
        output='screen',
        parameters=[Parameter('serial_number', '34'),
                    Parameter('serial_number', '34')]
    )

    # # launches node to run bottom flir camera
    # bottom_cam: Node = Node(
    #     package='spinnaker_camera_driver',
    #     executable='camera_driver_node',
    #     emulate_tty=True,
    #     name='bottom_Cam',
    #     output='screen',
    #     parameters=[Parameter('serial_number', '35')]
    # )

    return LaunchDescription([
        front_cam,
        # bottom_cam
    ])
