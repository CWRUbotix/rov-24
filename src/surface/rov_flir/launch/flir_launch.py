from launch.launch_description import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import Parameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    parameter_file = PathJoinSubstitution(
            [FindPackageShare('spinnaker_camera_driver'), 'config',
             'blackfly_s' + '.yaml'])

    # launches node to run front flir camera
    front_cam: Node = Node(
        package='spinnaker_camera_driver',
        executable='camera_driver_node',
        emulate_tty=True,
        name='front_cam',
        output='screen',
        parameters=[Parameter('serial_number', '23473577'),
                    Parameter('parameter_file', parameter_file)]
    )

    # launches node to run bottom flir camera
    bottom_cam: Node = Node(
        package='spinnaker_camera_driver',
        executable='camera_driver_node',
        emulate_tty=True,
        name='bottom_cam',
        output='screen',
        parameters=[Parameter('serial_number', '23473566'),
                    Parameter('parameter_file', parameter_file)]
    )

    return LaunchDescription([
        front_cam,
        bottom_cam
    ])
