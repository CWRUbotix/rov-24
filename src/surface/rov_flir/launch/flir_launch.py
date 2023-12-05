from launch.launch_description import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import Parameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    parameter_file = PathJoinSubstitution(
            [FindPackageShare('spinnaker_camera_driver'), 'config',
             'blackfly_s' + '.yaml'])

    parameters = {'debug': False,
                  'compute_brightness': False,
                  'adjust_timestamp': True,
                  'dump_node_map': False,
                  # set parameters defined in blackfly_s.yaml
                  'gain_auto': 'Continuous',
                  # 'pixel_format': 'BayerRG8',
                  'exposure_auto': 'Continuous',
                  # These are useful for GigE cameras
                  'device_link_throughput_limit': 380000000,
                  'gev_scps_packet_size': 9000,
                  # ---- to reduce the sensor width and shift the crop
                  # 'image_width': 1408,
                  # 'image_height': 1080,
                  # 'offset_x': 16,
                  # 'offset_y': 0,
                  'frame_rate_auto': 'Off',
                  #  'frame_rate': 40.0,
                  'frame_rate_enable': True,
                  'buffer_queue_size': 1,
                  'trigger_mode': 'Off',
                  'chunk_mode_active': True,
                  'chunk_selector_frame_id': 'FrameID',
                  'chunk_enable_frame_id': True,
                  'chunk_selector_exposure_time': 'ExposureTime',
                  'chunk_enable_exposure_time': True,
                  'chunk_selector_gain': 'Gain',
                  'chunk_enable_gain': True,
                  'chunk_selector_timestamp': 'Timestamp',
                  'chunk_enable_timestamp': True}

    # launches node to run front flir camera
    front_cam: Node = Node(
        package='spinnaker_camera_driver',
        executable='camera_driver_node',
        emulate_tty=True,
        output='screen',
        parameters=[Parameter('serial_number', '23473577'),
                    Parameter('binning', '2'),
                    Parameter('parameter_file', parameter_file),
                    parameters]
    )

    # launches node to run bottom flir camera
    bottom_cam: Node = Node(
        package='spinnaker_camera_driver',
        executable='camera_driver_node',
        emulate_tty=True,
        output='screen',
        parameters=[Parameter('serial_number', '23473566'),
                    Parameter('binning', '2'),
                    Parameter('parameter_file', parameter_file),
                    parameters]
    )

    return LaunchDescription([
        front_cam,
        bottom_cam
    ])
