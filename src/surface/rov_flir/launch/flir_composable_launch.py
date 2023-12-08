import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction
from launch.launch_description import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import Parameter
from launch.launch_context import LaunchContext

parameters = {'debug': False,
              'compute_brightness': False,
              'adjust_timestamp': True,
              'dump_node_map': False,
              # set parameters defined in blackfly_s.yaml
              'gain_auto': 'Continuous',
              # 'pixel_format': 'BayerRG8',
              'exposure_auto': 'Continuous',
              # These are useful for GigE cameras
              'device_link_throughput_limit': 125000000,
              'gev_scps_packet_size': 9000,
              # ---- to reduce the sensor width and shift the crop
              #   'image_width': 1280,
              #   'image_height': 720,
              # 'offset_x': 16,
              # 'offset_y': 0,
              'binning_x': 2,
              'binning_y': 2,
              'frame_rate_auto': 'Off',
              'frame_rate': 60.0,
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


parameter_file = os.path.join(get_package_share_directory('rov_flir'), 'config', 'blackfly_s.yaml')


def make_camera_node(name: str, serial: str) -> ComposableNode:
    """
    Generate Camera Node.

    Parameters
    ----------
    name : str
        Name of the camera.
    serial : str
        Serial number of the camera.

    Returns
    -------
    ComposableNode
        The composable camera node.
    """

    return ComposableNode(
        package='spinnaker_camera_driver',
        plugin='spinnaker_camera_driver::CameraDriver',
        name=name,
        extra_arguments=[{'use_intra_process_comms': True}],
        # parameters=[Parameter('serial_number', serial),
        #             Parameter('parameter_file', parameter_file),
        #             parameters]
        parameters=[{'serial_number': serial},
                    {'parameter_file': parameter_file},
                    parameters]
    )


def launch_setup(_: LaunchContext) -> list[ComposableNodeContainer]:
    """
    Launch setup for rov flir launch.
    Returns
    -------
    list[ComposableNodeContainer]
        List of composed nodes.
    """

    container = ComposableNodeContainer(
        name='flir_cameras',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            make_camera_node('front_camera', '23473577'),
            make_camera_node('bottom_camera', '23473566')
        ],
        output='screen',
        emulate_tty=True
    )

    return [container]


def generate_launch_description() -> LaunchDescription:
    """
    Generates LaunchDescription.

    Returns
    -------
    LaunchDescription
        Flir launch Description.
    """
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
