import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace

# from launch.substitutions import Command

NAMESPACE = "simulation"


def generate_launch_description() -> LaunchDescription:
    rov_gazebo_path: str = get_package_share_directory("rov_gazebo")
    surface_main_path: str = get_package_share_directory("surface_main")

    world_file = "rov24_coral.sdf"
    world_path: str = os.path.join(rov_gazebo_path, "worlds", world_file)

    params_file = "sub.parm"
    params_path: str = os.path.join(rov_gazebo_path, "config", params_file)

    # Process the URDF file
    # xacro_file = os.path.join(rov_gazebo_path, "description", "rov.xacro")
    # robot_description = Command(["xacro ", xacro_file])
    # params = {"robot_description": robot_description}

    # pool_file = os.path.join(rov_gazebo_path, "description", "pool.xacro")
    # pool_description = Command(["xacro ", pool_file])
    # pool_params = {"robot_description": pool_description}

    # # Create a robot_state_publisher node
    # robot_state_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     output="screen",
    #     parameters=[params],
    #     namespace=NS,
    #     emulate_tty=True
    # )

    # pool_state_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     output="screen",
    #     parameters=[pool_params],
    #     namespace=NS,
    #     remappings=[(f"/{NS}/robot_description", f"/{NS}/pool_description")],
    #     emulate_tty=True
    # )

    # # Launches Gazebo
    # gazeboLaunch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [os.path.join(ros_gz_sim_path, "launch", "gz_sim.launch.py")]
    #     ),
    #     launch_arguments={"gz_args": world_path}.items(),
    # )

    # TODO gz_sim launch might be nice
    start_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v', '3', '-r', world_path],
        output='screen'
    )

    start_ardusub = ExecuteProcess(
        cmd=["ardusub", '-S', '-w', '-M', 'JSON', '--defaults', params_path, '-I0', ],
        output='screen'
    )

    # Translate messages MAV <-> ROS
    mav_ros_node = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        namespace='mavros',
        parameters=[
            {"system_id": 255},
            # TODO check if needed
            {"component_id": 240},
            {"fcu_url": "tcp://localhost"},
            {"gcs_url": "udp://@localhost:14550"},
            {"plugin_allowlist": ["rc_io", "sys_status", "command"]}
        ],
        remappings=[
            (f'/{NAMESPACE}/mavros/state', '/tether/mavros/state'),
            (f'/{NAMESPACE}/mavros/rc/override', '/tether/mavros/rc/override'),
            (f'/{NAMESPACE}/mavros/cmd/arming', '/tether/mavros/cmd/arming'),
            (f'/{NAMESPACE}/mavros/cmd/command', '/tether/mavros/cmd/command'),
        ],
        emulate_tty=True
    )

    cam_bridge_node = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/front_cam'],
        output='screen',
    )

    # Not using keyboard launch file
    # TODO?
    # I think we should probably switch over all our single
    # Node launch files to something like this
    keyboard_control_node = Node(
        package="flight_control",
        executable="keyboard_control_node",
        output="screen",
        name="keyboard_control_node",
        remappings=[(f"/{NAMESPACE}/mavros/rc/override", "/tether/mavros/rc/override")],
        emulate_tty=True
    )

    # cam_bridge = Node(
    #     package="ros_gz_bridge",
    #     executable="parameter_bridge",
    #     namespace=NAMESPACE,
    #     name="cam_bridge",
    #     arguments=[
    #         "/bottom_cam/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
    #         "/bottom_cam/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
    #         "/front_cam/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
    #         "/front_cam/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
    #         "/manip_cam/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
    #         "/depth_cam@sensor_msgs/msg/Image@gz.msgs.Image",
    #         "/depth_cam/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
    #     ],
    #     remappings=[
    #         (f"/{NAMESPACE}/bottom_cam/image_raw", "/bottom_cam/image_raw"),
    #         (f"/{NAMESPACE}/bottom_cam/camera_info", "/bottom_cam/camera_info"),
    #         (f"/{NAMESPACE}/front_cam/image_raw", "/front_cam/image_raw"),
    #         (f"/{NAMESPACE}/front_cam/camera_info", "/front_cam/camera_info"),
    #         (f"/{NAMESPACE}/manip_cam/image_raw", "/manip_cam/image_raw"),
    #         (f"/{NAMESPACE}/depth_cam", "/depth_cam/image_raw"),
    #         (f"/{NAMESPACE}/depth_cam/points", "/depth_cam/points"),
    #     ],
    #     output="screen",
    #     emulate_tty=True
    # )

    # Launches the pi heartbeat node
    heartbeat_node = Node(
        package="heartbeat",
        executable="heartbeat_node",
        remappings=[(f"/{NAMESPACE}/pi_heartbeat", "/tether/pi_heartbeat")],
        emulate_tty=True,
        output="screen",

    )

    # Launches Surface Nodes
    surface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(surface_main_path, "launch", "surface_all_nodes_launch.py")]
        ),
    )

    namespace_launch = GroupAction(
        actions=[
            PushRosNamespace(NAMESPACE),
            mav_ros_node,
            heartbeat_node,
            cam_bridge_node,
            keyboard_control_node,
        ]
    )

    return LaunchDescription(
        [
            # robot_state_publisher,
            # pool_state_publisher,
            # gazeboLaunch,
            start_gazebo,
            # start_ardusub,
            namespace_launch,
            # gz_spawn_entity,
            # gz_spawn_pool,
            # thrust_bridge,
            # cam_bridge,
            surface_launch,
        ]
    )
