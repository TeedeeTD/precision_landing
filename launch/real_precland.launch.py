"""
Launch file: Real Drone Precision Landing Pipeline (Hardware Flight)
Note: MAVROS is run externally (e.g. `ros2 launch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600`)

Pipeline:
  - SIYI A8 Mini RTSP Publisher (GPU Accelerated NVDEC + flip 180 via GStreamer)
  - Fractal ArUco C++ Tracker (Multi-level nested target tracking)
  - Offboard Precision Landing Controller C++ (MAVROS Offboard setpoint control)
  - ImageToRtsp C++ (Zero-latency RTMP/RTSP stream output to MediaMTX)
  - Zero-Copy Intra-Process Communication via ComposableNodeContainer
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('precision_landing')
    rtsp_params_file = os.path.join(pkg_share, 'config', 'rtsp_publisher_params.yaml')
    offboard_params_file = os.path.join(pkg_share, 'config', 'offboard_precland_params.yaml')
    default_marker_config = os.path.join(pkg_share, 'config', 'custom_fractal.yml')

    # ── Launch Arguments ────────────────────────────────────────────

    rtsp_url_arg = DeclareLaunchArgument(
        'rtsp_url',
        default_value='rtsp://127.0.0.1:8554/my_camera',
        description='Input RTSP camera stream URL from MediaMTX or direct camera (e.g. rtsp://192.168.168.16:8554/main.264)'
    )

    flip_180_arg = DeclareLaunchArgument(
        'flip_180',
        default_value='true',
        description='Flip camera image 180 degrees (true if camera is mounted upside down on drone)'
    )

    stream_output_url_arg = DeclareLaunchArgument(
        'stream_output_url',
        default_value='rtmp://127.0.0.1:1935/siyi_aruco',
        description='Output RTMP stream URL pushing annotated debug video to MediaMTX'
    )

    marker_config_arg = DeclareLaunchArgument(
        'marker_configuration',
        default_value=default_marker_config,
        description='Absolute path to fractal marker YAML configuration file'
    )

    # ── Composable Node Container (Zero-Copy Intra-Process IPC) ──

    precland_container = ComposableNodeContainer(
        name='precision_landing_container',
        namespace='',
        package='precision_landing',
        executable='precland_container',
        composable_node_descriptions=[
            # 1. RTSP Camera Publisher (Decodes RTSP stream from camera/MediaMTX)
            ComposableNode(
                package='precision_landing',
                plugin='precision_landing::RtspPublisher',
                name='siyi_rtsp_publisher',
                parameters=[
                    rtsp_params_file,
                    {
                        'rtsp_url': LaunchConfiguration('rtsp_url'),
                        'flip_180': LaunchConfiguration('flip_180'),
                    }
                ],
                extra_arguments=[{'use_intra_process_comm': True}],
            ),

            # 2. ArUco Fractal Tracker (Multi-level target detection & 3D pose estimation)
            ComposableNode(
                package='precision_landing',
                plugin='fractal_tracker::ArucoFractalTracker',
                name='aruco_fractal_tracker',
                parameters=[
                    offboard_params_file,
                    {
                        'marker_configuration': LaunchConfiguration('marker_configuration'),
                        'use_sim_time': False,
                    }
                ],
                remappings=[
                    ('image_input_topic', '/siyi/image_raw'),
                    ('camera_info_topic', '/siyi/camera_info'),
                    ('image_output_topic', '/siyi/fractal_debug'),
                    ('poses_output_topic', '/siyi/fractal_pose'),
                    ('target_output_topic', '/siyi/landing_target'),
                ],
                extra_arguments=[{'use_intra_process_comm': True}],
            ),

            # 3. Offboard Precision Landing Controller (FSM Guidance to PX4 via MAVROS)
            ComposableNode(
                package='precision_landing',
                plugin='precision_landing::OffboardPreclandController',
                name='offboard_precland_controller',
                parameters=[
                    offboard_params_file,
                    {
                        'use_sim_time': False,
                        'target_topic': '/siyi/landing_target',
                        'target_pose_topic': '/siyi/fractal_pose',
                        'align_yaw_to_tag': True,
                    }
                ],
                extra_arguments=[{'use_intra_process_comm': True}],
            ),

            # 4. Image to RTSP Streamer (Pushes HUD annotated video back to MediaMTX via RTMP)
            ComposableNode(
                package='precision_landing',
                plugin='precision_landing::ImageToRtsp',
                name='image_to_rtsp',
                parameters=[{
                    'image_topic': '/siyi/fractal_debug',
                    'rtsp_url': LaunchConfiguration('stream_output_url'),
                    'fps': 25.0
                }],
                extra_arguments=[{'use_intra_process_comm': True}],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        rtsp_url_arg,
        flip_180_arg,
        stream_output_url_arg,
        marker_config_arg,
        precland_container,
    ])


