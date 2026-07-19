"""
Launch file: Real SIYI A8 Mini + Fractal ArUco Tracker + Offboard Precision Landing Controller + MAVROS (All C++)

Pipeline:
  1. MAVROS — connects to FCU (optional, can skip if no FCU connected)
  2. Composable Node Container — Runs RtspPublisher, ArucoFractalTracker, and OffboardPreclandController in a single process with Zero-copy IPC.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def _launch_bool(value: str) -> bool:
    return value.lower() in ('1', 'true', 'yes', 'on')


def _maybe_start_mavros(context):
    if not _launch_bool(LaunchConfiguration('enable_mavros').perform(context)):
        return []

    mavros_dir = get_package_share_directory('mavros')
    return [
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                os.path.join(mavros_dir, 'launch', 'px4.launch')
            ),
            launch_arguments={
                'fcu_url': LaunchConfiguration('fcu_url'),
            }.items(),
        )
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('precision_landing')
    rtsp_params_file = os.path.join(pkg_share, 'config', 'rtsp_publisher_params.yaml')
    offboard_params_file = os.path.join(pkg_share, 'config', 'offboard_precland_params.yaml')

    # ── Launch Arguments ────────────────────────────────────────────

    enable_mavros_arg = DeclareLaunchArgument(
        'enable_mavros',
        default_value='true',
        description='Enable MAVROS node (set false if no FCU connected)'
    )

    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://:14540@127.0.0.1:14580',
        description='MAVROS FCU URL (e.g. /dev/ttyACM0:57600 for USB Pixhawk)'
    )

    # ── 1. MAVROS (optional) ────────────────────────────────────────

    mavros_launch = OpaqueFunction(function=_maybe_start_mavros)

    # ── 2. Composable Node Container (Intra-Process Communication) ──

    precland_container = ComposableNodeContainer(
        name='precision_landing_container',
        namespace='',
        package='precision_landing',
        executable='precland_container',
        composable_node_descriptions=[
            ComposableNode(
                package='precision_landing',
                plugin='precision_landing::RtspPublisher',
                name='siyi_rtsp_publisher',
                parameters=[rtsp_params_file],
                extra_arguments=[{'use_intra_process_comm': True}],
            ),
            ComposableNode(
                package='precision_landing',
                plugin='fractal_tracker::ArucoFractalTracker',
                name='aruco_fractal_tracker',
                parameters=[
                    offboard_params_file,
                    {
                        'marker_configuration': os.path.join(
                            get_package_share_directory('precision_landing'),
                            'config',
                            'custom_fractal.yml'
                        ),
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
        ],
        output='screen',
    )

    return LaunchDescription([
        enable_mavros_arg,
        fcu_url_arg,
        mavros_launch,
        precland_container,
    ])
