import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('precision_landing')
    custom_fractal_config = os.path.join(pkg_share, 'config', 'custom_fractal.yml')

    # Launch arguments
    device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value='0',
        description='USB Webcam device ID (0 for /dev/video0)'
    )
    enable_morphology_arg = DeclareLaunchArgument(
        'enable_morphology',
        default_value='true',
        description='Enable morphology filter to close gaps in outer marker'
    )
    kernel_size_arg = DeclareLaunchArgument(
        'morphology_kernel_size',
        default_value='7',
        description='Kernel size for morphology closing (e.g. 5, 7, 9, 11)'
    )

    # 1. Webcam Node (uses image_tools cam2image)
    webcam_node = Node(
        package='image_tools',
        executable='cam2image',
        name='laptop_webcam',
        parameters=[{
            'device_id': LaunchConfiguration('device_id'),
            'width': 1280,
            'height': 720,
            'frequency': 30.0,
            'history': 'keep_last',
            'depth': 10,
        }],
        remappings=[
            ('image', '/siyi/image_raw'),
        ],
        output='screen'
    )

    # 2. ArUco Fractal Tracker Node
    tracker_node = Node(
        package='precision_landing',
        executable='aruco_fractal_tracker',
        name='aruco_fractal_tracker',
        parameters=[{
            'marker_configuration': custom_fractal_config,
            'marker_size': 0.5,
            'min_tracking_z': 0.1,
            'max_tracking_z': 20.0,
            'max_pose_jump_m': 2.0,
            'acquire_good_frames': 3,
            'lost_bad_frames': 5,
            'show_latency_overlay': True,
            'enable_morphology': LaunchConfiguration('enable_morphology'),
            'morphology_kernel_size': LaunchConfiguration('morphology_kernel_size'),
            'camera_x_to_body_east_sign': 1.0,
            'camera_y_to_body_north_sign': -1.0,
            'use_sim_time': False,
        }],
        remappings=[
            ('image_input_topic', '/siyi/image_raw'),
            ('camera_info_topic', '/siyi/camera_info'),
            ('image_output_topic', '/siyi/fractal_debug'),
            ('preprocessed_output_topic', '/siyi/preprocessed_image'),
            ('poses_output_topic', '/siyi/fractal_pose'),
            ('target_output_topic', '/siyi/landing_target'),
        ],
        output='screen'
    )

    # 3. Rqt Image View Node
    rqt_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        arguments=['/siyi/fractal_debug'],
        output='screen'
    )

    return LaunchDescription([
        device_id_arg,
        enable_morphology_arg,
        kernel_size_arg,
        webcam_node,
        tracker_node,
        rqt_node,
    ])
