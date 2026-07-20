import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('precision_landing')
    offboard_params_file = os.path.join(pkg_share, 'config', 'offboard_precland_params.yaml')

    # Node giải nén ảnh (compressed -> raw) cục bộ trên Jetson để tiết kiệm băng thông mạng Wifi
    decompress_node = Node(
        package='image_transport',
        executable='republish',
        name='image_decompressor',
        arguments=['compressed', 'raw'],
        remappings=[
            ('in/compressed', '/gimbal_camera/compressed'),
            ('out', '/gimbal_camera')
        ],
        parameters=[{
            'use_sim_time': True
        }],
        output='screen'
    )

    # Node dò tìm mục tiêu hạ cánh chạy trên Jetson (HITL)
    tracker_node = Node(
        package='precision_landing',
        executable='aruco_fractal_tracker',
        name='aruco_fractal_tracker',
        parameters=[
            offboard_params_file,
            {
                'marker_configuration': os.path.join(
                    pkg_share,
                    'config',
                    'custom_fractal.yml'
                ),
                'use_sim_time': True,
            }
        ],
        remappings=[
            ('image_input_topic', '/gimbal_camera'),
            ('camera_info_topic', '/gimbal_camera/camera_info'),
            ('image_output_topic', '/landing/annotated_image'),
            ('poses_output_topic', '/aruco_fractal_tracker/poses'),
            ('target_output_topic', '/landing/target_camera')
        ],
        output='screen'
    )

    # Node điều khiển UAV bám đuổi & hạ cánh chính xác chạy trên Jetson (HITL)
    controller_node = Node(
        package='precision_landing',
        executable='offboard_precland_controller',
        name='offboard_precland_controller',
        parameters=[
            offboard_params_file,
            {
                'use_sim_time': True,
                'align_yaw_to_tag': True
            }
        ],
        output='screen'
    )

    # Node nén ảnh debug kết quả nhận diện (raw -> compressed) cục bộ trên Jetson để gửi về PC xem mượt mà qua Wifi
    debug_compress_node = Node(
        package='image_transport',
        executable='republish',
        name='debug_image_compressor',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in', '/landing/annotated_image'),
            ('out/compressed', '/landing/annotated_image/compressed')
        ],
        parameters=[{
            'use_sim_time': True
        }],
        output='screen'
    )

    return LaunchDescription([
        decompress_node,
        tracker_node,
        controller_node,
        debug_compress_node
    ])
