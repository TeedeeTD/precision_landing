import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('precision_landing')

    # Path to modular configuration files in config/module/
    module_config_dir = os.path.join(pkg_share, 'config', 'module')
    vehicle_params_file = os.path.join(module_config_dir, 'vehicle_params.yaml')
    target_tracker_params_file = os.path.join(module_config_dir, 'target_tracker_params.yaml')
    guidance_params_file = os.path.join(module_config_dir, 'guidance_params.yaml')
    fsm_params_file = os.path.join(module_config_dir, 'fsm_params.yaml')
    tracker_node_params_file = os.path.join(module_config_dir, 'tracker_node_params.yaml')

    image_bridge_node = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/world/fractal_aruco_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/image'],
        remappings=[
            ('/world/fractal_aruco_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/image', '/gimbal_camera')
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    clock_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    camera_info_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/fractal_aruco_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        remappings=[
            ('/world/fractal_aruco_landing/model/x500_gimbal_0/link/camera_link/sensor/camera/camera_info', '/gimbal_camera/camera_info')
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    tracker_node = Node(
        package='precision_landing',
        executable='aruco_fractal_tracker',
        parameters=[
            tracker_node_params_file,
            target_tracker_params_file,
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

    controller_node = Node(
        package='precision_landing',
        executable='offboard_precland_controller',
        parameters=[
            vehicle_params_file,
            target_tracker_params_file,
            guidance_params_file,
            fsm_params_file,
            {
                'use_sim_time': True,
                'align_yaw_to_tag': True
            }
        ],
        output='screen'
    )

    return LaunchDescription([
        image_bridge_node,
        clock_bridge_node,
        camera_info_bridge_node,
        tracker_node,
        controller_node
    ])
