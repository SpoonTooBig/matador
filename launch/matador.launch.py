import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    matador_share_dir = get_package_share_directory('matador')
    
    params_file_path = os.path.join(matador_share_dir, 'config', 'tags_36h11_compressed.yaml')
    
    return LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_image',
            output='screen',
            parameters=[
                params_file_path,
                {'image_transport': 'compressed',
                'use_approximate_sync': True}],
            remappings=[
                ('image_rect', '/image_raw'),
                ('camera_info', '/camera_info'),
            ]
        ),
    ])
