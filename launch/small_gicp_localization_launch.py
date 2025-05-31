from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_map_localizer',
            executable='small_gicp_localization_node',
            name='map_localizer',
            output='screen',
            parameters=[{
                'map_file': '/home/shubham/Downloads/slam_output/2025-05-25_13-23-53/local_maps/plys/0000.pcd',
                'voxel_size': 0.25,
                'num_threads': 4,
                'max_corr_dist': 1.0,
                'corr_randomness': 20,
                'voxel_resolution': 1.0,
            }]
        )
    ])
