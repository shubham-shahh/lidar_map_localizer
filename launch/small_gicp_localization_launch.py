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
                'map_file': 'sample_maps/maze.pcd',
                'voxel_size': 0.25,
                'num_threads': 4,
                'max_corr_dist': 1.0,
                'corr_randomness': 20,
                'voxel_resolution': 1.0,
            }]
        )
    ])
