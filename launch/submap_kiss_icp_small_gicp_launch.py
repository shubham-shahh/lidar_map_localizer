# map_odom_corrector.launch.py

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import launch_ros.actions


def generate_launch_description():
    # Declare launch arguments
    ld = LaunchDescription()

    # Path to the global PCD map
    ld.add_action(DeclareLaunchArgument(
        'map_file',
        default_value='sample_maps/iris_warehouse_map.pcd',
        description='Full path to the global PCD file'))

    # Voxel‐grid leaf sizes for downsampling
    ld.add_action(DeclareLaunchArgument(
        'voxel_leaf_size_local',
        default_value='0.2',
        description='Leaf size (m) to downsample the incoming submap'))
    ld.add_action(DeclareLaunchArgument(
        'voxel_leaf_size_target',
        default_value='0.2',
        description='Leaf size (m) to downsample the cropped global region'))

    # Crop‐box dimensions (meters)
    ld.add_action(DeclareLaunchArgument(
        'crop_size_x',
        default_value='20.0',
        description='X dimension of the crop box (m)'))
    ld.add_action(DeclareLaunchArgument(
        'crop_size_y',
        default_value='20.0',
        description='Y dimension of the crop box (m)'))
    ld.add_action(DeclareLaunchArgument(
        'crop_size_z',
        default_value='5.0',
        description='Z dimension of the crop box (m)'))

    # Small‐GICP parameters
    ld.add_action(DeclareLaunchArgument(
        'map_downsample_voxel',
        default_value='0.1',
        description='Leaf size (m) to downsample the full map at startup'))
    ld.add_action(DeclareLaunchArgument(
        'gicp_num_threads',
        default_value='4',
        description='Number of OpenMP threads for small_gicp'))
    ld.add_action(DeclareLaunchArgument(
        'gicp_max_corr_dist',
        default_value='1.0',
        description='Maximum correspondence distance (m) for GICP'))
    ld.add_action(DeclareLaunchArgument(
        'gicp_corr_randomness',
        default_value='20',
        description='Number of random neighbors for covariance estimation'))

    # Topic names
    ld.add_action(DeclareLaunchArgument(
        'odom_topic',
        default_value='/kiss/odometry',
        description='Input raw odometry topic'))
    ld.add_action(DeclareLaunchArgument(
        'local_map_topic',
        default_value='/kiss/local_map',
        description='Input submap PointCloud2 topic'))
    ld.add_action(DeclareLaunchArgument(
        'corrected_odom_topic',
        default_value='/corrected_odometry',
        description='Output corrected odometry topic'))

    # Frame names
    ld.add_action(DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Robot base frame (child_frame_id in corrected odom)'))
    ld.add_action(DeclareLaunchArgument(
        'map_frame',
        default_value='map',
        description='Map frame id (frame_id in corrected odom)'))

    # Node: map_odom_corrector_small_gicp
    map_odom_corrector_node = launch_ros.actions.Node(
        package='lidar_map_localizer',
        executable='submap_kiss_icp_small_gicp',
        name='map_odom_corrector_small_gicp',
        output='screen',
        parameters=[
            {'map_file': LaunchConfiguration('map_file')},
            {'voxel_leaf_size_local': LaunchConfiguration('voxel_leaf_size_local')},
            {'voxel_leaf_size_target': LaunchConfiguration('voxel_leaf_size_target')},
            {'crop_size_x': LaunchConfiguration('crop_size_x')},
            {'crop_size_y': LaunchConfiguration('crop_size_y')},
            {'crop_size_z': LaunchConfiguration('crop_size_z')},
            {'map_downsample_voxel': LaunchConfiguration('map_downsample_voxel')},
            {'gicp_num_threads': LaunchConfiguration('gicp_num_threads')},
            {'gicp_max_corr_dist': LaunchConfiguration('gicp_max_corr_dist')},
            {'gicp_corr_randomness': LaunchConfiguration('gicp_corr_randomness')},
            {'odom_topic': LaunchConfiguration('odom_topic')},
            {'local_map_topic': LaunchConfiguration('local_map_topic')},
            {'corrected_odom_topic': LaunchConfiguration('corrected_odom_topic')},
            {'base_frame': LaunchConfiguration('base_frame')},
            {'map_frame': LaunchConfiguration('map_frame')}
        ]
    )

    ld.add_action(map_odom_corrector_node)
    return ld
