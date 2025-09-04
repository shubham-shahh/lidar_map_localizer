
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # === Map File ===
    ld.add_action(DeclareLaunchArgument(
        'map_file',
        default_value='/home/shubham/ardu_ws/src/lidar_map_localizer/sample_maps/iris_warehouse_map.pcd',
        description='Full path to the global PCD file'))

    # === NDT Parameters (Coarse Alignment) ===
    ld.add_action(DeclareLaunchArgument(
        'ndt_resolution',
        default_value='2.0',
        description='NDT grid resolution for coarse alignment (larger = faster but less accurate)'))
    ld.add_action(DeclareLaunchArgument(
        'ndt_step_size',
        default_value='0.1',
        description='NDT optimization step size'))
    ld.add_action(DeclareLaunchArgument(
        'ndt_epsilon',
        default_value='0.01',
        description='NDT transformation epsilon'))
    ld.add_action(DeclareLaunchArgument(
        'ndt_max_iter',
        default_value='30',
        description='Maximum NDT iterations'))
    ld.add_action(DeclareLaunchArgument(
        'ndt_voxel_leaf_size',
        default_value='0.5',
        description='Voxel size for NDT map/scan downsampling (coarse)'))
    ld.add_action(DeclareLaunchArgument(
        'ndt_fitness_threshold',
        default_value='1.0',
        description='Maximum acceptable NDT fitness score'))

    # === GICP Parameters (Fine Alignment) ===
    ld.add_action(DeclareLaunchArgument(
        'voxel_leaf_size_local',
        default_value='0.2',
        description='Leaf size (m) to downsample the incoming submap for GICP'))
    ld.add_action(DeclareLaunchArgument(
        'voxel_leaf_size_target',
        default_value='0.2',
        description='Leaf size (m) to downsample the cropped global region for GICP'))
    ld.add_action(DeclareLaunchArgument(
        'map_downsample_voxel',
        default_value='0.1',
        description='Leaf size (m) for initial map preprocessing'))
    ld.add_action(DeclareLaunchArgument(
        'gicp_num_threads',
        default_value='4',
        description='Number of OpenMP threads for small_gicp'))
    ld.add_action(DeclareLaunchArgument(
        'gicp_max_corr_dist',
        default_value='3.0',
        description='Maximum correspondence distance (m) for GICP'))
    ld.add_action(DeclareLaunchArgument(
        'gicp_corr_randomness',
        default_value='20',
        description='Number of random neighbors for GICP covariance estimation'))
    ld.add_action(DeclareLaunchArgument(
        'gicp_fitness_threshold',
        default_value='0.3',
        description='Maximum acceptable GICP error'))

    # === Crop Box Parameters ===
    ld.add_action(DeclareLaunchArgument(
        'crop_size_x',
        default_value='30.0',
        description='X dimension of the crop box around NDT pose (m)'))
    ld.add_action(DeclareLaunchArgument(
        'crop_size_y',
        default_value='30.0',
        description='Y dimension of the crop box around NDT pose (m)'))
    ld.add_action(DeclareLaunchArgument(
        'crop_size_z',
        default_value='10.0',
        description='Z dimension of the crop box around NDT pose (m)'))

    # === Topic Names ===
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

    # === Frame Names ===
    ld.add_action(DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Robot base frame (child_frame_id in corrected odom)'))
    ld.add_action(DeclareLaunchArgument(
        'map_frame',
        default_value='odom_lidar',
        description='Map frame id (frame_id in corrected odom)'))

    # === Initial Guess Configuration ===
    ld.add_action(DeclareLaunchArgument(
        'use_odom_as_initial_guess',
        default_value='true',
        description='Use odometry for initial guess (true) or use fixed initial pose (false)'))
    ld.add_action(DeclareLaunchArgument(
        'initial_x',
        default_value='0.0',
        description='Initial X position when use_odom_as_initial_guess is false'))
    ld.add_action(DeclareLaunchArgument(
        'initial_y',
        default_value='0.0',
        description='Initial Y position when use_odom_as_initial_guess is false'))
    ld.add_action(DeclareLaunchArgument(
        'initial_z',
        default_value='0.0',
        description='Initial Z position when use_odom_as_initial_guess is false'))
    ld.add_action(DeclareLaunchArgument(
        'initial_roll',
        default_value='0.0',
        description='Initial roll angle (radians) when use_odom_as_initial_guess is false'))
    ld.add_action(DeclareLaunchArgument(
        'initial_pitch',
        default_value='0.0',
        description='Initial pitch angle (radians) when use_odom_as_initial_guess is false'))
    ld.add_action(DeclareLaunchArgument(
        'initial_yaw',
        default_value='0.0',
        description='Initial yaw angle (radians) when use_odom_as_initial_guess is false'))

    # === Additional Options ===
    ld.add_action(DeclareLaunchArgument(
        'publish_pose_with_covariance',
        default_value='false',
        description='Also publish PoseWithCovarianceStamped message'))

    # === Node Definition ===
    hybrid_localizer_node = Node(
        package='lidar_map_localizer',
        executable='ndt_gicp_localization',
        name='hybrid_ndt_gicp_localization',
        output='screen',
        parameters=[
            # Map file
            {'map_file': LaunchConfiguration('map_file')},
            
            # NDT parameters
            {'ndt_resolution': LaunchConfiguration('ndt_resolution')},
            {'ndt_step_size': LaunchConfiguration('ndt_step_size')},
            {'ndt_epsilon': LaunchConfiguration('ndt_epsilon')},
            {'ndt_max_iter': LaunchConfiguration('ndt_max_iter')},
            {'ndt_voxel_leaf_size': LaunchConfiguration('ndt_voxel_leaf_size')},
            {'ndt_fitness_threshold': LaunchConfiguration('ndt_fitness_threshold')},
            
            # GICP parameters
            {'voxel_leaf_size_local': LaunchConfiguration('voxel_leaf_size_local')},
            {'voxel_leaf_size_target': LaunchConfiguration('voxel_leaf_size_target')},
            {'map_downsample_voxel': LaunchConfiguration('map_downsample_voxel')},
            {'gicp_num_threads': LaunchConfiguration('gicp_num_threads')},
            {'gicp_max_corr_dist': LaunchConfiguration('gicp_max_corr_dist')},
            {'gicp_corr_randomness': LaunchConfiguration('gicp_corr_randomness')},
            {'gicp_fitness_threshold': LaunchConfiguration('gicp_fitness_threshold')},
            
            # Crop box parameters
            {'crop_size_x': LaunchConfiguration('crop_size_x')},
            {'crop_size_y': LaunchConfiguration('crop_size_y')},
            {'crop_size_z': LaunchConfiguration('crop_size_z')},
            
            # Topics and frames
            {'odom_topic': LaunchConfiguration('odom_topic')},
            {'local_map_topic': LaunchConfiguration('local_map_topic')},
            {'corrected_odom_topic': LaunchConfiguration('corrected_odom_topic')},
            {'base_frame': LaunchConfiguration('base_frame')},
            {'map_frame': LaunchConfiguration('map_frame')},
            
            # Initial guess configuration
            {'use_odom_as_initial_guess': LaunchConfiguration('use_odom_as_initial_guess')},
            {'initial_x': LaunchConfiguration('initial_x')},
            {'initial_y': LaunchConfiguration('initial_y')},
            {'initial_z': LaunchConfiguration('initial_z')},
            {'initial_roll': LaunchConfiguration('initial_roll')},
            {'initial_pitch': LaunchConfiguration('initial_pitch')},
            {'initial_yaw': LaunchConfiguration('initial_yaw')},
            
            # Additional options
            {'publish_pose_with_covariance': LaunchConfiguration('publish_pose_with_covariance')}
        ]
    )

    ld.add_action(hybrid_localizer_node)
    return ld