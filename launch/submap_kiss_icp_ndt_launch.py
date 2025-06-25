from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_file', default_value='/home/shubham/ardu_ws/src/lidar_map_localizer/sample_maps/iris_warehouse_map.pcd',
                               description='Full path to the global map PCD'),
        DeclareLaunchArgument('map_leaf_size', default_value='0.2',
                               description='Voxel leaf size for global map downsampling'),
        DeclareLaunchArgument('scan_leaf_size', default_value='0.2',
                               description='Voxel leaf size for local scan downsampling'),
        DeclareLaunchArgument('ndt_resolution', default_value='1.0',
                               description='Resolution of NDT grid'),
        DeclareLaunchArgument('ndt_step_size', default_value='0.01',
                               description='Step size for NDT optimization'),
        DeclareLaunchArgument('ndt_epsilon', default_value='0.01',
                               description='Transformation epsilon for NDT'),
        DeclareLaunchArgument('ndt_max_iter', default_value='30',
                               description='Max iterations for NDT'),
        Node(
            package='lidar_map_localizer',
            executable='submap_kiss_icp_ndt',
            name='map_odom_corrector',
            output='screen',
            parameters=[
                {'map_file': LaunchConfiguration('map_file')},
                {'map_leaf_size': LaunchConfiguration('map_leaf_size')},
                {'scan_leaf_size': LaunchConfiguration('scan_leaf_size')},
                {'ndt_resolution': LaunchConfiguration('ndt_resolution')},
                {'ndt_step_size': LaunchConfiguration('ndt_step_size')},
                {'ndt_epsilon': LaunchConfiguration('ndt_epsilon')},
                {'ndt_max_iter': LaunchConfiguration('ndt_max_iter')},
                {'odom_topic': '/kiss/odometry'},
                {'local_map_topic': '/kiss/local_map'},
                {'corrected_topic': '/odometry_corrected'}
            ]
        )
    ])