# lidar_map_localizer


## Installation
To run this package there are a few dependencies

1. follow the steps mentioned here [Ardupilot-ROS2](https://ardupilot.org/dev/docs/ros2-gazebo.html) to install ROS2 Humble, Ardupilot, Gazebo and other essential components
2. install small_gicp with the instructions mentioned in the package here [small_gicp](https://github.com/koide3/small_gicp)
3. Install KISS_SLAM using `pip install kiss-slam`

## Build and Run the Package

1. Build the package
```bash

colcon build --packages-select lidar_map_localizer

```

2. Run the Simulation
```bash

ros2 launch ardupilot_gz_bringup iris_maze.launch.py rviz:=true use_gz_tf:=true lidar_dim:=3

```

3. Run the teleop node
```bash

cd scripts/
python3 teleop_control_node.py 



```

4. Build the map
Move across the map using the teleop node and record the rosbag and process it using KISS_SLAM add the path of generated map to `small_gicp_loclaization_launch.py` file, there is maze map in smaple_maps folder in case you want to skip map building
```bash

ros2 bag record -o scan_bag /cloud_in


```



5. Run the localization node

```bash

ros2 launch lidar_map_localizer localization_launch.py

```
The above node publishes odometry on /generate_pose topic