
# GSoC’ 25 Non-GPS Position Estimation Using 3D Camera and Pre-Generated Map - Final
![GSoC’ 25](https://github.com/shubham-shahh/lidar_map_localizer/blob/v_0_3_Aug_2025/assets/cover_image.png)

Hello Ardupilot family!

I have been working on  [GSoC’ 25 Non-GPS Position Estimation Using 3D Camera and Pre-Generated Map - Part 1](https://discuss.ardupilot.org/t/gsoc-25-non-gps-position-estimation-using-3d-camera-and-pre-generated-map-part-1/134712) this summer I am excited to inform the community it was a fun, exciting and challenging project to solve this research problem which can enable accurate localization on board with leveraging existing map below I will go throught the full journey, setup steps, datasets, harware setup and future steps and much more so buckle up for the ride.

## The Journey
I will go through all the things I worked on while working on this problem statement in as much detail as possible to make sure people can extend this work and build some cool applications on top of it

### Simulation

![Simulation](https://github.com/shubham-shahh/lidar_map_localizer/blob/v_0_3_Aug_2025/assets/gazebo_sim_startup.png)

I started this project from simulation as it should be. Before diving right into matching point cloud from a simulated limited FOV 3D camera with the existing map I started working on matching the 3D point cloud from lidar to the 3D map to progressively increase the difficulty levels.

To create the 3D maps in simulation I used [KISS-SLAM](https://github.com/PRBonn/kiss-slam), the main reason to use it over other SLAM algorithms is that it can be used right out of the box without much setup related to calibrations, extrensics or any other details. 

To match the lidar scans with the 3D lidar map I did a literature survey to find the existing frameworks rather than reinventing the wheel and I found couple of cool appraches that are in existance I have mentioned a list below if you're willing to explore them

1) [lidar_localization_ros2](https://github.com/rsasaki0109/lidar_localization_ros2)
2) [icp_localization](https://github.com/leggedrobotics/icp_localization)
3) [KISS-Matcher](https://github.com/MIT-SPARK/KISS-Matcher)
4) [hdl_localization](https://github.com/koide3/hdl_localization?tab=readme-ov-file)
5) [GLIM](https://github.com/koide3/glim)
6) [GLIL](https://koide3.github.io/glil_pubdoc/index.html) (closed source)
7) [FAST_LIO_LOCALIZATION](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION) (FAST_LIO + Map matching)
8) [KISS_ICP](https://github.com/PRBonn/kiss-icp)
9) [FAST-LIO-Localization-QN](https://github.com/engcang/FAST-LIO-Localization-QN) (yet to test, ROS 1 based)
10) [direct_lidar_odometry](https://github.com/vectr-ucla/direct_lidar_odometry?tab=readme-ov-file) (yet to test, ROS 1 based)

To keep my findings concise and limited to the scope of the discussion and not making this post boring I will list the most improtant ones. I implemented each of the above setup in the simulation environmnet to understand and benchmarks before finalizing a approach I can build on top of. all the appraoches have thier own strengths, weaknesses, requirments, environemnt constraints etc so rather than calculating ATE and RTE in simulation, I tried to query the % error in the system in the end of 500m trajectory. No approach was rock solid thoughout the trajectory all of them failed atelast once and drifted away during challanging scenarios (empty corridors,  low geometric features) etc 

| Algorithm              | % error  | Setup |
| :----------------------| :------: | ----: |
| lidar_localization_ros2|   12.4   | Easy  |
| icp_localization       |   25.3   | Medium|
| KISS-Matcher           |   37.2   | Hard  |
| hdl_localization       |   22.4   | Hard  |
| GLIM                   |   15.3   | Hard  |
| GLIL                   |   N/A    | N/A   |
| FAST_LIO_LOCALIZATION  |   39.2   | Easy  |

All the tested algorithms have thier own set of tuning params and some of the above mentioned apporaches do not do relocalization without adding additional components like KISS-Matcher and GLIM so the results might vary based on parameter tunung, setup and compute resourses, contact shubhams@udel.edu to get detailed param list, tuning parameters for specific appraoch. If I am missing out on any approach please add it in the comments I will be more than happy to add it to test it and add the results.

After deriving the results on full size scan to map matching I increased the difficulty by a notch and decreased the horizontal and vertical FOV to match common depth cameras like ZED and realsense. That's where all the above appraoches start falling apart since the amount of overlap needed to derive a meaningful and robust pose estimate becomes much harder.

Thats when I started building our own matcher based on simple foundations and learnings from above benchmarking. the approach works on the following learnings and principles

### Architecture

![Architecture V 1](https://github.com/shubham-shahh/lidar_map_localizer/blob/v_0_3_Aug_2025/assets/data_flow_architecture.gif)

#### Approaches Tried:

1) **NDT based scan to map matching**: Using just NDT to match the current scan to the global map 
    - It works well with a good initital estimate 
    - If there is a good amount of overlap between the sorunce and target scan
    - Rotational accuracy is poor
    - Gets lost easily in corridor/maze/long walls kind of environment
    - much less taxing on compute resources
    - Doesn't work well with unstructured point clouds

2) **ICP/GICP based scan to map matching**: Using just ICP/GICP to match the current scan to the global map
    - More accurate than NDT for most scenarios
    - Robust rotational estimate
    - reaches local minima when matches to a gobal map
    - Requires a lot of compute to match against a full map 

3) **Fusion with IMU**: Fusing IMU with estimates derived from any of the approaches
    - Improves rate and rotational accuracy
    - adds velocity estimates as well as a result of fusion
    - needs additional configuration, calibration, extrensics, well defined TF tree to work well


Based on the learings from benchnarkings and above trials, and to keep the appraoch simple and easy to setup, To make sure that the approach works efficiently, we do a NDT based scan to map search throughout the entire map with a coarse resolution and then dervice a coarse pose estimate and then use that pose estimate to make a configurable size moving ring buffer around that pose in the global map which is a smaller subset of the global map and we do a small_gicp based matching at a finer resolution to derive the final refined pose. The framework optionally takes in high rate intital estimate to derive high accuracy estimate. since the Overlap is considerably smaller complared to a full lidar scan and it is unstrucutred data, the data is published at a lower rate in the form of TF from `map` frame to `odom` frame which can be converted as pose in global frame and passed to Ardupilot EK3 with steps mentioned [here](https://ardupilot.org/copter/docs/common-vio-tracking-camera.html)


### Simulation Result
Here's the simulation result demonstrating robust transaltion and rotation accuracy in empty/ featureless spots

![Simulation Video](https://github.com/shubham-shahh/lidar_map_localizer/blob/v_0_3_Aug_2025/assets/sim_results.gif)



### Real World Tests
I tested the setup in real world on 2 Cameras, **Zed 2i** and **realsense D456**, Both camearas have different properties, point cloud accuracies which are tied to thier baseline and other camera properties. TO get the 3D designs and real world datasets contact me on shubhams@udel.edu because of the size of the datasets and storage limitations I am figuring how can  I share it but meanwhile if someone wants to give it a try, shoot me a email, I will be more than happy to share the designs and datsets

<div align="center">
<table>
<tr>
<td align="center" width="50%">
<img src="https://github.com/shubham-shahh/lidar_map_localizer/blob/v_0_3_Aug_2025/assets/hand_held_front.jpg" alt="Front View" style="width: 100%; height: 400px; object-fit: cover;">
<br><b>Front View</b>
</td>
<td align="center" width="50%">
<img src="https://github.com/shubham-shahh/lidar_map_localizer/blob/v_0_3_Aug_2025/assets/hand_held_side.jpg" alt="Side View" style="width: 100%; height: 400px; object-fit: cover;">
<br><b>Side View</b>
</td>
</tr>
</table>
</div>


I have tiled the camera a little but for this hand held setup to avoid moving people etc but on a drone the desireable angle would be forward facing.

![Real world data Video](https://github.com/shubham-shahh/lidar_map_localizer/blob/v_0_3_Aug_2025/assets/real_world_data.gif)



## Currect challanges and Future work

1) Exploring the learining based matching techniques for robust and rotational accuracies like [BUFFER_X](https://github.com/MIT-SPARK/BUFFER-X)
2) Add additional landmark detection module to add robust error flushing mechanism 
3) Better memory management to work with large scale maps
4) Improve the perforance of the system to improve the frame rate
5) Build a framework to maintain accurate and etremely sparse yet informational submaps to ensure the overlap is not a issue for calculating accurate odometry


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