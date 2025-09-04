

# GSoC '25 Non-GPS Position Estimation Using 3D Camera and Pre-Generated Map - Final

![GSoC '25](https://github.com/shubham-shahh/lidar_map_localizer/blob/v_0_3_Aug_2025/assets/cover_image.png)



Hello Ardupilot family!



I’ve been working on [GSoC '25 Non-GPS Position Estimation Using 3D Camera and Pre-Generated Map - Part 1](https://discuss.ardupilot.org/t/gsoc-25-non-gps-position-estimation-using-3d-camera-and-pre-generated-map-part-1/134712)

 this summer, and I’m excited to share that it’s been a fun, challenging project—one that enables accurate onboard localization by leveraging a pre-generated map. Below, I walk through the full journey: setup steps, datasets, hardware, future work, and more. Buckle up!



## The Journey

I will go through all the things I worked on while working on this problem statement in as much detail as possible to make sure people can extend this work and build some cool applications on top of it



I’ll cover everything I built and tested while tackling this problem statement, in as much detail as possible, enabling others to extend it and build new applications on top of it.



### Simulation



![Simulation](https://github.com/shubham-shahh/lidar_map_localizer/blob/v_0_3_Aug_2025/assets/gazebo_sim_startup.png)



I began in simulation. Before attempting limited-FOV 3D-camera–to-map matching, I first tackled 3D LiDAR scan-to-map and incrementally raised the difficulty. The maze world’s repetitive geometry was challenging, so I transitioned to an industrial-warehouse environment to uncover the issues most relevant to real deployments.



To create 3D maps in simulation, I used [KISS-SLAM](https://github.com/PRBonn/kiss-slam). The main reason: it works out of the box without heavy setup for calibrations, extrinsics, or other details.



To match LiDAR scans to the 3D LiDAR map, I surveyed existing frameworks rather than reinventing the wheel. Here are several solid approaches I explored:



1) [icp_localization](https://github.com/leggedrobotics/icp_localization)

2) [lidar_localization_ros2](https://github.com/rsasaki0109/lidar_localization_ros2)

3) [KISS-Matcher](https://github.com/MIT-SPARK/KISS-Matcher)

4) [hdl_localization](https://github.com/koide3/hdl_localization?tab=readme-ov-file)

5) [GLIM](https://github.com/koide3/glim)

6) [GLIL](https://koide3.github.io/glil_pubdoc/index.html) (closed source)

7) [FAST_LIO_LOCALIZATION](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION) (FAST_LIO + Map matching)

8) [KISS_ICP](https://github.com/PRBonn/kiss-icp)

9) [FAST-LIO-Localization-QN](https://github.com/engcang/FAST-LIO-Localization-QN) (yet to test, ROS 1 based)

10) [direct_lidar_odometry](https://github.com/vectr-ucla/direct_lidar_odometry?tab=readme-ov-file) (yet to test, ROS 1 based)



To keep things concise and on-topic, I’ll highlight the most important observations. I implemented each approach in simulation to understand behavior and rough benchmarks before choosing a direction to build on. Every approach has its own strengths, weaknesses, requirements, and environment constraints. Instead of reporting ATE/RTE in sim, I queried % error over a ~500 m trajectory to keep things simple. None of the approaches were rock solid end-to-end; each failed at least once and drifted during challenging scenarios (empty corridors, low geometric features, etc.).


| Algorithm              | % error  | Setup |
| :----------------------| :------: | ----: |
| lidar_localization_ros2|   12.4   | Easy  |
| icp_localization       |   25.3   | Medium|
| KISS-Matcher           |   37.2   | Hard  |
| hdl_localization       |   22.4   | Hard  |
| GLIM                   |   15.3   | Hard  |
| GLIL                   |   N/A    | N/A   |
| FAST_LIO_LOCALIZATION  |   39.2   | Easy  |


All the tested algorithms have their own set of tuning parameters, and some of the above-mentioned approaches do not perform relocalization without adding additional components (like KISS-Matcher and GLIM), so the results might vary based on parameter tuning, setup, and compute resources. Please contact [shubhams@udel.edu](shubhams@udel.edu) to get a detailed parameter list and tuning parameters for specific approaches. If I am missing any approach, please add it in the comments. I will be more than happy to test it and add the results.



After deriving the results on full-size scan-to-map matching, I increased the difficulty by a notch and decreased the horizontal and vertical FOV to match common depth cameras like ZED and RealSense. That's where all the above approaches start falling apart, since the amount of overlap needed to derive a meaningful and robust pose estimate becomes much harder.



That's when I started building our own matcher based on simple foundations and learnings from the above benchmarking. The approach works on the following learnings and principles:



### Architecture



![Architecture V 1](https://github.com/shubham-shahh/lidar_map_localizer/blob/v_0_3_Aug_2025/assets/data_flow_architecture.gif)



#### Approaches Tried:



1) **NDT based scan to map matching**: Using just NDT to match the current scan to the global map 

    - Works well with a good initial estimate 

    - Requires a good amount of overlap between the source and target scan

    - Rotational accuracy is poor

    - Gets lost easily in corridor/maze/long walls kind of environment

    - much less taxing on compute resources

    - Doesn't work well with unstructured point clouds



2) **ICP/GICP based scan to map matching**: Using just ICP/GICP to match the current scan to the global map

    - Generally more accurate than NDT for most scenarios

    - Robust rotational estimate

    - Prone to local minima against a full map

    - Requires a lot of compute to match against a full map 



3) **Fusion with IMU**: Fusing IMU with estimates derived from any of the approaches

    - Improves rate and rotational accuracy

    - adds velocity estimates as well as a result of fusion

    - needs additional configuration, calibration, extrensics, well-defined TF tree to work well





Based on the learnings from benchmarking and the above trials, and to keep the approach simple and easy to set up, I designed a hybrid solution. To ensure that the approach works efficiently, we perform an NDT-based scan-to-map search throughout the entire map with a coarse resolution to derive a coarse pose estimate. We then use that pose estimate to create a configurable-size moving ring buffer around that pose in the global map, which is a smaller subset of the global map. We perform small_gicp-based matching at a finer resolution to derive the final refined pose. The framework optionally takes in high-rate initial estimates to derive high-accuracy estimates. Since the overlap is considerably smaller compared to a full lidar scan and the data is unstructured, the data is published at a lower rate in the form of TF from map frame to odom frame, which can be converted as pose in the global frame and passed to ArduPilot EK3 with the steps mentioned  [here](https://ardupilot.org/copter/docs/common-vio-tracking-camera.html)





### Simulation Result

Here's the simulation result demonstrating robust translation and rotation accuracy even in empty/ featureless spots



![Simulation Video](https://github.com/shubham-shahh/lidar_map_localizer/blob/v_0_3_Aug_2025/assets/sim_results.gif)







### Real World Tests

I tested the setup in the real world on two cameras: **ZED 2i** and **RealSense D456**. Both cameras have different properties and point cloud accuracies, which are tied to their baseline and other camera properties. To get the 3D designs and real-world datasets, contact me at shubhams@udel.edu. Because of the size of the datasets and storage limitations, I am figuring out how I can share them, but meanwhile, if someone wants to give it a try, shoot me an email—I will be more than happy to share the designs and datasets.



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



I have tilted the camera slightly for this handheld setup to avoid capturing moving people, etc., but on a drone, the desirable angle would be forward-facing.



![Real world data Video](https://github.com/shubham-shahh/lidar_map_localizer/blob/v_0_3_Aug_2025/assets/real_world_data.gif)



## Current Challenges and Future Work



1) Exploring learning-based matching techniques for robust translational and rotational accuracies, like[BUFFER_X](https://github.com/MIT-SPARK/BUFFER-X)

2) Adding an additional landmark detection module to provide a robust error-flushing mechanism

3) Better memory management to work with large-scale maps

4) Improve the performance of the system to improve the frame rate

5) Building a framework to maintain accurate and extremely sparse yet informative submaps to ensure overlap is not an issue for calculating accurate odometry

6) Add Aruco-based initialization to support initialization from anywhere



## Gratitude to the ArduPilot Community

Heartfelt thanks to the pillars of this project: @rmackay9, @snktshrma, and [Ryan Friedman](https://www.linkedin.com/in/ryan-friedman-61a75ba6/?utm_source=chatgpt.com). I am deeply grateful for your guidance and encouragement. From early brainstorming to thoughtful suggestions and hands-on help at every stage, your support made this work possible.







## Installation

To run this package, there are a few dependencies



1. follow the steps mentioned here [Ardupilot-ROS2](https://ardupilot.org/dev/docs/ros2-gazebo.html) to install ROS2 Humble, Ardupilot, Gazebo and other essential components

2. Install small_gicp with the instructions mentioned in the package here [small_gicp](https://github.com/koide3/small_gicp)

3. Install KISS_SLAM using `pip install kiss-slam`



## Build and Run the Package



prereq: Build and install [small_gicp](https://github.com/koide3/small_gicp) and [kiss_icp](https://github.com/PRBonn/kiss-icp)



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

Move across the map using the teleop node and record the rosbag and process it using KISS_SLAM. There are some maps in sample_maps folder in case you want to skip map building

```bash



ros2 bag record -o scan_bag /cloud_in



```





5. Run the localization node for high-rate odom (you can use RTAB map/ KISS_ICP depending on your setup)



Note: Kiss_icp might not run on sparse maps directly. Modify [this line](https://github.com/PRBonn/kiss-icp/blob/8a5597bca71daaa6fe4fd02bf7ffef13578a3c92/cpp/kiss_icp/pipeline/KissICP.cpp#L74) to ` return {frame_downsample, frame_downsample};`



```bash



ros2 launch kiss_icp odometry.launch.py topic:=/cloud_i



```



5. Run the lidar_map_localizer node 



```bash



ros2 launch lidar_map_localizer ndt_gicp_localization_launch.py



```