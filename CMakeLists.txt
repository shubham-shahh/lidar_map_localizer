cmake_minimum_required(VERSION 3.5)
project(lidar_map_localizer)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(small_gicp REQUIRED)
find_package(OpenMP REQUIRED)

# Enable OpenMP for voxelgrid_sampling_omp
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
  add_compile_options(${OpenMP_CXX_FLAGS})
  link_libraries(${OpenMP_CXX_LIBRARIES})
endif()

include_directories(
  ${PCL_INCLUDE_DIRS}
)

add_executable(small_gicp_localization_node src/small_gicp_localization_node.cpp)

ament_target_dependencies(small_gicp_localization_node
  rclcpp sensor_msgs nav_msgs pcl_conversions pcl_ros small_gicp
)

target_link_libraries(small_gicp_localization_node
  ${PCL_LIBRARIES}
  small_gicp::small_gicp
)

install(TARGETS small_gicp_localization_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
