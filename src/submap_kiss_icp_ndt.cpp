// map_odom_corrector_node.cpp
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>

using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;

class MapOdomCorrector : public rclcpp::Node
{
public:
  MapOdomCorrector(const rclcpp::NodeOptions & options)
  : Node("map_odom_corrector", options)
  {
    // Parameters
    this->declare_parameter<std::string>("map_file", "");
    this->declare_parameter<double>("map_leaf_size", 0.2);
    this->declare_parameter<double>("scan_leaf_size", 0.2);
    this->declare_parameter<double>("ndt_resolution", 1.0);
    this->declare_parameter<double>("ndt_step_size", 0.01);
    this->declare_parameter<double>("ndt_epsilon", 0.01);
    this->declare_parameter<int>("ndt_max_iter", 30);
    this->declare_parameter<std::string>("odom_topic", "/kiss/odometry");
    this->declare_parameter<std::string>("local_map_topic", "/kiss/local_map");
    this->declare_parameter<std::string>("corrected_topic", "/kiss/odometry_corrected");

    // Get params
    std::string map_path = this->get_parameter("map_file").as_string();
    double map_leaf = this->get_parameter("map_leaf_size").as_double();
    double scan_leaf = this->get_parameter("scan_leaf_size").as_double();
    double res = this->get_parameter("ndt_resolution").as_double();
    double step = this->get_parameter("ndt_step_size").as_double();
    double eps = this->get_parameter("ndt_epsilon").as_double();
    int max_iter = this->get_parameter("ndt_max_iter").as_int();

    // Load global map
    PointCloud::Ptr global_map(new PointCloud);
    if (pcl::io::loadPCDFile<PointT>(map_path, *global_map) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load map file: %s", map_path.c_str());
      return;
    }
    // Remove NaNs
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*global_map, *global_map, indices);
    // Downsample
    pcl::VoxelGrid<PointT> vg_map;
    vg_map.setInputCloud(global_map);
    vg_map.setLeafSize(map_leaf, map_leaf, map_leaf);
    filtered_map_ = PointCloud::Ptr(new PointCloud);
    vg_map.filter(*filtered_map_);

    // Setup NDT
    ndt_.setInputTarget(filtered_map_);
    ndt_.setResolution(res);
    ndt_.setStepSize(step);
    ndt_.setTransformationEpsilon(eps);
    ndt_.setMaximumIterations(max_iter);

    // Publishers & Subscribers
    corrected_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      this->get_parameter("corrected_topic").as_string(), 10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      this->get_parameter("odom_topic").as_string(), 10,
      std::bind(&MapOdomCorrector::odomCallback, this, std::placeholders::_1));

    local_map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      this->get_parameter("local_map_topic").as_string(), 1,
      std::bind(&MapOdomCorrector::localMapCallback, this, std::placeholders::_1));
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odom_ = *msg;
  }

  void localMapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
  {
    if (!last_odom_.header.stamp.sec) return;

    // Convert to PCL
    PointCloud::Ptr scan(new PointCloud);
    pcl::fromROSMsg(*cloud_msg, *scan);
    // Remove NaNs
    std::vector<int> idx;
    pcl::removeNaNFromPointCloud(*scan, *scan, idx);
    // Downsample scan
    pcl::VoxelGrid<PointT> vg_scan;
    vg_scan.setInputCloud(scan);
    double scan_leaf = this->get_parameter("scan_leaf_size").as_double();
    vg_scan.setLeafSize(scan_leaf, scan_leaf, scan_leaf);
    PointCloud::Ptr filtered_scan(new PointCloud);
    vg_scan.filter(*filtered_scan);

    // Report point‐counts (proxy for “inliers”)
    RCLCPP_INFO_STREAM(this->get_logger(),
      "Filtered scan points: " << filtered_scan->size()
      << " | Filtered map points: " << filtered_map_->size());

    // Set source and align
    ndt_.setInputSource(filtered_scan);
    Eigen::Matrix4f init_guess = poseToMatrix(last_odom_);
    PointCloud::Ptr aligned(new PointCloud);
    ndt_.align(*aligned, init_guess);

    // Print convergence & error stats
    bool converged = ndt_.hasConverged();
    double fitness = ndt_.getFitnessScore();
    int iters = ndt_.getFinalNumIteration();
    RCLCPP_INFO_STREAM(this->get_logger(),
      "NDT converged: " << (converged ? "YES" : "NO")
      << " | Fitness score (mean sq. error): " << fitness
      << " | Iterations: " << iters);

    if (!converged) {
      RCLCPP_WARN(this->get_logger(), "NDT did not converge - skipping publish");
      return;
    }

    Eigen::Matrix4f trans = ndt_.getFinalTransformation();
    // Publish corrected odom
    nav_msgs::msg::Odometry out = last_odom_;
    out.header.stamp = this->now();
    matrixToPose(trans, out.pose.pose);
    corrected_pub_->publish(out);
  }

  static Eigen::Matrix4f poseToMatrix(const nav_msgs::msg::Odometry & odom)
  {
    Eigen::Translation3f t(odom.pose.pose.position.x,
                           odom.pose.pose.position.y,
                           odom.pose.pose.position.z);
    Eigen::Quaternionf q(odom.pose.pose.orientation.w,
                         odom.pose.pose.orientation.x,
                         odom.pose.pose.orientation.y,
                         odom.pose.pose.orientation.z);
    return (t * q).matrix();
  }

  static void matrixToPose(const Eigen::Matrix4f & m, geometry_msgs::msg::Pose & p)
  {
    Eigen::Matrix3f rot = m.block<3,3>(0,0);
    Eigen::Quaternionf q(rot);
    p.position.x = m(0,3);
    p.position.y = m(1,3);
    p.position.z = m(2,3);
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    p.orientation.w = q.w();
  }

  // --- members ---
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr corrected_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_sub_;
  pcl::NormalDistributionsTransform<PointT, PointT> ndt_;
  PointCloud::Ptr filtered_map_;              // downsampled global map
  nav_msgs::msg::Odometry last_odom_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapOdomCorrector>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
