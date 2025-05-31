#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Geometry>

#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>

using namespace std::placeholders;
using PointT = pcl::PointXYZ;

// small_gicp shorthands
using SGPointCloud = small_gicp::PointCloud;
using KdTree       = small_gicp::KdTree<SGPointCloud>;
using Registration = small_gicp::Registration<
  small_gicp::GICPFactor,
  small_gicp::ParallelReductionOMP
>;

class MapLocalizer : public rclcpp::Node {
public:
  MapLocalizer(const rclcpp::NodeOptions &opts)
  : Node("map_localizer", opts),
    last_pose_(Eigen::Isometry3d::Identity()),
    imu_q_(Eigen::Quaterniond::Identity())
  {
    // ---- parameters ----
    declare_parameter<std::string>("map_file", "");
    declare_parameter<double>("voxel_size", 0.1);
    declare_parameter<int>("num_threads", 2);
    declare_parameter<double>("max_corr_dist", 1.0);
    declare_parameter<int>("corr_randomness", 20);

    get_parameter("map_file", map_file_);
    get_parameter("voxel_size", voxel_size_);
    get_parameter("num_threads", num_threads_);
    get_parameter("max_corr_dist", max_corr_dist_);
    get_parameter("corr_randomness", corr_randomness_);

    // ---- load & preprocess map ----
    pcl::PointCloud<PointT>::Ptr raw_map(new pcl::PointCloud<PointT>);
    if (pcl::io::loadPCDFile(map_file_, *raw_map) < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to load map: %s", map_file_.c_str());
      rclcpp::shutdown();
      return;
    }
    //RCLCPP_INFO(get_logger(), "Loaded map (%zu points)", raw_map->size());

    pcl::PointCloud<PointT>::Ptr ds_map(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> vg_map;
    vg_map.setInputCloud(raw_map);
    vg_map.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    vg_map.filter(*ds_map);
    RCLCPP_INFO(get_logger(), "Downsampled map → %zu points", ds_map->size());

    // convert to small_gicp cloud
    std::vector<Eigen::Vector3d> map_pts;
    map_pts.reserve(ds_map->size());
    for (auto &p : ds_map->points) {
      map_pts.emplace_back(p.x, p.y, p.z);
    }
    target_ = std::make_shared<SGPointCloud>(map_pts);

    // build tree & covariances
    target_tree_ = std::make_shared<KdTree>(
      target_, small_gicp::KdTreeBuilderOMP(num_threads_));
    small_gicp::estimate_covariances_omp(
      *target_, *target_tree_, corr_randomness_, num_threads_);

    // configure GICP
    registration_.reduction.num_threads = num_threads_;
    registration_.rejector.max_dist_sq  = max_corr_dist_ * max_corr_dist_;

    // ---- ROS interfaces ----
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu", rclcpp::SensorDataQoS(),
      std::bind(&MapLocalizer::imuCallback, this, _1));

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud_in", rclcpp::SensorDataQoS(),
      std::bind(&MapLocalizer::cloudCallback, this, _1));

    pub_ = create_publisher<nav_msgs::msg::Odometry>("generate_pose", 10);

    RCLCPP_INFO(get_logger(), "MapLocalizer ready. Fusing IMU orientation into GICP.");
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
    // update the latest orientation from IMU
    imu_q_.w() = imu_msg->orientation.w;
    imu_q_.x() = imu_msg->orientation.x;
    imu_q_.y() = imu_msg->orientation.y;
    imu_q_.z() = imu_msg->orientation.z;
    imu_q_.normalize();
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // ROS→PCL
    pcl::PointCloud<PointT>::Ptr raw_scan(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *raw_scan);

    // downsample scan
    pcl::PointCloud<PointT>::Ptr ds_scan(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> vg_scan;
    vg_scan.setInputCloud(raw_scan);
    vg_scan.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    vg_scan.filter(*ds_scan);

    RCLCPP_INFO(get_logger(),
                "Map pts: %zu, Scan pts: %zu",
                target_->size(), ds_scan->size());
    if (ds_scan->empty()) {
      RCLCPP_WARN(get_logger(), "Empty downsampled scan, skipping");
      return;
    }

    // convert scan to small_gicp cloud + covariances
    std::vector<Eigen::Vector3d> scan_pts;
    scan_pts.reserve(ds_scan->size());
    for (auto &p : ds_scan->points) {
      scan_pts.emplace_back(p.x, p.y, p.z);
    }
    auto source = std::make_shared<SGPointCloud>(scan_pts);
    auto source_tree = std::make_shared<KdTree>(
      source, small_gicp::KdTreeBuilderOMP(num_threads_));
    small_gicp::estimate_covariances_omp(
      *source, *source_tree, corr_randomness_, num_threads_);

    // build initial guess: keep last translation, but orientation from IMU
    Eigen::Isometry3d init_guess = last_pose_;
    init_guess.linear() = imu_q_.toRotationMatrix();

    // align
    auto result = registration_.align(
      *target_, *source, *target_tree_, init_guess);

    // update last_pose_ and extract 4×4
    last_pose_ = result.T_target_source;
    Eigen::Matrix4d tf = last_pose_.matrix();

    // publish Odometry
    nav_msgs::msg::Odometry odom;
    odom.header         = msg->header;
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = tf(0,3);
    odom.pose.pose.position.y = tf(1,3);
    odom.pose.pose.position.z = tf(2,3);
    Eigen::Quaterniond q(last_pose_.rotation());
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    pub_->publish(odom);

    //RCLCPP_INFO_STREAM(get_logger(), "T =\n" << tf);
  }

  // parameters
  std::string map_file_;
  double      voxel_size_;
  int         num_threads_;
  double      max_corr_dist_;
  int         corr_randomness_;

  // small_gicp data
  std::shared_ptr<SGPointCloud> target_;
  std::shared_ptr<KdTree>       target_tree_;
  Registration                  registration_;
  Eigen::Isometry3d             last_pose_;

  // IMU orientation
  Eigen::Quaterniond            imu_q_;

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr           imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr   sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr            pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapLocalizer>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
