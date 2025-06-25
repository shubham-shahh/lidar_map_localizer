// ==============================
// map_odom_corrector_node.cpp
// (small_gicp version)
// ==============================

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>           // removeNaNFromPointCloud
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

// === small_gicp includes ===
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>

using std::placeholders::_1;
using PointT = pcl::PointXYZ;

// Aliases for small_gicp
using SGPointCloud = small_gicp::PointCloud;
using KdTree       = small_gicp::KdTree<SGPointCloud>;
using Registration = small_gicp::Registration<
  small_gicp::GICPFactor,
  small_gicp::ParallelReductionOMP
>;

class MapOdomCorrector : public rclcpp::Node
{
public:
  MapOdomCorrector(const rclcpp::NodeOptions & options)
  : Node("map_odom_corrector_small_gicp", options),
    odom_received_(false),
    last_map_to_odom_(Eigen::Isometry3d::Identity())
  {
    //
    // 1) Declare and read all parameters
    //
    this->declare_parameter<std::string>("map_file", "/path/to/your/global_map.pcd");
    this->declare_parameter<double>("voxel_leaf_size_local", 0.2);
    this->declare_parameter<double>("voxel_leaf_size_target", 0.2);
    this->declare_parameter<double>("crop_size_x", 20.0);
    this->declare_parameter<double>("crop_size_y", 20.0);
    this->declare_parameter<double>("crop_size_z", 5.0);
    // small_gicp‐specific
    this->declare_parameter<double>("map_downsample_voxel", 0.1);
    this->declare_parameter<int>("gicp_num_threads", 4);
    this->declare_parameter<double>("gicp_max_corr_dist", 5.0);
    this->declare_parameter<int>("gicp_corr_randomness", 20);

    this->declare_parameter<std::string>("odom_topic", "/kiss/odometry");
    this->declare_parameter<std::string>("local_map_topic", "/kiss/local_map");
    this->declare_parameter<std::string>("corrected_odom_topic", "/corrected_odometry");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<std::string>("map_frame", "map");

    // Read parameters
    this->get_parameter("map_file", map_file_);
    this->get_parameter("voxel_leaf_size_local", voxel_leaf_local_);
    this->get_parameter("voxel_leaf_size_target", voxel_leaf_target_);
    this->get_parameter("crop_size_x", crop_size_x_);
    this->get_parameter("crop_size_y", crop_size_y_);
    this->get_parameter("crop_size_z", crop_size_z_);
    this->get_parameter("map_downsample_voxel", map_downsample_voxel_);
    this->get_parameter("gicp_num_threads", num_threads_);
    this->get_parameter("gicp_max_corr_dist", max_corr_dist_);
    this->get_parameter("gicp_corr_randomness", corr_randomness_);
    this->get_parameter("odom_topic", odom_topic_);
    this->get_parameter("local_map_topic", local_map_topic_);
    this->get_parameter("corrected_odom_topic", corrected_odom_topic_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("map_frame", map_frame_);

    //
    // 2) Load + preprocess global PCD map
    //
    pcl::PointCloud<PointT>::Ptr raw_map(new pcl::PointCloud<PointT>);
    if (pcl::io::loadPCDFile<PointT>(map_file_, *raw_map) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load global PCD from '%s'.", map_file_.c_str());
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Loaded global map [%s] with %zu points.",
                map_file_.c_str(), raw_map->size());

    // Remove NaN from the raw map just in case
    {
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*raw_map, *raw_map, indices);
    }

    // Downsample the global map with a VoxelGrid
    pcl::PointCloud<PointT>::Ptr ds_map(new pcl::PointCloud<PointT>);
    {
      pcl::VoxelGrid<PointT> vg;
      vg.setInputCloud(raw_map);
      vg.setLeafSize(static_cast<float>(map_downsample_voxel_),
                     static_cast<float>(map_downsample_voxel_),
                     static_cast<float>(map_downsample_voxel_));
      vg.filter(*ds_map);
    }
    RCLCPP_INFO(this->get_logger(), "Downsampled global map → %zu points", ds_map->size());

    // Convert downsampled PCL map → small_gicp cloud (vector<Eigen::Vector3d>)
    std::vector<Eigen::Vector3d> map_pts;
    map_pts.reserve(ds_map->size());
    for (const auto &p : ds_map->points) {
      map_pts.emplace_back(p.x, p.y, p.z);
    }
    target_sg_cloud_ = std::make_shared<SGPointCloud>(map_pts);

    // Build KdTree + estimate covariances for the target
    target_tree_ = std::make_shared<KdTree>(
      target_sg_cloud_, small_gicp::KdTreeBuilderOMP(num_threads_));
    small_gicp::estimate_covariances_omp(
      *target_sg_cloud_, *target_tree_, corr_randomness_, num_threads_);

    // Configure the GICP registration object
    registration_.reduction.num_threads = num_threads_;
    registration_.rejector.max_dist_sq  = max_corr_dist_ * max_corr_dist_;

    //
    // 3) Create subscribers + publisher
    //
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10, std::bind(&MapOdomCorrector::odomCallback, this, _1));

    local_map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      local_map_topic_, 1, std::bind(&MapOdomCorrector::localMapCallback, this, _1));

    corrected_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      corrected_odom_topic_, 10);

    RCLCPP_INFO(this->get_logger(),
                "MapOdomCorrector (small_gicp) initialized. Subscribing to '%s' and '%s', publishing '%s'.",
                odom_topic_.c_str(), local_map_topic_.c_str(), corrected_odom_topic_.c_str());
  }

private:
  // === Member variables ===

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr corrected_odom_pub_;

  // Parameters
  std::string map_file_;
  double voxel_leaf_local_;
  double voxel_leaf_target_;
  double crop_size_x_, crop_size_y_, crop_size_z_;
  double map_downsample_voxel_;
  int num_threads_;
  double max_corr_dist_;
  int corr_randomness_;
  std::string odom_topic_, local_map_topic_, corrected_odom_topic_;
  std::string base_frame_, map_frame_;

  // PCL clouds
  // (We keep the original global_map_ only for cropping; downsampled version is in target_sg_cloud_)
  pcl::PointCloud<PointT>::Ptr global_map_{nullptr};

  // small_gicp data
  std::shared_ptr<SGPointCloud> target_sg_cloud_;
  std::shared_ptr<KdTree>       target_tree_;
  Registration                  registration_;
  Eigen::Isometry3d             last_map_to_odom_;

  // Keeps the latest odometry pose (in odom frame)
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  std::mutex odom_mutex_;
  bool odom_received_;

  // === Callbacks ===

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    latest_odom_ = msg;
    odom_received_ = true;
  }

  void localMapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pc2_msg)
  {
    // 1) Ensure we have received at least one odometry
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (!odom_received_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Waiting for initial odometry...");
        return;
      }
    }

    // 2) Convert incoming submap (PointCloud2) → PCL PointCloud, remove NaNs
    pcl::PointCloud<PointT>::Ptr local_cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*pc2_msg, *local_cloud);
    {
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*local_cloud, *local_cloud, indices);
    }

    // 3) Downsample local cloud
    pcl::PointCloud<PointT>::Ptr local_ds(new pcl::PointCloud<PointT>);
    {
      pcl::VoxelGrid<PointT> vg_local;
      vg_local.setInputCloud(local_cloud);
      vg_local.setLeafSize(static_cast<float>(voxel_leaf_local_),
                           static_cast<float>(voxel_leaf_local_),
                           static_cast<float>(voxel_leaf_local_));
      vg_local.filter(*local_ds);
    }

    // 4) Get current raw‐odom position to crop the global map
    Eigen::Vector3f odom_xyz;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      odom_xyz.x() = latest_odom_->pose.pose.position.x;
      odom_xyz.y() = latest_odom_->pose.pose.position.y;
      odom_xyz.z() = latest_odom_->pose.pose.position.z;
    }

    // 5) Crop the global map around odom position
    pcl::PointCloud<PointT>::Ptr cropped_map(new pcl::PointCloud<PointT>);
    {
      // Ensure global_map_ is set (we cached only the downsampled SG cloud, so load it now)
      if (!global_map_) {
        // Lazily load the full global PCD (needed for cropping).
        // In practice, you might have kept this loaded from startup.
        pcl::PointCloud<PointT>::Ptr full_map(new pcl::PointCloud<PointT>);
        if (pcl::io::loadPCDFile<PointT>(map_file_, *full_map) < 0) {
          RCLCPP_ERROR(this->get_logger(),
                       "Failed to reload global PCD from '%s' for cropping.",
                       map_file_.c_str());
          return;
        }
        {
          std::vector<int> idx;
          pcl::removeNaNFromPointCloud(*full_map, *full_map, idx);
        }
        global_map_ = full_map;
      }

      pcl::CropBox<PointT> crop;
      crop.setInputCloud(global_map_);

      Eigen::Vector4f min_pt, max_pt;
      min_pt[0] = odom_xyz.x() - crop_size_x_ / 2.0f;
      min_pt[1] = odom_xyz.y() - crop_size_y_ / 2.0f;
      min_pt[2] = odom_xyz.z() - crop_size_z_ / 2.0f;
      min_pt[3] = 1.0f;
      max_pt[0] = odom_xyz.x() + crop_size_x_ / 2.0f;
      max_pt[1] = odom_xyz.y() + crop_size_y_ / 2.0f;
      max_pt[2] = odom_xyz.z() + crop_size_z_ / 2.0f;
      max_pt[3] = 1.0f;
      crop.setMin(min_pt);
      crop.setMax(max_pt);
      crop.filter(*cropped_map);

      // Remove NaNs from cropped_map
      std::vector<int> idx;
      pcl::removeNaNFromPointCloud(*cropped_map, *cropped_map, idx);

      if (cropped_map->empty()) {
        RCLCPP_WARN(this->get_logger(),
                    "Cropped region from global map is empty. Skipping alignment.");
        return;
      }
    }

    // 6) Downsample the cropped portion → target_ds
    pcl::PointCloud<PointT>::Ptr target_ds(new pcl::PointCloud<PointT>);
    {
      pcl::VoxelGrid<PointT> vg_target;
      vg_target.setInputCloud(cropped_map);
      vg_target.setLeafSize(static_cast<float>(voxel_leaf_target_),
                            static_cast<float>(voxel_leaf_target_),
                            static_cast<float>(voxel_leaf_target_));
      vg_target.filter(*target_ds);
    }

    if (target_ds->empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "Downsampled crop is empty (maybe too small). Skipping alignment.");
      return;
    }

    //
    // 7) Convert local_ds → small_gicp source cloud + estimate covariances
    //
    std::vector<Eigen::Vector3d> scan_pts;
    scan_pts.reserve(local_ds->size());
    for (const auto &p : local_ds->points) {
      scan_pts.emplace_back(p.x, p.y, p.z);
    }
    auto source_sg_cloud = std::make_shared<SGPointCloud>(scan_pts);
    auto source_tree = std::make_shared<KdTree>(
      source_sg_cloud, small_gicp::KdTreeBuilderOMP(num_threads_));
    small_gicp::estimate_covariances_omp(
      *source_sg_cloud, *source_tree, corr_randomness_, num_threads_);

    // 8) Also convert target_ds → small_gicp target, rebuild its KdTree & covariances
    //     (Alternatively, you could reuse a cached “target_sg_cloud_” if you pre‐cropped
    //      the entire map into smaller tiles, but here we do it on‐the‐fly.)
    std::vector<Eigen::Vector3d> target_pts;
    target_pts.reserve(target_ds->size());
    for (const auto &p : target_ds->points) {
      target_pts.emplace_back(p.x, p.y, p.z);
    }
    auto cropped_sg_cloud = std::make_shared<SGPointCloud>(target_pts);
    auto cropped_tree = std::make_shared<KdTree>(
      cropped_sg_cloud, small_gicp::KdTreeBuilderOMP(num_threads_));
    small_gicp::estimate_covariances_omp(
      *cropped_sg_cloud, *cropped_tree, corr_randomness_, num_threads_);

    // 9) Build initial guess for GICP: reuse last_map_to_odom_, which is an Isometry3d
    Eigen::Isometry3d init_guess = last_map_to_odom_;

    // 10) Run small_gicp alignment: cropped_sg_cloud (target) ← source_sg_cloud
    auto result = registration_.align(
      *cropped_sg_cloud,
      *source_sg_cloud,
      *cropped_tree,
      init_guess);

    //std::cout << "--- T_target_source ---" << std::endl << result.T_target_source.matrix() << std::endl;
    std::cout << "converged:" << result.converged << std::endl;
    std::cout << "error:" << result.error << std::endl;
    std::cout << "iterations:" << result.iterations << std::endl;
    std::cout << "num_inliers:" << result.num_inliers << std::endl;
    //std::cout << "--- H ---" << std::endl << result.H << std::endl;
    //std::cout << "--- b ---" << std::endl << result.b.transpose() << std::endl;

    // 11) Extract new map→odom (T_target_source is actually T_map_odom if we treat
    //     source points as “odom‐framed submap” and target as “map‐framed crop”)
    last_map_to_odom_ = result.T_target_source;
    Eigen::Matrix4d tf_d = last_map_to_odom_.matrix();
    Eigen::Matrix4f tf_f = tf_d.cast<float>();

    //
    // 12) Build “corrected odometry” = (M_map_odom * M_odom_base)
    //
    //    First, reconstruct M_odom_base from raw odometry
    Eigen::Matrix4f M_odom_base = Eigen::Matrix4f::Identity();
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      double px = latest_odom_->pose.pose.position.x;
      double py = latest_odom_->pose.pose.position.y;
      double pz = latest_odom_->pose.pose.position.z;
      double qx = latest_odom_->pose.pose.orientation.x;
      double qy = latest_odom_->pose.pose.orientation.y;
      double qz = latest_odom_->pose.pose.orientation.z;
      double qw = latest_odom_->pose.pose.orientation.w;

      Eigen::Quaternionf q_base(
        static_cast<float>(qw),
        static_cast<float>(qx),
        static_cast<float>(qy),
        static_cast<float>(qz));
      Eigen::Matrix3f R_base = q_base.toRotationMatrix();
      M_odom_base.block<3,3>(0,0) = R_base;
      M_odom_base(0,3) = static_cast<float>(px);
      M_odom_base(1,3) = static_cast<float>(py);
      M_odom_base(2,3) = static_cast<float>(pz);
    }

    // Multiply: M_map_base = (map←odom) * (odom←base)
    Eigen::Matrix4f M_map_base = tf_f * M_odom_base;

    // Decompose M_map_base → position + quaternion
    Eigen::Vector3f map_t;
    map_t.x() = M_map_base(0,3);
    map_t.y() = M_map_base(1,3);
    map_t.z() = M_map_base(2,3);
    Eigen::Matrix3f map_R = M_map_base.block<3,3>(0,0);
    Eigen::Quaternionf map_q(map_R);

    // Build new Odometry message
    auto corrected_msg = std::make_shared<nav_msgs::msg::Odometry>();
    corrected_msg->header.stamp = this->get_clock()->now();
    corrected_msg->header.frame_id = map_frame_;       // “map”
    corrected_msg->child_frame_id  = base_frame_;      // “base_link”

    // Pose (corrected)
    corrected_msg->pose.pose.position.x = map_t.x();
    corrected_msg->pose.pose.position.y = map_t.y();
    corrected_msg->pose.pose.position.z = map_t.z();
    corrected_msg->pose.pose.orientation.x = map_q.x();
    corrected_msg->pose.pose.orientation.y = map_q.y();
    corrected_msg->pose.pose.orientation.z = map_q.z();
    corrected_msg->pose.pose.orientation.w = map_q.w();

    // Twist: rotate original odom’s linear velocity into map frame
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      double vx = latest_odom_->twist.twist.linear.x;
      double vy = latest_odom_->twist.twist.linear.y;
      double vz = latest_odom_->twist.twist.linear.z;
      Eigen::Vector3f v_odom(
        static_cast<float>(vx),
        static_cast<float>(vy),
        static_cast<float>(vz));
      Eigen::Vector3f v_map = map_R * v_odom;

      corrected_msg->twist.twist.linear.x  = v_map.x();
      corrected_msg->twist.twist.linear.y  = v_map.y();
      corrected_msg->twist.twist.linear.z  = v_map.z();

      // Copy angular velocities (assuming they are in robot’s local frame)
      corrected_msg->twist.twist.angular = latest_odom_->twist.twist.angular;
    }

    // 13) Publish corrected odometry
    corrected_odom_pub_->publish(*corrected_msg);
    RCLCPP_DEBUG(this->get_logger(),
                 "Published corrected odometry (small_gicp) at time %u.%u",
                 corrected_msg->header.stamp.sec,
                 corrected_msg->header.stamp.nanosec);
  }
};  // class MapOdomCorrector

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<MapOdomCorrector>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
