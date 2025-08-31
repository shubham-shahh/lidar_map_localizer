// hybrid_ndt_gicp_localizer.cpp
// Combines NDT for coarse alignment and small_gicp for fine alignment

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/registration/ndt.h>

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
using PointCloud = pcl::PointCloud<PointT>;

// Aliases for small_gicp
using SGPointCloud = small_gicp::PointCloud;
using KdTree       = small_gicp::KdTree<SGPointCloud>;
using Registration = small_gicp::Registration<
  small_gicp::GICPFactor,
  small_gicp::ParallelReductionOMP
>;

class HybridNDTGICPLocalizer : public rclcpp::Node
{
public:
  HybridNDTGICPLocalizer(const rclcpp::NodeOptions & options)
  : Node("hybrid_ndt_gicp_localizer", options),
    odom_received_(false),
    last_map_to_odom_(Eigen::Isometry3d::Identity()),
    ndt_coarse_pose_(Eigen::Matrix4f::Identity()),
    first_alignment_(true)
  {
    //
    // 1) Declare and read all parameters
    //
    
    // Map file
    this->declare_parameter<std::string>("map_file", "/path/to/your/global_map.pcd");
    
    // NDT parameters (for coarse alignment)
    this->declare_parameter<double>("ndt_resolution", 2.0);  // Coarse resolution for NDT
    this->declare_parameter<double>("ndt_step_size", 0.1);
    this->declare_parameter<double>("ndt_epsilon", 0.01);
    this->declare_parameter<int>("ndt_max_iter", 30);
    this->declare_parameter<double>("ndt_voxel_leaf_size", 0.5);  // Coarse downsampling for NDT
    
    // GICP parameters (for fine alignment)
    this->declare_parameter<double>("voxel_leaf_size_local", 0.2);
    this->declare_parameter<double>("voxel_leaf_size_target", 0.2);
    this->declare_parameter<double>("crop_size_x", 30.0);
    this->declare_parameter<double>("crop_size_y", 30.0);
    this->declare_parameter<double>("crop_size_z", 10.0);
    this->declare_parameter<double>("map_downsample_voxel", 0.1);
    this->declare_parameter<int>("gicp_num_threads", 4);
    this->declare_parameter<double>("gicp_max_corr_dist", 3.0);
    this->declare_parameter<int>("gicp_corr_randomness", 20);
    
    // Topics and frames
    this->declare_parameter<std::string>("odom_topic", "/kiss/odometry");
    this->declare_parameter<std::string>("local_map_topic", "/kiss/local_map");
    this->declare_parameter<std::string>("corrected_odom_topic", "/corrected_odometry");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<std::string>("map_frame", "map");
    
    // Localization mode
    this->declare_parameter<bool>("use_odom_as_initial_guess", true);
    this->declare_parameter<double>("initial_x", 0.0);
    this->declare_parameter<double>("initial_y", 0.0);
    this->declare_parameter<double>("initial_z", 0.0);
    this->declare_parameter<double>("initial_roll", 0.0);
    this->declare_parameter<double>("initial_pitch", 0.0);
    this->declare_parameter<double>("initial_yaw", 0.0);
    
    // Thresholds
    this->declare_parameter<double>("ndt_fitness_threshold", 1.0);
    this->declare_parameter<double>("gicp_fitness_threshold", 0.3);
    this->declare_parameter<bool>("publish_pose_with_covariance", false);

    // Read parameters
    this->get_parameter("map_file", map_file_);
    
    // NDT params
    this->get_parameter("ndt_resolution", ndt_resolution_);
    this->get_parameter("ndt_step_size", ndt_step_size_);
    this->get_parameter("ndt_epsilon", ndt_epsilon_);
    this->get_parameter("ndt_max_iter", ndt_max_iter_);
    this->get_parameter("ndt_voxel_leaf_size", ndt_voxel_leaf_);
    
    // GICP params
    this->get_parameter("voxel_leaf_size_local", voxel_leaf_local_);
    this->get_parameter("voxel_leaf_size_target", voxel_leaf_target_);
    this->get_parameter("crop_size_x", crop_size_x_);
    this->get_parameter("crop_size_y", crop_size_y_);
    this->get_parameter("crop_size_z", crop_size_z_);
    this->get_parameter("map_downsample_voxel", map_downsample_voxel_);
    this->get_parameter("gicp_num_threads", num_threads_);
    this->get_parameter("gicp_max_corr_dist", max_corr_dist_);
    this->get_parameter("gicp_corr_randomness", corr_randomness_);
    
    // Other params
    this->get_parameter("odom_topic", odom_topic_);
    this->get_parameter("local_map_topic", local_map_topic_);
    this->get_parameter("corrected_odom_topic", corrected_odom_topic_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("map_frame", map_frame_);
    this->get_parameter("use_odom_as_initial_guess", use_odom_as_initial_guess_);
    this->get_parameter("initial_x", initial_x_);
    this->get_parameter("initial_y", initial_y_);
    this->get_parameter("initial_z", initial_z_);
    this->get_parameter("initial_roll", initial_roll_);
    this->get_parameter("initial_pitch", initial_pitch_);
    this->get_parameter("initial_yaw", initial_yaw_);
    this->get_parameter("ndt_fitness_threshold", ndt_fitness_threshold_);
    this->get_parameter("gicp_fitness_threshold", gicp_fitness_threshold_);
    this->get_parameter("publish_pose_with_covariance", publish_pose_with_covariance_);

    //
    // 2) Load global PCD map
    //
    PointCloud::Ptr raw_map(new PointCloud);
    if (pcl::io::loadPCDFile<PointT>(map_file_, *raw_map) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load global PCD from '%s'.", map_file_.c_str());
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Loaded global map [%s] with %zu points.",
                map_file_.c_str(), raw_map->size());

    // Remove NaN from the raw map
    {
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*raw_map, *raw_map, indices);
    }
    
    // Store the full map for cropping later
    global_map_ = raw_map;

    //
    // 3) Prepare coarse map for NDT
    //
    PointCloud::Ptr ndt_map(new PointCloud);
    {
      pcl::VoxelGrid<PointT> vg;
      vg.setInputCloud(global_map_);
      vg.setLeafSize(static_cast<float>(ndt_voxel_leaf_),
                     static_cast<float>(ndt_voxel_leaf_),
                     static_cast<float>(ndt_voxel_leaf_));
      vg.filter(*ndt_map);
    }
    RCLCPP_INFO(this->get_logger(), "NDT coarse map: %zu points", ndt_map->size());
    
    // Setup NDT with coarse map
    ndt_.setInputTarget(ndt_map);
    ndt_.setResolution(ndt_resolution_);
    ndt_.setStepSize(ndt_step_size_);
    ndt_.setTransformationEpsilon(ndt_epsilon_);
    ndt_.setMaximumIterations(ndt_max_iter_);

    //
    // 4) Prepare fine map for GICP (will be cropped dynamically)
    //
    // Configure the GICP registration object
    registration_.reduction.num_threads = num_threads_;
    registration_.rejector.max_dist_sq  = max_corr_dist_ * max_corr_dist_;

    //
    // 5) Create subscribers and publishers
    //
    if (use_odom_as_initial_guess_) {
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10, std::bind(&HybridNDTGICPLocalizer::odomCallback, this, _1));
    }

    local_map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      local_map_topic_, 1, std::bind(&HybridNDTGICPLocalizer::localMapCallback, this, _1));

    corrected_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      corrected_odom_topic_, 10);
      
    if (publish_pose_with_covariance_) {
      pose_with_cov_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/localization_pose_with_covariance", 10);
    }

    RCLCPP_INFO(this->get_logger(),
                "HybridNDTGICPLocalizer initialized. NDT for coarse, GICP for fine alignment.");
  }

private:
  // === Member variables ===

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr corrected_odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_with_cov_pub_;

  // Parameters
  std::string map_file_;
  
  // NDT params
  double ndt_resolution_, ndt_step_size_, ndt_epsilon_;
  int ndt_max_iter_;
  double ndt_voxel_leaf_;
  double ndt_fitness_threshold_;
  
  // GICP params
  double voxel_leaf_local_, voxel_leaf_target_;
  double crop_size_x_, crop_size_y_, crop_size_z_;
  double map_downsample_voxel_;
  int num_threads_;
  double max_corr_dist_;
  int corr_randomness_;
  double gicp_fitness_threshold_;
  
  // Other params
  std::string odom_topic_, local_map_topic_, corrected_odom_topic_;
  std::string base_frame_, map_frame_;
  bool use_odom_as_initial_guess_;
  double initial_x_, initial_y_, initial_z_;
  double initial_roll_, initial_pitch_, initial_yaw_;
  bool publish_pose_with_covariance_;

  // Map data
  PointCloud::Ptr global_map_;
  
  // NDT
  pcl::NormalDistributionsTransform<PointT, PointT> ndt_;
  Eigen::Matrix4f ndt_coarse_pose_;
  
  // GICP
  Registration registration_;
  Eigen::Isometry3d last_map_to_odom_;

  // State
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  std::mutex odom_mutex_;
  bool odom_received_;
  bool first_alignment_;

  // === Helper functions ===
  
  Eigen::Matrix4f createInitialGuessMatrix(
    double x, double y, double z, 
    double roll, double pitch, double yaw)
  {
    Eigen::Translation3f translation(x, y, z);
    Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
    Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
    return (translation * q).matrix();
  }

  // === Callbacks ===

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    latest_odom_ = msg;
    odom_received_ = true;
  }

  void localMapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pc2_msg)
  {
    // 1) Check if we have initial guess
    if (use_odom_as_initial_guess_) {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (!odom_received_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Waiting for initial odometry...");
        return;
      }
    }

    // 2) Convert incoming submap to PCL and preprocess
    PointCloud::Ptr local_cloud(new PointCloud);
    pcl::fromROSMsg(*pc2_msg, *local_cloud);
    {
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*local_cloud, *local_cloud, indices);
    }

    // 3) Downsample for NDT (coarse)
    PointCloud::Ptr local_ndt_cloud(new PointCloud);
    {
      pcl::VoxelGrid<PointT> vg_ndt;
      vg_ndt.setInputCloud(local_cloud);
      vg_ndt.setLeafSize(static_cast<float>(ndt_voxel_leaf_),
                        static_cast<float>(ndt_voxel_leaf_),
                        static_cast<float>(ndt_voxel_leaf_));
      vg_ndt.filter(*local_ndt_cloud);
    }

    //
    // === STEP 1: NDT Coarse Alignment ===
    //
    Eigen::Matrix4f ndt_init_guess;
    if (first_alignment_) {
      if (use_odom_as_initial_guess_) {
        // Use odometry for initial guess
        std::lock_guard<std::mutex> lock(odom_mutex_);
        Eigen::Translation3f t(latest_odom_->pose.pose.position.x,
                               latest_odom_->pose.pose.position.y,
                               latest_odom_->pose.pose.position.z);
        Eigen::Quaternionf q(latest_odom_->pose.pose.orientation.w,
                             latest_odom_->pose.pose.orientation.x,
                             latest_odom_->pose.pose.orientation.y,
                             latest_odom_->pose.pose.orientation.z);
        ndt_init_guess = (t * q).matrix();
      } else {
        // Use provided initial pose
        ndt_init_guess = createInitialGuessMatrix(
          initial_x_, initial_y_, initial_z_,
          initial_roll_, initial_pitch_, initial_yaw_);
      }
      first_alignment_ = false;
    } else {
      // Use last result as initial guess
      ndt_init_guess = ndt_coarse_pose_;
    }

    // Run NDT
    ndt_.setInputSource(local_ndt_cloud);
    PointCloud::Ptr ndt_aligned(new PointCloud);
    ndt_.align(*ndt_aligned, ndt_init_guess);

    bool ndt_converged = ndt_.hasConverged();
    double ndt_fitness = ndt_.getFitnessScore();
    
    RCLCPP_INFO(this->get_logger(),
                "[NDT] Converged: %s | Fitness: %.4f | Iterations: %d",
                ndt_converged ? "YES" : "NO", ndt_fitness, ndt_.getFinalNumIteration());

    if (!ndt_converged || ndt_fitness > ndt_fitness_threshold_) {
      RCLCPP_WARN(this->get_logger(), 
                  "NDT failed (converged=%d, fitness=%.4f). Skipping this frame.",
                  ndt_converged, ndt_fitness);
      return;
    }

    // Store NDT result
    ndt_coarse_pose_ = ndt_.getFinalTransformation();

    //
    // === STEP 2: Crop map around NDT pose for GICP ===
    //
    Eigen::Vector3f ndt_position(
      ndt_coarse_pose_(0, 3),
      ndt_coarse_pose_(1, 3),
      ndt_coarse_pose_(2, 3)
    );

    PointCloud::Ptr cropped_map(new PointCloud);
    {
      pcl::CropBox<PointT> crop;
      crop.setInputCloud(global_map_);

      Eigen::Vector4f min_pt, max_pt;
      min_pt[0] = ndt_position.x() - crop_size_x_ / 2.0f;
      min_pt[1] = ndt_position.y() - crop_size_y_ / 2.0f;
      min_pt[2] = ndt_position.z() - crop_size_z_ / 2.0f;
      min_pt[3] = 1.0f;
      max_pt[0] = ndt_position.x() + crop_size_x_ / 2.0f;
      max_pt[1] = ndt_position.y() + crop_size_y_ / 2.0f;
      max_pt[2] = ndt_position.z() + crop_size_z_ / 2.0f;
      max_pt[3] = 1.0f;
      crop.setMin(min_pt);
      crop.setMax(max_pt);
      crop.filter(*cropped_map);

      std::vector<int> idx;
      pcl::removeNaNFromPointCloud(*cropped_map, *cropped_map, idx);

      if (cropped_map->empty()) {
        RCLCPP_WARN(this->get_logger(),
                    "Cropped region from global map is empty. Using NDT result only.");
        publishResult(ndt_coarse_pose_);
        return;
      }
    }

    // Downsample cropped map for GICP
    PointCloud::Ptr target_ds(new PointCloud);
    {
      pcl::VoxelGrid<PointT> vg_target;
      vg_target.setInputCloud(cropped_map);
      vg_target.setLeafSize(static_cast<float>(voxel_leaf_target_),
                            static_cast<float>(voxel_leaf_target_),
                            static_cast<float>(voxel_leaf_target_));
      vg_target.filter(*target_ds);
    }

    // Downsample local cloud for GICP
    PointCloud::Ptr local_ds(new PointCloud);
    {
      pcl::VoxelGrid<PointT> vg_local;
      vg_local.setInputCloud(local_cloud);
      vg_local.setLeafSize(static_cast<float>(voxel_leaf_local_),
                           static_cast<float>(voxel_leaf_local_),
                           static_cast<float>(voxel_leaf_local_));
      vg_local.filter(*local_ds);
    }

    //
    // === STEP 3: GICP Fine Alignment ===
    //
    
    // Convert local_ds to small_gicp format
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

    // Convert target_ds to small_gicp format
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

    // Use NDT result as initial guess for GICP
    Eigen::Isometry3d gicp_init_guess;
    gicp_init_guess.matrix() = ndt_coarse_pose_.cast<double>();

    // Run GICP
    auto result = registration_.align(
      *cropped_sg_cloud,
      *source_sg_cloud,
      *cropped_tree,
      gicp_init_guess);

    RCLCPP_INFO(this->get_logger(),
                "[GICP] Converged: %s | Error: %.4f | Iterations: %zu | Inliers: %zu",
                result.converged ? "YES" : "NO", 
                result.error, 
                result.iterations, 
                result.num_inliers);

    // Check GICP quality
    if (!result.converged || result.error > gicp_fitness_threshold_) {
      RCLCPP_WARN(this->get_logger(), 
                  "GICP refinement failed. Using NDT result.");
      publishResult(ndt_coarse_pose_);
    } else {
      // Use refined GICP result
      last_map_to_odom_ = result.T_target_source;
      Eigen::Matrix4f refined_pose = last_map_to_odom_.matrix().cast<float>();
      publishResult(refined_pose);
      
      // Update NDT coarse pose with refined result for next iteration
      ndt_coarse_pose_ = refined_pose;
    }
  }

  void publishResult(const Eigen::Matrix4f& tf_matrix)
  {
    // Get current odometry for twist information
    nav_msgs::msg::Odometry corrected_msg;
    corrected_msg.header.stamp = this->get_clock()->now();
    corrected_msg.header.frame_id = map_frame_;
    corrected_msg.child_frame_id = base_frame_;

    // Extract position and orientation
    Eigen::Vector3f position(tf_matrix(0,3), tf_matrix(1,3), tf_matrix(2,3));
    Eigen::Matrix3f rotation = tf_matrix.block<3,3>(0,0);
    Eigen::Quaternionf q(rotation);

    corrected_msg.pose.pose.position.x = position.x();
    corrected_msg.pose.pose.position.y = position.y();
    corrected_msg.pose.pose.position.z = position.z();
    corrected_msg.pose.pose.orientation.x = q.x();
    corrected_msg.pose.pose.orientation.y = q.y();
    corrected_msg.pose.pose.orientation.z = q.z();
    corrected_msg.pose.pose.orientation.w = q.w();

    // Copy twist if available
    if (use_odom_as_initial_guess_ && odom_received_) {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      corrected_msg.twist = latest_odom_->twist;
    }

    corrected_odom_pub_->publish(corrected_msg);
    
    // Optionally publish pose with covariance
    if (publish_pose_with_covariance_) {
      geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_msg;
      pose_cov_msg.header = corrected_msg.header;
      pose_cov_msg.pose.pose = corrected_msg.pose.pose;
      // Simple covariance based on algorithm performance
      // You can make this more sophisticated based on fitness scores
      for (int i = 0; i < 36; ++i) {
        pose_cov_msg.pose.covariance[i] = 0.0;
      }
      pose_cov_msg.pose.covariance[0] = 0.01;  // x
      pose_cov_msg.pose.covariance[7] = 0.01;  // y
      pose_cov_msg.pose.covariance[14] = 0.01; // z
      pose_cov_msg.pose.covariance[21] = 0.001; // roll
      pose_cov_msg.pose.covariance[28] = 0.001; // pitch
      pose_cov_msg.pose.covariance[35] = 0.001; // yaw
      
      pose_with_cov_pub_->publish(pose_cov_msg);
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<HybridNDTGICPLocalizer>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}