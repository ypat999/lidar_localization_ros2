#include <chrono>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <mutex>
#include <vector>

#include <pcl/registration/registration.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>

#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "lidar_localization/lidar_undistortion.hpp"

using namespace std::chrono_literals;

class PCLLocalization : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit PCLLocalization(const rclcpp::NodeOptions & options);

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &);
  CallbackReturn on_activate(const rclcpp_lifecycle::State &);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);
  CallbackReturn on_error(const rclcpp_lifecycle::State & state);

  void initializeParameters();
  void initializePubSub();
  void initializeRegistration();
  void initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void mapReceived(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void odomReceived(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void imuReceived(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void cloudReceived(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void timerPublishPose();
  // void gnssReceived();

  tf2_ros::TransformBroadcaster broadcaster_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
  rclcpp::Clock clock_;
  tf2_ros::Buffer tfbuffer_;
  tf2_ros::TransformListener tflistener_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::ConstSharedPtr
    initial_pose_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    map_odom_pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
    path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    initial_map_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::ConstSharedPtr
    map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr
    odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::ConstSharedPtr
    cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr
    imu_sub_;

  rclcpp::TimerBase::SharedPtr pose_publish_timer_;

  boost::shared_ptr<pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>> registration_;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter_;
  pcl::VoxelGrid<pcl::PointXYZI> map_downsample_filter_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr corrent_pose_with_cov_stamped_ptr_;
  nav_msgs::msg::Path::SharedPtr path_ptr_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr last_scan_ptr_;

  bool map_recieved_{false};
  bool initialpose_recieved_{false};

  // parameters
  std::string global_frame_id_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string map_topic_;
  std::string registration_method_;
  double scan_max_range_;
  double scan_min_range_;
  double scan_period_;
  double score_threshold_;
  double ndt_resolution_;
  double ndt_step_size_;
  double transform_epsilon_;
  double voxel_leaf_size_;
  bool use_pcd_map_{false};
  std::string map_path_;
  bool set_initial_pose_{false};
  double initial_pose_x_;
  double initial_pose_y_;
  double initial_pose_z_;
  double initial_pose_qx_;
  double initial_pose_qy_;
  double initial_pose_qz_;
  double initial_pose_qw_;

  bool use_odom_{false};
  geometry_msgs::msg::Pose odom_at_localization_;  // odom->base pose at last successful localization
  bool use_imu_{false};
  bool enable_debug_{false};
  bool enable_map_odom_tf_{false};
  bool enable_timer_publishing{false};
  double pose_publish_frequency_{1.0};

  int ndt_num_threads_;
  int ndt_max_iterations_;

  // GICP-specific parameters
  double gicp_corr_dist_threshold_{5.0};
  double gicp_rotation_epsilon_{0.002};
  int gicp_k_correspondences_{20};
  int gicp_max_optimizer_iterations_{20};
  double gicp_epsilon_{0.01};

  // imu
  LidarUndistortion lidar_undistortion_;

  // New parameters for improved localization
  double displacement_threshold_{0.3};  // meters
  double search_radius_{3.0};           // meters
  int search_grid_size_{5};             // grid points per dimension
  bool enable_displacement_check_{true};
  bool enable_search_optimization_{true};
  double map_downsample_leaf_size_{2.0};  // meters
  double last_localization_x_{0.0};
  double last_localization_y_{0.0};
  double last_localization_z_{0.0};
  bool first_localization_done_{false};
  int initial_localization_accumulate_frames_{10};
  pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_cloud_ptr_{new pcl::PointCloud<pcl::PointXYZI>};
  int accumulated_frame_count_{0};
  
  // Odom-based displacement tracking（基于odom的位移累计）
  double accumulated_odom_distance_{0.0};
  
  // Angle search optimization parameters
  bool enable_angle_search_{true};       // Enable angle search for better rotation convergence
  double angle_search_range_{0.349};     // Angle search range in radians (±20 degrees)
  int angle_search_steps_{9};            // Number of angle steps to try
  
  // Z-axis search parameters
  bool enable_z_axis_search_{false};     // Enable Z-axis search for all triggers
  
  // Dynamic score threshold mechanism
  double current_fitness_score_{std::numeric_limits<double>::max()};  // Track current fitness score
  double best_fitness_score_{std::numeric_limits<double>::max()};     // Track minimum fitness score ever seen
  bool enable_dynamic_threshold_{true};  // Enable dynamic threshold mechanism
  double dynamic_threshold_factor_{2.0};  // Factor for dynamic threshold (new score must be <= current * factor)
  
  // Helper methods
  double calculateDisplacement(const geometry_msgs::msg::Pose& current_pose);
  bool shouldUpdateLocalization(const geometry_msgs::msg::Pose& current_pose);
  
  // Performance statistics methods
  void performanceTimerCallback();
  void addPerformanceStatistics(const std::string& method, double duration);
  
  struct SearchResult {
    Eigen::Matrix4f transformation;
    bool has_converged;
    double fitness_score;
  };
  
  SearchResult searchOptimalTransformation(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_ptr,
    const Eigen::Matrix4f& initial_guess,
    bool search_z_axis);
    
  // Performance statistics variables
  rclcpp::TimerBase::SharedPtr performance_timer_;
  std::vector<double> icp_performance_stats_;
  std::vector<double> ndt_performance_stats_;
  std::mutex performance_stats_mutex_;
};
