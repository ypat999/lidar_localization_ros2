#include <lidar_localization/lidar_localization_component.hpp>
#include <algorithm>
#include <chrono>
#include <numeric>
#include <thread>

PCLLocalization::PCLLocalization(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("lidar_localization", options),
  clock_(RCL_ROS_TIME),
  tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
  tflistener_(tfbuffer_),
  broadcaster_(this),
  static_broadcaster_(this)
{
  declare_parameter("global_frame_id", "map");
  declare_parameter("odom_frame_id", "odom");
  declare_parameter("base_frame_id", "base_link");
  declare_parameter("map_topic", "map3d");
  declare_parameter("enable_map_odom_tf", false);
  declare_parameter("registration_method", "NDT");
  declare_parameter("initial_score_threshold", 0.5);  // 初始定位阈值（宽松）
  declare_parameter("ongoing_score_threshold", 0.1);  // 持续定位阈值（严格）
  // Per-axis fitness score thresholds（默认与 ongoing_score_threshold 相同，保持向后兼容）
  declare_parameter("ongoing_score_threshold_x", 0.1);
  declare_parameter("ongoing_score_threshold_y", 0.1);
  declare_parameter("ongoing_score_threshold_z", 0.1);
  declare_parameter("score_threshold", 0.0001);
  declare_parameter("ndt_resolution", 1.0);
  declare_parameter("ndt_step_size", 0.1);
  declare_parameter("ndt_max_iterations", 35);
  declare_parameter("ndt_num_threads", 4);
  declare_parameter("transform_epsilon", 0.01);
  declare_parameter("voxel_leaf_size", 0.2);
  declare_parameter("scan_max_range", 100.0);
  declare_parameter("scan_min_range", 1.0);
  declare_parameter("scan_period", 0.1);
  declare_parameter("use_pcd_map", false);
  declare_parameter("map_path", "/map/map.pcd");
  declare_parameter("set_initial_pose", false);
  declare_parameter("initial_pose_x", 0.0);
  declare_parameter("initial_pose_y", 0.0);
  declare_parameter("initial_pose_z", 0.0);
  declare_parameter("initial_pose_qx", 0.0);
  declare_parameter("initial_pose_qy", 0.0);
  declare_parameter("initial_pose_qz", 0.0);
  declare_parameter("initial_pose_qw", 1.0);
  declare_parameter("use_odom", false);
  declare_parameter("use_imu", false);
  declare_parameter("enable_debug", false);
  declare_parameter("enable_timer_publishing", false);
  declare_parameter("pose_publish_frequency", 10.0);

  // New parameters for improved localization
  declare_parameter("displacement_threshold", 0.3);  // meters
  declare_parameter("search_radius", 3.0);           // meters
  declare_parameter("search_grid_size", 5);          // grid points per dimension
  declare_parameter("min_localization_interval", 2.0);  // 最小重试间隔（秒）
  declare_parameter("enable_displacement_check", true);
  declare_parameter("enable_search_optimization", true);
  
  // New parameters for map downsampling
  declare_parameter("map_downsample_leaf_size", 2.0);  // meters
  
  // New parameters for angle search optimization
  declare_parameter("enable_angle_search", true);
  declare_parameter("angle_search_range", 0.349);   // radians (±20 degrees)
  declare_parameter("angle_search_steps", 9);
  
  // New parameters for Z-axis search
  declare_parameter("enable_z_axis_search", false);  // Enable Z-axis search for all triggers
  
  // New parameters for dynamic score threshold mechanism
  declare_parameter("enable_dynamic_threshold", true);
  declare_parameter("dynamic_threshold_factor", 2.0);
  declare_parameter("initial_localization_accumulate_frames", 10);

  // Origin baseline precision landing (原点基准精准降落)
  declare_parameter("enable_origin_baseline", false);
  declare_parameter("origin_baseline_frames", 10);
  declare_parameter("origin_baseline_radius", 1.5);
  declare_parameter("origin_baseline_match_interval", 1.0);
  declare_parameter("origin_baseline_base_frame", "base_link");
  declare_parameter("origin_baseline_pcd_path", "/tmp/origin_baseline.pcd");
  
  // GICP-specific parameters
  declare_parameter("gicp_corr_dist_threshold", 5.0);
  declare_parameter("gicp_rotation_epsilon", 0.002);
  declare_parameter("gicp_k_correspondences", 20);
  declare_parameter("gicp_max_optimizer_iterations", 20);
  declare_parameter("gicp_epsilon", 0.01);

  // 参数运行时修改回调
  param_handler_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      for (const auto & p : params) {
        const auto & name = p.get_name();
        if (name == "global_frame_id") global_frame_id_ = p.as_string();
        else if (name == "odom_frame_id") odom_frame_id_ = p.as_string();
        else if (name == "base_frame_id") base_frame_id_ = p.as_string();
        else if (name == "enable_map_odom_tf") enable_map_odom_tf_ = p.as_bool();
        else if (name == "registration_method") registration_method_ = p.as_string();
        else if (name == "ndt_resolution") ndt_resolution_ = p.as_double();
        else if (name == "ndt_step_size") ndt_step_size_ = p.as_double();
        else if (name == "ndt_num_threads") ndt_num_threads_ = p.as_int();
        else if (name == "ndt_max_iterations") ndt_max_iterations_ = p.as_int();
        else if (name == "transform_epsilon") transform_epsilon_ = p.as_double();
        else if (name == "voxel_leaf_size") voxel_leaf_size_ = p.as_double();
        else if (name == "scan_max_range") scan_max_range_ = p.as_double();
        else if (name == "scan_min_range") scan_min_range_ = p.as_double();
        else if (name == "scan_period") scan_period_ = p.as_double();
        else if (name == "use_pcd_map") use_pcd_map_ = p.as_bool();
        else if (name == "map_path") map_path_ = p.as_string();
        else if (name == "set_initial_pose") set_initial_pose_ = p.as_bool();
        else if (name == "initial_pose_x") initial_pose_x_ = p.as_double();
        else if (name == "initial_pose_y") initial_pose_y_ = p.as_double();
        else if (name == "initial_pose_z") initial_pose_z_ = p.as_double();
        else if (name == "initial_pose_qx") initial_pose_qx_ = p.as_double();
        else if (name == "initial_pose_qy") initial_pose_qy_ = p.as_double();
        else if (name == "initial_pose_qz") initial_pose_qz_ = p.as_double();
        else if (name == "initial_pose_qw") initial_pose_qw_ = p.as_double();
        else if (name == "use_odom") use_odom_ = p.as_bool();
        else if (name == "use_imu") use_imu_ = p.as_bool();
        else if (name == "enable_debug") enable_debug_ = p.as_bool();
        else if (name == "enable_timer_publishing") enable_timer_publishing = p.as_bool();
        else if (name == "pose_publish_frequency") pose_publish_frequency_ = p.as_double();
        else if (name == "displacement_threshold") displacement_threshold_ = p.as_double();
        else if (name == "min_localization_interval") min_localization_interval_ = p.as_double();
        else if (name == "search_radius") search_radius_ = p.as_double();
        else if (name == "search_grid_size") search_grid_size_ = p.as_int();
        else if (name == "enable_displacement_check") enable_displacement_check_ = p.as_bool();
        else if (name == "enable_search_optimization") enable_search_optimization_ = p.as_bool();
        else if (name == "map_downsample_leaf_size") map_downsample_leaf_size_ = p.as_double();
        else if (name == "enable_angle_search") enable_angle_search_ = p.as_bool();
        else if (name == "angle_search_range") angle_search_range_ = p.as_double();
        else if (name == "angle_search_steps") angle_search_steps_ = p.as_int();
        else if (name == "enable_z_axis_search") enable_z_axis_search_ = p.as_bool();
        else if (name == "enable_dynamic_threshold") enable_dynamic_threshold_ = p.as_bool();
        else if (name == "dynamic_threshold_factor") dynamic_threshold_factor_ = p.as_double();
        else if (name == "ongoing_score_threshold_x") ongoing_score_threshold_x_ = p.as_double();
        else if (name == "ongoing_score_threshold_y") ongoing_score_threshold_y_ = p.as_double();
        else if (name == "ongoing_score_threshold_z") ongoing_score_threshold_z_ = p.as_double();
        else if (name == "initial_localization_accumulate_frames") initial_localization_accumulate_frames_ = p.as_int();
        else if (name == "enable_origin_baseline") enable_origin_baseline_ = p.as_bool();
        else if (name == "origin_baseline_frames") origin_baseline_frames_ = p.as_int();
        else if (name == "origin_baseline_radius") origin_baseline_radius_ = p.as_double();
        else if (name == "origin_baseline_match_interval") origin_baseline_match_interval_ = p.as_double();
        else if (name == "origin_baseline_base_frame") origin_baseline_base_frame_ = p.as_string();
        else if (name == "origin_baseline_pcd_path") origin_baseline_pcd_path_ = p.as_string();
        else if (name == "gicp_corr_dist_threshold") gicp_corr_dist_threshold_ = p.as_double();
        else if (name == "gicp_rotation_epsilon") gicp_rotation_epsilon_ = p.as_double();
        else if (name == "gicp_k_correspondences") gicp_k_correspondences_ = p.as_int();
        else if (name == "gicp_max_optimizer_iterations") gicp_max_optimizer_iterations_ = p.as_int();
        else if (name == "gicp_epsilon") gicp_epsilon_ = p.as_double();
        else if (name == "map_topic") map_topic_ = p.as_string();
      }
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      return result;
    });
}

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn PCLLocalization::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  initializeParameters();
  initializePubSub();
  initializeRegistration();

  path_ptr_ = std::make_shared<nav_msgs::msg::Path>();
  path_ptr_->header.frame_id = global_frame_id_;

  RCLCPP_INFO(get_logger(), "Configuring end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");

  pose_pub_->on_activate();
  map_odom_pose_pub_->on_activate();
  path_pub_->on_activate();
  initial_map_pub_->on_activate();

  if (set_initial_pose_) {
    auto msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

    msg->header.stamp = now();
    msg->header.frame_id = global_frame_id_;
    msg->pose.pose.position.x = initial_pose_x_;
    msg->pose.pose.position.y = initial_pose_y_;
    msg->pose.pose.position.z = initial_pose_z_;
    msg->pose.pose.orientation.x = initial_pose_qx_;
    msg->pose.pose.orientation.y = initial_pose_qy_;
    msg->pose.pose.orientation.z = initial_pose_qz_;
    msg->pose.pose.orientation.w = initial_pose_qw_;

    geometry_msgs::msg::PoseStamped::SharedPtr pose_stamped(new geometry_msgs::msg::PoseStamped);
    pose_stamped->header.stamp = msg->header.stamp;
    pose_stamped->header.frame_id = global_frame_id_;
    pose_stamped->pose = msg->pose.pose;
    path_ptr_->poses.push_back(*pose_stamped);

    // Initialize last localization position and reset first localization flag
    last_localization_x_ = initial_pose_x_;
    last_localization_y_ = initial_pose_y_;
    last_localization_z_ = initial_pose_z_;
    first_localization_done_ = false;  // Force first localization on next cloud
    first_localization_attempted_ = false;
    
    accumulated_cloud_ptr_->clear();
    accumulated_frame_count_ = 0;

    initialPoseReceived(msg);
  }

  if (use_pcd_map_) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    // load a pcd or ply file
    if (map_path_.rfind(".pcd") != std::string::npos) {
      RCLCPP_INFO(get_logger(), "Loading pcd map from: %s", map_path_.c_str());
      if (pcl::io::loadPCDFile(map_path_, *map_cloud_ptr) == -1) {
        RCLCPP_ERROR(get_logger(), "Failed to load pcd file: %s", map_path_.c_str());
        return CallbackReturn::FAILURE;
      }
    } else if (map_path_.rfind(".ply") != std::string::npos) {
      RCLCPP_INFO(get_logger(), "Loading ply map from: %s", map_path_.c_str());
      if (pcl::io::loadPLYFile(map_path_, *map_cloud_ptr) == -1) {
        RCLCPP_ERROR(get_logger(), "Failed to load ply file: %s", map_path_.c_str());
        return CallbackReturn::FAILURE;
      }
    } else {
      RCLCPP_ERROR(
          get_logger(), "Unsupported map file format. Please use .pcd or .ply: %s",
          map_path_.c_str());
      return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(get_logger(), "Map Size %ld", map_cloud_ptr->size());
    
    // Downsample the map to reduce size to ~100k points
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    map_downsample_filter_.setInputCloud(map_cloud_ptr);
    map_downsample_filter_.filter(*downsampled_cloud_ptr);
    RCLCPP_INFO(get_logger(), "Downsampled Map Size %ld", downsampled_cloud_ptr->size());
    
    // Save downsampled map in same folder as original map with _down suffix
    std::string downsampled_map_path = map_path_;
    size_t ext_pos = downsampled_map_path.rfind('.');
    if (ext_pos != std::string::npos) {
      downsampled_map_path.insert(ext_pos, "_down");
    } else {
      downsampled_map_path += "_down";
    }
    if (pcl::io::savePCDFileASCII(downsampled_map_path, *downsampled_cloud_ptr) == -1) {
      RCLCPP_ERROR(get_logger(), "Failed to save downsampled map to: %s", downsampled_map_path.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Downsampled map saved to: %s", downsampled_map_path.c_str());
    }
    
    // Publish downsampled map
    sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*downsampled_cloud_ptr, *map_msg_ptr);
    map_msg_ptr->header.frame_id = global_frame_id_;
    initial_map_pub_->publish(*map_msg_ptr);
    RCLCPP_INFO(get_logger(), "Initial Map Published");

    if (registration_method_ == "GICP" || registration_method_ == "GICP_OMP") {
      pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
      voxel_grid_filter_.setInputCloud(map_cloud_ptr);
      voxel_grid_filter_.filter(*filtered_cloud_ptr);
      registration_->setInputTarget(filtered_cloud_ptr);
    } else {
      registration_->setInputTarget(map_cloud_ptr);
    }

    buildTargetKdTree();

    map_recieved_ = true;
  }

  // Create performance statistics timer (30 seconds interval)
  performance_timer_ = create_wall_timer(
    std::chrono::seconds(120),
    std::bind(&PCLLocalization::performanceTimerCallback, this));

  RCLCPP_INFO(get_logger(), "Activating end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  pose_pub_->on_deactivate();
  map_odom_pose_pub_->on_deactivate();
  path_pub_->on_deactivate();
  initial_map_pub_->on_deactivate();

  RCLCPP_INFO(get_logger(), "Deactivating end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning Up");
  initial_pose_sub_.reset();
  initial_map_pub_.reset();
  path_pub_.reset();
  pose_pub_.reset();
  odom_sub_.reset();
  cloud_sub_.reset();
  imu_sub_.reset();

  if (enable_timer_publishing){
    pose_publish_timer_.reset();
  }

  RCLCPP_INFO(get_logger(), "Cleaning Up end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Shutting Down from %s", state.label().c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_FATAL(get_logger(), "Error Processing from %s, shutting down for respawn...", state.label().c_str());

  // 退出进程以触发 launch 的 respawn 机制
  // 如果不退出，节点会卡在 error 状态不恢复
  std::thread([]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    rclcpp::shutdown();
  }).detach();

  return CallbackReturn::SUCCESS;
}

void PCLLocalization::initializeParameters()
{
  RCLCPP_INFO(get_logger(), "initializeParameters");
  get_parameter("global_frame_id", global_frame_id_);
  get_parameter("odom_frame_id", odom_frame_id_);
  get_parameter("base_frame_id", base_frame_id_);
  get_parameter("enable_map_odom_tf", enable_map_odom_tf_);
  get_parameter("registration_method", registration_method_);
  get_parameter("initial_score_threshold", initial_score_threshold_);
  get_parameter("ongoing_score_threshold", ongoing_score_threshold_);
  // Per-axis thresholds: default to ongoing_score_threshold_ if not explicitly set (backward compatible)
  get_parameter("ongoing_score_threshold_x", ongoing_score_threshold_x_);
  get_parameter("ongoing_score_threshold_y", ongoing_score_threshold_y_);
  get_parameter("ongoing_score_threshold_z", ongoing_score_threshold_z_);
  best_fitness_score_ = ongoing_score_threshold_;  // 初始化为严格阈值，只追踪低于"严格阈值"的好分数
  get_parameter("ndt_resolution", ndt_resolution_);
  get_parameter("ndt_step_size", ndt_step_size_);
  get_parameter("ndt_num_threads", ndt_num_threads_);
  get_parameter("ndt_max_iterations", ndt_max_iterations_);
  get_parameter("transform_epsilon", transform_epsilon_);
  get_parameter("voxel_leaf_size", voxel_leaf_size_);
  get_parameter("scan_max_range", scan_max_range_);
  get_parameter("scan_min_range", scan_min_range_);
  get_parameter("scan_period", scan_period_);
  get_parameter("use_pcd_map", use_pcd_map_);
  get_parameter("map_path", map_path_);
  get_parameter("set_initial_pose", set_initial_pose_);
  get_parameter("initial_pose_x", initial_pose_x_);
  get_parameter("initial_pose_y", initial_pose_y_);
  get_parameter("initial_pose_z", initial_pose_z_);
  get_parameter("initial_pose_qx", initial_pose_qx_);
  get_parameter("initial_pose_qy", initial_pose_qy_);
  get_parameter("initial_pose_qz", initial_pose_qz_);
  get_parameter("initial_pose_qw", initial_pose_qw_);
  get_parameter("use_odom", use_odom_);
  get_parameter("use_imu", use_imu_);
  get_parameter("enable_debug", enable_debug_);
  get_parameter("enable_timer_publishing", enable_timer_publishing);
  get_parameter("pose_publish_frequency", pose_publish_frequency_);

  // New parameters for improved localization
  get_parameter("displacement_threshold", displacement_threshold_);
  get_parameter("min_localization_interval", min_localization_interval_);
  get_parameter("search_radius", search_radius_);
  get_parameter("search_grid_size", search_grid_size_);
  get_parameter("enable_displacement_check", enable_displacement_check_);
  get_parameter("enable_search_optimization", enable_search_optimization_);

  // New parameters for map downsampling
  get_parameter("map_downsample_leaf_size", map_downsample_leaf_size_);

  // New parameters for angle search optimization
  get_parameter("enable_angle_search", enable_angle_search_);
  get_parameter("angle_search_range", angle_search_range_);
  get_parameter("angle_search_steps", angle_search_steps_);

  // New parameters for Z-axis search
  get_parameter("enable_z_axis_search", enable_z_axis_search_);

  // New parameters for dynamic score threshold mechanism
  get_parameter("enable_dynamic_threshold", enable_dynamic_threshold_);
  get_parameter("dynamic_threshold_factor", dynamic_threshold_factor_);
  get_parameter("initial_localization_accumulate_frames", initial_localization_accumulate_frames_);

  get_parameter("enable_origin_baseline", enable_origin_baseline_);
  get_parameter("origin_baseline_frames", origin_baseline_frames_);
  get_parameter("origin_baseline_radius", origin_baseline_radius_);
  get_parameter("origin_baseline_match_interval", origin_baseline_match_interval_);
  get_parameter("origin_baseline_base_frame", origin_baseline_base_frame_);
  get_parameter("origin_baseline_pcd_path", origin_baseline_pcd_path_);

  RCLCPP_INFO(get_logger(),"global_frame_id: %s", global_frame_id_.c_str());
  RCLCPP_INFO(get_logger(),"odom_frame_id: %s", odom_frame_id_.c_str());
  RCLCPP_INFO(get_logger(),"base_frame_id: %s", base_frame_id_.c_str());
  RCLCPP_INFO(get_logger(),"enable_map_odom_tf: %d", enable_map_odom_tf_);
  RCLCPP_INFO(get_logger(),"registration_method: %s", registration_method_.c_str());
  RCLCPP_INFO(get_logger(),"ndt_resolution: %lf", ndt_resolution_);
  RCLCPP_INFO(get_logger(),"ndt_step_size: %lf", ndt_step_size_);
  RCLCPP_INFO(get_logger(),"ndt_num_threads: %d", ndt_num_threads_);
  RCLCPP_INFO(get_logger(),"transform_epsilon: %lf", transform_epsilon_);
  RCLCPP_INFO(get_logger(),"voxel_leaf_size: %lf", voxel_leaf_size_);
  RCLCPP_INFO(get_logger(),"scan_max_range: %lf", scan_max_range_);
  RCLCPP_INFO(get_logger(),"scan_min_range: %lf", scan_min_range_);
  RCLCPP_INFO(get_logger(),"scan_period: %lf", scan_period_);
  RCLCPP_INFO(get_logger(),"use_pcd_map: %d", use_pcd_map_);
  RCLCPP_INFO(get_logger(),"map_path: %s", map_path_.c_str());
  RCLCPP_INFO(get_logger(),"set_initial_pose: %d", set_initial_pose_);
  RCLCPP_INFO(get_logger(),"use_odom: %d", use_odom_);
  RCLCPP_INFO(get_logger(),"use_imu: %d", use_imu_);
  RCLCPP_INFO(get_logger(),"enable_debug: %d", enable_debug_);
  RCLCPP_INFO(get_logger(),"enable_timer_publishing: %d", enable_timer_publishing);
  RCLCPP_INFO(get_logger(),"pose_publish_frequency: %lf", pose_publish_frequency_);
  RCLCPP_INFO(get_logger(),"displacement_threshold: %lf", displacement_threshold_);
  RCLCPP_INFO(get_logger(),"min_localization_interval: %lf", min_localization_interval_);
  RCLCPP_INFO(get_logger(),"search_radius: %lf", search_radius_);
  RCLCPP_INFO(get_logger(),"search_grid_size: %d", search_grid_size_);
  RCLCPP_INFO(get_logger(),"enable_displacement_check: %d", enable_displacement_check_);
  RCLCPP_INFO(get_logger(),"enable_search_optimization: %d", enable_search_optimization_);
  RCLCPP_INFO(get_logger(),"map_downsample_leaf_size: %lf", map_downsample_leaf_size_);
  RCLCPP_INFO(get_logger(),"enable_z_axis_search: %d", enable_z_axis_search_);
  RCLCPP_INFO(get_logger(),"enable_angle_search: %d", enable_angle_search_);
  RCLCPP_INFO(get_logger(),"angle_search_range: %lf (deg: %lf)", angle_search_range_, angle_search_range_ * 180.0 / M_PI);
  RCLCPP_INFO(get_logger(),"angle_search_steps: %d", angle_search_steps_);
  RCLCPP_INFO(get_logger(),"enable_dynamic_threshold: %d", enable_dynamic_threshold_);
  RCLCPP_INFO(get_logger(),"dynamic_threshold_factor: %lf", dynamic_threshold_factor_);
  RCLCPP_INFO(get_logger(),"ongoing_score_threshold_x: %lf", ongoing_score_threshold_x_);
  RCLCPP_INFO(get_logger(),"ongoing_score_threshold_y: %lf", ongoing_score_threshold_y_);
  RCLCPP_INFO(get_logger(),"ongoing_score_threshold_z: %lf", ongoing_score_threshold_z_);
  RCLCPP_INFO(get_logger(),"initial_localization_accumulate_frames: %d", initial_localization_accumulate_frames_);
  RCLCPP_INFO(get_logger(),
              "enable_origin_baseline: %d, frames: %d, radius: %.2f, match_interval: %.2f, anchor_frame: %s",
              enable_origin_baseline_, origin_baseline_frames_, origin_baseline_radius_,
              origin_baseline_match_interval_, origin_baseline_base_frame_.c_str());

  // GICP-specific parameters
  get_parameter("gicp_corr_dist_threshold", gicp_corr_dist_threshold_);
  get_parameter("gicp_rotation_epsilon", gicp_rotation_epsilon_);
  get_parameter("gicp_k_correspondences", gicp_k_correspondences_);
  get_parameter("gicp_max_optimizer_iterations", gicp_max_optimizer_iterations_);
  get_parameter("gicp_epsilon", gicp_epsilon_);
  
  RCLCPP_INFO(get_logger(),"gicp_corr_dist_threshold: %lf", gicp_corr_dist_threshold_);
  RCLCPP_INFO(get_logger(),"gicp_rotation_epsilon: %lf", gicp_rotation_epsilon_);
  RCLCPP_INFO(get_logger(),"gicp_k_correspondences: %d", gicp_k_correspondences_);
  RCLCPP_INFO(get_logger(),"gicp_max_optimizer_iterations: %d", gicp_max_optimizer_iterations_);
  RCLCPP_INFO(get_logger(),"gicp_epsilon: %lf", gicp_epsilon_);
  get_parameter("map_topic", map_topic_);
  RCLCPP_INFO(get_logger(),"map_topic: %s", map_topic_.c_str());
}

void PCLLocalization::initializePubSub()
{
  RCLCPP_INFO(get_logger(), "initializePubSub");

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "pcl_pose",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  map_odom_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "lidar_localization_pose",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "path",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  initial_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "initial_map",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", rclcpp::SystemDefaultsQoS(),
    std::bind(&PCLLocalization::initialPoseReceived, this, std::placeholders::_1));

  map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&PCLLocalization::mapReceived, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
    std::bind(&PCLLocalization::odomReceived, this, std::placeholders::_1));

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
    std::bind(&PCLLocalization::cloudReceived, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "imu", rclcpp::QoS(rclcpp::KeepLast(100)).best_effort(),
    std::bind(&PCLLocalization::imuReceived, this, std::placeholders::_1));

  if (enable_timer_publishing) {
    auto period = std::chrono::duration<double>(1.0 / pose_publish_frequency_);
    pose_publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PCLLocalization::timerPublishPose, this));
  }

  RCLCPP_INFO(get_logger(), "initializePubSub end");
}

void PCLLocalization::initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "initialPoseReceived");
  if (msg->header.frame_id != global_frame_id_) {
    RCLCPP_WARN(this->get_logger(), "initialpose_frame_id does not match global_frame_id");
    return;
  }
  initialpose_recieved_ = true;
  corrent_pose_with_cov_stamped_ptr_ = msg;
  
  // Initialize last localization position and force next cloud to trigger NDT
  last_localization_x_ = msg->pose.pose.position.x;
  last_localization_y_ = msg->pose.pose.position.y;
  last_localization_z_ = msg->pose.pose.position.z;
  first_localization_done_ = false;  // Force NDT on next cloud
  
  // Do NOT clear accumulated_cloud_ptr_ — accumulation data should persist.
  // Displacement is now computed as absolute distance from odom_at_localization_
  // (set after each successful localization), so repeated initialPose won't
  // cause continuous NDT when the robot is stationary.
  
  pose_pub_->publish(*corrent_pose_with_cov_stamped_ptr_);

  if(last_scan_ptr_) {
    cloudReceived(last_scan_ptr_);
  }

  RCLCPP_INFO(get_logger(), "initialPoseReceived end");
}

void PCLLocalization::mapReceived(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "mapReceived");
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  if (msg->header.frame_id != global_frame_id_) {
    RCLCPP_WARN(this->get_logger(), "map_frame_id does not match　global_frame_id");
    return;
  }

  pcl::fromROSMsg(*msg, *map_cloud_ptr);

  if (registration_method_ == "GICP" || registration_method_ == "GICP_OMP") {
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    voxel_grid_filter_.setInputCloud(map_cloud_ptr);
    voxel_grid_filter_.filter(*filtered_cloud_ptr);
    registration_->setInputTarget(filtered_cloud_ptr);

  } else {
    registration_->setInputTarget(map_cloud_ptr);
  }

  buildTargetKdTree();

  map_recieved_ = true;
  RCLCPP_INFO(get_logger(), "mapReceived end");
}

void PCLLocalization::odomReceived(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  // 生命周期节点激活前不处理消息，避免未激活的 LifecyclePublisher 触发 WARN
  if (!pose_pub_->is_activated()) return;

  if (!use_odom_) {
    RCLCPP_WARN(get_logger(), "use_odom is disabled, ignoring odom data");
    return;
  }
  
  if (!corrent_pose_with_cov_stamped_ptr_) {
    RCLCPP_WARN(get_logger(), "corrent_pose_with_cov_stamped_ptr_ is null, attempting to initialize from odom data");
    
    auto initial_pose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    initial_pose->header = msg->header;
    initial_pose->header.frame_id = global_frame_id_;
    initial_pose->pose.pose = msg->pose.pose;
    
    for (int i = 0; i < 36; ++i) {
      initial_pose->pose.covariance[i] = 0.0;
    }
    initial_pose->pose.covariance[0] = 1.0;
    initial_pose->pose.covariance[7] = 1.0;
    initial_pose->pose.covariance[14] = 1.0;
    initial_pose->pose.covariance[21] = 0.1;
    initial_pose->pose.covariance[28] = 0.1;
    initial_pose->pose.covariance[35] = 0.1;
    
    initialpose_recieved_ = true;
    corrent_pose_with_cov_stamped_ptr_ = initial_pose;
    
    last_localization_x_ = initial_pose->pose.pose.position.x;
    last_localization_y_ = initial_pose->pose.pose.position.y;
    last_localization_z_ = initial_pose->pose.pose.position.z;
    first_localization_done_ = false;
    accumulated_cloud_ptr_->clear();
    accumulated_frame_count_ = 0;
    
    RCLCPP_INFO(get_logger(), "Initialized pose from odom: x=%.3f, y=%.3f, z=%.3f",
                initial_pose->pose.pose.position.x,
                initial_pose->pose.pose.position.y,
                initial_pose->pose.pose.position.z);
    
    pose_pub_->publish(*corrent_pose_with_cov_stamped_ptr_);
    
    if(last_scan_ptr_) {
      cloudReceived(last_scan_ptr_);
    }
    
    return;
  }
  
  // Update init_guess for NDT by composing map->odom * odom->base from TF tree.
  // This is only used as the initial guess for the next NDT alignment — it does NOT
  // affect the published map->odom transform (that comes from NDT output).
  // When TF timestamps are out of sync, we fall back to the latest available transform
  // rather than discarding the update entirely.
  
  // Step 1: Look up odom->base transform
  geometry_msgs::msg::TransformStamped odom_to_base_stamped;
  try {
    odom_to_base_stamped = tfbuffer_.lookupTransform(
      odom_frame_id_, base_frame_id_, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
  } catch (const tf2::TransformException & ex) {
    // Fallback to latest available transform
    try {
      odom_to_base_stamped = tfbuffer_.lookupTransform(
        odom_frame_id_, base_frame_id_, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return;  // No TF data at all, skip this odom message
    }
  }
  
  // Step 2: Look up map->odom transform (published by this node after each localization)
  geometry_msgs::msg::TransformStamped map_to_odom_stamped;
  bool map_to_odom_valid = true;
  try {
    map_to_odom_stamped = tfbuffer_.lookupTransform(
      global_frame_id_, odom_frame_id_, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    map_to_odom_valid = false;
  }
  
  if (map_to_odom_valid && first_localization_done_) {
    // Compose: map->base = map->odom * odom->base
    tf2::Transform map_to_odom_tf, odom_to_base_tf, map_to_base_tf;
    tf2::fromMsg(map_to_odom_stamped.transform, map_to_odom_tf);
    tf2::fromMsg(odom_to_base_stamped.transform, odom_to_base_tf);
    map_to_base_tf = map_to_odom_tf * odom_to_base_tf;
    
    corrent_pose_with_cov_stamped_ptr_->header.stamp = msg->header.stamp;
    corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x = map_to_base_tf.getOrigin().x();
    corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y = map_to_base_tf.getOrigin().y();
    corrent_pose_with_cov_stamped_ptr_->pose.pose.position.z = map_to_base_tf.getOrigin().z();
    corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation = tf2::toMsg(map_to_base_tf.getRotation());
  }
  
  // Compare current odom position against odom position at last localization (absolute, not accumulated)
  // Accumulated delta is fragile when TF timestamps are out of sync, causing artificial drift.
  double dx = odom_to_base_stamped.transform.translation.x - odom_at_localization_.position.x;
  double dy = odom_to_base_stamped.transform.translation.y - odom_at_localization_.position.y;
  double dz = odom_to_base_stamped.transform.translation.z - odom_at_localization_.position.z;
  accumulated_odom_distance_ = std::sqrt(dx*dx + dy*dy + dz*dz);
}

void PCLLocalization::imuReceived(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if (!use_imu_) {return;}

  sensor_msgs::msg::Imu tf_converted_imu;

  try {
    const geometry_msgs::msg::TransformStamped transform = tfbuffer_.lookupTransform(
     base_frame_id_, msg->header.frame_id, tf2::TimePointZero);

    geometry_msgs::msg::Vector3Stamped angular_velocity, linear_acceleration, transformed_angular_velocity, transformed_linear_acceleration;
    geometry_msgs::msg::Quaternion  transformed_quaternion;

    angular_velocity.header = msg->header;
    angular_velocity.vector = msg->angular_velocity;
    linear_acceleration.header = msg->header;
    linear_acceleration.vector = msg->linear_acceleration;

    tf2::doTransform(angular_velocity, transformed_angular_velocity, transform);
    tf2::doTransform(linear_acceleration, transformed_linear_acceleration, transform);

    tf_converted_imu.angular_velocity = transformed_angular_velocity.vector;
    tf_converted_imu.linear_acceleration = transformed_linear_acceleration.vector;
    tf_converted_imu.orientation = transformed_quaternion;

  }
  catch (tf2::TransformException& ex)
  {
    std::cout << "Failed to lookup transform" << std::endl;
    RCLCPP_WARN(this->get_logger(), "Failed to lookup transform.");
    return;
  }

  Eigen::Vector3f angular_velo{static_cast<float>(tf_converted_imu.angular_velocity.x), static_cast<float>(tf_converted_imu.angular_velocity.y),
    static_cast<float>(tf_converted_imu.angular_velocity.z)};
  Eigen::Vector3f acc{static_cast<float>(tf_converted_imu.linear_acceleration.x), static_cast<float>(tf_converted_imu.linear_acceleration.y), static_cast<float>(tf_converted_imu.linear_acceleration.z)};
  Eigen::Quaternionf quat{static_cast<float>(msg->orientation.w), static_cast<float>(msg->orientation.x), static_cast<float>(msg->orientation.y),
    static_cast<float>(msg->orientation.z)};
  double imu_time = msg->header.stamp.sec +
    msg->header.stamp.nanosec * 1e-9;

  lidar_undistortion_.getImu(angular_velo, acc, quat, imu_time);

}

bool PCLLocalization::processOriginBaseline(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  // 锚定系：与航点/控制点一致（/lio/robo/odom child = base_link）
  const std::string anchor_frame =
    origin_baseline_base_frame_.empty() ? base_frame_id_ : origin_baseline_base_frame_;

  if (origin_baseline_aborted_) {
    return false;  // 已放弃基准，不再拦截全局定位，也免去点云变换开销
  }

  // 当前 map->anchor（TF 优先按帧时间戳，退化到最新）
  geometry_msgs::msg::TransformStamped map_to_anchor_stamped;
  try {
    map_to_anchor_stamped = tfbuffer_.lookupTransform(
      global_frame_id_, anchor_frame, msg->header.stamp,
      rclcpp::Duration::from_seconds(0.1));
  } catch (const tf2::TransformException &) {
    try {
      map_to_anchor_stamped = tfbuffer_.lookupTransform(
        global_frame_id_, anchor_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return false;  // TF 尚不可用，本帧交给正常流程
    }
  }
  tf2::Transform map_to_anchor;
  tf2::fromMsg(map_to_anchor_stamped.transform, map_to_anchor);
  const tf2::Vector3 & pos = map_to_anchor.getOrigin();
  const double dist_xy = std::hypot(pos.x(), pos.y());
  const bool in_zone = dist_xy < origin_baseline_radius_;

  // 基准已就绪且不在圈内（或正在离圈）：不处理点云，交回主流程
  if (origin_baseline_ready_ && !in_zone) {
    // 首次离圈 = 起飞动作，此后才武装降落匹配（防止坪上待机时误接管 map->odom）
    if (!origin_baseline_departed_) {
      origin_baseline_departed_ = true;
      RCLCPP_INFO(get_logger(),
        "Origin baseline armed: left takeoff zone (xy=%.2f m), landing refinement will engage on return",
        dist_xy);
    }
    if (in_landing_zone_) {
      in_landing_zone_ = false;
      RCLCPP_INFO(get_logger(),
        "Left origin landing zone (xy=%.2f m), best baseline fitness=%.6f, resuming global map localization",
        dist_xy, landing_best_fitness_);
    }
    return false;
  }

  // 未起飞（基准已就绪但从未离圈）：降落匹配不接管，全局定位保持权威。
  // 飞机还停在坪上，此时做"降落匹配"只会用恒等修正干扰 map->odom。
  // 注意：必须在 ready_ 判定之后，否则会挡住上面的基准积累。
  if (origin_baseline_ready_ && !origin_baseline_departed_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
      "Origin baseline ready on pad; landing refinement arms after first takeoff (xy=%.2f m)",
      dist_xy);
    return false;
  }

  // ===== 原始点云独立变换到锚定系（不依赖主流程 base_footprint 变换） =====
  pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *raw_cloud);
  if (msg->header.frame_id != anchor_frame) {
    geometry_msgs::msg::TransformStamped anchor_to_src_stamped;
    try {
      anchor_to_src_stamped = tfbuffer_.lookupTransform(
        anchor_frame, msg->header.frame_id, msg->header.stamp,
        rclcpp::Duration::from_seconds(0.1));
    } catch (const tf2::TransformException &) {
      try {
        anchor_to_src_stamped = tfbuffer_.lookupTransform(
          anchor_frame, msg->header.frame_id, tf2::TimePointZero);
      } catch (const tf2::TransformException &) {
        return false;  // 无法变换到锚定系，本帧交给正常流程
      }
    }
    Eigen::Matrix4f anchor_to_src =
      tf2::transformToEigen(anchor_to_src_stamped.transform).matrix().cast<float>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*raw_cloud, *transformed, anchor_to_src);
    raw_cloud = transformed;
  }
  // 体素滤波 + 距离裁剪（与主管线同参数，保持源/目标密度一致）
  pcl::PointCloud<pcl::PointXYZI>::Ptr voxelized(new pcl::PointCloud<pcl::PointXYZI>);
  voxel_grid_filter_.setInputCloud(raw_cloud);
  voxel_grid_filter_.filter(*voxelized);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_anchor(
    new pcl::PointCloud<pcl::PointXYZI>);
  for (const auto & p : voxelized->points) {
    const double r = std::sqrt(p.x * p.x + p.y * p.y);
    if (scan_min_range_ < r && r < scan_max_range_) {
      cloud_anchor->push_back(p);
    }
  }

  if (!origin_baseline_ready_) {
    // ===== 启动阶段：先积累基准，期间挂起全局图定位（定位延后做，不冲突） =====
    if (in_zone) {
      if (origin_baseline_frame_count_ == 0) {
        RCLCPP_INFO(get_logger(),
          "Collecting origin baseline in %s frame (%d frames within %.1fm radius); "
          "global map localization paused until baseline ready",
          anchor_frame.c_str(), origin_baseline_frames_, origin_baseline_radius_);
      }
      Eigen::Matrix4f map_to_anchor_eigen =
        tf2::transformToEigen(map_to_anchor_stamped.transform).matrix().cast<float>();
      pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::transformPointCloud(*cloud_anchor, *map_cloud, map_to_anchor_eigen);
      *origin_baseline_cloud_ptr_ += *map_cloud;
      origin_baseline_frame_count_++;

      if (origin_baseline_frame_count_ >= origin_baseline_frames_) {
        // 基准点云完整保留（不降采样），作为降落阶段的独立高精度匹配地图
        origin_baseline_ready_ = true;
        createOriginRegistration();
        origin_registration_->setInputTarget(origin_baseline_cloud_ptr_);
        RCLCPP_INFO(get_logger(),
          "Origin baseline ready in %s frame: %d frames, %lu points (full resolution, no downsample), "
          "landing refinement arms within %.1fm of world origin; resuming global map localization",
          anchor_frame.c_str(), origin_baseline_frames_, origin_baseline_cloud_ptr_->size(),
          origin_baseline_radius_);

        // 基准点云按 map 系落盘供离线检查（CloudCompare/RViz 打开验证拼接质量）
        if (!origin_baseline_pcd_path_.empty()) {
          const int save_ret =
            pcl::io::savePCDFileBinary(origin_baseline_pcd_path_, *origin_baseline_cloud_ptr_);
          if (save_ret == 0) {
            RCLCPP_INFO(get_logger(),
              "Origin baseline cloud saved to %s (%s frame, %lu points)",
              origin_baseline_pcd_path_.c_str(), global_frame_id_.c_str(),
              origin_baseline_cloud_ptr_->size());
          } else {
            RCLCPP_WARN(get_logger(),
              "Failed to save origin baseline PCD to %s (ret=%d)",
              origin_baseline_pcd_path_.c_str(), save_ret);
          }
        }
        // 就绪当帧即放行，交给下面的初始定位流程处理
        return false;
      }
      return true;  // 积累中：挂起全局图定位，避免初始匹配修正 map->odom 打断基准拼接
    }

    // 未攒满就离开原点半径（如开机即起飞）：基准作废，立即恢复全局定位流程
    origin_baseline_aborted_ = true;
    if (origin_baseline_frame_count_ > 0) {
      RCLCPP_WARN(get_logger(),
        "Left origin radius (xy=%.2fm) with incomplete baseline (%d/%d frames), discarding",
        dist_xy, origin_baseline_frame_count_, origin_baseline_frames_);
      origin_baseline_cloud_ptr_->clear();
      origin_baseline_frame_count_ = 0;
    }
    RCLCPP_INFO(get_logger(),
      "Origin baseline aborted (drone outside %.1fm radius before collection finished); "
      "global map localization proceeds, landing falls back to global map matching",
      origin_baseline_radius_);
    return false;
  }

  // ===== 降落阶段：进圈立即开始，以配置周期(默认1Hz)与基准持续匹配 =====
  rclcpp::Time now_t = this->now();
  if (!in_landing_zone_) {
    in_landing_zone_ = true;
    landing_best_fitness_ = std::numeric_limits<double>::max();
    // 回拨一个周期，使首帧立即触发匹配
    last_baseline_match_time_ =
      rclcpp::Time(now_t.nanoseconds(), now_t.get_clock_type()) -
      rclcpp::Duration::from_seconds(origin_baseline_match_interval_);
    has_last_baseline_match_time_ = true;
    RCLCPP_INFO(get_logger(),
      "Entered origin landing zone (xy=%.2f m < %.2f m), matching baseline in %s frame every %.1fs, "
      "static TF keeps lowest-error result",
      dist_xy, origin_baseline_radius_, anchor_frame.c_str(), origin_baseline_match_interval_);
  }

  if (!has_last_baseline_match_time_ ||
      now_t - last_baseline_match_time_ >=
      rclcpp::Duration::from_seconds(origin_baseline_match_interval_))
  {
    last_baseline_match_time_ = now_t;
    has_last_baseline_match_time_ = true;
    runBaselineLandingMatch(cloud_anchor, map_to_anchor, rclcpp::Time(msg->header.stamp));
  }
  return true;  // 进圈后本帧由基准降落逻辑接管
}

void PCLLocalization::runBaselineLandingMatch(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud_anchor,
  const tf2::Transform & map_to_anchor_cur, const rclcpp::Time & cloud_stamp)
{
  // 单帧匹配：当前帧（锚定系）按本次已解析的 map->anchor 变换到 map 系作为源，
  // 不做多帧拼接（降落段飞机在动，拼接会产生拖影；也避免额外 TF 解析）
  const tf2::Vector3 & ao = map_to_anchor_cur.getOrigin();
  const tf2::Quaternion & ar = map_to_anchor_cur.getRotation();
  Eigen::Matrix4d map_to_anchor_mat = Eigen::Matrix4d::Identity();
  map_to_anchor_mat.block<3, 3>(0, 0) =
    Eigen::Quaterniond(ar.w(), ar.x(), ar.y(), ar.z()).toRotationMatrix();
  map_to_anchor_mat(0, 3) = ao.x();
  map_to_anchor_mat(1, 3) = ao.y();
  map_to_anchor_mat(2, 3) = ao.z();

  pcl::PointCloud<pcl::PointXYZI>::Ptr source_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(
    *cloud_anchor, *source_ptr, map_to_anchor_mat.cast<float>());

  const size_t min_points = 100;
  if (source_ptr->size() < min_points) {
    RCLCPP_DEBUG(get_logger(),
      "Baseline match skipped: merged source too sparse (%zu < %zu points)",
      source_ptr->size(), min_points);
    return;
  }

  rclcpp::Clock system_clock;
  rclcpp::Time t0 = system_clock.now();
  origin_registration_->setInputSource(source_ptr);
  pcl::PointCloud<pcl::PointXYZI>::Ptr align_out(new pcl::PointCloud<pcl::PointXYZI>);
  Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();  // 源已在map系，只需小幅精修
  origin_registration_->align(*align_out, guess);
  rclcpp::Time t1 = system_clock.now();
  addPerformanceStatistics(
    registration_method_.find("GICP") != std::string::npos ? "ICP" : "NDT",
    std::chrono::duration_cast<std::chrono::milliseconds>((t1 - t0).to_chrono<std::chrono::nanoseconds>()).count());

  if (!origin_registration_->hasConverged()) {
    RCLCPP_DEBUG(get_logger(), "Baseline match not converged, skipping");
    return;
  }
  const double fitness = origin_registration_->getFitnessScore();

  // 只接受比本次进圈后历史最低误差更好的结果
  if (!(fitness < landing_best_fitness_)) {
    RCLCPP_DEBUG(get_logger(),
      "Baseline match fitness %.6f >= best %.6f, keeping previous static TF",
      fitness, landing_best_fitness_);
    return;
  }

  // 精修结果（近似单位阵的修正量）叠加到当前 map->base
  Eigen::Matrix4f refined = origin_registration_->getFinalTransformation();

  // 合理性门：修正量应是小幅度调整，防止离群误匹配污染TF
  const double max_corr_translation = 1.0;   // m
  const double max_corr_rotation = 0.26;     // rad (~15deg)
  const double corr_translation = refined.block<3, 1>(0, 3).norm();
  const Eigen::AngleAxisf corr_angle(refined.block<3, 3>(0, 0).cast<float>());
  if (corr_translation > max_corr_translation || corr_angle.angle() > max_corr_rotation) {
    RCLCPP_WARN(get_logger(),
      "Baseline match rejected as outlier: corr_t=%.2fm corr_rot=%.1fdeg (fitness %.6f)",
      corr_translation, corr_angle.angle() * 180.0 / M_PI, fitness);
    return;
  }

  // 组合法（重要教训 2026-09-20 实机）：不要用 (refined*map->anchor)*inv(odom->anchor)
  // 这种"两次独立 anchor 查询"的组合——map->anchor 与 odom->anchor 各走一次 TF 解析，
  // 在 /tf_static 双发布竞态或带时间戳查询超时回退时，两者可能落在不同采样上，
  // 产生与锚定系无关的 ~179.4° 纯 yaw 组合差（fitness 正常、平移仅厘米级）。
  // 代数上 out = refined * old_map->odom（refined 是 map 系左乘量，尾链严格对消），
  // 因此直接单次查询当前 map->odom 后左乘 refined，全程只有一次 TF 解析。
  geometry_msgs::msg::TransformStamped prev_m2o_stamped;
  try {
    prev_m2o_stamped = tfbuffer_.lookupTransform(
      global_frame_id_, odom_frame_id_, cloud_stamp, rclcpp::Duration::from_seconds(0.2));
  } catch (const tf2::TransformException &) {
    try {
      prev_m2o_stamped = tfbuffer_.lookupTransform(
        global_frame_id_, odom_frame_id_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(),
        "Baseline match: no %s->%s TF available, skipping update: %s",
        global_frame_id_.c_str(), odom_frame_id_.c_str(), ex.what());
      return;
    }
  }
  tf2::Transform prev_m2o;
  tf2::fromMsg(prev_m2o_stamped.transform, prev_m2o);

  tf2::Transform refined_tf;
  refined_tf.setOrigin(
    tf2::Vector3(refined(0, 3), refined(1, 3), refined(2, 3)));
  Eigen::Quaterniond q_refined(refined.block<3, 3>(0, 0).cast<double>());
  q_refined.normalize();
  tf2::Quaternion q_t(q_refined.x(), q_refined.y(), q_refined.z(), q_refined.w());
  q_t.normalize();
  refined_tf.setRotation(q_t);

  const tf2::Transform map_to_odom_tf = refined_tf * prev_m2o;

  // 新历史最低误差：以静态TF发布，锁定为本次降落的 map->odom
  landing_best_fitness_ = fitness;

  geometry_msgs::msg::TransformStamped map_to_odom_stamped;
  map_to_odom_stamped.header.stamp = cloud_stamp;
  map_to_odom_stamped.header.frame_id = global_frame_id_;
  map_to_odom_stamped.child_frame_id = odom_frame_id_;
  map_to_odom_stamped.transform = tf2::toMsg(map_to_odom_tf);
  static_broadcaster_.sendTransform(map_to_odom_stamped);

  geometry_msgs::msg::PoseWithCovarianceStamped map_odom_pose_msg;
  map_odom_pose_msg.header.stamp = cloud_stamp;
  map_odom_pose_msg.header.frame_id = global_frame_id_;
  map_odom_pose_msg.pose.pose.position.x = map_to_odom_tf.getOrigin().x();
  map_odom_pose_msg.pose.pose.position.y = map_to_odom_tf.getOrigin().y();
  map_odom_pose_msg.pose.pose.position.z = map_to_odom_tf.getOrigin().z();
  map_odom_pose_msg.pose.pose.orientation = tf2::toMsg(map_to_odom_tf.getRotation());
  for (int i = 0; i < 36; ++i) {
    map_odom_pose_msg.pose.covariance[i] = 0.0;
  }
  map_odom_pose_msg.pose.covariance[0] = fitness;
  map_odom_pose_msg.pose.covariance[7] = fitness;
  map_odom_pose_msg.pose.covariance[14] = fitness;
  map_odom_pose_msg.pose.covariance[21] = fitness;
  map_odom_pose_msg.pose.covariance[28] = fitness;
  map_odom_pose_msg.pose.covariance[35] = fitness;
  if (map_odom_pose_pub_ && map_odom_pose_pub_->is_activated()) {
    map_odom_pose_pub_->publish(map_odom_pose_msg);
  }

  RCLCPP_INFO(get_logger(),
    "Landing baseline NEW BEST fitness=%.6f | map->odom t=(%.3f, %.3f, %.3f) q=(%.4f, %.4f, %.4f, %.4f) "
    "| corr t=%.3fm rot=%.1fdeg",
    fitness,
    map_to_odom_tf.getOrigin().x(), map_to_odom_tf.getOrigin().y(), map_to_odom_tf.getOrigin().z(),
    map_to_odom_tf.getRotation().x(), map_to_odom_tf.getRotation().y(),
    map_to_odom_tf.getRotation().z(), map_to_odom_tf.getRotation().w(),
    corr_translation, corr_angle.angle() * 180.0 / M_PI);
}

void PCLLocalization::cloudReceived(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (!msg) {
    RCLCPP_WARN(get_logger(), "Received null point cloud message");
    return;
  }

  if (!map_recieved_ || !initialpose_recieved_) {return;}
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud_ptr);

  // If your cloud is not robot-centric, convert to base_frame.
  if (msg->header.frame_id != base_frame_id_) {
    RCLCPP_DEBUG(
        this->get_logger(), "Transforming point cloud from %s to %s",
        msg->header.frame_id.c_str(), base_frame_id_.c_str());
    geometry_msgs::msg::TransformStamped base_to_lidar_stamped;
    try {
      base_to_lidar_stamped = tfbuffer_.lookupTransform(
          base_frame_id_, msg->header.frame_id, msg->header.stamp,
          rclcpp::Duration::from_seconds(0.5));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(
          this->get_logger(), "Could not transform %s to %s: %s",
          msg->header.frame_id.c_str(), base_frame_id_.c_str(), ex.what());
      return;
    }

    Eigen::Matrix4f initial_transformation =
      tf2::transformToEigen(base_to_lidar_stamped.transform).matrix().cast<float>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*cloud_ptr, *transformed_cloud, initial_transformation);
    cloud_ptr = transformed_cloud;
  }

  if (use_imu_) {
    double received_time = msg->header.stamp.sec +
      msg->header.stamp.nanosec * 1e-9;
    lidar_undistortion_.adjustDistortion(cloud_ptr, received_time);
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  voxel_grid_filter_.setInputCloud(cloud_ptr);
  voxel_grid_filter_.filter(*filtered_cloud_ptr);

  double r;
  pcl::PointCloud<pcl::PointXYZI> tmp;
  for (const auto & p : filtered_cloud_ptr->points) {
    r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
    if (scan_min_range_ < r && r < scan_max_range_) {
      tmp.push_back(p);
    }
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>(tmp));
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_for_registration = tmp_ptr;

  // 原点基准精准降落：先做基准积累/进圈匹配处理（内部按锚定系 base_link 自行变换）。
  // 返回 true 表示该帧已由基准逻辑接管：
  //  - 启动积累期：挂起全局图定位（先收基准，定位延后做，避免 map->odom 中途被修正）
  //  - 降落进圈后：跳过全局图匹配，由基准匹配的最低误差结果独占 map->odom 静态TF
  if (enable_origin_baseline_ && processOriginBaseline(msg)) {
    last_scan_ptr_ = msg;
    return;
  }

  if (!first_localization_done_ && initial_localization_accumulate_frames_ > 1) {
    *accumulated_cloud_ptr_ += *tmp_ptr;
    accumulated_frame_count_++;

    if (accumulated_frame_count_ < initial_localization_accumulate_frames_) {
      RCLCPP_DEBUG(get_logger(), "Accumulating frames for initial localization: %d/%d (points: %lu)",
                  accumulated_frame_count_, initial_localization_accumulate_frames_,
                  accumulated_cloud_ptr_->size());
      last_scan_ptr_ = msg;
      return;
    }

    RCLCPP_INFO(get_logger(), "Accumulated %d frames for initial localization, total points: %lu",
                accumulated_frame_count_, accumulated_cloud_ptr_->size());
    cloud_for_registration = accumulated_cloud_ptr_;
  }
  
  // 检查点云数量是否足够进行配准（GICP 需要 >= k_correspondences 个点）
  const size_t min_points_for_gicp = 100;  // 至少需要 100 个点
  if (cloud_for_registration->size() < min_points_for_gicp) {
    RCLCPP_WARN(get_logger(), 
      "Point cloud too sparse (%zu points < %zu), skipping localization",
      cloud_for_registration->size(), min_points_for_gicp);
    return;
  }
  
  registration_->setInputSource(cloud_for_registration);

  Eigen::Affine3d affine;
  tf2::fromMsg(corrent_pose_with_cov_stamped_ptr_->pose.pose, affine);

  Eigen::Matrix4f init_guess = affine.matrix().cast<float>();

  // Check if we should update localization based on displacement
  if (!shouldUpdateLocalization(corrent_pose_with_cov_stamped_ptr_->pose.pose)) {
    RCLCPP_DEBUG(get_logger(), "Displacement check failed, skipping localization");
    return;
  }
  
  // 记录定位尝试时间（用于限制重试频率）
  last_localization_attempt_time_ = this->now();

  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  rclcpp::Clock system_clock;
  rclcpp::Time time_align_start = system_clock.now();
  
  // Use search optimization to find best transformation
  SearchResult search_result = searchOptimalTransformation(cloud_for_registration, init_guess, enable_z_axis_search_);
  Eigen::Matrix4f final_transformation = search_result.transformation;
  
  rclcpp::Time time_align_end = system_clock.now();

  bool has_converged = search_result.has_converged;
  double fitness_score = search_result.fitness_score;
  
  // 质量检查：根据阶段使用不同阈值
  // - 初始定位（!first_localization_done_）: 使用宽松阈值 initial_score_threshold_
  // - 持续定位（first_localization_done_）: 使用严格阈值 ongoing_score_threshold_
  double effective_threshold = first_localization_done_ ? ongoing_score_threshold_ : initial_score_threshold_;
  
  // 初始定位时始终输出 fitness score
  if (!first_localization_done_) {
    RCLCPP_INFO(get_logger(), "Initial localization fitness score: %lf (threshold: %lf%s)",
                fitness_score, initial_score_threshold_,
                has_converged ? "" : ", not converged");
  }
  
  if (!has_converged) {
    // GICP may report hasConverged=false even with a valid solution (BFGS inner exception).
    // Use configured threshold to determine if the fitness score is acceptable.
    if (fitness_score >= effective_threshold) {
      RCLCPP_WARN(get_logger(), 
        "Registration not converged and fitness %.6f exceeds threshold %.6f, rejecting",
        fitness_score, effective_threshold);
      // 更新 odom 参考点，避免立即重试
      try {
        geometry_msgs::msg::TransformStamped odom_to_base = tfbuffer_.lookupTransform(
          odom_frame_id_, base_frame_id_, tf2::TimePointZero);
        odom_at_localization_.position.x = odom_to_base.transform.translation.x;
        odom_at_localization_.position.y = odom_to_base.transform.translation.y;
        odom_at_localization_.position.z = odom_to_base.transform.translation.z;
        odom_at_localization_.orientation = odom_to_base.transform.rotation;
      } catch (const tf2::TransformException &) {}
      return;
    }
    RCLCPP_WARN(get_logger(), 
      "Registration not converged but fitness %.6f within threshold %.6f, accepting",
      fitness_score, effective_threshold);
  }
  
  if (fitness_score > effective_threshold) {
    if (!first_localization_done_) {
      RCLCPP_INFO(get_logger(), "Initial localization REJECTED: fitness %lf > threshold %lf",
                  fitness_score, effective_threshold);
      RCLCPP_INFO(get_logger(), "Falling back to initial pose, first_localization_done_ set to true");

      // 初始定位失败时，直接使用初始位姿，标记完成，不再重复尝试匹配
      first_localization_done_ = true;
      accumulated_cloud_ptr_->clear();
      accumulated_frame_count_ = 0;

      // 用初始猜测位姿（即初始位姿）代替注册结果，继续后续发布/TF 流程
      final_transformation = init_guess;
      // 不修改 fitness_score，保持实际值（大于 best_fitness_score_，不会触发最小值更新）
      // 不 return，继续往下走到发布和 TF 广播逻辑
    } else {
      RCLCPP_DEBUG(get_logger(), "Localization skipped: fitness %lf > threshold %lf (phase: %s)",
                   fitness_score, effective_threshold,
                   first_localization_done_ ? "ongoing" : "initial");
      // 更新 odom 参考点，避免在失败位置立即重试
      try {
        geometry_msgs::msg::TransformStamped odom_to_base = tfbuffer_.lookupTransform(
          odom_frame_id_, base_frame_id_, tf2::TimePointZero);
        odom_at_localization_.position.x = odom_to_base.transform.translation.x;
        odom_at_localization_.position.y = odom_to_base.transform.translation.y;
        odom_at_localization_.position.z = odom_to_base.transform.translation.z;
        odom_at_localization_.orientation = odom_to_base.transform.rotation;
      } catch (const tf2::TransformException &) {}
      return;
    }
  }
  
  // Per-axis fitness score check（各轴独立阈值检查）
  // 仅在总体 fitness 通过后才执行（跳过初始定位失败的 fallback 场景）
  // 仅在持续定位阶段运行（初始定位时总体 fitness 阈值已足够，per-axis 可能因初始偏差集中在一轴导致无限重试）
  if (first_localization_done_ && fitness_score <= effective_threshold && target_kdtree_ && !target_kdtree_->getInputCloud()->empty()) {
    double fitness_x, fitness_y, fitness_z;
    computePerAxisFitnessScore(cloud_for_registration, final_transformation, fitness_x, fitness_y, fitness_z);
    
    if (fitness_x > ongoing_score_threshold_x_ ||
        fitness_y > ongoing_score_threshold_y_ ||
        fitness_z > ongoing_score_threshold_z_) {
      RCLCPP_INFO(get_logger(),
        "Localization REJECTED per-axis: x=%.6f(thresh=%.6f) y=%.6f(thresh=%.6f) z=%.6f(thresh=%.6f)",
        fitness_x, ongoing_score_threshold_x_,
        fitness_y, ongoing_score_threshold_y_,
        fitness_z, ongoing_score_threshold_z_);
      // 更新 odom 参考点，避免在失败位置立即重试
      try {
        geometry_msgs::msg::TransformStamped odom_to_base = tfbuffer_.lookupTransform(
          odom_frame_id_, base_frame_id_, tf2::TimePointZero);
        odom_at_localization_.position.x = odom_to_base.transform.translation.x;
        odom_at_localization_.position.y = odom_to_base.transform.translation.y;
        odom_at_localization_.position.z = odom_to_base.transform.translation.z;
        odom_at_localization_.orientation = odom_to_base.transform.rotation;
      } catch (const tf2::TransformException &) {}
      return;
    }
    
    search_result.fitness_score_x = fitness_x;
    search_result.fitness_score_y = fitness_y;
    search_result.fitness_score_z = fitness_z;
    
    RCLCPP_INFO(get_logger(), "Localization ACCEPTED per-axis: x=%.6f(thresh=%.6f) y=%.6f(thresh=%.6f) z=%.6f(thresh=%.6f)",
                 fitness_x, ongoing_score_threshold_x_,
                 fitness_y, ongoing_score_threshold_y_,
                 fitness_z, ongoing_score_threshold_z_);
  }
  
  // Update current fitness score
  current_fitness_score_ = fitness_score;
  if (fitness_score < best_fitness_score_) {
    best_fitness_score_ = fitness_score;
    RCLCPP_INFO(get_logger(), "New minimum fitness score: %lf", fitness_score);
  }
  
  Eigen::Matrix3d rot_mat = final_transformation.block<3, 3>(0, 0).cast<double>();
  Eigen::Quaterniond quat_eig(rot_mat);
  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);

  corrent_pose_with_cov_stamped_ptr_->header.stamp = msg->header.stamp;
  corrent_pose_with_cov_stamped_ptr_->header.frame_id = global_frame_id_;
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x = static_cast<double>(final_transformation(0, 3));
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y = static_cast<double>(final_transformation(1, 3));
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.z = static_cast<double>(final_transformation(2, 3));
  corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation = quat_msg;
  
  // Mark first localization as done after successful pose update
  if (!first_localization_done_) {
    first_localization_done_ = true;
    accumulated_cloud_ptr_->clear();
    accumulated_frame_count_ = 0;
    RCLCPP_INFO(get_logger(), "First localization completed successfully");
  }
    
  // publish here if timer is not enabled

  if (!enable_timer_publishing){
    pose_pub_->publish(*corrent_pose_with_cov_stamped_ptr_);
  }

  geometry_msgs::msg::TransformStamped map_to_base_link_stamped;
  map_to_base_link_stamped.header.stamp = msg->header.stamp;
  map_to_base_link_stamped.header.frame_id = global_frame_id_;
  map_to_base_link_stamped.child_frame_id = base_frame_id_;
  map_to_base_link_stamped.transform.translation.x = static_cast<double>(final_transformation(0, 3));
  map_to_base_link_stamped.transform.translation.y = static_cast<double>(final_transformation(1, 3));
  map_to_base_link_stamped.transform.translation.z = static_cast<double>(final_transformation(2, 3));
  map_to_base_link_stamped.transform.rotation = quat_msg;

  tf2::Transform map_to_base_link_tf;
  tf2::fromMsg(map_to_base_link_stamped.transform, map_to_base_link_tf);

  geometry_msgs::msg::TransformStamped odom_to_base_link_msg;
  try {
    odom_to_base_link_msg = tfbuffer_.lookupTransform(
      odom_frame_id_, base_frame_id_, msg->header.stamp, rclcpp::Duration::from_seconds(1.0));
  } catch (tf2::TransformException & ex) {
    // Try with latest available time as fallback
    RCLCPP_WARN(this->get_logger(),
      "Could not get transform %s to %s at stamp %.3f, trying latest: %s",
      odom_frame_id_.c_str(), base_frame_id_.c_str(),
      rclcpp::Time(msg->header.stamp).seconds(), ex.what());
    try {
      odom_to_base_link_msg = tfbuffer_.lookupTransform(
        odom_frame_id_, base_frame_id_, tf2::TimePointZero);
    } catch (tf2::TransformException & ex2) {
      RCLCPP_ERROR(this->get_logger(),
        "Could not get transform %s to %s even at latest: %s. Skipping TF publish.",
        odom_frame_id_.c_str(), base_frame_id_.c_str(), ex2.what());
      return;
    }
  }
  tf2::Transform odom_to_base_link_tf;
  tf2::fromMsg(odom_to_base_link_msg.transform, odom_to_base_link_tf);

  // Update odom reference for displacement check
  odom_at_localization_.position.x = odom_to_base_link_msg.transform.translation.x;
  odom_at_localization_.position.y = odom_to_base_link_msg.transform.translation.y;
  odom_at_localization_.position.z = odom_to_base_link_msg.transform.translation.z;
  odom_at_localization_.orientation = odom_to_base_link_msg.transform.rotation;

  tf2::Transform map_to_odom_tf = map_to_base_link_tf * odom_to_base_link_tf.inverse();
  geometry_msgs::msg::TransformStamped map_to_odom_stamped;
  map_to_odom_stamped.header.stamp = msg->header.stamp;
  map_to_odom_stamped.header.frame_id = global_frame_id_;
  map_to_odom_stamped.child_frame_id = odom_frame_id_;
  map_to_odom_stamped.transform = tf2::toMsg(map_to_odom_tf);
  static_broadcaster_.sendTransform(map_to_odom_stamped);

  geometry_msgs::msg::PoseWithCovarianceStamped map_odom_pose_msg;
  map_odom_pose_msg.header.stamp = msg->header.stamp;
  map_odom_pose_msg.header.frame_id = global_frame_id_;
  map_odom_pose_msg.pose.pose.position.x = map_to_odom_tf.getOrigin().x();
  map_odom_pose_msg.pose.pose.position.y = map_to_odom_tf.getOrigin().y();
  map_odom_pose_msg.pose.pose.position.z = map_to_odom_tf.getOrigin().z();
  map_odom_pose_msg.pose.pose.orientation.x = map_to_odom_tf.getRotation().x();
  map_odom_pose_msg.pose.pose.orientation.y = map_to_odom_tf.getRotation().y();
  map_odom_pose_msg.pose.pose.orientation.z = map_to_odom_tf.getRotation().z();
  map_odom_pose_msg.pose.pose.orientation.w = map_to_odom_tf.getRotation().w();
  
  for (int i = 0; i < 36; ++i) {
    map_odom_pose_msg.pose.covariance[i] = 0.0;
  }
  map_odom_pose_msg.pose.covariance[0] = fitness_score;
  map_odom_pose_msg.pose.covariance[7] = fitness_score;
  map_odom_pose_msg.pose.covariance[14] = fitness_score;
  map_odom_pose_msg.pose.covariance[21] = fitness_score;
  map_odom_pose_msg.pose.covariance[28] = fitness_score;
  map_odom_pose_msg.pose.covariance[35] = fitness_score;
  
  map_odom_pose_pub_->publish(map_odom_pose_msg);

  RCLCPP_INFO(get_logger(),
    "map->odom TF | t=(%.3f, %.3f, %.3f) q=(%.4f, %.4f, %.4f, %.4f) | fitness=%.6f",
    map_to_odom_tf.getOrigin().x(),
    map_to_odom_tf.getOrigin().y(),
    map_to_odom_tf.getOrigin().z(),
    map_to_odom_tf.getRotation().x(),
    map_to_odom_tf.getRotation().y(),
    map_to_odom_tf.getRotation().z(),
    map_to_odom_tf.getRotation().w(),
    fitness_score);

  geometry_msgs::msg::PoseStamped::SharedPtr pose_stamped_ptr(new geometry_msgs::msg::PoseStamped);
  pose_stamped_ptr->header.stamp = msg->header.stamp;
  pose_stamped_ptr->header.frame_id = global_frame_id_;
  pose_stamped_ptr->pose = corrent_pose_with_cov_stamped_ptr_->pose.pose;
  path_ptr_->poses.push_back(*pose_stamped_ptr);
  path_pub_->publish(*path_ptr_);

  last_scan_ptr_ = msg;

  if (enable_debug_) {
    std::cout << "number of filtered cloud points: " << filtered_cloud_ptr->size() << std::endl;
    std::cout << "align time:" << time_align_end.seconds() - time_align_start.seconds() <<
      "[sec]" << std::endl;
    std::cout << "has converged: " << has_converged << std::endl;
    std::cout << "fitness score: " << fitness_score << std::endl;
    std::cout << "final transformation:" << std::endl;
    std::cout << final_transformation << std::endl;
    /* delta_angle check
     * trace(RotationMatrix) = 2(cos(theta) + 1)
     */
    double init_cos_angle = 0.5 *
      (init_guess.coeff(0, 0) + init_guess.coeff(1, 1) + init_guess.coeff(2, 2) - 1);
    double cos_angle = 0.5 *
      (final_transformation.coeff(0,
      0) + final_transformation.coeff(1, 1) + final_transformation.coeff(2, 2) - 1);
    double init_angle = acos(init_cos_angle);
    double angle = acos(cos_angle);
    // Ref:https://twitter.com/Atsushi_twi/status/1185868416864808960
    double delta_angle = abs(atan2(sin(init_angle - angle), cos(init_angle - angle)));
    std::cout << "delta_angle:" << delta_angle * 180 / M_PI << "[deg]" << std::endl;
    std::cout << "-----------------------------------------------------" << std::endl;
  }
}

void PCLLocalization::timerPublishPose()
{
  // 生命周期节点激活前不发布位姿
  if (!pose_pub_->is_activated()) return;

  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.stamp = now();
  pose_msg.header.frame_id = global_frame_id_;

  geometry_msgs::msg::TransformStamped map_to_base_link_stamped;
  try {
    map_to_base_link_stamped = tfbuffer_.lookupTransform(
      global_frame_id_, base_frame_id_, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not get transform %s to %s: %s",
      global_frame_id_.c_str(), base_frame_id_.c_str(), ex.what());
    return;
  }

  pose_msg.pose.pose.position.x = map_to_base_link_stamped.transform.translation.x;
  pose_msg.pose.pose.position.y = map_to_base_link_stamped.transform.translation.y;
  pose_msg.pose.pose.position.z = map_to_base_link_stamped.transform.translation.z;
  pose_msg.pose.pose.orientation = map_to_base_link_stamped.transform.rotation;

  geometry_msgs::msg::PoseStamped stamped;
  stamped.header = pose_msg.header;
  stamped.header.frame_id = global_frame_id_;
  stamped.pose = pose_msg.pose.pose;
  path_ptr_->poses.push_back(stamped);

  nav_msgs::msg::Path path_copy = *path_ptr_;

  pose_pub_->publish(pose_msg);
  path_pub_->publish(path_copy);
}

double PCLLocalization::calculateDisplacement(const geometry_msgs::msg::Pose& current_pose)
{
  double dx = current_pose.position.x - last_localization_x_;
  double dy = current_pose.position.y - last_localization_y_;
  double dz = current_pose.position.z - last_localization_z_;
  return sqrt(dx*dx + dy*dy + dz*dz);
}

bool PCLLocalization::shouldUpdateLocalization(const geometry_msgs::msg::Pose& current_pose)
{
  if (!enable_displacement_check_) {
    return true; // Always update if displacement check is disabled
  }
  
  // Force first localization to execute
  if (!first_localization_attempted_) {
    first_localization_attempted_ = true;
    return true;
  }
  
  // 初始定位阶段（first_localization_done_ == false）始终执行，不需等位移触发
  if (!first_localization_done_) {
    return true;
  }
  
  // 持续定位阶段：检查时间间隔（避免频繁重试）
  auto now = this->now();
  double elapsed = (now - last_localization_attempt_time_).seconds();
  if (elapsed < min_localization_interval_) {
    RCLCPP_DEBUG(get_logger(), 
      "Time since last localization attempt: %.1f s < %.1f s, skipping",
      elapsed, min_localization_interval_);
    return false;
  }
  
  if (accumulated_odom_distance_ > displacement_threshold_) {
    RCLCPP_DEBUG(get_logger(), "Odom displacement %.3f m from last localization exceeds threshold %.3f m, triggering localization",
      accumulated_odom_distance_, displacement_threshold_);
    return true;
  }
  
  return false;
}

void PCLLocalization::addPerformanceStatistics(const std::string& method, double duration)
{
  std::lock_guard<std::mutex> lock(performance_stats_mutex_);
  
  if (method == "ICP") {
    icp_performance_stats_.push_back(duration);
  } else if (method == "NDT") {
    ndt_performance_stats_.push_back(duration);
  }
}

void PCLLocalization::performanceTimerCallback()
{
  std::lock_guard<std::mutex> lock(performance_stats_mutex_);
  
  // Calculate and output ICP performance statistics
  if (!icp_performance_stats_.empty()) {
    double total_icp_time = std::accumulate(icp_performance_stats_.begin(), icp_performance_stats_.end(), 0.0);
    double avg_icp_time = total_icp_time / icp_performance_stats_.size();
    RCLCPP_INFO(get_logger(), "ICP Performance: %zu calculations, average time: %.3f ms", 
                icp_performance_stats_.size(), avg_icp_time);
    icp_performance_stats_.clear();
  }
  
  // Calculate and output NDT performance statistics
  if (!ndt_performance_stats_.empty()) {
    double total_ndt_time = std::accumulate(ndt_performance_stats_.begin(), ndt_performance_stats_.end(), 0.0);
    double avg_ndt_time = total_ndt_time / ndt_performance_stats_.size();
    RCLCPP_INFO(get_logger(), "NDT Performance: %zu calculations, average time: %.3f ms", 
                ndt_performance_stats_.size(), avg_ndt_time);
    ndt_performance_stats_.clear();
  }
  
  if (icp_performance_stats_.empty() && ndt_performance_stats_.empty()) {
    RCLCPP_DEBUG(get_logger(), "No ICP/NDT performance data in the last 30 seconds");
  }
}

void PCLLocalization::buildTargetKdTree()
{
  auto target_cloud = registration_->getInputTarget();
  if (!target_cloud || target_cloud->empty()) {
    RCLCPP_WARN(get_logger(), "buildTargetKdTree: target cloud is empty, skipping");
    return;
  }
  
  target_kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  target_kdtree_->setInputCloud(target_cloud);
  RCLCPP_INFO(get_logger(), "Target kd-tree built with %lu points", target_cloud->size());
}

void PCLLocalization::computePerAxisFitnessScore(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& source_cloud,
  const Eigen::Matrix4f& transformation,
  double& fitness_x, double& fitness_y, double& fitness_z)
{
  fitness_x = 0.0;
  fitness_y = 0.0;
  fitness_z = 0.0;
  
  if (!target_kdtree_ || target_kdtree_->getInputCloud()->empty()) {
    return;
  }
  
  // Transform source cloud by the final registration transformation
  pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*source_cloud, *aligned_cloud, transformation);
  
  std::vector<int> indices(1);
  std::vector<float> distances(1);
  int valid_count = 0;
  
  for (const auto& pt : aligned_cloud->points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
    
    pcl::PointXYZI search_pt = pt;
    if (target_kdtree_->nearestKSearch(search_pt, 1, indices, distances) > 0) {
      const auto& target_pt = target_kdtree_->getInputCloud()->points[indices[0]];
      double dx = pt.x - target_pt.x;
      double dy = pt.y - target_pt.y;
      double dz = pt.z - target_pt.z;
      fitness_x += dx * dx;
      fitness_y += dy * dy;
      fitness_z += dz * dz;
      valid_count++;
    }
  }
  
  // 归一化：除以对应点数，使其与 PCL getFitnessScore() 量纲一致（平均距离平方和）
  if (valid_count > 0) {
    fitness_x /= valid_count;
    fitness_y /= valid_count;
    fitness_z /= valid_count;
  }
  
  RCLCPP_DEBUG(get_logger(), "Per-axis fitness: x=%.6f y=%.6f z=%.6f (from %d correspondences, avg err: x=%.4f y=%.4f z=%.4f m)",
               fitness_x, fitness_y, fitness_z, valid_count,
               valid_count > 0 ? std::sqrt(fitness_x / valid_count) : 0.0,
               valid_count > 0 ? std::sqrt(fitness_y / valid_count) : 0.0,
               valid_count > 0 ? std::sqrt(fitness_z / valid_count) : 0.0);
}
