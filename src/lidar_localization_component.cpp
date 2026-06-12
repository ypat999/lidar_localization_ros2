#include <lidar_localization/lidar_localization_component.hpp>
#include <chrono>

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
  declare_parameter("enable_map_odom_tf", false);
  declare_parameter("registration_method", "NDT");
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
    
    // Save downsampled map to 3dmap_down.pcd
    std::string downsampled_map_path = "/home/cat/slam_data/3d_map/3dmap_down.pcd";
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
      voxel_grid_filter_.setInputCloud(downsampled_cloud_ptr);
      voxel_grid_filter_.filter(*filtered_cloud_ptr);
      registration_->setInputTarget(filtered_cloud_ptr);
    } else {
      registration_->setInputTarget(downsampled_cloud_ptr);
    }

    map_recieved_ = true;
  }

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
  RCLCPP_FATAL(get_logger(), "Error Processing from %s", state.label().c_str());

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
  get_parameter("score_threshold", score_threshold_);
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
  RCLCPP_INFO(get_logger(),"initial_localization_accumulate_frames: %d", initial_localization_accumulate_frames_);
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
    "map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
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

void PCLLocalization::initializeRegistration()
{
  RCLCPP_INFO(get_logger(), "initializeRegistration");

  if (registration_method_ == "GICP") {
    boost::shared_ptr<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>> gicp(
      new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    gicp->setTransformationEpsilon(transform_epsilon_);
    registration_ = gicp;
  }
  else if (registration_method_ == "NDT") {
    boost::shared_ptr<pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>> ndt(
      new pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt->setStepSize(ndt_step_size_);
    ndt->setResolution(ndt_resolution_);
    ndt->setTransformationEpsilon(transform_epsilon_);
    registration_ = ndt;
  }
  else if (registration_method_ == "NDT_OMP") {
    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt_omp(
      new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt_omp->setStepSize(ndt_step_size_);
    ndt_omp->setResolution(ndt_resolution_);
    ndt_omp->setTransformationEpsilon(transform_epsilon_);
    if (ndt_num_threads_ > 0) {
      ndt_omp->setNumThreads(ndt_num_threads_);
    } else {
      ndt_omp->setNumThreads(omp_get_max_threads());
    }
    registration_ = ndt_omp;
  }
  else if (registration_method_ == "GICP_OMP") {
    pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr gicp_omp(
      new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    gicp_omp->setTransformationEpsilon(transform_epsilon_);
    registration_ = gicp_omp;
  }
  else {
    RCLCPP_ERROR(get_logger(), "Invalid registration method.");
    exit(EXIT_FAILURE);
  }
  registration_->setMaximumIterations(ndt_max_iterations_);

  voxel_grid_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  map_downsample_filter_.setLeafSize(map_downsample_leaf_size_, map_downsample_leaf_size_, map_downsample_leaf_size_);
  RCLCPP_INFO(get_logger(), "initializeRegistration end");
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
  
  // Initialize last localization position and reset first localization flag
  last_localization_x_ = msg->pose.pose.position.x;
  last_localization_y_ = msg->pose.pose.position.y;
  last_localization_z_ = msg->pose.pose.position.z;
  first_localization_done_ = false;  // Force first localization on next cloud
  accumulated_cloud_ptr_->clear();
  accumulated_frame_count_ = 0;
  
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

  map_recieved_ = true;
  RCLCPP_INFO(get_logger(), "mapReceived end");
}

void PCLLocalization::odomReceived(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
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
  

  double current_odom_received_time = msg->header.stamp.sec +
    msg->header.stamp.nanosec * 1e-9;
  if (last_odom_received_time_ == 0.0) {
    last_odom_received_time_ = current_odom_received_time;
    return;
  }
  double dt_odom = current_odom_received_time - last_odom_received_time_;
  last_odom_received_time_ = current_odom_received_time;
  if (dt_odom > 1.0 /* [sec] */) {
    RCLCPP_WARN(this->get_logger(), "odom time interval is too large: %f", dt_odom);
    return;
  }
  if (dt_odom < 0.0 /* [sec] */) {
    RCLCPP_WARN(this->get_logger(), "odom time interval is negative: %f", dt_odom);
    return;
  }

  tf2::Quaternion previous_quat_tf;
  double roll, pitch, yaw;
  tf2::fromMsg(corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation, previous_quat_tf);

  tf2::Matrix3x3(previous_quat_tf).getRPY(roll, pitch, yaw);

  roll += msg->twist.twist.angular.x * dt_odom;
  pitch += msg->twist.twist.angular.y * dt_odom;
  yaw += msg->twist.twist.angular.z * dt_odom;

  Eigen::Quaterniond quat_eig =
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);

  Eigen::Vector3d odom{
    msg->twist.twist.linear.x,
    msg->twist.twist.linear.y,
    msg->twist.twist.linear.z};
  Eigen::Vector3d delta_position = quat_eig.matrix() * dt_odom * odom;

  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x += delta_position.x();
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y += delta_position.y();
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.z += delta_position.z();
  corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation = quat_msg;
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

  Eigen::Vector3f angular_velo{tf_converted_imu.angular_velocity.x, tf_converted_imu.angular_velocity.y,
    tf_converted_imu.angular_velocity.z};
  Eigen::Vector3f acc{tf_converted_imu.linear_acceleration.x, tf_converted_imu.linear_acceleration.y, tf_converted_imu.linear_acceleration.z};
  Eigen::Quaternionf quat{msg->orientation.w, msg->orientation.x, msg->orientation.y,
    msg->orientation.z};
  double imu_time = msg->header.stamp.sec +
    msg->header.stamp.nanosec * 1e-9;

  lidar_undistortion_.getImu(angular_velo, acc, quat, imu_time);

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
          rclcpp::Duration::from_seconds(0.1));
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

  if (!first_localization_done_ && initial_localization_accumulate_frames_ > 1) {
    *accumulated_cloud_ptr_ += *tmp_ptr;
    accumulated_frame_count_++;

    if (accumulated_frame_count_ < initial_localization_accumulate_frames_) {
      RCLCPP_INFO(get_logger(), "Accumulating frames for initial localization: %d/%d (points: %lu)",
                  accumulated_frame_count_, initial_localization_accumulate_frames_,
                  accumulated_cloud_ptr_->size());
      last_scan_ptr_ = msg;
      return;
    }

    RCLCPP_INFO(get_logger(), "Accumulated %d frames for initial localization, total points: %lu",
                accumulated_frame_count_, accumulated_cloud_ptr_->size());
    cloud_for_registration = accumulated_cloud_ptr_;
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

  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  rclcpp::Clock system_clock;
  rclcpp::Time time_align_start = system_clock.now();
  
  // Use search optimization to find best transformation
  SearchResult search_result = searchOptimalTransformation(cloud_for_registration, init_guess, enable_z_axis_search_);
  Eigen::Matrix4f final_transformation = search_result.transformation;
  
  rclcpp::Time time_align_end = system_clock.now();

  bool has_converged = search_result.has_converged;
  double fitness_score = search_result.fitness_score;
  if (!has_converged) {
    RCLCPP_WARN(get_logger(), "The registration didn't converge.");
    return;
  }
  
  // Dynamic score threshold mechanism
  double effective_threshold = score_threshold_;
  if (enable_dynamic_threshold_) {
    if (!first_localization_done_) {
      // First localization: accept any score that meets the basic threshold
      effective_threshold = score_threshold_;
      RCLCPP_INFO(get_logger(), "First localization, using base threshold: %lf", effective_threshold);
    } else {
      // Subsequent localizations: use dynamic threshold based on current best score
      effective_threshold = std::min(current_fitness_score_ * dynamic_threshold_factor_, score_threshold_);
      RCLCPP_DEBUG(get_logger(), "Dynamic threshold: %lf (current score: %lf, factor: %lf)", 
                   effective_threshold, current_fitness_score_, dynamic_threshold_factor_);
    }
  }
  
  if (fitness_score > effective_threshold) {
    RCLCPP_WARN(get_logger(), "The fitness score %lf is over threshold %lf. Rejecting transformation.", 
                fitness_score, effective_threshold);
    return;
  }
  
  // Update current fitness score
  current_fitness_score_ = fitness_score;
  RCLCPP_INFO(get_logger(), "Updated current fitness score to: %lf", current_fitness_score_);
  
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
    RCLCPP_WARN(
      this->get_logger(), "Could not get transform %s to %s: %s",
      base_frame_id_.c_str(), odom_frame_id_.c_str(), ex.what());
    return;
  }
  tf2::Transform odom_to_base_link_tf;
  tf2::fromMsg(odom_to_base_link_msg.transform, odom_to_base_link_tf);

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
  if (!first_localization_done_) {
    first_localization_done_ = true;
    return true;
  }
  
  double displacement = calculateDisplacement(current_pose);
  
  // RCLCPP_INFO(get_logger(), "Current pose: (%.3f, %.3f, %.3f), Last localization: (%.3f, %.3f, %.3f), Displacement: %.3f m",
  //   current_pose.position.x, current_pose.position.y, current_pose.position.z,
  //   last_localization_x_, last_localization_y_, last_localization_z_,
  //   displacement);
  
  if (displacement > displacement_threshold_) {
    // Update last localization position
    last_localization_x_ = current_pose.position.x;
    last_localization_y_ = current_pose.position.y;
    last_localization_z_ = current_pose.position.z;
    
    RCLCPP_INFO(get_logger(), "Displacement %.3f m exceeds threshold %.3f m, updating localization",
      displacement, displacement_threshold_);
    return true;
  }
  
  
  return false;
}

PCLLocalization::SearchResult PCLLocalization::searchOptimalTransformation(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_ptr,
  const Eigen::Matrix4f& initial_guess,
  bool search_z_axis)
{
  SearchResult result;
  result.transformation = initial_guess;
  result.has_converged = false;
  result.fitness_score = std::numeric_limits<double>::max();
  
  if (!enable_search_optimization_) {
    // Use standard single-point registration
    registration_->setInputSource(cloud_ptr);
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    registration_->align(*output_cloud, initial_guess);
    result.transformation = registration_->getFinalTransformation();
    result.has_converged = registration_->hasConverged();
    result.fitness_score = registration_->getFitnessScore();
    return result;
  }
  
  RCLCPP_DEBUG(this->get_logger(), "Performing two-stage search optimization with radius %lf m", search_radius_);
  if (enable_angle_search_) {
    RCLCPP_DEBUG(this->get_logger(), "Angle search enabled: range %.3f rad (%.1f deg), %d steps", 
      angle_search_range_, angle_search_range_ * 180.0 / M_PI, angle_search_steps_);
  }
  RCLCPP_DEBUG(this->get_logger(), "Z-axis search: %s", search_z_axis ? "enabled" : "disabled");
  
  // Extract initial position and rotation from transformation matrix
  Eigen::Vector3f initial_position = initial_guess.block<3,1>(0,3);
  Eigen::Matrix3f initial_rotation = initial_guess.block<3,3>(0,0);
  Eigen::Vector3f initial_euler = initial_rotation.eulerAngles(0, 1, 2);
  
  double best_fitness_score = std::numeric_limits<double>::max();
  Eigen::Matrix4f best_transformation = initial_guess;
  bool best_has_converged = false;
  double best_z_offset = 0.0;
  
  // Calculate step size for z-axis search
  double z_step_size = (2.0 * search_radius_) / (search_grid_size_ - 1);
  
  // Calculate angle step size for yaw search
  double angle_step_size = (2.0 * angle_search_range_) / (angle_search_steps_ - 1);
  
  int total_searches = 0;
  if (search_z_axis) {
    total_searches = search_grid_size_;
  }
  if (enable_angle_search_) {
    total_searches += (angle_search_steps_ - 1);
  }
  
  RCLCPP_DEBUG(this->get_logger(), "Total search iterations (two-stage): %d", total_searches);
  
  int search_count = 0;
  
  // Stage 1: Search z-axis only (only if search_z_axis is true)
  if (search_z_axis) {
    for (int k = 0; k < search_grid_size_; ++k) {
      search_count++;
      
      // Calculate z offset for this grid point
      double z_offset = -search_radius_ + k * z_step_size;
      
      // Create transformation matrix with z offset only
      Eigen::Matrix4f test_guess = initial_guess;
      test_guess(0,3) = initial_position.x();  // Keep x unchanged
      test_guess(1,3) = initial_position.y();  // Keep y unchanged
      test_guess(2,3) = initial_position.z() + z_offset;  // Apply z offset
      
      // Keep original rotation (no angle offset in stage 1)
      test_guess.block<3,3>(0,0) = initial_rotation;
      
      // Create a new registration object for each test to avoid state issues
      boost::shared_ptr<pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>> test_registration;
      
      if (registration_method_ == "NDT_OMP") {
        pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt_omp(
          new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
        ndt_omp->setStepSize(ndt_step_size_);
        ndt_omp->setResolution(ndt_resolution_);
        ndt_omp->setTransformationEpsilon(transform_epsilon_);
        if (ndt_num_threads_ > 0) {
          ndt_omp->setNumThreads(ndt_num_threads_);
        }
        ndt_omp->setMaximumIterations(ndt_max_iterations_);
        ndt_omp->setInputTarget(registration_->getInputTarget());
        test_registration = ndt_omp;
      } else if (registration_method_ == "GICP_OMP") {
        pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr gicp_omp(
          new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
        gicp_omp->setTransformationEpsilon(transform_epsilon_);
        gicp_omp->setMaximumIterations(ndt_max_iterations_);
        gicp_omp->setInputTarget(registration_->getInputTarget());
        test_registration = gicp_omp;
      } else {
        // Fallback to standard registration
        test_registration = registration_;
      }
      
      // Perform registration with this initial guess
      test_registration->setInputSource(cloud_ptr);
      pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      test_registration->align(*output_cloud, test_guess);
      
      if (test_registration->hasConverged()) {
        double fitness_score = test_registration->getFitnessScore();
        
        if (fitness_score < best_fitness_score) {
          best_fitness_score = fitness_score;
          best_transformation = test_registration->getFinalTransformation();
          best_has_converged = true;
          best_z_offset = z_offset;
          
          RCLCPP_DEBUG(get_logger(), "Stage 1 Search %d/%d: z=%.2f, score=%.4f (best)", 
            search_count, total_searches, z_offset, fitness_score);
        }
      }
    }
  } else {
    // If not searching z-axis, use initial guess as baseline
    best_z_offset = 0.0;
    RCLCPP_DEBUG(get_logger(), "Stage 1 skipped (z-axis search disabled)");
  }
  
  // Stage 2: Search angles only at the best z position
  if (enable_angle_search_) {
    RCLCPP_DEBUG(get_logger(), "Stage 2: Searching angles at best z=%.2f", best_z_offset);
    
    for (int a = 0; a < angle_search_steps_; ++a) {
      // Skip the first angle (0 offset) as it was already tested in Stage 1
      if (a == (angle_search_steps_ - 1) / 2) {
        continue;
      }
      
      search_count++;
      
      // Calculate yaw angle offset
      double yaw_offset = -angle_search_range_ + a * angle_step_size;
      
      // Create transformation matrix with best z offset and angle offset
      Eigen::Matrix4f test_guess = initial_guess;
      test_guess(0,3) = initial_position.x();  // Keep x unchanged
      test_guess(1,3) = initial_position.y();  // Keep y unchanged
      test_guess(2,3) = initial_position.z() + best_z_offset;  // Use best z offset
      
      // Apply yaw rotation (only modify yaw, keep roll and pitch)
      Eigen::AngleAxisf roll_angle(initial_euler(0), Eigen::Vector3f::UnitX());
      Eigen::AngleAxisf pitch_angle(initial_euler(1), Eigen::Vector3f::UnitY());
      Eigen::AngleAxisf yaw_angle(initial_euler(2) + yaw_offset, Eigen::Vector3f::UnitZ());
      
      Eigen::Matrix3f rotation_matrix = (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();
      test_guess.block<3,3>(0,0) = rotation_matrix;
      
      // Create a new registration object for each test to avoid state issues
      boost::shared_ptr<pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>> test_registration;
      
      if (registration_method_ == "NDT_OMP") {
        pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt_omp(
          new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
        ndt_omp->setStepSize(ndt_step_size_);
        ndt_omp->setResolution(ndt_resolution_);
        ndt_omp->setTransformationEpsilon(transform_epsilon_);
        if (ndt_num_threads_ > 0) {
          ndt_omp->setNumThreads(ndt_num_threads_);
        }
        ndt_omp->setMaximumIterations(ndt_max_iterations_);
        ndt_omp->setInputTarget(registration_->getInputTarget());
        test_registration = ndt_omp;
      } else if (registration_method_ == "GICP_OMP") {
        pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr gicp_omp(
          new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
        gicp_omp->setTransformationEpsilon(transform_epsilon_);
        gicp_omp->setMaximumIterations(ndt_max_iterations_);
        gicp_omp->setInputTarget(registration_->getInputTarget());
        test_registration = gicp_omp;
      } else {
        // Fallback to standard registration
        test_registration = registration_;
      }
      
      // Perform registration with this initial guess
      test_registration->setInputSource(cloud_ptr);
      pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      test_registration->align(*output_cloud, test_guess);
      
      if (test_registration->hasConverged()) {
        double fitness_score = test_registration->getFitnessScore();
        
        if (fitness_score < best_fitness_score) {
          best_fitness_score = fitness_score;
          best_transformation = test_registration->getFinalTransformation();
          best_has_converged = true;
          
          RCLCPP_DEBUG(get_logger(), "Stage 2 Search %d/%d: z=%.2f, yaw=%.2f deg, score=%.4f (best)", 
            search_count, total_searches, best_z_offset, yaw_offset * 180.0 / M_PI, fitness_score);
        }
      }
    }
  }
  
  // If no valid result found (still max value), use initial guess with standard registration
  if (best_fitness_score >= std::numeric_limits<double>::max() / 2.0) {
    RCLCPP_WARN(get_logger(), "No valid result found in search, using initial guess with standard registration");
    registration_->setInputSource(cloud_ptr);
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    registration_->align(*output_cloud, initial_guess);
    best_transformation = registration_->getFinalTransformation();
    best_has_converged = registration_->hasConverged();
    best_fitness_score = registration_->getFitnessScore();
  }
  
  RCLCPP_DEBUG(this->get_logger(), "Best fitness score after search: %lf", best_fitness_score);
  result.transformation = best_transformation;
  result.has_converged = best_has_converged;
  result.fitness_score = best_fitness_score;
  return result;
}
