// registration_impl.cpp - Heavy template instantiations for GICP/NDT registration.
// Isolated here so that changes to lidar_localization_component.cpp do not trigger
// recompilation of the large pclomp template headers.

#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <pclomp/ndt_omp.h>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp.h>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>
#include <pclomp/gicp_omp.h>
#include <pclomp/gicp_omp_impl.hpp>
#include <lidar_localization/lidar_localization_component.hpp>

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
    gicp_omp->setMaxCorrespondenceDistance(gicp_corr_dist_threshold_);
    gicp_omp->setRotationEpsilon(gicp_rotation_epsilon_);
    gicp_omp->setCorrespondenceRandomness(gicp_k_correspondences_);
    gicp_omp->setMaximumOptimizerIterations(gicp_max_optimizer_iterations_);
    gicp_omp->setGICPEpsilon(gicp_epsilon_);
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

// ===== searchOptimalTransformation =====

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
    
    // Measure performance for ICP/NDT
    auto start_time = std::chrono::high_resolution_clock::now();
    registration_->align(*output_cloud, initial_guess);
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    
    // Add to performance statistics
    std::string method = (registration_method_.find("GICP") != std::string::npos) ? "ICP" : "NDT";
    addPerformanceStatistics(method, duration);
    
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
        gicp_omp->setMaxCorrespondenceDistance(gicp_corr_dist_threshold_);
        gicp_omp->setRotationEpsilon(gicp_rotation_epsilon_);
        gicp_omp->setCorrespondenceRandomness(gicp_k_correspondences_);
        gicp_omp->setMaximumOptimizerIterations(gicp_max_optimizer_iterations_);
        gicp_omp->setGICPEpsilon(gicp_epsilon_);
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
      
      // Measure performance for ICP/NDT in Stage 1
      auto start_time = std::chrono::high_resolution_clock::now();
      test_registration->align(*output_cloud, test_guess);
      auto end_time = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
      
      // Add to performance statistics
      std::string method = (registration_method_.find("GICP") != std::string::npos) ? "ICP" : "NDT";
      addPerformanceStatistics(method, duration);
      
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
        gicp_omp->setMaxCorrespondenceDistance(gicp_corr_dist_threshold_);
        gicp_omp->setRotationEpsilon(gicp_rotation_epsilon_);
        gicp_omp->setCorrespondenceRandomness(gicp_k_correspondences_);
        gicp_omp->setMaximumOptimizerIterations(gicp_max_optimizer_iterations_);
        gicp_omp->setGICPEpsilon(gicp_epsilon_);
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
      
      // Measure performance for ICP/NDT in Stage 2
      auto start_time = std::chrono::high_resolution_clock::now();
      test_registration->align(*output_cloud, test_guess);
      auto end_time = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
      
      // Add to performance statistics
      std::string method = (registration_method_.find("GICP") != std::string::npos) ? "ICP" : "NDT";
      addPerformanceStatistics(method, duration);
      
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
    registration_->setInputSource(cloud_ptr);
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    
    // Measure performance for ICP/NDT fallback
    auto start_time = std::chrono::high_resolution_clock::now();
    registration_->align(*output_cloud, initial_guess);
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    
    // Add to performance statistics
    std::string method = (registration_method_.find("GICP") != std::string::npos) ? "ICP" : "NDT";
    addPerformanceStatistics(method, duration);
    
    best_transformation = registration_->getFinalTransformation();
    best_has_converged = registration_->hasConverged();
    best_fitness_score = registration_->getFitnessScore();
  }
  
  result.transformation = best_transformation;
  result.has_converged = best_has_converged;
  result.fitness_score = best_fitness_score;
  return result;
}
