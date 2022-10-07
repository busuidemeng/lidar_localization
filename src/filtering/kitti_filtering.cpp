/*
 * @Description: IMU-lidar fusion for localization workflow
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-27 23:23:35
 */
#include "lidar_localization/filtering/kitti_filtering.h"

namespace lidar_localization {

KITTIFiltering::KITTIFiltering()
    : global_map_ptr_(new CloudData::CloudType())
    , local_map_ptr_(new CloudData::CloudType())
    , current_scan_ptr_(new CloudData::CloudType()) {
  // load ROS config:
  InitWithConfig();
}

bool KITTIFiltering::Init(const Eigen::Matrix4f& init_pose,
                          const Eigen::Vector3f& init_vel,
                          const IMUData& init_imu_data) {
  if (SetInitGNSS(init_pose)) {
    current_vel_ = init_vel;

    // kalman_filter_ptr_->Init(current_vel_.cast<double>(), init_imu_data);
    kalman_filter_ptr_->Init(
        init_pose.cast<double>(), current_vel_.cast<double>(), init_imu_data);

    return true;
  }

  return false;
}

bool KITTIFiltering::Update(const IMUData& imu_data) {
  if (kalman_filter_ptr_->Update(imu_data)) {
    kalman_filter_ptr_->GetOdometry(current_pose_, current_vel_);
    return true;
  }

  return false;
}

bool KITTIFiltering::Correct(const IMUData& imu_data,
                             const CloudData& cloud_data,
                             const PosVelData& pos_vel_data,
                             Eigen::Matrix4f& cloud_pose) {
  static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
  static Eigen::Matrix4f last_pose = init_pose_;
  static Eigen::Matrix4f predict_pose = init_pose_;

  // remove invalid measurements:
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(
      *cloud_data.cloud_ptr_, *cloud_data.cloud_ptr_, indices);

  // downsample:
  CloudData::CloudTypePtr filtered_cloud_ptr(new CloudData::CloudType());
  current_scan_filter_ptr_->Filter(cloud_data.cloud_ptr_, filtered_cloud_ptr);

  if (!has_inited_) {
    predict_pose = current_gnss_pose_;
  }

  // matching:
  CloudData::CloudTypePtr result_cloud_ptr(new CloudData::CloudType());
  registration_ptr_->ScanMatch(
      filtered_cloud_ptr, predict_pose, result_cloud_ptr, cloud_pose);
  pcl::transformPointCloud(
      *cloud_data.cloud_ptr_, *current_scan_ptr_, cloud_pose);

  // update predicted pose:
  step_pose = last_pose.inverse() * cloud_pose;
  predict_pose = cloud_pose * step_pose;
  last_pose = cloud_pose;

  // shall the local map be updated:
  std::vector<float> edge = local_map_segmenter_ptr_->edge();
  for (int i = 0; i < 3; i++) {
    if (fabs(cloud_pose(i, 3) - edge.at(2 * i)) > 50.0 &&
        fabs(cloud_pose(i, 3) - edge.at(2 * i + 1)) > 50.0) {
      continue;
    }

    ResetLocalMap(cloud_pose(0, 3), cloud_pose(1, 3), cloud_pose(2, 3));
    break;
  }

  // set lidar measurement:
  current_measurement_.time = cloud_data.time_;
  // current_measurement_.T_nb =
  //     (init_pose_.inverse() * cloud_pose).cast<double>();
  current_measurement_.T_nb = cloud_pose.cast<double>();
  current_measurement_.v_b = pos_vel_data.vel_.cast<double>();
  current_measurement_.w_b = Eigen::Vector3d(imu_data.angular_velocity_.x,
                                             imu_data.angular_velocity_.y,
                                             imu_data.angular_velocity_.z);
  current_measurement_.p_nb = pos_vel_data.pos_.cast<double>();

  // Kalman correction:
  if (kalman_filter_ptr_->Correct(
          imu_data, config_.fusion_strategy, current_measurement_)) {
    kalman_filter_ptr_->GetOdometry(current_pose_, current_vel_);

    return true;
  }

  return false;
}

void KITTIFiltering::GetGlobalMap(CloudData::CloudTypePtr& global_map) {
  // downsample global map for visualization:
  global_map_filter_ptr_->Filter(global_map_ptr_, global_map);
  has_new_global_map_ = false;
}

void KITTIFiltering::GetOdometry(Eigen::Matrix4f& pose, Eigen::Vector3f& vel) {
  // pose = init_pose_ * current_pose_;
  // vel = init_pose_.block<3, 3>(0, 0) * current_vel_;
  pose = current_pose_;
  vel = current_vel_;
}

bool KITTIFiltering::InitWithConfig() {
  const std::string config_file_path =
      WORK_SPACE_PATH + "/config/filtering/kitti_filtering.yaml";

  YAML::Node config_node = YAML::LoadFile(config_file_path);

  LOG(INFO) << std::endl
            << "-----------------Init IMU-Lidar Fusion for "
               "Localization-------------------"
            << std::endl;

  // a. init filters:
  InitFilters(config_node);
  // b. init map:
  InitGlobalMap(config_node);
  // c. init frontend:
  InitRegistration(config_node, registration_ptr_);
  // d. init fusion:
  InitFusion(config_node);

  // init local map for frontend matching:
  ResetLocalMap(0.0, 0.0, 0.0);

  return true;
}

bool KITTIFiltering::InitFilter(
    const YAML::Node& config_node,
    const std::string& filter_user,
    std::shared_ptr<CloudFilterInterface>& filter_ptr) {
  const std::string filter_mothod =
      config_node[filter_user + "_filter"].as<std::string>();

  std::cout << "\tFilter Method for " << filter_user << ": " << filter_mothod
            << std::endl;

  if (filter_mothod == "voxel_filter") {
    filter_ptr =
        std::make_shared<VoxelFilter>(config_node[filter_mothod][filter_user]);
  } else if (filter_mothod == "no_filter") {
    filter_ptr = std::make_shared<NoFilter>();
  } else {
    LOG(ERROR) << "Filter method " << filter_mothod << " for " << filter_user
               << " NOT FOUND!";
    return false;
  }

  return true;
}

bool KITTIFiltering::InitLocalMapSegmenter(const YAML::Node& config_node) {
  local_map_segmenter_ptr_ = std::make_shared<BoxFilter>(config_node);
  return true;
}

bool KITTIFiltering::InitFilters(const YAML::Node& config_node) {
  // a. global map filter -- downsample point cloud map for visualization:
  InitFilter(config_node, "global_map", global_map_filter_ptr_);
  // b. local map filter -- downsample & ROI filtering for scan-map matching:
  InitLocalMapSegmenter(config_node);
  InitFilter(config_node, "local_map", local_map_filter_ptr_);
  // c. scan filter --
  InitFilter(config_node, "current_scan", current_scan_filter_ptr_);

  return true;
}

bool KITTIFiltering::InitGlobalMap(const YAML::Node& config_node) {
  map_path_ = config_node["map_path"].as<std::string>();

  pcl::io::loadPCDFile(map_path_, *global_map_ptr_);
  LOG(INFO) << "Load global map, size:" << global_map_ptr_->points.size();

  // since scan-map matching is used, here apply the same filter to local map &
  // scan:
  local_map_filter_ptr_->Filter(global_map_ptr_, global_map_ptr_);
  LOG(INFO) << "Filtered global map, size:" << global_map_ptr_->points.size();

  has_new_global_map_ = true;

  return true;
}

bool KITTIFiltering::InitRegistration(
    const YAML::Node& config_node,
    std::shared_ptr<RegistrationInterface>& registration_ptr) {
  std::string registration_method =
      config_node["registration_method"].as<std::string>();

  LOG(INFO) << "\tPoint Cloud Registration Method: " << registration_method
            << std::endl;

  if (registration_method == "NDT") {
    registration_ptr =
        std::make_shared<NDTRegistration>(config_node[registration_method]);
  } else {
    LOG(ERROR) << "Registration method " << registration_method
               << " NOT FOUND!";
    return false;
  }

  return true;
}

bool KITTIFiltering::InitFusion(const YAML::Node& config_node) {
  // set up fusion strategy:
  config_.fusion_strategy_id["pose"] = KalmanFilter::MeasurementType::POSE;
  config_.fusion_strategy_id["pose_position"] =
      KalmanFilter::MeasurementType::POSE_POSI;
  config_.fusion_strategy_id["pose_velocity"] =
      KalmanFilter::MeasurementType::POSE_VEL;
  config_.fusion_strategy_id["pose_velocity_constrain"] =
      KalmanFilter::MeasurementType::POSE_VEL_CONS;
  config_.fusion_strategy_id["position"] = KalmanFilter::MeasurementType::POSI;
  config_.fusion_strategy_id["position_velocity"] =
      KalmanFilter::MeasurementType::POSI_VEL;

  std::string fusion_strategy =
      config_node["fusion_strategy"].as<std::string>();

  if (config_.fusion_strategy_id.end() !=
      config_.fusion_strategy_id.find(fusion_strategy)) {
    config_.fusion_strategy = config_.fusion_strategy_id.at(fusion_strategy);
  } else {
    LOG(ERROR) << "Fusion strategy " << fusion_strategy << " NOT FOUND!";
    return false;
  }
  std::cout << "\tGNSS-INS-Sim Localization Fusion Strategy: "
            << fusion_strategy << std::endl;

  // set up fusion method:
  config_.fusion_method = config_node["fusion_method"].as<std::string>();

  if (config_.fusion_method == "error_state_kalman_filter") {
    kalman_filter_ptr_ = std::make_shared<ErrorStateKalmanFilter>(
        config_node[config_.fusion_method]);
  } else {
    LOG(ERROR) << "Fusion method " << config_.fusion_method << " NOT FOUND!";
    return false;
  }
  LOG(INFO) << "\tKITTI Localization Fusion Method: " << config_.fusion_method
            << std::endl;

  return true;
}

bool KITTIFiltering::SetInitGNSS(const Eigen::Matrix4f& gnss_pose) {
  SetInitPose(gnss_pose);
  has_inited_ = true;

  return true;
}

bool KITTIFiltering::SetInitPose(const Eigen::Matrix4f& init_pose) {
  init_pose_ = init_pose;

  ResetLocalMap(init_pose(0, 3), init_pose(1, 3), init_pose(2, 3));

  return true;
}

bool KITTIFiltering::ResetLocalMap(const float x,
                                   const float y,
                                   const float z) {
  const std::vector<float> origin = {x, y, z};

  // segment local map from global map:
  local_map_segmenter_ptr_->set_origin(origin);
  local_map_segmenter_ptr_->Filter(global_map_ptr_, local_map_ptr_);

  registration_ptr_->SetInputTarget(local_map_ptr_);

  has_new_local_map_ = true;

  const std::vector<float> edge = local_map_segmenter_ptr_->edge();

  LOG(INFO) << "New local map:" << edge.at(0) << "," << edge.at(1) << ","
            << edge.at(2) << "," << edge.at(3) << "," << edge.at(4) << ","
            << edge.at(5) << std::endl
            << std::endl;

  return true;
}

}  // namespace lidar_localization