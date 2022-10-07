/*
 * @Description: LIO localization frontend, implementation
 * @Author: Ge Yao
 * @Date: 2021-01-02 10:47:23
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-22 21:02:33
 */
#include "lidar_localization/matching/front_end/matching.h"

namespace lidar_localization {

Matching::Matching()
    : global_map_ptr_(new CloudData::CloudType())
    , local_map_ptr_(new CloudData::CloudType())
    , current_scan_ptr_(new CloudData::CloudType()) {
  InitWithConfig();

  ResetLocalMap(0.0, 0.0, 0.0);
}

bool Matching::InitWithConfig() {
  //
  // load lio localization frontend config file:
  //
  std::string config_file_path =
      WORK_SPACE_PATH + "/config/matching/matching.yaml";
  YAML::Node config_node = YAML::LoadFile(config_file_path);

  // prompt:
  LOG(INFO)
      << std::endl
      << "----------------- Init LIO Localization, Frontend -------------------"
      << std::endl;

  // a. init point cloud map & measurement processors:
  InitPointCloudProcessors(config_node);
  // b. load global map:
  InitGlobalMap(config_node);
  // c. init lidar frontend for relative pose estimation:
  InitRegistration(config_node, registration_ptr_);

  return true;
}

bool Matching::InitFilter(const YAML::Node& config_node,
                          std::string filter_user,
                          std::shared_ptr<CloudFilterInterface>& filter_ptr) {
  std::string filter_mothod =
      config_node[filter_user + "_filter"].as<std::string>();

  // prompt:
  LOG(INFO) << "\t\tFilter Method for " << filter_user << ": " << filter_mothod
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

bool Matching::InitLocalMapSegmenter(const YAML::Node& config_node) {
  local_map_segmenter_ptr_ = std::make_shared<BoxFilter>(config_node);
  return true;
}

bool Matching::InitPointCloudProcessors(const YAML::Node& config_node) {
  // prompt:
  LOG(INFO) << "\tInit Point Cloud Processors:" << std::endl;

  // a. global map filter:
  InitFilter(config_node, "global_map", global_map_filter_ptr_);

  // b.1. local map segmenter:
  InitLocalMapSegmenter(config_node);
  // b.2. local map filter:
  InitFilter(config_node, "local_map", local_map_filter_ptr_);

  // c. scan filter --
  InitFilter(config_node, "frame", frame_filter_ptr_);

  return true;
}

bool Matching::InitGlobalMap(const YAML::Node& config_node) {
  std::string map_path = config_node["map_path"].as<std::string>();

  // load map:
  pcl::io::loadPCDFile(map_path, *global_map_ptr_);
  // apply local map filter to global map for later scan-map matching:
  local_map_filter_ptr_->Filter(global_map_ptr_, global_map_ptr_);

  // prompt:
  LOG(INFO) << "\tLoad Global Map, size:" << global_map_ptr_->points.size();

  has_new_global_map_ = true;

  return true;
}

bool Matching::InitRegistration(
    const YAML::Node& config_node,
    std::shared_ptr<RegistrationInterface>& registration_ptr) {
  std::string registration_method =
      config_node["registration_method"].as<std::string>();

  // prompt:
  LOG(INFO) << "\tLidar Frontend Estimation Method: " << registration_method
            << std::endl;

  if (registration_method == "NDT") {
    registration_ptr =
        std::make_shared<NDTRegistration>(config_node[registration_method]);
  } else {
    LOG(ERROR) << "Estimation method " << registration_method << " NOT FOUND!";
    return false;
  }

  return true;
}

bool Matching::ResetLocalMap(float x, float y, float z) {
  std::vector<float> origin = {x, y, z};

  // use ROI filtering for local map segmentation:
  local_map_segmenter_ptr_->set_origin(origin);
  local_map_segmenter_ptr_->Filter(global_map_ptr_, local_map_ptr_);

  registration_ptr_->SetInputTarget(local_map_ptr_);

  has_new_local_map_ = true;

  const std::vector<float>& edge = local_map_segmenter_ptr_->edge();
  LOG(INFO) << "New local map:" << edge.at(0) << "," << edge.at(1) << ","
            << edge.at(2) << "," << edge.at(3) << "," << edge.at(4) << ","
            << edge.at(5) << std::endl
            << std::endl;

  return true;
}

bool Matching::SetInitPose(const Eigen::Matrix4f& init_pose) {
  init_pose_ = init_pose;

  ResetLocalMap(init_pose(0, 3), init_pose(1, 3), init_pose(2, 3));

  return true;
}

bool Matching::SetGNSSPose(const Eigen::Matrix4f& gnss_pose) {
  static int gnss_cnt = 0;

  current_gnss_pose_ = gnss_pose;

  if (gnss_cnt == 0) {
    SetInitPose(gnss_pose);
  } else if (gnss_cnt > 3) {
    has_inited_ = true;
  }
  gnss_cnt++;

  return true;
}

bool Matching::Update(const CloudData& cloud_data,
                      Eigen::Matrix4f& laser_pose,
                      Eigen::Matrix4f& map_matching_pose) {
  static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
  static Eigen::Matrix4f last_pose = init_pose_;
  static Eigen::Matrix4f predict_pose = init_pose_;

  // remove invalid measurements:
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(
      *cloud_data.cloud_ptr_, *cloud_data.cloud_ptr_, indices);

  // downsample current scan:
  CloudData::CloudTypePtr filtered_cloud_ptr(new CloudData::CloudType());
  frame_filter_ptr_->Filter(cloud_data.cloud_ptr_, filtered_cloud_ptr);

  if (!has_inited_) {
    predict_pose = current_gnss_pose_;
  }

  // matching:
  CloudData::CloudTypePtr result_cloud_ptr(new CloudData::CloudType());
  registration_ptr_->ScanMatch(
      filtered_cloud_ptr, predict_pose, result_cloud_ptr, laser_pose);
  pcl::transformPointCloud(
      *cloud_data.cloud_ptr_, *current_scan_ptr_, laser_pose);

  // update predicted pose:
  step_pose = last_pose.inverse() * laser_pose;
  last_pose = laser_pose;
  predict_pose = laser_pose * step_pose;

  // shall the local map be updated:
  const std::vector<float>& edge = local_map_segmenter_ptr_->edge();
  for (int i = 0; i < 3; i++) {
    if (fabs(laser_pose(i, 3) - edge.at(2 * i)) > 50.0 &&
        fabs(laser_pose(i, 3) - edge.at(2 * i + 1)) > 50.0) {
      continue;
    }

    ResetLocalMap(laser_pose(0, 3), laser_pose(1, 3), laser_pose(2, 3));
    break;
  }

  return true;
}

// bool Matching::HasInited(void) { return has_inited_; }

// bool Matching::HasNewGlobalMap(void) { return has_new_global_map_; }

// bool Matching::HasNewLocalMap(void) { return has_new_local_map_; }

// Eigen::Matrix4f Matching::GetInitPose(void) { return init_pose_; }

// CloudData::CloudTypePtr& Matching::GetGlobalMap(void) {
//   has_new_global_map_ = false;
//   return global_map_ptr_;
// }

// CloudData::CloudTypePtr& Matching::GetLocalMap(void) { return local_map_ptr_;
// }

// CloudData::CloudTypePtr& Matching::GetCurrentScan(void) {
//   return current_scan_ptr_;
// }

}  // namespace lidar_localization