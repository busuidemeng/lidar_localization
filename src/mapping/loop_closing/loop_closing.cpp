/*
 * @Description: 闭环检测算法
 * @Author: Ren Qian
 * @Date: 2020-02-04 18:53:06
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-26 23:10:58
 */
#include "lidar_localization/mapping/loop_closing/loop_closing.h"

namespace lidar_localization {
LoopClosing::LoopClosing() { InitWithConfig(); }

bool LoopClosing::InitWithConfig() {
  const std::string config_file_path =
      WORK_SPACE_PATH + "/config/mapping/loop_closing.yaml";
  YAML::Node config_node = YAML::LoadFile(config_file_path);

  std::cout << "-----------------Init Loop-Closing Detection-------------------"
            << std::endl;
  InitParam(config_node);
  InitDataPath(config_node);

  InitFilter(config_node, "map", map_filter_ptr_);
  InitFilter(config_node, "scan", scan_filter_ptr_);

  InitRegistration(config_node, registration_ptr_);

  return true;
}

bool LoopClosing::InitParam(const YAML::Node& config_node) {
  extend_frame_num_ = config_node["extend_frame_num"].as<int>();
  loop_step_ = config_node["loop_step"].as<int>();
  diff_num_ = config_node["diff_num"].as<int>();
  detect_area_ = config_node["detect_area"].as<float>();
  fitness_score_limit_ = config_node["fitness_score_limit"].as<float>();
  key_frame_distance_ = config_node["key_frame_distance"].as<float>();

  return true;
}

bool LoopClosing::InitDataPath(const YAML::Node& config_node) {
  std::string data_path = config_node["data_path"].as<std::string>();
  if (data_path == "./") {
    data_path = WORK_SPACE_PATH;
  }

  key_frames_path_ = data_path + "/slam_data/key_frames";

  return true;
}

bool LoopClosing::InitFilter(
    const YAML::Node& config_node,
    const std::string filter_user,
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

bool LoopClosing::InitRegistration(
    const YAML::Node& config_node,
    std::shared_ptr<RegistrationInterface>& registration_ptr) {
  const std::string registration_method =
      config_node["registration_method"].as<std::string>();
  std::cout << "\tPoint Cloud Registration Method: " << registration_method
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

bool LoopClosing::Update(const CloudData& key_scan,
                         const KeyFrame& key_frame,
                         const KeyFrame& key_gnss) {
  static int key_frame_index = 0;
  static float yaw_change_in_rad = 0.0f;

  has_new_loop_pose_ = false;

  //   scan_context_manager_ptr_->Update(key_scan, key_gnss);

  all_key_frames_.push_back(key_frame);
  all_key_gnss_.push_back(key_gnss);

  if (!DetectNearestKeyFrame(key_frame_index, yaw_change_in_rad))
    return false;

  if (!CloudRegistration(key_frame_index, yaw_change_in_rad))
    return false;

  has_new_loop_pose_ = true;
  return true;
}

bool LoopClosing::DetectNearestKeyFrame(int& key_frame_index,
                                        float& yaw_change_in_rad) {
  static int skip_cnt = 0;
  static int skip_num = loop_step_;

  // only perform loop closure detection for every skip_num key frames:
  if (++skip_cnt < skip_num)
    return false;

  // perform loop closure via GNSS measurement to prevent LO drift

  // total number of GNSS/IMU key frame poses:
  const size_t N = all_key_gnss_.size();

  // ensure valid loop closure match:
  if (N < static_cast<size_t>(diff_num_ + 1))
    return false;

  const KeyFrame& current_key_frame = all_key_gnss_.back();

  int proposed_key_frame_id = -1;
  // this orientation compensation is not available for GNSS/IMU proposal:
  const float proposed_yaw_change = 0.0f;
  float key_frame_distance = std::numeric_limits<float>::max();
  // find the nearest frame w.r.t. current frame
  for (size_t i = 0; i < N - 1; ++i) {
    // ensure key frame seq. distance:
    // close to current frame do not perform loop closure
    if (N < static_cast<size_t>(i + diff_num_))
      break;

    const KeyFrame& proposed_key_frame = all_key_gnss_.at(i);

    const Eigen::Vector3f translation =
        (current_key_frame.pose_.block<3, 1>(0, 3) -
         proposed_key_frame.pose_.block<3, 1>(0, 3));
    float distance = translation.norm();

    // get closest proposal:
    if (distance < key_frame_distance) {
      key_frame_distance = distance;
      proposed_key_frame_id = i;
    }
  }

  // this is needed for valid local map build:
  // 闭环使用proposed_key_frame_id前后extend_frame_num_个关键帧组成local_map进行scan2map配准
  // 如果proposed_key_frame_id小于extend_frame_num_则不能拼成local_map
  if (proposed_key_frame_id < extend_frame_num_)
    return false;

  // update detection interval:
  skip_cnt = 0;
  skip_num = static_cast<int>(key_frame_distance);
  // if the nearest frame is further away than detect_area_
  // we consider that there is no loop closure in key_frame_distance
  if (key_frame_distance > detect_area_) {
    skip_num =
        std::max((int)(key_frame_distance / key_frame_distance_), loop_step_);
    return false;
  } else {
    key_frame_index = proposed_key_frame_id;
    yaw_change_in_rad = proposed_yaw_change;

    skip_num = loop_step_;
    return true;
  }
}

bool LoopClosing::CloudRegistration(const int key_frame_index,
                                    const float yaw_change_in_rad) {
  // 生成地图
  CloudData::CloudTypePtr map_cloud_ptr(new CloudData::CloudType());
  Eigen::Matrix4f map_pose = Eigen::Matrix4f::Identity();
  JointMap(key_frame_index, yaw_change_in_rad, map_cloud_ptr, map_pose);

  // 生成当前scan
  CloudData::CloudTypePtr scan_cloud_ptr(new CloudData::CloudType());
  Eigen::Matrix4f scan_pose = Eigen::Matrix4f::Identity();
  JointScan(scan_cloud_ptr, scan_pose);

  // 匹配
  Eigen::Matrix4f result_pose = Eigen::Matrix4f::Identity();
  Registration(map_cloud_ptr, scan_cloud_ptr, scan_pose, result_pose);

  // 计算相对位姿
  current_loop_pose_.pose_ = map_pose.inverse() * result_pose;

  // 判断是否有效
  if (registration_ptr_->GetFitnessScore() > fitness_score_limit_)
    return false;

  static int loop_close_cnt = 0;
  loop_close_cnt++;

  LOG(INFO) << std::endl
            << "[ICP Registration] Loop-Closure Detected "
            << current_loop_pose_.index0_ << "<-->"
            << current_loop_pose_.index1_ << std::endl
            << "\tFitness Score " << registration_ptr_->GetFitnessScore()
            << std::endl
            << std::endl;

  return true;
}

bool LoopClosing::JointMap(const int key_frame_index,
                           const float yaw_change_in_rad,
                           CloudData::CloudTypePtr& map_cloud_ptr,
                           Eigen::Matrix4f& map_pose) {
  // init map pose as loop closure pose:
  map_pose = all_key_gnss_.at(key_frame_index).pose_;

  // apply yaw change estimation from scan context match:
  const Eigen::AngleAxisf orientation_change(yaw_change_in_rad,
                                             Eigen::Vector3f::UnitZ());
  map_pose.block<3, 3>(0, 0) =
      map_pose.block<3, 3>(0, 0) * orientation_change.toRotationMatrix();

  current_loop_pose_.index0_ = all_key_frames_.at(key_frame_index).index_;

  // create local map:
  const Eigen::Matrix4f pose_to_gnss =
      map_pose * all_key_frames_.at(key_frame_index).pose_.inverse();
  for (int i = key_frame_index - extend_frame_num_;
       i < key_frame_index + extend_frame_num_;
       ++i) {
    // a. load back surrounding key scan:
    const std::string file_path = key_frames_path_ + "/key_frame_" +
                                  std::to_string(all_key_frames_.at(i).index_) +
                                  ".pcd";
    CloudData::CloudTypePtr cloud_ptr(new CloudData::CloudType());
    pcl::io::loadPCDFile(file_path, *cloud_ptr);

    // b. transform surrounding key scan to map frame:
    const Eigen::Matrix4f cloud_pose =
        pose_to_gnss * all_key_frames_.at(i).pose_;
    pcl::transformPointCloud(*cloud_ptr, *cloud_ptr, cloud_pose);

    *map_cloud_ptr += *cloud_ptr;
  }
  // pre-process current map:
  map_filter_ptr_->Filter(map_cloud_ptr, map_cloud_ptr);

  return true;
}

bool LoopClosing::JointScan(CloudData::CloudTypePtr& scan_cloud_ptr,
                            Eigen::Matrix4f& scan_pose) {
  // set scan pose as GNSS estimation:
  scan_pose = all_key_gnss_.back().pose_;
  current_loop_pose_.index1_ = all_key_frames_.back().index_;
  current_loop_pose_.time_ = all_key_frames_.back().time_;

  // load back current key scan:
  const std::string file_path = key_frames_path_ + "/key_frame_" +
                                std::to_string(all_key_frames_.back().index_) +
                                ".pcd";
  pcl::io::loadPCDFile(file_path, *scan_cloud_ptr);

  // pre-process current scan:
  scan_filter_ptr_->Filter(scan_cloud_ptr, scan_cloud_ptr);

  return true;
}

bool LoopClosing::Registration(CloudData::CloudTypePtr& map_cloud_ptr,
                               CloudData::CloudTypePtr& scan_cloud_ptr,
                               Eigen::Matrix4f& scan_pose,
                               Eigen::Matrix4f& result_pose) {
  // point cloud registration:
  CloudData::CloudTypePtr result_cloud_ptr(new CloudData::CloudType());
  registration_ptr_->SetInputTarget(map_cloud_ptr);
  registration_ptr_->ScanMatch(
      scan_cloud_ptr, scan_pose, result_cloud_ptr, result_pose);

  return true;
}

// bool LoopClosing::HasNewLoopPose() { return has_new_loop_pose_; }

// LoopPose& LoopClosing::GetCurrentLoopPose() { return current_loop_pose_; }

}  // namespace lidar_localization