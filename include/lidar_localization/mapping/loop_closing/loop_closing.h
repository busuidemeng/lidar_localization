/*
 * @Description: 闭环检测算法
 * @Author: Ren Qian
 * @Date: 2020-02-04 18:52:45
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-25 22:15:22
 */
#ifndef LIDAR_LOCALIZATION_MAPPING_LOOP_CLOSING_LOOP_CLOSING_H_
#define LIDAR_LOCALIZATION_MAPPING_LOOP_CLOSING_LOOP_CLOSING_H_

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>

#include <glog/logging.h>

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>

// models
#include "lidar_localization/models/cloud_filter/cloud_filter_interface.h"
#include "lidar_localization/models/cloud_filter/no_filter.h"
#include "lidar_localization/models/cloud_filter/voxel_filter.h"
#include "lidar_localization/models/registration/ndt_registration.h"
#include "lidar_localization/models/registration/registration_interface.h"
// sensor data
#include "lidar_localization/sensor_data/key_frame.h"
#include "lidar_localization/sensor_data/loop_pose.h"
// tools
#include "lidar_localization/tools/print_info.h"
// global
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {
class LoopClosing {
 public:
  LoopClosing();

  bool Update(const CloudData& key_scan,
              const KeyFrame& key_frame,
              const KeyFrame& key_gnss);

  // bool HasNewLoopPose();
  // LoopPose& GetCurrentLoopPose();
  bool has_new_loop_pose() const { return has_new_loop_pose_; }

  const LoopPose& current_loop_pose() const { return current_loop_pose_; }

 private:
  bool InitWithConfig();

  bool InitParam(const YAML::Node& config_node);

  bool InitDataPath(const YAML::Node& config_node);

  bool InitFilter(const YAML::Node& config_node,
                  const std::string filter_user,
                  std::shared_ptr<CloudFilterInterface>& filter_ptr);

  bool InitLoopClosure(const YAML::Node& config_node);

  bool InitRegistration(
      const YAML::Node& config_node,
      std::shared_ptr<RegistrationInterface>& registration_ptr);

  bool DetectNearestKeyFrame(int& key_frame_index, float& yaw_change_in_rad);

  bool CloudRegistration(const int key_frame_index,
                         const float yaw_change_in_rad);

  bool JointMap(const int key_frame_index,
                const float yaw_change_in_rad,
                CloudData::CloudTypePtr& map_cloud_ptr,
                Eigen::Matrix4f& map_pose);

  bool JointScan(CloudData::CloudTypePtr& scan_cloud_ptr,
                 Eigen::Matrix4f& scan_pose);

  bool Registration(CloudData::CloudTypePtr& map_cloud_ptr,
                    CloudData::CloudTypePtr& scan_cloud_ptr,
                    Eigen::Matrix4f& scan_pose,
                    Eigen::Matrix4f& result_pose);

  std::string key_frames_path_{};

  int extend_frame_num_{3};
  int loop_step_{10};
  int diff_num_{100};
  float detect_area_{10.0};  // loop closure detect area
  float fitness_score_limit_{2.0};
  float key_frame_distance_{2.0};

  std::shared_ptr<CloudFilterInterface> scan_filter_ptr_{};
  std::shared_ptr<CloudFilterInterface> map_filter_ptr_{};
  std::shared_ptr<RegistrationInterface> registration_ptr_{};

  std::deque<KeyFrame> all_key_frames_;
  std::deque<KeyFrame> all_key_gnss_;

  LoopPose current_loop_pose_;
  bool has_new_loop_pose_{false};
};
}  // namespace lidar_localization

#endif