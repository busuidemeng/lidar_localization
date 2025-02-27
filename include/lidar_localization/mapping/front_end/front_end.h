/*
 * @Description: 前端里程计算法
 * @Author: Ren Qian
 * @Date: 2020-02-04 18:52:45
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-25 21:55:56
 */
#ifndef LIDAR_LOCALIZATION_MAPPING_FRONT_END_FRONT_END_H_
#define LIDAR_LOCALIZATION_MAPPING_FRONT_END_FRONT_END_H_

#include <deque>
#include <fstream>

#include <yaml-cpp/yaml.h>

#include <glog/logging.h>

#include <Eigen/Dense>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <boost/filesystem.hpp>

// models
#include "lidar_localization/models/cloud_filter/cloud_filter_interface.h"
#include "lidar_localization/models/cloud_filter/no_filter.h"
#include "lidar_localization/models/cloud_filter/voxel_filter.h"
#include "lidar_localization/models/registration/ndt_registration.h"
#include "lidar_localization/models/registration/registration_interface.h"
// sensor data
#include "lidar_localization/sensor_data/cloud_data.h"
// tools
#include "lidar_localization/tools/print_info.h"
// global
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {
class FrontEnd {
 public:
  struct Frame {
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    CloudData cloud_data;
  };

  FrontEnd();

  bool Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose);

  bool SetInitPose(const Eigen::Matrix4f& init_pose);

 private:
  bool InitWithConfig();
  bool InitParam(const YAML::Node& config_node);
  bool InitRegistration(
      std::shared_ptr<RegistrationInterface>& registration_ptr,
      const YAML::Node& config_node);
  bool InitFilter(std::string filter_user,
                  std::shared_ptr<CloudFilterInterface>& filter_ptr,
                  const YAML::Node& config_node);
  bool UpdateWithNewFrame(const Frame& new_key_frame);

  std::string data_path_{};

  // scan filter:
  std::shared_ptr<CloudFilterInterface> frame_filter_ptr_{};
  // local map filter:
  std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_{};
  // point cloud registrator:
  std::shared_ptr<RegistrationInterface> registration_ptr_{};

  std::deque<Frame> local_map_frames_;

  CloudData::CloudTypePtr local_map_ptr_;
  Frame current_frame_;

  Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();

  float key_frame_distance_{2.0};  // m
  int local_frame_num_{20};
};
}  // namespace lidar_localization

#endif