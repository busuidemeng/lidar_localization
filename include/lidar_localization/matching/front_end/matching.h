/*
 * @Description: lidar localization frontend, interface
 * @Author: Ren Qian
 * @Date: 2020-02-04 18:52:45
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-22 23:24:56
 */
#ifndef LIDAR_LOCALIZATION_MATCHING_FRONT_END_MATCHING_H_
#define LIDAR_LOCALIZATION_MATCHING_FRONT_END_MATCHING_H_

#include <deque>

#include <glog/logging.h>

#include <Eigen/Dense>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <yaml-cpp/yaml.h>

// models
#include "lidar_localization/models/cloud_filter/box_filter.h"
#include "lidar_localization/models/cloud_filter/cloud_filter_interface.h"
#include "lidar_localization/models/cloud_filter/no_filter.h"
#include "lidar_localization/models/cloud_filter/voxel_filter.h"
#include "lidar_localization/models/registration/ndt_registration.h"
#include "lidar_localization/models/registration/registration_interface.h"
// sensor_data
#include "lidar_localization/sensor_data/cloud_data.h"
#include "lidar_localization/sensor_data/pose_data.h"
// global
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {

class Matching {
 public:
  Matching();

  bool Update(const CloudData& cloud_data,
              Eigen::Matrix4f& laser_pose,
              Eigen::Matrix4f& map_matching_pose);

  bool has_inited() const { return has_inited_; }

  bool has_new_global_map() const { return has_new_global_map_; }

  bool has_new_local_map() const { return has_new_local_map_; }

  const Eigen::Matrix4f& init_pose() const { return init_pose_; }

  const CloudData::CloudTypePtr& local_map_ptr() const {
    return local_map_ptr_;
  }

  const CloudData::CloudTypePtr& current_scan_ptr() const {
    return current_scan_ptr_;
  }

  const CloudData::CloudTypePtr& GetGlobalMapPtr() {
    has_new_global_map_ = false;
    return global_map_ptr_;
  };

  bool SetGNSSPose(const Eigen::Matrix4f& init_pose);

 private:
  bool InitWithConfig();
  //
  // point cloud map & measurement processors:
  //
  bool InitFilter(const YAML::Node& config_node,
                  std::string filter_user,
                  std::shared_ptr<CloudFilterInterface>& filter_ptr);
  bool InitLocalMapSegmenter(const YAML::Node& config_node);
  bool InitPointCloudProcessors(const YAML::Node& config_node);
  //
  // global map:
  //
  bool InitGlobalMap(const YAML::Node& config_node);
  //
  // lidar frontend for relative pose estimation:
  //
  bool InitRegistration(
      const YAML::Node& config_node,
      std::shared_ptr<RegistrationInterface>& registration_ptr);

  bool SetInitPose(const Eigen::Matrix4f& init_pose);
  bool ResetLocalMap(float x, float y, float z);

  std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_;

  std::shared_ptr<BoxFilter> local_map_segmenter_ptr_;
  std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;

  std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;

  CloudData::CloudTypePtr global_map_ptr_;
  CloudData::CloudTypePtr local_map_ptr_;
  CloudData::CloudTypePtr current_scan_ptr_;

  std::shared_ptr<RegistrationInterface> registration_ptr_;

  Eigen::Matrix4f current_pose_ = Eigen::Matrix4f::Identity();

  Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f current_gnss_pose_ = Eigen::Matrix4f::Identity();

  bool has_inited_ = false;
  bool has_new_global_map_ = false;
  bool has_new_local_map_ = false;
};

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_MATCHING_FRONTEND_MATCHING_HPP_