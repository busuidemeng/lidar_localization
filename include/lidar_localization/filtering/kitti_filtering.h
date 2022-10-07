/*
 * @Description: IMU-lidar fusion for localization workflow
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-25 15:40:35
 */
#ifndef LIDAR_LOCALIZATION_FILTERING_KITTI_FILTERING_H_
#define LIDAR_LOCALIZATION_FILTERING_KITTI_FILTERING_H_

#include <deque>
#include <unordered_map>

#include <glog/logging.h>

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include "lidar_localization/sensor_data/cloud_data.h"
#include "lidar_localization/sensor_data/imu_data.h"
#include "lidar_localization/sensor_data/pos_vel_data.h"
#include "lidar_localization/sensor_data/pose_data.h"
// models
#include "lidar_localization/models/cloud_filter/box_filter.h"
#include "lidar_localization/models/cloud_filter/cloud_filter_interface.h"
#include "lidar_localization/models/cloud_filter/no_filter.h"
#include "lidar_localization/models/cloud_filter/voxel_filter.h"
#include "lidar_localization/models/kalman_filter/error_state_kalman_filter.h"
#include "lidar_localization/models/kalman_filter/kalman_filter.h"
#include "lidar_localization/models/registration/ndt_registration.h"
#include "lidar_localization/models/registration/registration_interface.h"
// global
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {

class KITTIFiltering {
 public:
  KITTIFiltering();

  bool Init(const CloudData& init_scan,
            const Eigen::Vector3f& init_vel,
            const IMUData& init_imu_data);

  bool Init(const Eigen::Matrix4f& init_pose,
            const Eigen::Vector3f& init_vel,
            const IMUData& init_imu_data);

  bool Update(const IMUData& imu_data);
  bool Correct(const IMUData& imu_data,
               const CloudData& cloud_data,
               const PosVelData& pos_vel_data,
               Eigen::Matrix4f& cloud_pose);

  // getters:
  bool has_inited() const { return has_inited_; }
  bool has_new_global_map() const { return has_new_global_map_; }
  bool has_new_local_map() const { return has_new_local_map_; }

  void GetGlobalMap(CloudData::CloudTypePtr& global_map);
  CloudData::CloudTypePtr& local_map_ptr() { return local_map_ptr_; }
  CloudData::CloudTypePtr& current_scan_ptr() { return current_scan_ptr_; }

  double GetTime() const { return kalman_filter_ptr_->time(); }
  const Eigen::Matrix4f& current_pose() const { return current_pose_; }
  const Eigen::Vector3f& current_vel() const { return current_vel_; }
  void GetOdometry(Eigen::Matrix4f& pose, Eigen::Vector3f& vel);

 private:
  bool InitWithConfig();
  // a. filter initializer:
  bool InitFilter(const YAML::Node& config_node,
                  const std::string& filter_user,
                  std::shared_ptr<CloudFilterInterface>& filter_ptr);
  bool InitLocalMapSegmenter(const YAML::Node& config_node);
  bool InitFilters(const YAML::Node& config_node);
  // b. map initializer:
  bool InitGlobalMap(const YAML::Node& config_node);
  // c. scan context manager initializer:
  bool InitScanContextManager(const YAML::Node& config_node);
  // d. frontend initializer:
  bool InitRegistration(
      const YAML::Node& config_node,
      std::shared_ptr<RegistrationInterface>& registration_ptr);
  // e. IMU-lidar fusion initializer:
  bool InitFusion(const YAML::Node& config_node);

  // local map setter:
  bool ResetLocalMap(const float x, const float y, const float z);

  // init pose setter:
  bool SetInitScan(const CloudData& init_scan);
  bool SetInitGNSS(const Eigen::Matrix4f& init_pose);
  bool SetInitPose(const Eigen::Matrix4f& init_pose);

  std::string map_path_{};

  // a. global map:
  std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_{};
  // b. local map:
  std::shared_ptr<BoxFilter> local_map_segmenter_ptr_{};
  std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_{};
  // c. current scan:
  std::shared_ptr<CloudFilterInterface> current_scan_filter_ptr_{};

  // // scan context manager:
  // std::shared_ptr<ScanContextManager> scan_context_manager_ptr_;
  // frontend:
  std::shared_ptr<RegistrationInterface> registration_ptr_{};
  // IMU-lidar Kalman filter:
  struct {
    std::string fusion_method{};
    std::unordered_map<std::string, KalmanFilter::MeasurementType>
        fusion_strategy_id;
    KalmanFilter::MeasurementType fusion_strategy;
  } config_;
  std::shared_ptr<KalmanFilter> kalman_filter_ptr_{};
  KalmanFilter::Measurement current_measurement_;

  CloudData::CloudTypePtr global_map_ptr_{};
  CloudData::CloudTypePtr local_map_ptr_{};
  CloudData::CloudTypePtr current_scan_ptr_{};

  Eigen::Matrix4f current_gnss_pose_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f current_pose_ = Eigen::Matrix4f::Identity();
  Eigen::Vector3f current_vel_ = Eigen::Vector3f::Zero();

  bool has_inited_{false};
  bool has_new_global_map_{false};
  bool has_new_local_map_{false};
};

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_FILTERING_KITTI_FILTERING_H_