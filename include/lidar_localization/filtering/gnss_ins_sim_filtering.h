/*
 * @Description: Kalman filter based localization on GNSS-INS-Sim
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-25 15:43:20
 */
#ifndef LIDAR_LOCALIZATION_FILTERING_GNSS_INS_SIM_FILTERING_H_
#define LIDAR_LOCALIZATION_FILTERING_GNSS_INS_SIM_FILTERING_H_

#include <deque>
#include <string>
#include <unordered_map>

#include <glog/logging.h>

#include <Eigen/Dense>

#include <yaml-cpp/yaml.h>

#include "lidar_localization/sensor_data/imu_data.h"
#include "lidar_localization/sensor_data/pos_vel_data.h"
#include "lidar_localization/sensor_data/pose_data.h"
// message
#include "lidar_localization/EKFStd.h"
// models
#include "lidar_localization/models/kalman_filter/error_state_kalman_filter.h"
#include "lidar_localization/models/kalman_filter/kalman_filter.h"
// global
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {

class GNSSINSSimFiltering {
 public:
  GNSSINSSimFiltering();

  bool Init(const Eigen::Matrix4f& init_pose,
            const Eigen::Vector3f& init_vel,
            const IMUData& init_imu_data);

  bool Update(const IMUData& imu_data);
  bool Correct(const IMUData& imu_data, const PosVelData& pos_vel_data);

  // getters:
  bool has_inited() const { return has_inited_; }
  const Eigen::Matrix4f& current_pose() const { return current_pose_; }
  const Eigen::Vector3f& current_vel() const { return current_vel_; }

  double GetTime(void) const { return kalman_filter_ptr_->time(); }

  void GetOdometry(Eigen::Matrix4f& pose, Eigen::Vector3f& vel);
  void GetStandardDeviation(EKFStd& kf_std_msg);
  void SaveObservabilityAnalysis();

 private:
  bool InitWithConfig();
  bool InitFusion(const YAML::Node& config_node);

  // init pose setter:
  bool SetInitGNSS(const Eigen::Matrix4f& init_pose);
  bool SetInitPose(const Eigen::Matrix4f& init_pose);

  bool has_inited_ = false;

  // Kalman filter:
  struct {
    std::string fusion_method{};
    std::unordered_map<std::string, KalmanFilter::MeasurementType>
        fusion_strategy_id;
    KalmanFilter::MeasurementType fusion_strategy;
  } config_;
  std::shared_ptr<KalmanFilter> kalman_filter_ptr_{};
  KalmanFilter::Measurement current_measurement_;

  Eigen::Matrix4f current_gnss_pose_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f current_pose_ = Eigen::Matrix4f::Identity();
  Eigen::Vector3f current_vel_ = Eigen::Vector3f::Zero();
  KalmanFilter::Cov current_cov_;
};

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_FILTERING_GNSS_INS_SIM_FILTERING_H_