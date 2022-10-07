/*
 * @Description: data preprocess for urbanNav dataset
 * @Author: ZiJieChen
 * @Date: 2022-10-03 14:28:23
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-10-03 15:33:37
 */
#ifndef LIDAR_LOCALIZATION_DATA_HK_PRETREAT_DATA_PRETREAT_FLOW_H_
#define LIDAR_LOCALIZATION_DATA_HK_PRETREAT_DATA_PRETREAT_FLOW_H_

#include <glog/logging.h>

#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
// subscriber
#include "lidar_localization/subscriber/cloud_subscriber.h"
#include "lidar_localization/subscriber/gnss_subscriber.h"
#include "lidar_localization/subscriber/imu_subscriber.h"
#include "lidar_localization/subscriber/odometry_subscriber.h"
#include "lidar_localization/subscriber/velocity_subscriber.h"
#include "lidar_localization/tf_listener/tf_listener.h"
// publisher
// a. synced lidar measurement
#include "lidar_localization/publisher/cloud_publisher.h"
// b. synced IMU measurement
#include "lidar_localization/publisher/imu_publisher.h"
// c. synced GNSS-odo measurement:
#include "lidar_localization/publisher/pos_vel_publisher.h"
// d. synced reference trajectory:
#include "lidar_localization/publisher/odometry_publisher.h"
// models
#include "lidar_localization/models/scan_adjust/distortion_adjust.h"
// tools
#include "lidar_localization/tools/file_manager.h"
// global
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {
class HKDataPretreatFlow {
 public:
  HKDataPretreatFlow(const ros::NodeHandle& nh, const std::string cloud_topic);

  bool Run();

  bool SaveGnssOrigin();

 private:
  bool ReadData();
  bool InitCalibration();
  bool InitGNSS();
  bool HasData();
  bool ValidData();
  bool TransformData();
  bool PublishData() const;

  // subscriber
  std::shared_ptr<IMUSubscriber> imu_sub_ptr_{};
  std::shared_ptr<VelocitySubscriber> velocity_sub_ptr_{};
  std::shared_ptr<CloudSubscriber> cloud_sub_ptr_{};
  // std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_{};
  std::shared_ptr<OdometrySubscriber> gnss_sub_ptr_{};
  std::shared_ptr<TFListener> lidar_to_imu_ptr_{};
  // publisher
  std::shared_ptr<CloudPublisher> cloud_pub_ptr_{};
  std::shared_ptr<IMUPublisher> imu_pub_ptr_{};
  std::shared_ptr<PosVelPublisher> pos_vel_pub_ptr_{};
  std::shared_ptr<OdometryPublisher> gnss_pub_ptr_{};
  // models
  std::shared_ptr<DistortionAdjust> distortion_adjust_ptr_{};

  Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();

  std::deque<CloudData> cloud_data_buff_;
  std::deque<IMUData> imu_data_buff_;
  std::deque<VelocityData> velocity_data_buff_;
  std::deque<PoseData> gnss_data_buff_;

  CloudData current_cloud_data_;
  IMUData current_imu_data_;
  VelocityData current_velocity_data_;
  PoseData current_gnss_data_;

  PosVelData pos_vel_;
  Eigen::Matrix4f gnss_pose_ = Eigen::Matrix4f::Identity();

  bool is_mapping_{true};
  GNSSData gnss_init_data_;

  Eigen::Matrix4f T_imu_lidar_ = Eigen::Matrix4f::Identity();
};
}  // namespace lidar_localization

#endif