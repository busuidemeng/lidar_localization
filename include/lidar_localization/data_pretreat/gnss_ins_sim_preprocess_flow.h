/*
 * @Description: GNSS-INS-Sim measurement preprocessing workflow
 * @Author: Ge Yao
 * @Date: 2020-11-21 15:39:24
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-24 22:48:04
 */
#ifndef LIDAR_LOCALIZATION_DATA_PRETREAT_GNSS_INS_SIM_PREPROCESS_FLOW_H_
#define LIDAR_LOCALIZATION_DATA_PRETREAT_GNSS_INS_SIM_PREPROCESS_FLOW_H_

#include <glog/logging.h>

// ROS common:
#include <ros/ros.h>

// subscriber:
#include "lidar_localization/subscriber/gnss_subscriber.h"
#include "lidar_localization/subscriber/imu_subscriber.h"
#include "lidar_localization/subscriber/odometry_subscriber.h"
#include "lidar_localization/subscriber/velocity_subscriber.h"
// publisher:
// a. IMU measurement:
#include "lidar_localization/publisher/imu_publisher.h"
// b. synced GNSS-odo measurement:
#include "lidar_localization/publisher/pos_vel_publisher.h"
// c. reference trajectory:
#include "lidar_localization/publisher/odometry_publisher.h"

#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {

class GNSSINSSimPreprocessFlow {
 public:
  GNSSINSSimPreprocessFlow(ros::NodeHandle& nh);
  bool Run();

 private:
  bool ReadData();
  bool InitGNSS();
  bool HasData();
  bool ValidData();
  bool TransformData();
  bool PublishData();

  // subscriber:
  std::shared_ptr<IMUSubscriber> imu_sub_ptr_{};
  std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_{};
  std::shared_ptr<VelocitySubscriber> odo_sub_ptr_{};
  std::shared_ptr<OdometrySubscriber> ref_pose_sub_ptr_{};
  // publisher:
  std::shared_ptr<IMUPublisher> imu_pub_ptr_{};
  std::shared_ptr<PosVelPublisher> pos_vel_pub_ptr_{};
  std::shared_ptr<OdometryPublisher> gnss_pose_pub_ptr_{};
  std::shared_ptr<OdometryPublisher> ref_pose_pub_ptr_{};

  std::deque<IMUData> imu_data_buff_;
  std::deque<GNSSData> gnss_data_buff_;
  std::deque<VelocityData> odo_data_buff_;
  std::deque<PoseData> ref_pose_data_buff_;

  IMUData current_imu_data_;
  GNSSData current_gnss_data_;
  VelocityData current_odo_data_;
  PoseData current_ref_pose_data_;

  PosVelData pos_vel_;
  Eigen::Matrix4f gnss_pose_ = Eigen::Matrix4f::Identity();
};

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_DATA_PRETREAT_GNSS_INS_SIM_PREPROCESS_FLOW_H_