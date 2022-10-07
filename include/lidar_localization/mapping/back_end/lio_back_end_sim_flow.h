/*
 * @Description:
 * @Autor: ZiJieChen
 * @Date: 2022-09-28 14:48:35
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-28 19:32:08
 */
#ifndef LIDAR_LOCALIZATION_MAPPING_BACK_END_LIO_BACK_END_SIM_FLOW_H_
#define LIDAR_LOCALIZATION_MAPPING_BACK_END_LIO_BACK_END_SIM_FLOW_H_

#include <glog/logging.h>

#include <ros/ros.h>

#include "lidar_localization/subscriber/imu_subscriber.h"
#include "lidar_localization/subscriber/odometry_subscriber.h"
#include "lidar_localization/subscriber/pos_vel_subscriber.h"

#include "lidar_localization/publisher/key_frame_publisher.h"
#include "lidar_localization/publisher/key_frames_publisher.h"
#include "lidar_localization/publisher/odometry_publisher.h"

#include "lidar_localization/mapping/back_end/lio_back_end_sim.h"

namespace lidar_localization {

class LIOBackEndSimFlow {
 public:
  LIOBackEndSimFlow(ros::NodeHandle& nh);

  bool Run();

  bool ForceOptimize();

  bool SaveOptimizedOdometry();

 private:
  bool ReadData();

  bool HasData();

  bool ValidData();

  bool UpdateBackEnd();

  bool UpdateIMUPreIntegration();

  bool PublishData();

  // subscriber
  // synced IMU
  std::shared_ptr<IMUSubscriber> imu_sub_ptr_{};
  std::deque<IMUData> imu_data_buff_;
  // synced posi-vel
  std::shared_ptr<PosVelSubscriber> pos_vel_sub_ptr_{};
  std::deque<PosVelData> pos_vel_data_buff_;
  // synced reference trajectory
  std::shared_ptr<OdometrySubscriber> ref_pose_sub_ptr_{};
  std::deque<PoseData> ref_pose_data_buff_;

  // publisher
  std::shared_ptr<OdometryPublisher> lidar_odom_pub_ptr_{};
  std::shared_ptr<OdometryPublisher> gnss_odom_pub_ptr_{};
  std::shared_ptr<KeyFramesPublisher> key_frames_pub_ptr_{};

  std::shared_ptr<LIOBackEndSim> back_end_sim_ptr_{};

  IMUData current_imu_data_;
  PosVelData current_pos_vel_data_;
  PoseData current_ref_pos_data_;
};
}  // namespace lidar_localization
#endif