/*
 * @Description: Kalman filter based localization on GNSS-INS-Sim workflow
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-25 09:56:06
 */
#ifndef LIDAR_LOCALIZATION_FILTERING_GNSS_INS_SIM_FILTERING_FLOW_H_
#define LIDAR_LOCALIZATION_FILTERING_GNSS_INS_SIM_FILTERING_FLOW_H_

#include <ostream>

#include <glog/logging.h>

#include <ros/ros.h>
// message
#include "lidar_localization/EKFStd.h"
// subscribers:
// a. IMU:
#include "lidar_localization/subscriber/imu_subscriber.h"
// b. synced GNSS-odo measurements:
#include "lidar_localization/subscriber/pos_vel_subscriber.h"
// c. reference pose:
#include "lidar_localization/subscriber/odometry_subscriber.h"
// publishers:
#include "lidar_localization/publisher/odometry_publisher.h"
#include "lidar_localization/publisher/tf_broadcaster.h"
// filtering:
#include "lidar_localization/filtering/gnss_ins_sim_filtering.h"
// tools
#include "lidar_localization/tools/file_manager.h"

#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {

class GNSSINSSimFilteringFlow {
 public:
  GNSSINSSimFilteringFlow(ros::NodeHandle& nh);
  bool Run();

  // save odometry for evo evaluation:
  bool SaveOdometry();
  // save TOM measurements for observability analysis:
  bool SaveObservabilityAnalysis() const;

 private:
  bool ReadData();
  bool HasInited() const;

  bool HasData() const;

  bool HasIMUData() const { return (!imu_data_buff_.empty()); }
  bool HasPosVelData() const { return (!pos_vel_data_buff_.empty()); }
  bool HasIMUComesFirst() const {
    return imu_data_buff_.front().time_ < pos_vel_data_buff_.front().time_;
  }

  bool ValidIMUData();
  bool ValidPosVelData();

  bool InitLocalization();

  bool UpdateLocalization();
  bool CorrectLocalization();

  bool PublishFusionOdom();
  bool PublishFusionStandardDeviation();

  bool UpdateOdometry(const double time);
  /**
   * @brief  save pose in KITTI format for evo evaluation
   * @param  pose, input pose
   * @param  ofs, output file stream
   * @return true if success otherwise false
   */
  static bool SavePose(const Eigen::Matrix4f& pose, std::ofstream& ofs);

  // subscriber:
  // a. IMU:
  std::shared_ptr<IMUSubscriber> imu_sub_ptr_{};
  std::deque<IMUData> imu_data_buff_;
  // b. synced GNSS-odo-mag measurement:
  std::shared_ptr<PosVelSubscriber> pos_vel_sub_ptr_{};
  std::deque<PosVelData> pos_vel_data_buff_;
  // c. reference trajectory:
  std::shared_ptr<OdometrySubscriber> ref_pose_sub_ptr_{};
  std::deque<PoseData> ref_pose_data_buff_;

  // publisher:
  // a. odometry:
  std::shared_ptr<OdometryPublisher> fused_odom_pub_ptr_{};
  // b. tf:
  std::shared_ptr<TFBroadCaster> imu_tf_pub_ptr_{};
  // c. standard deviation:
  ros::Publisher fused_std_pub_;
  // filtering instance:
  std::shared_ptr<GNSSINSSimFiltering> filtering_ptr_{};

  IMUData current_imu_data_;
  PosVelData current_pos_vel_data_;
  PoseData current_ref_pose_data_;

  // fused odometry:
  Eigen::Matrix4f gnss_pose_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f fused_pose_ = Eigen::Matrix4f::Identity();
  Eigen::Vector3f fused_vel_ = Eigen::Vector3f::Zero();
  EKFStd fused_std_;

  // trajectory for evo evaluation:
  struct {
    size_t N = 0;
    std::deque<double> time;
    std::deque<Eigen::Matrix4f> fused;
    std::deque<Eigen::Matrix4f> gnss;
    std::deque<Eigen::Matrix4f> ref;
  } trajectory_;
};

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_FILTERING_GNSS_INS_SIM_FILTERING_FLOW_HPP_