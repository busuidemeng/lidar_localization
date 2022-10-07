/*
 * @Description: IMU-lidar fusion for localization workflow
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-25 10:22:07
 */
#ifndef LIDAR_LOCALIZATION_FILTERING_KITTI_FILTERING_FLOW_H_
#define LIDAR_LOCALIZATION_FILTERING_KITTI_FILTERING_FLOW_H_

#include <ostream>

#include <glog/logging.h>

#include <ros/ros.h>

// subscriber:
// a. IMU:
#include "lidar_localization/subscriber/imu_subscriber.h"
// b. lidar:
#include "lidar_localization/subscriber/cloud_subscriber.h"
// c. synced GNSS-odo measurements:
#include "lidar_localization/subscriber/pos_vel_subscriber.h"
// d. GNSS:
#include "lidar_localization/subscriber/odometry_subscriber.h"
// e. lidar to imu:
#include "lidar_localization/tf_listener/tf_listener.h"
// publisher:
#include "lidar_localization/publisher/cloud_publisher.h"
#include "lidar_localization/publisher/odometry_publisher.h"
#include "lidar_localization/publisher/tf_broadcaster.h"
// tools
#include "lidar_localization/tools/file_manager.h"
// filtering instance:
#include "lidar_localization/filtering/kitti_filtering.h"

#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {

class KITTIFilteringFlow {
 public:
  KITTIFilteringFlow(ros::NodeHandle& nh);
  bool Run();
  // save odometry for evo evaluation:
  bool SaveOdometry();

 private:
  bool ReadData();
  bool HasInited() const;

  bool HasData();

  bool HasIMUData() const {
    if (!imu_raw_data_buff_.empty()) {
      double diff_filter_time =
          current_imu_raw_data_.time_ - filtering_ptr_->GetTime();

      if (diff_filter_time <= 0.01) {
        return true;
      }
    }

    return false;
  }

  bool HasLidarData() const {
    return (!cloud_data_buff_.empty() && !imu_synced_data_buff_.empty() &&
            !pos_vel_data_buff_.empty());
  }

  bool HasIMUComesFirst() const {
    return imu_raw_data_buff_.front().time_ < cloud_data_buff_.front().time_;
  }

  bool ValidIMUData();
  bool ValidLidarData();

  bool InitCalibration();
  bool InitLocalization() const;

  bool UpdateLocalization();
  bool CorrectLocalization();

  bool PublishGlobalMap() const;
  bool PublishLocalMap() const;
  bool PublishLidarOdom() const;
  bool PublishFusionOdom();

  bool UpdateOdometry(const double time);

  /**
   * @brief  save pose in KITTI format for evo evaluation
   * @param  pose, input pose
   * @param  ofs, output file stream
   * @return true if success otherwise false
   */
  static bool SavePose(const Eigen::Matrix4f& pose, std::ofstream& ofs);

  // subscriber:
  // a. IMU raw:
  std::shared_ptr<IMUSubscriber> imu_raw_sub_ptr_{};
  std::deque<IMUData> imu_raw_data_buff_;
  // b. lidar:
  std::shared_ptr<CloudSubscriber> cloud_sub_ptr_{};
  std::deque<CloudData> cloud_data_buff_;
  // c. synced GNSS-odo measurement:
  std::shared_ptr<PosVelSubscriber> pos_vel_sub_ptr_{};
  std::deque<PosVelData> pos_vel_data_buff_;
  // c. IMU synced:
  std::shared_ptr<IMUSubscriber> imu_synced_sub_ptr_{};
  std::deque<IMUData> imu_synced_data_buff_;
  // e. GNSS:
  std::shared_ptr<OdometrySubscriber> gnss_sub_ptr_{};
  std::deque<PoseData> gnss_data_buff_;
  // f. lidar to imu tf:
  std::shared_ptr<TFListener> lidar_to_imu_ptr_{};
  Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();

  // publisher:
  // a. global-local map and current scan:
  std::shared_ptr<CloudPublisher> global_map_pub_ptr_{};
  std::shared_ptr<CloudPublisher> local_map_pub_ptr_{};
  std::shared_ptr<CloudPublisher> current_scan_pub_ptr_{};
  // b. odometry:
  std::shared_ptr<OdometryPublisher> fused_odom_pub_ptr_{};
  std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_{};
  std::shared_ptr<OdometryPublisher> ref_odom_pub_ptr_{};
  // c. tf:
  std::shared_ptr<TFBroadCaster> laser_tf_pub_ptr_{};

  // filtering instance:
  std::shared_ptr<KITTIFiltering> filtering_ptr_{};

  IMUData current_imu_raw_data_;

  CloudData current_cloud_data_;
  IMUData current_imu_synced_data_;
  PosVelData current_pos_vel_data_;
  PoseData current_gnss_data_;

  // lidar odometry frame in map frame:
  Eigen::Matrix4f fused_pose_ = Eigen::Matrix4f::Identity();
  Eigen::Vector3f fused_vel_ = Eigen::Vector3f::Zero();

  Eigen::Matrix4f laser_pose_ = Eigen::Matrix4f::Identity();

  // trajectory for evo evaluation:
  struct {
    size_t N{0};
    std::deque<double> time;
    std::deque<Eigen::Matrix4f> fused;
    std::deque<Eigen::Matrix4f> lidar;
    std::deque<Eigen::Matrix4f> ref;
  } trajectory_;
};

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_FILTERING_KITTI_FILTERING_FLOW_HPP_