/*
 * @Description: Kalman filter based localization on GNSS-INS-Sim workflow
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-10-03 22:00:06
 */

#include "lidar_localization/filtering/gnss_ins_sim_filtering_flow.h"

namespace lidar_localization {

GNSSINSSimFilteringFlow::GNSSINSSimFilteringFlow(ros::NodeHandle& nh) {
  // subscriber:
  // a. IMU raw measurement:
  imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/synced_imu", 1000000);
  // b. synced GNSS-odo-mag measurement:
  pos_vel_sub_ptr_ =
      std::make_shared<PosVelSubscriber>(nh, "/synced_pos_vel", 100000);
  // c. reference trajectory:
  ref_pose_sub_ptr_ = std::make_shared<OdometrySubscriber>(
      nh, "/synced_reference_pose", 100000);

  // publisher:
  // a. fused pose in map frame:
  fused_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(
      nh, "/fused_pose", "/map", "/imu_link", 100);
  // b. tf, map -> imu_link:
  imu_tf_pub_ptr_ = std::make_shared<TFBroadCaster>("/map", "/imu_link");
  // c. covariance:
  fused_std_pub_ = nh.advertise<lidar_localization::EKFStd>("/fused_std", 100);
  fused_std_.header.frame_id = "/imu_link";

  // filtering instance:
  filtering_ptr_ = std::make_shared<GNSSINSSimFiltering>();
}

bool GNSSINSSimFilteringFlow::Run() {
  ReadData();

  while (HasData()) {
    if (!HasInited()) {
      if (HasPosVelData() && ValidPosVelData() && HasIMUData() &&
          ValidIMUData()) {
        InitLocalization();
      }
    } else {
      // TODO: handle timestamp chaos in an more elegant way
      if (HasPosVelData() && ValidPosVelData()) {
        if (HasIMUData()) {
          while (HasIMUData() && ValidIMUData() &&
                 current_imu_data_.time_ < current_pos_vel_data_.time_) {
            UpdateLocalization();
          }

          if (current_imu_data_.time_ >= current_pos_vel_data_.time_) {
            imu_data_buff_.push_back(current_imu_data_);
          }
        }

        CorrectLocalization();
      }

      if (HasIMUData() && ValidIMUData()) {
        UpdateLocalization();
      }
    }
  }

  return true;
}

bool GNSSINSSimFilteringFlow::SaveOdometry() {
  if (0 == trajectory_.N) {
    return false;
  }

  // init output files:
  std::ofstream fused_odom_ofs;
  std::ofstream gnss_odom_ofs;
  std::ofstream ref_odom_ofs;
  if (!FileManager::CreateFile(
          WORK_SPACE_PATH + "/slam_data/trajectory/fused.txt",
          fused_odom_ofs) ||
      !FileManager::CreateFile(
          WORK_SPACE_PATH + "/slam_data/trajectory/gnss.txt", gnss_odom_ofs) ||
      !FileManager::CreateFile(
          WORK_SPACE_PATH + "/slam_data/trajectory/ground_truth.txt",
          ref_odom_ofs)) {
    return false;
  }

  // write outputs:
  const double half_sampling_time = 0.005;  // 100hz
  for (size_t i = 0; i < trajectory_.N; ++i) {
    // sync ref pose with gnss measurement:
    while (!ref_pose_data_buff_.empty() &&
           (ref_pose_data_buff_.front().time_ - trajectory_.time.at(i) <=
            -half_sampling_time)) {
      ref_pose_data_buff_.pop_front();
    }

    if (ref_pose_data_buff_.empty()) {
      break;
    }
    current_ref_pose_data_ = ref_pose_data_buff_.front();

    SavePose(trajectory_.fused.at(i), fused_odom_ofs);
    SavePose(trajectory_.gnss.at(i), gnss_odom_ofs);
    SavePose(current_ref_pose_data_.pose_, ref_odom_ofs);
  }

  return true;
}

bool GNSSINSSimFilteringFlow::SaveObservabilityAnalysis() const {
  filtering_ptr_->SaveObservabilityAnalysis();

  return true;
}

bool GNSSINSSimFilteringFlow::ReadData() {
  //
  // pipe synced IMU-GNSS measurements into buffer:
  //
  imu_sub_ptr_->ParseData(imu_data_buff_);
  pos_vel_sub_ptr_->ParseData(pos_vel_data_buff_);
  ref_pose_sub_ptr_->ParseData(ref_pose_data_buff_);

  return true;
}

bool GNSSINSSimFilteringFlow::HasInited() const {
  return filtering_ptr_->has_inited();
}

bool GNSSINSSimFilteringFlow::HasData() const {
  if (!HasInited()) {
    if (!HasIMUData() || !HasPosVelData()) {
      return false;
    }
  } else {
    if (!HasIMUData() && !HasPosVelData()) {
      return false;
    }
  }

  return true;
}

bool GNSSINSSimFilteringFlow::ValidIMUData() {
  current_imu_data_ = imu_data_buff_.front();

  imu_data_buff_.pop_front();

  return true;
}

bool GNSSINSSimFilteringFlow::ValidPosVelData() {
  current_pos_vel_data_ = pos_vel_data_buff_.front();

  pos_vel_data_buff_.pop_front();

  return true;
}

bool GNSSINSSimFilteringFlow::InitLocalization() {
  // init pos & vel:
  gnss_pose_(0, 3) = current_pos_vel_data_.pos_.x();
  gnss_pose_(1, 3) = current_pos_vel_data_.pos_.y();
  gnss_pose_(2, 3) = current_pos_vel_data_.pos_.z();

  Eigen::Vector3f init_vel =
      current_imu_data_.GetOrientationMatrix() * current_pos_vel_data_.vel_;

  // set:
  filtering_ptr_->Init(gnss_pose_, init_vel, current_imu_data_);

  LOG(INFO) << "Init localization Kalman filter with first IMU & GNSS-odo-mag "
               "measurement"
            << std::endl;

  return true;
}

bool GNSSINSSimFilteringFlow::UpdateLocalization() {
  if (filtering_ptr_->Update(current_imu_data_)) {
    // publish new odom estimation:
    PublishFusionOdom();

    return true;
  }

  return false;
}

bool GNSSINSSimFilteringFlow::CorrectLocalization() {
  static int count = 0;

  if (
      // downsample GNSS measurement:
      // note: 把gnss数据从400hz降到10hz
      0 == (++count % 40) &&
      // successful correct:
      filtering_ptr_->Correct(current_imu_data_, current_pos_vel_data_)) {
    // reset downsample counter:
    count = 0;

    // publish new odom estimation:
    PublishFusionOdom();

    // add to odometry output for evo evaluation:
    UpdateOdometry(current_pos_vel_data_.time_);

    return true;
  }

  return false;
}

bool GNSSINSSimFilteringFlow::PublishFusionOdom() {
  // get odometry from Kalman filter:
  filtering_ptr_->GetOdometry(fused_pose_, fused_vel_);

  // a. publish fusion odometry:
  fused_odom_pub_ptr_->Publish(
      fused_pose_, fused_vel_, current_imu_data_.time_);
  // b. publish tf:
  imu_tf_pub_ptr_->SendTransform(fused_pose_, current_imu_data_.time_);

  // publish standard deviation:
  PublishFusionStandardDeviation();

  return true;
}

bool GNSSINSSimFilteringFlow::PublishFusionStandardDeviation() {
  // get standard deviation from Kalman filter:
  filtering_ptr_->GetStandardDeviation(fused_std_);
  // c. publish standard deviation:
  fused_std_pub_.publish(fused_std_);

  return true;
}

bool GNSSINSSimFilteringFlow::UpdateOdometry(const double time) {
  trajectory_.time.push_back(time);

  trajectory_.fused.push_back(fused_pose_);

  gnss_pose_(0, 3) = current_pos_vel_data_.pos_.x();
  gnss_pose_(1, 3) = current_pos_vel_data_.pos_.y();
  gnss_pose_(2, 3) = current_pos_vel_data_.pos_.z();

  trajectory_.gnss.push_back(gnss_pose_);

  ++trajectory_.N;

  return true;
}

/**
 * @brief  save pose in KITTI format for evo evaluation
 * @param  pose, input pose
 * @param  ofs, output file stream
 * @return true if success otherwise false
 */
bool GNSSINSSimFilteringFlow::SavePose(const Eigen::Matrix4f& pose,
                                       std::ofstream& ofs) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 4; ++j) {
      ofs << pose(i, j);

      if (i == 2 && j == 3) {
        ofs << std::endl;
      } else {
        ofs << " ";
      }
    }
  }

  return true;
}

}  // namespace lidar_localization