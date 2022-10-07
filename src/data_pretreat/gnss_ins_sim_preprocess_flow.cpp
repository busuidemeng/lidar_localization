/*
 * @Description: GNSS-INS-Sim measurement preprocessing workflow
 * @Author: Ge Yao
 * @Date: 2020-11-21 15:39:24
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-30 09:20:07
 */
#include "lidar_localization/data_pretreat/gnss_ins_sim_preprocess_flow.h"

namespace lidar_localization {

GNSSINSSimPreprocessFlow::GNSSINSSimPreprocessFlow(ros::NodeHandle& nh) {
  // subscribers:
  // a. GNSS-INS-Sim IMU:
  imu_sub_ptr_ =
      std::make_shared<IMUSubscriber>(nh, "/sim/sensor/imu", 1000000);
  // b. GNSS-INS-Sim GNSS:
  gnss_sub_ptr_ =
      std::make_shared<GNSSSubscriber>(nh, "/sim/sensor/gps/fix", 1000000);
  // c. GNSS-INS-Sim odo:
  odo_sub_ptr_ =
      std::make_shared<VelocitySubscriber>(nh, "/sim/sensor/odo", 1000000);
  // d. reference trajectory:
  ref_pose_sub_ptr_ =
      std::make_shared<OdometrySubscriber>(nh, "/reference_pose", 100000);

  // publishers:
  imu_pub_ptr_ =
      std::make_shared<IMUPublisher>(nh, "/synced_imu", "/imu_link", 100);
  pos_vel_pub_ptr_ = std::make_shared<PosVelPublisher>(
      nh, "/synced_pos_vel", "/map", "/imu_link", 100);
  gnss_pose_pub_ptr_ = std::make_shared<OdometryPublisher>(
      nh, "/synced_gnss_pose", "/map", "/imu_link", 100);
  ref_pose_pub_ptr_ = std::make_shared<OdometryPublisher>(
      nh, "/synced_reference_pose", "/map", "/imu_link", 100);
}

bool GNSSINSSimPreprocessFlow::Run() {
  if (!ReadData())
    return false;

  while (InitGNSS() && HasData()) {
    if (!ValidData()) {
      LOG(INFO) << "Invalid data. Skip." << std::endl;
      continue;
    }

    TransformData();
    PublishData();
  }

  return true;
}

bool GNSSINSSimPreprocessFlow::ReadData() {
  // pipe sensor measurements into buffer:
  imu_sub_ptr_->ParseData(imu_data_buff_);
  gnss_sub_ptr_->ParseData(gnss_data_buff_);
  odo_sub_ptr_->ParseData(odo_data_buff_);
  ref_pose_sub_ptr_->ParseData(ref_pose_data_buff_);

  return true;
}

bool GNSSINSSimPreprocessFlow::InitGNSS() {
  static bool gnss_inited = false;

  if (!gnss_inited && !gnss_data_buff_.empty()) {
    GNSSData gnss_data = gnss_data_buff_.front();
    gnss_data.InitOriginPosition();
    gnss_inited = true;

    LOG(INFO) << "Init local map frame at: " << gnss_data.latitude_ << ", "
              << gnss_data.longitude_ << ", " << gnss_data.altitude_
              << std::endl;
  }

  return gnss_inited;
}

bool GNSSINSSimPreprocessFlow::HasData() {
  if (imu_data_buff_.size() == 0 || gnss_data_buff_.size() == 0 ||
      odo_data_buff_.size() == 0 || ref_pose_data_buff_.size() == 0) {
    return false;
  }

  return true;
}

bool GNSSINSSimPreprocessFlow::ValidData() {
  current_imu_data_ = imu_data_buff_.front();
  current_gnss_data_ = gnss_data_buff_.front();
  current_odo_data_ = odo_data_buff_.front();
  current_ref_pose_data_ = ref_pose_data_buff_.front();

  const double diff_gnss_time =
      current_imu_data_.time_ - current_gnss_data_.time_;
  const double diff_odo_time =
      current_imu_data_.time_ - current_odo_data_.time_;
  const double diff_ref_pose_time =
      current_imu_data_.time_ - current_ref_pose_data_.time_;

  //
  // this check assumes the frequency of GNSS-INS-Sim is 400Hz:
  //
  // half_sampling_time = 1/400/2 采样周期的一半
  const double half_sampling_time = 0.005;  // 0.00125
  if (diff_gnss_time < -half_sampling_time ||
      diff_odo_time < -half_sampling_time ||
      diff_ref_pose_time < -half_sampling_time) {
    imu_data_buff_.pop_front();
    return false;
  }

  if (diff_gnss_time > half_sampling_time) {
    gnss_data_buff_.pop_front();
    return false;
  }

  if (diff_odo_time > half_sampling_time) {
    odo_data_buff_.pop_front();
    return false;
  }

  if (diff_ref_pose_time > half_sampling_time) {
    ref_pose_data_buff_.pop_front();
    return false;
  }

  imu_data_buff_.pop_front();
  gnss_data_buff_.pop_front();
  odo_data_buff_.pop_front();
  ref_pose_data_buff_.pop_front();

  return true;
}

bool GNSSINSSimPreprocessFlow::TransformData() {
  // a. get synced GNSS-odo measurement:
  current_gnss_data_.UpdateXYZ();

  gnss_pose_ = Eigen::Matrix4f::Identity();

  gnss_pose_(0, 3) = +current_gnss_data_.local_N_;
  gnss_pose_(1, 3) = +current_gnss_data_.local_E_;
  gnss_pose_(2, 3) = -current_gnss_data_.local_U_;

  pos_vel_.pos_.x() = +current_gnss_data_.local_N_;
  pos_vel_.pos_.y() = +current_gnss_data_.local_E_;
  pos_vel_.pos_.z() = -current_gnss_data_.local_U_;

  pos_vel_.vel_.x() = current_odo_data_.linear_velocity_.x;
  pos_vel_.vel_.y() = current_odo_data_.linear_velocity_.y;
  pos_vel_.vel_.z() = current_odo_data_.linear_velocity_.z;

  // b. transform reference pose position from LLA to xyz:
  GNSSData ref_position;

  ref_position.latitude_ = current_ref_pose_data_.pose_(0, 3);
  ref_position.longitude_ = current_ref_pose_data_.pose_(1, 3);
  ref_position.altitude_ = current_ref_pose_data_.pose_(2, 3);

  ref_position.UpdateXYZ();

  current_ref_pose_data_.pose_(0, 3) = +ref_position.local_N_;
  current_ref_pose_data_.pose_(1, 3) = +ref_position.local_E_;
  current_ref_pose_data_.pose_(2, 3) = -ref_position.local_U_;
  current_ref_pose_data_.vel_.v.x() = current_odo_data_.linear_velocity_.x;
  current_ref_pose_data_.vel_.v.y() = current_odo_data_.linear_velocity_.y;
  current_ref_pose_data_.vel_.v.z() = current_odo_data_.linear_velocity_.z;

  return true;
}

bool GNSSINSSimPreprocessFlow::PublishData() {
  imu_pub_ptr_->Publish(current_imu_data_, current_imu_data_.time_);

  pos_vel_pub_ptr_->Publish(pos_vel_, current_imu_data_.time_);

  gnss_pose_pub_ptr_->Publish(gnss_pose_, current_imu_data_.time_);

  ref_pose_pub_ptr_->Publish(current_ref_pose_data_.pose_,
                             current_ref_pose_data_.vel_.v,
                             current_imu_data_.time_);

  return true;
}

}  // namespace lidar_localization