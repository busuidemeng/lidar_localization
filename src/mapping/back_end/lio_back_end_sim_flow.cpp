/*
 * @Description:
 * @Autor: ZiJieChen
 * @Date: 2022-09-28 14:57:42
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-10-03 19:42:25
 */
#include "lidar_localization/mapping/back_end/lio_back_end_sim_flow.h"

namespace lidar_localization {

LIOBackEndSimFlow::LIOBackEndSimFlow(ros::NodeHandle& nh) {
  imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/sim/imu", 1000000);
  ref_pose_sub_ptr_ =
      std::make_shared<OdometrySubscriber>(nh, "/sim/ground_truth", 100000);

  lidar_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(
      nh, "/transformed_odom", "/map", "/lidar", 100);
  gnss_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(
      nh, "/synced_gnss", "/map", "/map", 100);
  key_frames_pub_ptr_ = std::make_shared<KeyFramesPublisher>(
      nh, "/optimized_key_frames", "/map", 100);

  back_end_sim_ptr_ = std::make_shared<LIOBackEndSim>();
}

bool LIOBackEndSimFlow::Run() {
  ReadData();

  while (HasData()) {
    if (!ValidData()) {
      continue;
    }

    UpdateBackEnd();
  }

  return true;
}

bool LIOBackEndSimFlow::ReadData() {
  imu_sub_ptr_->ParseData(imu_data_buff_);
  ref_pose_sub_ptr_->ParseData(ref_pose_data_buff_);

  return true;
}

bool LIOBackEndSimFlow::HasData() {
  if (imu_data_buff_.empty() || ref_pose_data_buff_.empty()) {
    return false;
  }

  return true;
}

bool LIOBackEndSimFlow::ValidData() {
  current_imu_data_ = imu_data_buff_.front();
  current_ref_pos_data_ = ref_pose_data_buff_.front();

  const double diff_ref_pos_time =
      current_imu_data_.time_ - current_ref_pos_data_.time_;

  const double half_sampling_time = 0.005;  // sim data is 100hz

  if (diff_ref_pos_time < -diff_ref_pos_time) {
    imu_data_buff_.pop_front();
    return false;
  }

  if (diff_ref_pos_time > half_sampling_time) {
    ref_pose_data_buff_.pop_front();
    return false;
  }

  imu_data_buff_.pop_front();
  ref_pose_data_buff_.pop_front();
  return true;
}

bool LIOBackEndSimFlow::UpdateBackEnd() {
  static size_t count = 0;
  count++;

  // downsampling measurement from 100hz to 10hz
  if (count % 10 == 0) {
    back_end_sim_ptr_->Update(
        current_ref_pos_data_, current_ref_pos_data_, current_imu_data_);

    PublishData();

    count = 0;
  } else {
    // update IMU pre-integration
    UpdateIMUPreIntegration();
  }

  return true;
}

bool LIOBackEndSimFlow::UpdateIMUPreIntegration() {
  back_end_sim_ptr_->UpdateIMUPreIntegration(current_imu_data_);
  return true;
}

bool LIOBackEndSimFlow::PublishData() {
  lidar_odom_pub_ptr_->Publish(current_ref_pos_data_.pose_,
                               current_ref_pos_data_.time_);

  if (back_end_sim_ptr_->has_new_key_frame()) {
    KeyFrame key_frame;
    back_end_sim_ptr_->GetLatestKeyGNSS(key_frame);
  }

  return true;
}

bool LIOBackEndSimFlow::ForceOptimize() {
  static std::deque<KeyFrame> optimized_key_frames;

  back_end_sim_ptr_->ForceOptimize();

  if (back_end_sim_ptr_->has_new_optimized()) {
    back_end_sim_ptr_->GetOptimizedKeyFrames(optimized_key_frames);
    key_frames_pub_ptr_->Publish(optimized_key_frames);
  }
  return true;
}

bool LIOBackEndSimFlow::SaveOptimizedOdometry() {
  back_end_sim_ptr_->SaveOptimizedPose();

  return true;
}

}  // namespace lidar_localization