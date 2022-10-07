/*
 * @Description: LIO mapping backend workflow for ubarnNav dataset
 * @Author: ZiJieChen
 * @Date: 2022-10-03 15:38:05
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-10-03 15:43:12
 */

#include "lidar_localization/mapping/back_end/hk_lio_back_end_flow.h"

namespace lidar_localization {

HKLIOBackEndFlow::HKLIOBackEndFlow(ros::NodeHandle& nh,
                                   const std::string& cloud_topic,
                                   const std::string& odom_topic) {
  //
  // subscribers:
  //
  // a. lidar scan, key frame measurement:
  cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, cloud_topic, 100000);
  // b. lidar odometry:
  laser_odom_sub_ptr_ =
      std::make_shared<OdometrySubscriber>(nh, odom_topic, 100000);
  // c. GNSS position:
  gnss_pose_sub_ptr_ =
      std::make_shared<OdometrySubscriber>(nh, "/synced_gnss", 100000);
  // d. loop closure detection:
  loop_pose_sub_ptr_ =
      std::make_shared<LoopPoseSubscriber>(nh, "/loop_pose", 100000);
  // e. IMU measurement, for pre-integration:
  imu_raw_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/imu/data", 1000000);
  imu_synced_sub_ptr_ =
      std::make_shared<IMUSubscriber>(nh, "/synced_imu", 100000);

  transformed_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(
      nh, "/transformed_odom", "/map", "/lidar", 100);
  key_scan_pub_ptr_ =
      std::make_shared<CloudPublisher>(nh, "/key_scan", "/velo_link", 100);
  key_frame_pub_ptr_ =
      std::make_shared<KeyFramePublisher>(nh, "/key_frame", "/map", 100);
  key_gnss_pub_ptr_ =
      std::make_shared<KeyFramePublisher>(nh, "/key_gnss", "/map", 100);
  key_frames_pub_ptr_ = std::make_shared<KeyFramesPublisher>(
      nh, "/optimized_key_frames", "/map", 100);

  const std::string config_file_path =
      WORK_SPACE_PATH + "/config/mapping/hk_lio_back_end.yaml";
  back_end_ptr_ = std::make_shared<LIOBackEnd>(config_file_path);
}

bool HKLIOBackEndFlow::Run() {
  // load messages into buffer:
  if (!ReadData())
    return false;

  // add loop poses for graph optimization:
  InsertLoopClosurePose();

  while (HasData()) {
    // make sure undistorted Velodyne measurement -- lidar pose in map frame --
    // lidar odometry are synced:
    if (!ValidData())
      continue;

    UpdateBackEnd();

    PublishData();
  }

  return true;
}

bool HKLIOBackEndFlow::ForceOptimize() {
  static std::deque<KeyFrame> optimized_key_frames;

  back_end_ptr_->ForceOptimize();

  if (back_end_ptr_->has_new_optimized()) {
    back_end_ptr_->GetOptimizedKeyFrames(optimized_key_frames);
    key_frames_pub_ptr_->Publish(optimized_key_frames);
  }
  return true;
}

bool HKLIOBackEndFlow::SaveOptimizedOdometry() {
  back_end_ptr_->SaveOptimizedPose();

  return true;
}

bool HKLIOBackEndFlow::ReadData() {
  // a. lidar scan, key frame measurement:
  cloud_sub_ptr_->ParseData(cloud_data_buff_);
  // b. lidar odometry:
  laser_odom_sub_ptr_->ParseData(laser_odom_data_buff_);
  // c. GNSS position:
  gnss_pose_sub_ptr_->ParseData(gnss_pose_data_buff_);
  // d. loop closure detection:
  loop_pose_sub_ptr_->ParseData(loop_pose_data_buff_);
  // e. IMU measurement, for pre-integration:
  imu_raw_sub_ptr_->ParseData(imu_raw_data_buff_);
  imu_synced_sub_ptr_->ParseData(imu_synced_data_buff_);

  return true;
}

/**
 * @brief  add loop closure for backend optimization
 * @param  void
 * @return true if success false otherwise
 */
bool HKLIOBackEndFlow::InsertLoopClosurePose() {
  while (loop_pose_data_buff_.size() > 0) {
    back_end_ptr_->InsertLoopPose(loop_pose_data_buff_.front());
    loop_pose_data_buff_.pop_front();
  }

  return true;
}

bool HKLIOBackEndFlow::HasData() const {
  if (cloud_data_buff_.empty() || laser_odom_data_buff_.empty() ||
      gnss_pose_data_buff_.empty() || imu_synced_data_buff_.empty()) {
    return false;
  }

  return true;
}

bool HKLIOBackEndFlow::ValidData() {
  current_cloud_data_ = cloud_data_buff_.front();
  current_laser_odom_data_ = laser_odom_data_buff_.front();
  current_gnss_pose_data_ = gnss_pose_data_buff_.front();
  current_imu_data_ = imu_synced_data_buff_.front();

  const double diff_laser_time =
      current_cloud_data_.time_ - current_laser_odom_data_.time_;
  const double diff_gnss_time =
      current_cloud_data_.time_ - current_gnss_pose_data_.time_;
  const double diff_imu_time =
      current_cloud_data_.time_ - current_imu_data_.time_;

  const double half_sampling_time = 0.05;  // LiDAR is 10hz
  if (diff_laser_time < -half_sampling_time ||
      diff_gnss_time < -half_sampling_time ||
      diff_imu_time < -half_sampling_time) {
    cloud_data_buff_.pop_front();
    return false;
  }

  if (diff_laser_time > half_sampling_time) {
    laser_odom_data_buff_.pop_front();
    return false;
  }

  if (diff_gnss_time > half_sampling_time) {
    gnss_pose_data_buff_.pop_front();
    return false;
  }

  if (diff_imu_time > half_sampling_time) {
    imu_synced_data_buff_.pop_front();
    return false;
  }

  cloud_data_buff_.pop_front();
  laser_odom_data_buff_.pop_front();
  gnss_pose_data_buff_.pop_front();
  imu_synced_data_buff_.pop_front();

  return true;
}

bool HKLIOBackEndFlow::UpdateIMUPreIntegration() {
  while (!imu_raw_data_buff_.empty() &&
         imu_raw_data_buff_.front().time_ < current_imu_data_.time_ &&
         back_end_ptr_->UpdateIMUPreIntegration(imu_raw_data_buff_.front())) {
    imu_raw_data_buff_.pop_front();
  }

  return true;
}

// bool HKLIOBackEndFlow::UpdateOdoPreIntegration() {
//   while (!velocity_data_buff_.empty() &&
//          velocity_data_buff_.front().time_ < current_imu_data_.time_ &&
//          back_end_ptr_->UpdateOdoPreIntegration(velocity_data_buff_.front()))
//          {
//     velocity_data_buff_.pop_front();
//   }

//   return true;
// }

bool HKLIOBackEndFlow::UpdateBackEnd() {
  static bool odometry_inited = false;
  static Eigen::Matrix4f odom_init_pose = Eigen::Matrix4f::Identity();

  if (!odometry_inited) {
    // the origin of lidar odometry frame in map frame as init pose:
    odom_init_pose = current_gnss_pose_data_.pose_ *
                     current_laser_odom_data_.pose_.inverse();

    odometry_inited = true;
  }

  // update IMU pre-integration:
  UpdateIMUPreIntegration();

  // odom constraints is not available in HK dataset
  // // update odo pre-integration:
  // UpdateOdoPreIntegration();

  // current lidar odometry in map frame:
  current_laser_odom_data_.pose_ =
      odom_init_pose * current_laser_odom_data_.pose_;

  // optimization is carried out in map frame:
  return back_end_ptr_->Update(current_cloud_data_,
                               current_laser_odom_data_,
                               current_gnss_pose_data_,
                               current_imu_data_);
}

bool HKLIOBackEndFlow::PublishData() {
  transformed_odom_pub_ptr_->Publish(current_laser_odom_data_.pose_,
                                     current_laser_odom_data_.time_);

  if (back_end_ptr_->has_new_key_frame()) {
    CloudData key_scan;

    back_end_ptr_->GetLatestKeyScan(key_scan);
    key_scan_pub_ptr_->Publish(key_scan.cloud_ptr_, key_scan.time_);

    KeyFrame key_frame;

    back_end_ptr_->GetLatestKeyFrame(key_frame);
    key_frame_pub_ptr_->Publish(key_frame);

    back_end_ptr_->GetLatestKeyGNSS(key_frame);
    key_gnss_pub_ptr_->Publish(key_frame);
  }

  if (back_end_ptr_->has_new_optimized()) {
    std::deque<KeyFrame> optimized_key_frames;
    back_end_ptr_->GetOptimizedKeyFrames(optimized_key_frames);
    key_frames_pub_ptr_->Publish(optimized_key_frames);
  }

  return true;
}

}  // namespace lidar_localization