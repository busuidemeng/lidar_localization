/*
 * @Description: lio localization backend workflow, implementation
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-26 23:12:33
 */
#include "lidar_localization/matching/back_end/sliding_window_flow.h"

namespace lidar_localization {

SlidingWindowFlow::SlidingWindowFlow(const ros::NodeHandle& nh) {
  //
  // subscribers:
  //
  // a. lidar odometry:
  // 当前帧与地图匹配得到的位姿
  laser_odom_sub_ptr_ =
      std::make_shared<OdometrySubscriber>(nh, "/laser_odometry", 100000);
  // b. map matching odometry:
  // 当前帧在scancontext中找匹配得到的位姿
  map_matching_odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(
      nh, "/map_matching_odometry", 100000);
  // c. IMU measurement, for pre-integration:
  imu_raw_sub_ptr_ =
      std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu/extract", 1000000);
  imu_synced_sub_ptr_ =
      std::make_shared<IMUSubscriber>(nh, "/synced_imu", 100000);
  // d. GNSS position:
  gnss_pose_sub_ptr_ =
      std::make_shared<OdometrySubscriber>(nh, "/synced_gnss", 100000);

  //
  //  publishers:
  //
  // a. current lidar key frame:
  key_frame_pub_ptr_ =
      std::make_shared<KeyFramePublisher>(nh, "/key_frame", "/map", 100);
  // b. current reference GNSS frame:
  key_gnss_pub_ptr_ =
      std::make_shared<KeyFramePublisher>(nh, "/key_gnss", "/map", 100);
  // c. optimized odometry:
  optimized_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(
      nh, "/optimized_odometry", "/map", "/lidar", 100);
  // d. optimized trajectory:
  optimized_trajectory_pub_ptr_ = std::make_shared<KeyFramesPublisher>(
      nh, "/optimized_trajectory", "/map", 100);
  // e. lidar frame
  laser_tf_pub_ptr_ = std::make_shared<TFBroadCaster>("/map", "/velo_link");

  //
  // backend:
  //
  sliding_window_ptr_ = std::make_shared<SlidingWindow>();
}

bool SlidingWindowFlow::Run() {
  // load messages into buffer:
  if (!ReadData()) {
    return false;
  }

  while (HasData()) {
    // make sure all the measurements are synced:
    if (!ValidData())
      continue;

    UpdateBackEnd();
    PublishData();
  }

  return true;
}

bool SlidingWindowFlow::SaveOptimizedTrajectory() {
  sliding_window_ptr_->SaveOptimizedTrajectory();

  return true;
}

bool SlidingWindowFlow::ReadData() {
  // a. lidar odometry:
  laser_odom_sub_ptr_->ParseData(laser_odom_data_buff_);
  // b. map matching odometry:
  map_matching_odom_sub_ptr_->ParseData(map_matching_odom_data_buff_);
  // c. IMU measurement, for pre-integration:
  imu_raw_sub_ptr_->ParseData(imu_raw_data_buff_);
  imu_synced_sub_ptr_->ParseData(imu_synced_data_buff_);
  // d. GNSS position:
  gnss_pose_sub_ptr_->ParseData(gnss_pose_data_buff_);

  return true;
}

bool SlidingWindowFlow::HasData() {
  if (laser_odom_data_buff_.empty() || map_matching_odom_data_buff_.empty() ||
      imu_synced_data_buff_.empty() || gnss_pose_data_buff_.empty()) {
    return false;
  }

  return true;
}

bool SlidingWindowFlow::ValidData() {
  current_laser_odom_data_ = laser_odom_data_buff_.front();
  current_map_matching_odom_data_ = map_matching_odom_data_buff_.front();
  current_imu_data_ = imu_synced_data_buff_.front();
  current_gnss_pose_data_ = gnss_pose_data_buff_.front();

  const double diff_map_matching_odom_time =
      current_laser_odom_data_.time_ - current_map_matching_odom_data_.time_;
  const double diff_imu_time =
      current_laser_odom_data_.time_ - current_imu_data_.time_;
  const double diff_gnss_pose_time =
      current_laser_odom_data_.time_ - current_gnss_pose_data_.time_;

  const double half_sampling_time = 0.05;  // LiDAR is 10hz
  if (diff_map_matching_odom_time < -half_sampling_time ||
      diff_imu_time < -half_sampling_time ||
      diff_gnss_pose_time < -half_sampling_time) {
    laser_odom_data_buff_.pop_front();
    return false;
  }

  if (diff_map_matching_odom_time > half_sampling_time) {
    map_matching_odom_data_buff_.pop_front();
    return false;
  }

  if (diff_imu_time > half_sampling_time) {
    imu_synced_data_buff_.pop_front();
    return false;
  }

  if (diff_gnss_pose_time > half_sampling_time) {
    gnss_pose_data_buff_.pop_front();
    return false;
  }

  laser_odom_data_buff_.pop_front();
  map_matching_odom_data_buff_.pop_front();
  imu_synced_data_buff_.pop_front();
  gnss_pose_data_buff_.pop_front();

  return true;
}

bool SlidingWindowFlow::UpdateIMUPreIntegration(void) {
  while (!imu_raw_data_buff_.empty() &&
         imu_raw_data_buff_.front().time_ < current_imu_data_.time_ &&
         sliding_window_ptr_->UpdateIMUPreIntegration(
             imu_raw_data_buff_.front())) {
    imu_raw_data_buff_.pop_front();
  }

  return true;
}

bool SlidingWindowFlow::UpdateBackEnd() {
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

  // current lidar odometry in map frame:
  current_laser_odom_data_.pose_ =
      odom_init_pose * current_laser_odom_data_.pose_;

  // optimization is carried out in map frame:
  return sliding_window_ptr_->Update(current_laser_odom_data_,
                                     current_map_matching_odom_data_,
                                     current_imu_data_,
                                     current_gnss_pose_data_);
}

bool SlidingWindowFlow::PublishData() {
  if (sliding_window_ptr_->has_new_key_frame()) {
    // KeyFrame key_frame;

    // sliding_window_ptr_->GetLatestKeyFrame(key_frame);
    // key_frame_pub_ptr_->Publish(key_frame);

    // sliding_window_ptr_->GetLatestKeyGNSS(key_frame);
    // key_gnss_pub_ptr_->Publish(key_frame);

    const KeyFrame& current_key_frame =
        sliding_window_ptr_->get_current_key_frame();
    key_frame_pub_ptr_->Publish(current_key_frame);

    const KeyFrame& current_key_gnss =
        sliding_window_ptr_->get_current_key_gnss();
    key_gnss_pub_ptr_->Publish(current_key_gnss);
  }

  if (sliding_window_ptr_->has_new_optimized()) {
    KeyFrame key_frame;
    sliding_window_ptr_->GetLatestOptimizedOdometry(key_frame);
    optimized_odom_pub_ptr_->Publish(key_frame.pose_, key_frame.time_);

    // publish lidar TF:
    laser_tf_pub_ptr_->SendTransform(key_frame.pose_, key_frame.time_);
  }

  return true;
}

}  // namespace lidar_localization