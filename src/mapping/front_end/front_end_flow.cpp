/*
 * @Description: front end 任务管理， 放在类里使代码更清晰
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:38:42
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-10-02 21:36:41
 */
#include "lidar_localization/mapping/front_end/front_end_flow.h"

namespace lidar_localization {
FrontEndFlow::FrontEndFlow(ros::NodeHandle& nh,
                           const std::string& cloud_topic,
                           const std::string& odom_topic) {
  cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, cloud_topic, 100000);
  laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(
      nh, odom_topic, "/map", "/lidar", 100);

  front_end_ptr_ = std::make_shared<FrontEnd>();
}

bool FrontEndFlow::Run() {
  if (!ReadData())
    return false;

  while (HasData()) {
    if (!ValidData())
      continue;

    if (UpdateLaserOdometry()) {
      PublishData();
    }
  }

  return true;
}

bool FrontEndFlow::ReadData() {
  cloud_sub_ptr_->ParseData(cloud_data_buff_);
  return true;
}

bool FrontEndFlow::ValidData() {
  current_cloud_data_ = cloud_data_buff_.front();
  cloud_data_buff_.pop_front();

  return true;
}

bool FrontEndFlow::UpdateLaserOdometry() {
  static bool odometry_inited = false;
  if (!odometry_inited) {
    odometry_inited = true;
    // init lidar odometry:
    front_end_ptr_->SetInitPose(Eigen::Matrix4f::Identity());
  }

  // update lidar odometry using current undistorted measurement:
  return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);
}

bool FrontEndFlow::PublishData() {
  laser_odom_pub_ptr_->Publish(laser_odometry_, current_cloud_data_.time_);

  return true;
}
}  // namespace lidar_localization