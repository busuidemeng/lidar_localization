/*
 * @Description: 订阅imu数据
 * @Author: Ren Qian
 * @Date: 2019-06-14 16:44:18
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-28 22:03:30
 */
#include "lidar_localization/subscriber/imu_subscriber.h"

namespace lidar_localization {
IMUSubscriber::IMUSubscriber(const ros::NodeHandle& nh,
                             const std::string topic_name,
                             const size_t buff_size)
    : nh_(nh) {
  subscriber_ = nh_.subscribe(topic_name,
                              buff_size,
                              &IMUSubscriber::MsgCallback,
                              this,
                              ros::TransportHints().tcpNoDelay());
}

void IMUSubscriber::MsgCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr) {
  buff_mutex_.lock();

  // convert ROS IMU to GeographicLib compatible GNSS message:
  IMUData imu_data;
  imu_data.time_ = imu_msg_ptr->header.stamp.toSec();

  imu_data.linear_acceleration_.x = imu_msg_ptr->linear_acceleration.x;
  imu_data.linear_acceleration_.y = imu_msg_ptr->linear_acceleration.y;
  imu_data.linear_acceleration_.z = imu_msg_ptr->linear_acceleration.z;

  imu_data.angular_velocity_.x = imu_msg_ptr->angular_velocity.x;
  imu_data.angular_velocity_.y = imu_msg_ptr->angular_velocity.y;
  imu_data.angular_velocity_.z = imu_msg_ptr->angular_velocity.z;

  imu_data.orientation_.x = imu_msg_ptr->orientation.x;
  imu_data.orientation_.y = imu_msg_ptr->orientation.y;
  imu_data.orientation_.z = imu_msg_ptr->orientation.z;
  imu_data.orientation_.w = imu_msg_ptr->orientation.w;

  // add new message to buffer:
  new_imu_data_.push_back(imu_data);

  buff_mutex_.unlock();
}

void IMUSubscriber::ParseData(std::deque<IMUData>& imu_data_buff) {
  buff_mutex_.lock();

  // pipe all available measurements to output buffer:
  if (new_imu_data_.size() > 0) {
    imu_data_buff.insert(
        imu_data_buff.end(), new_imu_data_.begin(), new_imu_data_.end());
    new_imu_data_.clear();
  }

  buff_mutex_.unlock();
}
}  // namespace lidar_localization