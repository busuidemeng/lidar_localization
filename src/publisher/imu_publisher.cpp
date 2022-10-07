/*
 * @Description: 通过ros发布点云
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:27:30
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-22 09:41:11
 */
#include "lidar_localization/publisher/imu_publisher.h"

namespace lidar_localization {

IMUPublisher::IMUPublisher(const ros::NodeHandle& nh,
                           const std::string topic_name,
                           const std::string frame_id,
                           const size_t buff_size)
    : nh_(nh)
    , frame_id_(frame_id) {
  publisher_ = nh_.advertise<sensor_msgs::Imu>(topic_name, buff_size);

  imu_.header.frame_id = frame_id_;
}

void IMUPublisher::Publish(const IMUData& imu_data, const double time) {
  const ros::Time ros_time(time);
  PublishData(imu_data, ros_time);
}

void IMUPublisher::Publish(const IMUData& imu_data){
  const ros::Time time = ros::Time::now();
  PublishData(imu_data, time);
}

void IMUPublisher::PublishData(const IMUData& imu_data,
                               const ros::Time time){
  imu_.header.stamp = time;

  // set orientation:
  imu_.orientation.w = imu_data.orientation_.w;
  imu_.orientation.x = imu_data.orientation_.x;
  imu_.orientation.y = imu_data.orientation_.y;
  imu_.orientation.z = imu_data.orientation_.z;

  // set angular velocity:
  imu_.angular_velocity.x = imu_data.angular_velocity_.x;
  imu_.angular_velocity.y = imu_data.angular_velocity_.y;
  imu_.angular_velocity.z = imu_data.angular_velocity_.z;

  // set linear acceleration:
  imu_.linear_acceleration.x = imu_data.linear_acceleration_.x;
  imu_.linear_acceleration.y = imu_data.linear_acceleration_.y;
  imu_.linear_acceleration.z = imu_data.linear_acceleration_.z;

  publisher_.publish(imu_);
}

}  // namespace lidar_localization