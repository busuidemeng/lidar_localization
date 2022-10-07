/*
 * @Description: 在ros中发布IMU数据
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:27:30
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-22 23:16:24
 */
#ifndef LIDAR_LOCALIZATION_PUBLISHER_IMU_PUBLISHER_H_
#define LIDAR_LOCALIZATION_PUBLISHER_IMU_PUBLISHER_H_

#include <glog/logging.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "lidar_localization/sensor_data/imu_data.h"

namespace lidar_localization {
class IMUPublisher {
 public:
  IMUPublisher(const ros::NodeHandle& nh,
               const std::string topic_name,
               const std::string frame_id,
               const size_t buff_size);

  IMUPublisher() = default;

  void Publish(const IMUData& imu_data, const double time);

  void Publish(const IMUData& imu_data);

  bool HasSubscribers() const { return publisher_.getNumSubscribers() != 0; }

 private:
  void PublishData(const IMUData& imu_data, const ros::Time time);

  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  std::string frame_id_{};

  sensor_msgs::Imu imu_;
};
}  // namespace lidar_localization
#endif