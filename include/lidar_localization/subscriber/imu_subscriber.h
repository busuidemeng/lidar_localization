/*
 * @Description: 订阅imu数据
 * @Author: Ren Qian
 * @Date: 2019-08-19 19:22:17
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-21 23:03:01
 */
#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_IMU_SUBSCRIBER_H_
#define LIDAR_LOCALIZATION_SUBSCRIBER_IMU_SUBSCRIBER_H_

#include <deque>
#include <mutex>
#include <thread>

#include <glog/logging.h>

#include <ros/ros.h>
#include "sensor_msgs/Imu.h"

#include "lidar_localization/sensor_data/imu_data.h"

namespace lidar_localization {
class IMUSubscriber {
 public:
  IMUSubscriber(const ros::NodeHandle& nh,
                const std::string topic_name,
                const size_t buff_size);

  IMUSubscriber() = default;

  void ParseData(std::deque<IMUData>& deque_imu_data);

 private:
  void MsgCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);

  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  std::deque<IMUData> new_imu_data_;

  std::mutex buff_mutex_;
};
}  // namespace lidar_localization
#endif