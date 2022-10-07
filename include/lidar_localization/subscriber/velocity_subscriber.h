/*
 * @Description: 订阅velocity数据
 * @Author: Ren Qian
 * @Date: 2019-08-19 19:22:17
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-21 23:27:33
 */
#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_VELOCITY_SUBSCRIBER_H_
#define LIDAR_LOCALIZATION_SUBSCRIBER_VELOCITY_SUBSCRIBER_H_

#include <deque>
#include <mutex>
#include <thread>

#include <glog/logging.h>

#include <ros/ros.h>
#include "geometry_msgs/TwistStamped.h"

#include "lidar_localization/sensor_data/velocity_data.h"

namespace lidar_localization {
class VelocitySubscriber {
 public:
  VelocitySubscriber(const ros::NodeHandle& nh,
                     const std::string topic_name,
                     const size_t buff_size);

  VelocitySubscriber() = default;

  void ParseData(std::deque<VelocityData>& deque_velocity_data);

 private:
  void MsgCallback(const geometry_msgs::TwistStampedConstPtr& twist_msg_ptr);

  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  std::deque<VelocityData> new_velocity_data_;

  std::mutex buff_mutex_;
};
}  // namespace lidar_localization
#endif