/*
 * @Description: 订阅odometry数据
 * @Author: Ren Qian
 * @Date: 2019-08-19 19:22:17
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-21 23:23:59
 */
#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_ODOMETRY_SUBSCRIBER_H_
#define LIDAR_LOCALIZATION_SUBSCRIBER_ODOMETRY_SUBSCRIBER_H_

#include <deque>
#include <mutex>
#include <thread>

#include <glog/logging.h>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include "lidar_localization/sensor_data/pose_data.h"

namespace lidar_localization {
class OdometrySubscriber {
 public:
  OdometrySubscriber(const ros::NodeHandle& nh,
                     const std::string topic_name,
                     const size_t buff_size);

  OdometrySubscriber() = default;

  void ParseData(std::deque<PoseData>& deque_pose_data);

 private:
  void MsgCallback(const nav_msgs::OdometryConstPtr& odom_msg_ptr);

  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  std::deque<PoseData> new_pose_data_;

  std::mutex buff_mutex_;
};
}  // namespace lidar_localization
#endif