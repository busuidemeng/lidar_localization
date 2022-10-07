/*
 * @Description: Subscribe to PosVel messages
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-21 23:26:14
 */
#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_POS_VEL_SUBSCRIBER_H_
#define LIDAR_LOCALIZATION_SUBSCRIBER_POS_VEL_SUBSCRIBER_H_

#include <deque>
#include <mutex>
#include <thread>

#include <glog/logging.h>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include "lidar_localization/PosVel.h"
#include "lidar_localization/sensor_data/pos_vel_data.h"

namespace lidar_localization {

class PosVelSubscriber {
 public:
  PosVelSubscriber(const ros::NodeHandle& nh,
                   const std::string topic_name,
                   const size_t buff_size);

  PosVelSubscriber() = default;

  void ParseData(std::deque<PosVelData>& pos_vel_data_buff);

 private:
  void MsgCallback(const PosVelConstPtr& pos_vel_msg_ptr);

  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  std::deque<PosVelData> new_pos_vel_data_;

  std::mutex buff_mutex_;
};

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_SUBSCRIBER_POS_VEL_SUBSCRIBER_H_