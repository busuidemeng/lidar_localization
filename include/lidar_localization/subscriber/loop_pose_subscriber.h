/*
 * @Description: 订阅 闭环检测位姿 数据
 * @Author: Ren Qian
 * @Date: 2019-08-19 19:22:17
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-21 23:20:32
 */
#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_LOOP_POSE_SUBSCRIBER_H_
#define LIDAR_LOCALIZATION_SUBSCRIBER_LOOP_POSE_SUBSCRIBER_H_

#include <deque>
#include <mutex>
#include <thread>

#include <glog/logging.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>

#include "lidar_localization/sensor_data/loop_pose.h"

namespace lidar_localization {
class LoopPoseSubscriber {
 public:
  LoopPoseSubscriber(const ros::NodeHandle& nh,
                     const std::string topic_name,
                     const size_t buff_size);

  LoopPoseSubscriber() = default;

  void ParseData(std::deque<LoopPose>& loop_pose_buff);

 private:
  void MsgCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr&
                       loop_pose_msg_ptr);

  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  std::deque<LoopPose> new_loop_pose_;

  std::mutex buff_mutex_;
};
}  // namespace lidar_localization
#endif