/*
 * @Description: 订阅 key frame 数据
 * @Author: Ren Qian
 * @Date: 2019-08-19 19:22:17
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-21 23:07:56
 */
#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_KEY_FRAMES_SUBSCRIBER_H_
#define LIDAR_LOCALIZATION_SUBSCRIBER_KEY_FRAMES_SUBSCRIBER_H_

#include <deque>
#include <mutex>
#include <thread>

#include <glog/logging.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include "lidar_localization/sensor_data/key_frame.h"

namespace lidar_localization {
class KeyFramesSubscriber {
 public:
  KeyFramesSubscriber(const ros::NodeHandle& nh,
                      const std::string topic_name,
                      const size_t buff_size);

  KeyFramesSubscriber() = default;

  void ParseData(std::deque<KeyFrame>& deque_key_frames);

 private:
  void MsgCallback(const nav_msgs::Path::ConstPtr& key_frames_msg_ptr);

  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  std::deque<KeyFrame> new_key_frames_;

  std::mutex buff_mutex_;
};
}  // namespace lidar_localization
#endif