/*
 * @Description: 订阅 key frame 数据
 * @Author: Ren Qian
 * @Date: 2019-08-19 19:22:17
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-21 23:05:55
 */
#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_KEY_FRAME_SUBSCRIBER_H_
#define LIDAR_LOCALIZATION_SUBSCRIBER_KEY_FRAME_SUBSCRIBER_H_

#include <deque>
#include <mutex>
#include <thread>

#include <glog/logging.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>

#include <Eigen/Dense>

#include "lidar_localization/sensor_data/key_frame.h"

namespace lidar_localization {
class KeyFrameSubscriber {
 public:
  KeyFrameSubscriber(const ros::NodeHandle& nh,
                     const std::string topic_name,
                     const size_t buff_size);

  KeyFrameSubscriber() = default;

  void ParseData(std::deque<KeyFrame>& key_frame_buff);

 private:
  void MsgCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr&
                       key_frame_msg_ptr);

  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  std::deque<KeyFrame> new_key_frame_;

  std::mutex buff_mutex_;
};
}  // namespace lidar_localization
#endif