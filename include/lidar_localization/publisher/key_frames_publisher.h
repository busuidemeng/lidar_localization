/*
 * @Description: key frames 信息发布
 * @Author: Ren Qian
 * @Date: 2020-02-06 21:05:47
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-22 23:16:37
 */
#ifndef LIDAR_LOCALIZATION_PUBLISHER_KEY_FRAMES_PUBLISHER_H_
#define LIDAR_LOCALIZATION_PUBLISHER_KEY_FRAMES_PUBLISHER_H_

#include <deque>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <Eigen/Dense>

#include "lidar_localization/sensor_data/key_frame.h"

namespace lidar_localization {
class KeyFramesPublisher {
 public:
  KeyFramesPublisher(const ros::NodeHandle& nh,
                     const std::string topic_name,
                     const std::string frame_id,
                     const int buff_size);

  KeyFramesPublisher() = default;

  void Publish(const std::deque<KeyFrame>& key_frames) const;

  bool HasSubscribers() const { return publisher_.getNumSubscribers() != 0; }

 private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  std::string frame_id_{};
};
}  // namespace lidar_localization
#endif