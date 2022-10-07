/*
 * @Description: 单个 key frame 信息发布
 * @Author: Ren Qian
 * @Date: 2020-02-06 21:05:47
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-22 23:16:33
 */
#ifndef LIDAR_LOCALIZATION_PUBLISHER_KEY_FRAME_PUBLISHER_H_
#define LIDAR_LOCALIZATION_PUBLISHER_KEY_FRAME_PUBLISHER_H_

#include <string>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>

#include <Eigen/Dense>

#include "lidar_localization/sensor_data/key_frame.h"

namespace lidar_localization {
class KeyFramePublisher {
 public:
  KeyFramePublisher(const ros::NodeHandle& nh,
                    const std::string topic_name,
                    const std::string frame_id,
                    const int buff_size);

  KeyFramePublisher() = default;

  void Publish(const KeyFrame& key_frame) const;

  bool HasSubscribers() const { return publisher_.getNumSubscribers() != 0; }

 private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  std::string frame_id_{};
};
}  // namespace lidar_localization
#endif