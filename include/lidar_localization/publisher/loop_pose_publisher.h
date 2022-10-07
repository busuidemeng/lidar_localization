/*
 * @Description: 发送闭环检测的相对位姿
 * @Author: Ren Qian
 * @Date: 2020-02-06 21:05:47
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-22 23:16:49
 */
#ifndef LIDAR_LOCALIZATION_PUBLISHER_LOOP_POSE_PUBLISHER_H_
#define LIDAR_LOCALIZATION_PUBLISHER_LOOP_POSE_PUBLISHER_H_

#include <string>

#include <glog/logging.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>

#include <Eigen/Dense>

#include "lidar_localization/sensor_data/loop_pose.h"

namespace lidar_localization {
class LoopPosePublisher {
 public:
  LoopPosePublisher(const ros::NodeHandle& nh,
                    const std::string topic_name,
                    const std::string frame_id,
                    const int buff_size);

  LoopPosePublisher() = default;

  void Publish(const LoopPose& loop_pose) const;

  bool HasSubscribers() const { return publisher_.getNumSubscribers() != 0; }

 private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  std::string frame_id_{};
};
}  // namespace lidar_localization
#endif