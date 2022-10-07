/*
 * @Description: 发布tf的类
 * @Author: Ren Qian
 * @Date: 2020-03-05 15:23:26
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-22 10:26:13
 */
#ifndef LIDAR_LOCALIZATION_PUBLISHER_TF_BROADCASTER_H_
#define LIDAR_LOCALIZATION_PUBLISHER_TF_BROADCASTER_H_

#include <string>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Dense>

namespace lidar_localization {
class TFBroadCaster {
 public:
  TFBroadCaster(const std::string frame_id, const std::string child_frame_id);

  TFBroadCaster() = default;

  void SendTransform(const Eigen::Matrix4f pose, const double time);

 protected:
  tf::StampedTransform transform_;
  tf::TransformBroadcaster broadcaster_;
};
}  // namespace lidar_localization
#endif