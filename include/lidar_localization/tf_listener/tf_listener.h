/*
 * @Description: tf监听模块
 * @Author: Ren Qian
 * @Date: 2020-02-06 16:01:21
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-22 10:32:24
 */
#ifndef LIDAR_LOCALIZATION_TF_LISTENER_H_
#define LIDAR_LOCALIZATION_TF_LISTENER_H_

#include <string>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace lidar_localization {
class TFListener {
 public:
  TFListener(const ros::NodeHandle& nh,
             const std::string base_frame_id,
             const std::string child_frame_id);

  TFListener() = default;

  bool LookupData(Eigen::Matrix4f& transform_matrix) const;

 private:
  static bool TransformToMatrix(const tf::StampedTransform& transform,
                                Eigen::Matrix4f& transform_matrix);

  ros::NodeHandle nh_;
  tf::TransformListener listener_;
  std::string base_frame_id_{};
  std::string child_frame_id_{};
};
}  // namespace lidar_localization

#endif