/*
 * @Description: 发布tf的类
 * @Author: Ren Qian
 * @Date: 2020-03-05 15:23:26
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-22 10:26:11
 */

#include "lidar_localization/publisher/tf_broadcaster.h"

namespace lidar_localization {
TFBroadCaster::TFBroadCaster(const std::string frame_id,
                             const std::string child_frame_id) {
  transform_.frame_id_ = frame_id;
  transform_.child_frame_id_ = child_frame_id;
}

void TFBroadCaster::SendTransform(const Eigen::Matrix4f pose,
                                  const double time) {
  Eigen::Quaternionf q(pose.block<3, 3>(0, 0));
  ros::Time ros_time(time);
  transform_.stamp_ = ros_time;
  transform_.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
  transform_.setOrigin(tf::Vector3(pose(0, 3), pose(1, 3), pose(2, 3)));
  broadcaster_.sendTransform(transform_);
}
}  // namespace lidar_localization