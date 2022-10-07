/*
 * @Description: 发送闭环检测的相对位姿
 * @Author: Ren Qian
 * @Date: 2020-02-06 21:11:44
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-22 10:04:43
 */
#include "lidar_localization/publisher/loop_pose_publisher.h"

namespace lidar_localization {
LoopPosePublisher::LoopPosePublisher(const ros::NodeHandle& nh,
                                     const std::string topic_name,
                                     const std::string frame_id,
                                     const int buff_size)
    : nh_(nh)
    , frame_id_(frame_id) {
  publisher_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      topic_name, buff_size);
}

void LoopPosePublisher::Publish(const LoopPose& loop_pose) const {
  geometry_msgs::PoseWithCovarianceStamped pose_stamped;

  const ros::Time ros_time(loop_pose.time_);
  pose_stamped.header.stamp = ros_time;
  pose_stamped.header.frame_id = frame_id_;

  pose_stamped.pose.pose.position.x = loop_pose.pose_(0, 3);
  pose_stamped.pose.pose.position.y = loop_pose.pose_(1, 3);
  pose_stamped.pose.pose.position.z = loop_pose.pose_(2, 3);

  Eigen::Quaternionf q = loop_pose.GetQuaternion();
  pose_stamped.pose.pose.orientation.x = q.x();
  pose_stamped.pose.pose.orientation.y = q.y();
  pose_stamped.pose.pose.orientation.z = q.z();
  pose_stamped.pose.pose.orientation.w = q.w();

  pose_stamped.pose.covariance[0] = (double)loop_pose.index0_;
  pose_stamped.pose.covariance[1] = (double)loop_pose.index1_;

  publisher_.publish(pose_stamped);
}
}  // namespace lidar_localization