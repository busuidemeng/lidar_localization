/*
 * @Description: 单个 key frame 信息发布
 * @Author: Ren Qian
 * @Date: 2020-02-06 21:11:44
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-22 09:39:39
 */
#include "lidar_localization/publisher/key_frame_publisher.h"

namespace lidar_localization {
KeyFramePublisher::KeyFramePublisher(const ros::NodeHandle& nh,
                                     const std::string topic_name,
                                     const std::string frame_id,
                                     const int buff_size)
    : nh_(nh)
    , frame_id_(frame_id) {
  publisher_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      topic_name, buff_size);
}

void KeyFramePublisher::Publish(const KeyFrame& key_frame) const {
  geometry_msgs::PoseWithCovarianceStamped pose_stamped;

  const ros::Time ros_time(key_frame.time_);
  pose_stamped.header.stamp = ros_time;
  pose_stamped.header.frame_id = frame_id_;

  pose_stamped.pose.pose.position.x = key_frame.pose_(0, 3);
  pose_stamped.pose.pose.position.y = key_frame.pose_(1, 3);
  pose_stamped.pose.pose.position.z = key_frame.pose_(2, 3);

  Eigen::Quaternionf q = key_frame.GetQuaternion();
  pose_stamped.pose.pose.orientation.x = q.x();
  pose_stamped.pose.pose.orientation.y = q.y();
  pose_stamped.pose.pose.orientation.z = q.z();
  pose_stamped.pose.pose.orientation.w = q.w();

  pose_stamped.pose.covariance[0] = (double)key_frame.index_;

  publisher_.publish(pose_stamped);
}
}  // namespace lidar_localization