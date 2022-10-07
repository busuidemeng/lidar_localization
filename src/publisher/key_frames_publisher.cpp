/*
 * @Description: key frames 信息发布
 * @Author: Ren Qian
 * @Date: 2020-02-06 21:11:44
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-22 09:49:28
 */
#include "lidar_localization/publisher/key_frames_publisher.h"

namespace lidar_localization {
KeyFramesPublisher::KeyFramesPublisher(const ros::NodeHandle& nh,
                                       const std::string topic_name,
                                       const std::string frame_id,
                                       const int buff_size)
    : nh_(nh)
    , frame_id_(frame_id) {
  publisher_ = nh_.advertise<nav_msgs::Path>(topic_name, buff_size);
}

void KeyFramesPublisher::Publish(const std::deque<KeyFrame>& key_frames) const {
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = frame_id_;

  for (size_t i = 0; i < key_frames.size(); ++i) {
    KeyFrame key_frame = key_frames.at(i);

    geometry_msgs::PoseStamped pose_stamped;
    const ros::Time ros_time(key_frame.time_);
    pose_stamped.header.stamp = ros_time;
    pose_stamped.header.frame_id = frame_id_;

    pose_stamped.header.seq = key_frame.index_;

    pose_stamped.pose.position.x = key_frame.pose_(0, 3);
    pose_stamped.pose.position.y = key_frame.pose_(1, 3);
    pose_stamped.pose.position.z = key_frame.pose_(2, 3);

    Eigen::Quaternionf q = key_frame.GetQuaternion();
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();

    path.poses.push_back(pose_stamped);
  }

  publisher_.publish(path);
}
}  // namespace lidar_localization