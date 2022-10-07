/*
 * @Description: 订阅 loop pose 数据
 * @Author: Ren Qian
 * @Date: 2019-06-14 16:44:18
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-28 22:04:10
 */
#include "lidar_localization/subscriber/loop_pose_subscriber.h"

namespace lidar_localization {
LoopPoseSubscriber::LoopPoseSubscriber(const ros::NodeHandle& nh,
                                       const std::string topic_name,
                                       const size_t buff_size)
    : nh_(nh) {
  subscriber_ = nh_.subscribe(topic_name,
                              buff_size,
                              &LoopPoseSubscriber::MsgCallback,
                              this,
                              ros::TransportHints().tcpNoDelay());
}

void LoopPoseSubscriber::MsgCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& loop_pose_msg_ptr) {
  buff_mutex_.lock();
  LoopPose loop_pose;
  loop_pose.time_ = loop_pose_msg_ptr->header.stamp.toSec();
  loop_pose.index0_ = (unsigned int)loop_pose_msg_ptr->pose.covariance[0];
  loop_pose.index1_ = (unsigned int)loop_pose_msg_ptr->pose.covariance[1];

  loop_pose.pose_(0, 3) = loop_pose_msg_ptr->pose.pose.position.x;
  loop_pose.pose_(1, 3) = loop_pose_msg_ptr->pose.pose.position.y;
  loop_pose.pose_(2, 3) = loop_pose_msg_ptr->pose.pose.position.z;

  Eigen::Quaternionf q;
  q.x() = loop_pose_msg_ptr->pose.pose.orientation.x;
  q.y() = loop_pose_msg_ptr->pose.pose.orientation.y;
  q.z() = loop_pose_msg_ptr->pose.pose.orientation.z;
  q.w() = loop_pose_msg_ptr->pose.pose.orientation.w;
  loop_pose.pose_.block<3, 3>(0, 0) = q.matrix();

  new_loop_pose_.push_back(loop_pose);
  buff_mutex_.unlock();
}

void LoopPoseSubscriber::ParseData(std::deque<LoopPose>& loop_pose_buff) {
  buff_mutex_.lock();
  if (new_loop_pose_.size() > 0) {
    loop_pose_buff.insert(
        loop_pose_buff.end(), new_loop_pose_.begin(), new_loop_pose_.end());
    new_loop_pose_.clear();
  }
  buff_mutex_.unlock();
}
}  // namespace lidar_localization