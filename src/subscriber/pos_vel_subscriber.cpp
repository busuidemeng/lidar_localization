/*
 * @Description: Subscribe to PosVel messages
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-28 22:04:15
 */
#include "lidar_localization/subscriber/pos_vel_subscriber.h"

namespace lidar_localization {

PosVelSubscriber::PosVelSubscriber(const ros::NodeHandle& nh,
                                   const std::string topic_name,
                                   const size_t buff_size)
    : nh_(nh) {
  subscriber_ = nh_.subscribe(topic_name,
                              buff_size,
                              &PosVelSubscriber::MsgCallback,
                              this,
                              ros::TransportHints().tcpNoDelay());
}

void PosVelSubscriber::MsgCallback(const PosVelConstPtr& pos_vel_msg_ptr) {
  buff_mutex_.lock();

  PosVelData pos_vel_data;
  pos_vel_data.time_ = pos_vel_msg_ptr->header.stamp.toSec();

  // a. set the position:
  pos_vel_data.pos_.x() = pos_vel_msg_ptr->position.x;
  pos_vel_data.pos_.y() = pos_vel_msg_ptr->position.y;
  pos_vel_data.pos_.z() = pos_vel_msg_ptr->position.z;

  // b. set the body frame velocity:
  pos_vel_data.vel_.x() = pos_vel_msg_ptr->velocity.x;
  pos_vel_data.vel_.y() = pos_vel_msg_ptr->velocity.y;
  pos_vel_data.vel_.z() = pos_vel_msg_ptr->velocity.z;

  new_pos_vel_data_.push_back(pos_vel_data);

  buff_mutex_.unlock();
}

void PosVelSubscriber::ParseData(std::deque<PosVelData>& pos_vel_data_buff) {
  buff_mutex_.lock();

  if (new_pos_vel_data_.size() > 0) {
    pos_vel_data_buff.insert(pos_vel_data_buff.end(),
                             new_pos_vel_data_.begin(),
                             new_pos_vel_data_.end());
    new_pos_vel_data_.clear();
  }

  buff_mutex_.unlock();
}

}  // namespace lidar_localization