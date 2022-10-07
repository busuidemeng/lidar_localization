/*
 * @Description: synced PosVelData publisher
 * @Author: Ge Yao
 * @Date: 2020-11-21 15:39:24
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-22 10:21:28
 */
#include "lidar_localization/publisher/pos_vel_publisher.h"

namespace lidar_localization {

PosVelPublisher::PosVelPublisher(const ros::NodeHandle& nh,
                                 const std::string topic_name,
                                 const std::string base_frame_id,
                                 const std::string child_frame_id,
                                 const int buff_size)
    : nh_(nh) {
  publisher_ = nh_.advertise<PosVel>(topic_name, buff_size);
  pos_vel_msg_.header.frame_id = base_frame_id;
  pos_vel_msg_.child_frame_id = child_frame_id;
}

void PosVelPublisher::Publish(const PosVelData& pos_vel_data,
                              const double& time) {
  const ros::Time ros_time(time);
  PublishData(pos_vel_data, ros_time);
}

void PosVelPublisher::Publish(const PosVelData& pos_vel_data) {
  PublishData(pos_vel_data, ros::Time::now());
}

void PosVelPublisher::PublishData(const PosVelData& pos_vel_data,
                                  const ros::Time time) {
  pos_vel_msg_.header.stamp = time;

  // a. set position
  pos_vel_msg_.position.x = pos_vel_data.pos_.x();
  pos_vel_msg_.position.y = pos_vel_data.pos_.y();
  pos_vel_msg_.position.z = pos_vel_data.pos_.z();

  // b. set velocity:
  pos_vel_msg_.velocity.x = pos_vel_data.vel_.x();
  pos_vel_msg_.velocity.y = pos_vel_data.vel_.y();
  pos_vel_msg_.velocity.z = pos_vel_data.vel_.z();

  publisher_.publish(pos_vel_msg_);
}

}  // namespace lidar_localization