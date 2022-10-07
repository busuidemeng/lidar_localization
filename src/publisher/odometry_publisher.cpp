/*
 * @Description: 里程计信息发布
 * @Author: Ren Qian
 * @Date: 2020-02-06 21:11:44
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-22 10:11:59
 */
#include "lidar_localization/publisher/odometry_publisher.h"

namespace lidar_localization {
OdometryPublisher::OdometryPublisher(const ros::NodeHandle& nh,
                                     const std::string topic_name,
                                     const std::string base_frame_id,
                                     const std::string child_frame_id,
                                     const int buff_size)
    : nh_(nh) {
  publisher_ = nh_.advertise<nav_msgs::Odometry>(topic_name, buff_size);
  odometry_.header.frame_id = base_frame_id;
  odometry_.child_frame_id = child_frame_id;
}

void OdometryPublisher::Publish(const Eigen::Matrix4f& transform_matrix,
                                const double time) {
  const ros::Time ros_time(time);
  PublishData(transform_matrix, velocity_data_, ros_time);
}

void OdometryPublisher::Publish(const Eigen::Matrix4f& transform_matrix) {
  PublishData(transform_matrix, velocity_data_, ros::Time::now());
}

void OdometryPublisher::Publish(const Eigen::Matrix4f& transform_matrix,
                                const VelocityData& velocity_data,
                                const double time) {
  const ros::Time ros_time(time);
  PublishData(transform_matrix, velocity_data, ros_time);
}

void OdometryPublisher::Publish(const Eigen::Matrix4f& transform_matrix,
                                const VelocityData& velocity_data) {
  PublishData(transform_matrix, velocity_data, ros::Time::now());
}

void OdometryPublisher::Publish(const Eigen::Matrix4f& transform_matrix,
                                const Eigen::Vector3f& vel,
                                const double time) {
  const ros::Time ros_time(time);

  velocity_data_.linear_velocity_.x = vel.x();
  velocity_data_.linear_velocity_.y = vel.y();
  velocity_data_.linear_velocity_.z = vel.z();

  PublishData(transform_matrix, velocity_data_, ros_time);

  velocity_data_.linear_velocity_.x = velocity_data_.linear_velocity_.y =
      velocity_data_.linear_velocity_.z = 0.0;
}

void OdometryPublisher::Publish(const Eigen::Matrix4f& transform_matrix,
                                const Eigen::Vector3f& vel) {
  velocity_data_.linear_velocity_.x = vel.x();
  velocity_data_.linear_velocity_.y = vel.y();
  velocity_data_.linear_velocity_.z = vel.z();

  PublishData(transform_matrix, velocity_data_, ros::Time::now());

  velocity_data_.linear_velocity_.x = velocity_data_.linear_velocity_.y =
      velocity_data_.linear_velocity_.z = 0.0;
}

void OdometryPublisher::PublishData(const Eigen::Matrix4f& transform_matrix,
                                    const VelocityData& velocity_data,
                                    const ros::Time time) {
  odometry_.header.stamp = time;

  // set the pose
  odometry_.pose.pose.position.x = transform_matrix(0, 3);
  odometry_.pose.pose.position.y = transform_matrix(1, 3);
  odometry_.pose.pose.position.z = transform_matrix(2, 3);

  Eigen::Quaternionf q;
  q = transform_matrix.block<3, 3>(0, 0);
  odometry_.pose.pose.orientation.x = q.x();
  odometry_.pose.pose.orientation.y = q.y();
  odometry_.pose.pose.orientation.z = q.z();
  odometry_.pose.pose.orientation.w = q.w();

  // set the twist:
  // a. linear:
  odometry_.twist.twist.linear.x = velocity_data.linear_velocity_.x;
  odometry_.twist.twist.linear.y = velocity_data.linear_velocity_.y;
  odometry_.twist.twist.linear.z = velocity_data.linear_velocity_.z;
  // b. angular:
  odometry_.twist.twist.angular.x = velocity_data.angular_velocity_.x;
  odometry_.twist.twist.angular.y = velocity_data.angular_velocity_.y;
  odometry_.twist.twist.angular.z = velocity_data.angular_velocity_.z;

  publisher_.publish(odometry_);
}
}  // namespace lidar_localization