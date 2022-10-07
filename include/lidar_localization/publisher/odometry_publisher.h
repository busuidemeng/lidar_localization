/*
 * @Description: odometry 信息发布
 * @Author: Ren Qian
 * @Date: 2020-02-06 21:05:47
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-22 23:16:56
 */
#ifndef LIDAR_LOCALIZATION_PUBLISHER_ODOMETRY_PUBLISHER_H_
#define LIDAR_LOCALIZATION_PUBLISHER_ODOMETRY_PUBLISHER_H_

#include <string>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <Eigen/Dense>

#include "lidar_localization/sensor_data/velocity_data.h"

namespace lidar_localization {
class OdometryPublisher {
 public:
  OdometryPublisher(const ros::NodeHandle& nh,
                    const std::string topic_name,
                    const std::string base_frame_id,
                    const std::string child_frame_id,
                    const int buff_size);

  OdometryPublisher() = default;

  void Publish(const Eigen::Matrix4f& transform_matrix, const double time);

  void Publish(const Eigen::Matrix4f& transform_matrix);

  void Publish(const Eigen::Matrix4f& transform_matrix,
               const VelocityData& velocity_data,
               const double time);

  void Publish(const Eigen::Matrix4f& transform_matrix,
               const VelocityData& velocity_data);

  void Publish(const Eigen::Matrix4f& transform_matrix,
               const Eigen::Vector3f& vel,
               const double time);

  void Publish(const Eigen::Matrix4f& transform_matrix,
               const Eigen::Vector3f& vel);

  bool HasSubscribers() const { return publisher_.getNumSubscribers() != 0; }

 private:
  void PublishData(const Eigen::Matrix4f& transform_matrix,
                   const VelocityData& velocity_data,
                   const ros::Time time);

  ros::NodeHandle nh_;
  ros::Publisher publisher_;

  VelocityData velocity_data_;
  nav_msgs::Odometry odometry_;
};
}  // namespace lidar_localization
#endif