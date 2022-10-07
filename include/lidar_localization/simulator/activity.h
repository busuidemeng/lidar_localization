/*
 * @Description: IMU measurement generation activity
 * @Author: Ge Yao
 * @Date: 2020-11-10 14:25:03
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-10-03 22:14:31
 */
#ifndef LIDAR_LOCALIZATION_SIMULATOR_ACTIVITY_H_
#define LIDAR_LOCALIZATION_SIMULATOR_ACTIVITY_H_

#include <math.h>
#include <random>
#include <string>

#include <glog/logging.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "lidar_localization/simulator/simulator_config.h"
#include "lidar_localization/simulator/simulator_constants.h"

namespace lidar_localization {

namespace generator {

class Activity {
 public:
  Activity();
  void Init(void);
  void Run(void);

 private:
  // get groud truth from motion equation:
  void GetGroundTruth();
  // random walk & measurement noise generation:
  void AddNoise(double delta_t);
  // convert to ROS messages:
  void SetIMUMessage();
  void SetOdometryMessage();
  // publish:
  void PublishMessages() const;

  // utilities:
  Eigen::Vector3d GetGaussianNoise(double stddev);
  static Eigen::Matrix3d EulerAnglesToRotation(
      const Eigen::Vector3d& euler_angles);
  static Eigen::Vector3d EulerAngleRatesToBodyAngleRates(
      const Eigen::Vector3d& euler_angles,
      const Eigen::Vector3d& euler_angle_rates);

  // node handler:
  ros::NodeHandle private_nh_;

  ros::Publisher pub_imu_;
  // TODO: separate odometry estimation from IMU device
  ros::Publisher pub_odom_;

  // config:
  IMUConfig imu_config_;
  OdomConfig odom_config_;

  // noise generator:
  std::default_random_engine normal_generator_;
  std::normal_distribution<double> normal_distribution_;

  // measurements:
  ros::Time timestamp_;
  // a. gravity constant:
  Eigen::Vector3d G_ = Eigen::Vector3d::Zero();
  // b. pose:
  Eigen::Matrix3d R_gt_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_gt_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_gt_ = Eigen::Vector3d::Zero();
  // c. angular velocity:
  Eigen::Vector3d angular_vel_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_vel_bias_ = Eigen::Vector3d::Zero();
  // d. linear acceleration:
  Eigen::Vector3d linear_acc_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_acc_bias_ = Eigen::Vector3d::Zero();
  // ROS IMU message:
  sensor_msgs::Imu message_imu_;
  nav_msgs::Odometry message_odom_;
};

}  // namespace generator

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_SIMULATOR_ACTIVITY_H_