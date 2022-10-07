/*
 * @Description: IMU integration configs
 * @Author: Ge Yao
 * @Date: 2020-11-10 14:25:03
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-10-03 22:08:28
 */
#ifndef LIDAR_LOCALIZATION_SIMULATOR_SIMULATOR_CONFIG_H_
#define LIDAR_LOCALIZATION_SIMULATOR_SIMULATOR_CONFIG_H_

#include <string>

namespace lidar_localization {

struct IMUConfig {
  // general info:
  std::string device_name{};
  std::string frame_id{};
  std::string topic_name{};

  // gravity constant:
  struct {
    double x{0.0};
    double y{0.0};
    double z{0.0};
  } gravity;

  // bias:
  struct {
    struct {
      double x{0.0};
      double y{0.0};
      double z{0.0};
    } angular_velocity;
    struct {
      double x{0.0};
      double y{0.0};
      double z{0.0};
    } linear_acceleration;
  } bias;

  // angular velocity noises:
  double gyro_bias_stddev{0.0};
  double gyro_noise_stddev{0.0};

  // linear acceleration noises:
  double acc_bias_stddev{0.0};
  double acc_noise_stddev{0.0};
};

struct OdomConfig {
  // general info:
  std::string frame_id;
  struct {
    std::string ground_truth{};
    std::string estimation{};
  } topic_name;
};

}  // namespace lidar_localization

#endif