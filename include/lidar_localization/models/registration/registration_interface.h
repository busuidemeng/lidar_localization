/*
 * @Description: 点云匹配模块的基类
 * @Author: Ren Qian
 * @Date: 2020-02-08 21:25:11
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-20 22:55:14
 */

#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_INTERFACE_H_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_INTERFACE_H_

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>

#include "lidar_localization/sensor_data/cloud_data.h"

namespace lidar_localization {
class RegistrationInterface {
 public:
  virtual ~RegistrationInterface() = default;

  virtual bool SetInputTarget(const CloudData::CloudTypePtr& input_target) = 0;
  virtual bool ScanMatch(const CloudData::CloudTypePtr& input_source,
                         const Eigen::Matrix4f& predict_pose,
                         CloudData::CloudTypePtr& result_cloud_ptr,
                         Eigen::Matrix4f& result_pose) = 0;
  virtual float GetFitnessScore() = 0;
};
}  // namespace lidar_localization

#endif