/*
 * @Description:
 * @Autor: ZiJieChen
 * @Date: 2022-09-29 15:21:17
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-10-03 22:05:33
 */
#ifndef LIDAR_LOCALIZATION_SIMULATOR_SIMULATOR_CONSTANTS_H_
#define LIDAR_LOCALIZATION_SIMULATOR_SIMULATOR_CONSTANTS_H_

#include <cmath>

#include <ros/ros.h>

namespace lidar_localization {

namespace generator {

constexpr double kRhoX = 30.0;  // 3.0
constexpr double kRhoY = 40.0;  // 4.0
constexpr double kRhoZ = 10.0;  // 1.0

constexpr double kOmegaXY = M_PI / 10.0;
constexpr double kOmegaZ = 10.0 * kOmegaXY;

constexpr double kYaw = M_PI / 10.0;
constexpr double kPitch = 0.20;
constexpr double kRoll = 0.10;

}  // namespace generator

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_SIMULATOR_SIMULATOR_CONSTANTS_H_