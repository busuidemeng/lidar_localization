/*
 * @Description: synced PosVelData publisher
 * @Author: Ge Yao
 * @Date: 2020-11-21 15:39:24
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-22 23:17:01
 */

#ifndef LIDAR_LOCALIZATION_PUBLISHER_POS_VEL_PUBLISHER_H_
#define LIDAR_LOCALIZATION_PUBLISHER_POS_VEL_PUBLISHER_H_

#include <ros/ros.h>

#include <Eigen/Dense>
#include <string>

#include "lidar_localization/PosVel.h"
#include "lidar_localization/sensor_data/pos_vel_data.h"

namespace lidar_localization {

class PosVelPublisher {
 public:
  PosVelPublisher(const ros::NodeHandle& nh,
                  const std::string topic_name,
                  const std::string base_frame_id,
                  const std::string child_frame_id,
                  const int buff_size);

  PosVelPublisher() = default;

  void Publish(const PosVelData& pos_vel_data, const double& time);

  void Publish(const PosVelData& pos_vel_data);

  bool HasSubscribers() const { return publisher_.getNumSubscribers() != 0; }

 private:
  void PublishData(const PosVelData& pos_vel_data, const ros::Time time);

  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  PosVel pos_vel_msg_;
};

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_PUBLISHER_POS_VEL_PUBLISHER_H_