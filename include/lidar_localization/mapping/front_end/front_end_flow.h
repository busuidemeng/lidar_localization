/*
 * @Description: front end 任务管理， 放在类里使代码更清晰
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:31:22
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-25 21:56:18
 */
#ifndef LIDAR_LOCALIZATION_MAPPING_FRONT_END_FRONT_END_FLOW_H_
#define LIDAR_LOCALIZATION_MAPPING_FRONT_END_FRONT_END_FLOW_H_

#include <glog/logging.h>

#include <ros/ros.h>

// publisher
#include "lidar_localization/publisher/odometry_publisher.h"
// subscriber
#include "lidar_localization/subscriber/cloud_subscriber.h"
// mapping
#include "lidar_localization/mapping/front_end/front_end.h"
// global
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {
class FrontEndFlow {
 public:
  FrontEndFlow(ros::NodeHandle& nh,
               const std::string& cloud_topic,
               const std::string& odom_topic);

  bool Run();

 private:
  bool ReadData();

  bool HasData() const { return cloud_data_buff_.size() > 0; }

  bool ValidData();
  bool UpdateLaserOdometry();
  bool PublishData();

  std::shared_ptr<CloudSubscriber> cloud_sub_ptr_{};
  std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_{};
  std::shared_ptr<FrontEnd> front_end_ptr_{};

  std::deque<CloudData> cloud_data_buff_;

  CloudData current_cloud_data_;

  Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();
};
}  // namespace lidar_localization

#endif
