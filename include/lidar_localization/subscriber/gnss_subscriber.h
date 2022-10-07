/*
 * @Description:
 * @Author: Ren Qian
 * @Date: 2019-03-31 12:58:10
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-21 22:58:35
 */
#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_GNSS_SUBSCRIBER_H_
#define LIDAR_LOCALIZATION_SUBSCRIBER_GNSS_SUBSCRIBER_H_

#include <deque>
#include <mutex>
#include <thread>

#include <glog/logging.h>

#include <ros/ros.h>
#include "sensor_msgs/NavSatFix.h"

#include "lidar_localization/sensor_data/gnss_data.h"

namespace lidar_localization {
class GNSSSubscriber {
 public:
  GNSSSubscriber(const ros::NodeHandle& nh,
                 const std::string topic_name,
                 const size_t buff_size);

  GNSSSubscriber() = default;

  void ParseData(std::deque<GNSSData>& deque_gnss_data);

 private:
  void MsgCallback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr);

  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  std::deque<GNSSData> new_gnss_data_;

  std::mutex buff_mutex_;
};
}  // namespace lidar_localization
#endif