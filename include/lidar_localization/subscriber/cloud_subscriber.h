/*
 * @Description: 订阅激光点云信息，并解析数据
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:27:30
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-21 22:53:09
 */

#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_CLOUD_SUBSCRIBER_H_
#define LIDAR_LOCALIZATION_SUBSCRIBER_CLOUD_SUBSCRIBER_H_

#include <deque>
#include <mutex>
#include <thread>

#include <glog/logging.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "lidar_localization/sensor_data/cloud_data.h"

namespace lidar_localization {
class CloudSubscriber {
 public:
  CloudSubscriber(const ros::NodeHandle& nh,
                  const std::string topic_name,
                  const size_t buff_size);

  CloudSubscriber() = default;

  void ParseData(std::deque<CloudData>& deque_cloud_data);

 private:
  void MsgCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);

  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  std::deque<CloudData> new_cloud_data_;

  std::mutex buff_mutex_;
};
}  // namespace lidar_localization

#endif