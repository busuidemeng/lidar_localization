/*
 * @Description: 在ros中发布点云
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:27:30
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-22 23:16:18
 */
#ifndef LIDAR_LOCALIZATION_PUBLISHER_CLOUD_PUBLISHER_H_
#define LIDAR_LOCALIZATION_PUBLISHER_CLOUD_PUBLISHER_H_

#include <glog/logging.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "lidar_localization/sensor_data/cloud_data.h"

namespace lidar_localization {
class CloudPublisher {
 public:
  CloudPublisher(const ros::NodeHandle& nh,
                 const std::string topic_name,
                 const std::string frame_id,
                 const size_t buff_size);

  CloudPublisher() = default;

  void Publish(const CloudData::CloudTypePtr& cloud_ptr_input,
               const double time) const;

  void Publish(const CloudData::CloudTypePtr& cloud_ptr_input) const;

  bool HasSubscribers() const { return publisher_.getNumSubscribers() != 0; }

 private:
  void PublishData(const CloudData::CloudTypePtr& cloud_ptr_input,
                   const ros::Time time) const;

  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  std::string frame_id_{};
};
}  // namespace lidar_localization
#endif