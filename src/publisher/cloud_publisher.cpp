/*
 * @Description: 通过ros发布点云
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:27:30
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-22 09:44:37
 */

#include "lidar_localization/publisher/cloud_publisher.h"

namespace lidar_localization {
CloudPublisher::CloudPublisher(const ros::NodeHandle& nh,
                               const std::string topic_name,
                               const std::string frame_id,
                               const size_t buff_size)
    : nh_(nh)
    , frame_id_(frame_id) {
  publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, buff_size);
}

void CloudPublisher::Publish(const CloudData::CloudTypePtr& cloud_ptr_input,
                             const double time) const {
  const ros::Time ros_time(time);
  PublishData(cloud_ptr_input, ros_time);
}

void CloudPublisher::Publish(
    const CloudData::CloudTypePtr& cloud_ptr_input) const {
  const ros::Time time = ros::Time::now();
  PublishData(cloud_ptr_input, time);
}

void CloudPublisher::PublishData(const CloudData::CloudTypePtr& cloud_ptr_input,
                                 const ros::Time time) const {
  sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
  pcl::toROSMsg(*cloud_ptr_input, *cloud_ptr_output);

  cloud_ptr_output->header.stamp = time;
  cloud_ptr_output->header.frame_id = frame_id_;
  publisher_.publish(*cloud_ptr_output);
}
}  // namespace lidar_localization