/*
 * @Description: 订阅激光点云信息，并解析数据
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:27:30
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-28 22:03:51
 */
#include "lidar_localization/subscriber/cloud_subscriber.h"

namespace lidar_localization {
CloudSubscriber::CloudSubscriber(const ros::NodeHandle& nh,
                                 const std::string topic_name,
                                 const size_t buff_size)
    : nh_(nh) {
  subscriber_ = nh_.subscribe(topic_name,
                              buff_size,
                              &CloudSubscriber::MsgCallback,
                              this,
                              ros::TransportHints().tcpNoDelay());
}

void CloudSubscriber::MsgCallback(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr) {
  buff_mutex_.lock();

  // convert ROS PointCloud2 to pcl::PointCloud<pcl::PointXYZ>:
  CloudData cloud_data;
  cloud_data.time_ = cloud_msg_ptr->header.stamp.toSec();
  pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.cloud_ptr_));
  // add new message to buffer:
  new_cloud_data_.push_back(cloud_data);

  buff_mutex_.unlock();
}

void CloudSubscriber::ParseData(std::deque<CloudData>& cloud_data_buff) {
  buff_mutex_.lock();

  // pipe all available measurements to output buffer:
  if (new_cloud_data_.size() > 0) {
    cloud_data_buff.insert(
        cloud_data_buff.end(), new_cloud_data_.begin(), new_cloud_data_.end());
    new_cloud_data_.clear();
  }

  buff_mutex_.unlock();
}
}  // namespace lidar_localization