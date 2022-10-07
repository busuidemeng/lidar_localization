/*
 * @Description:
 * @Author: Ren Qian
 * @Date: 2019-03-31 13:10:51
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-28 22:03:39
 */
#include "lidar_localization/subscriber/gnss_subscriber.h"

namespace lidar_localization {
GNSSSubscriber::GNSSSubscriber(const ros::NodeHandle& nh,
                               const std::string topic_name,
                               const size_t buff_size)
    : nh_(nh) {
  subscriber_ = nh_.subscribe(topic_name,
                              buff_size,
                              &GNSSSubscriber::MsgCallback,
                              this,
                              ros::TransportHints().tcpNoDelay());
}

void GNSSSubscriber::MsgCallback(
    const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr) {
  buff_mutex_.lock();

  // convert ROS NavSatFix to GeographicLib compatible GNSS message:
  GNSSData gnss_data;
  gnss_data.time_ = nav_sat_fix_ptr->header.stamp.toSec();
  gnss_data.latitude_ = nav_sat_fix_ptr->latitude;
  gnss_data.longitude_ = nav_sat_fix_ptr->longitude;
  gnss_data.altitude_ = nav_sat_fix_ptr->altitude;
  gnss_data.status_ = nav_sat_fix_ptr->status.status;
  gnss_data.service_ = nav_sat_fix_ptr->status.service;
  // add new message to buffer:
  new_gnss_data_.push_back(gnss_data);

  buff_mutex_.unlock();
}

void GNSSSubscriber::ParseData(std::deque<GNSSData>& gnss_data_buff) {
  buff_mutex_.lock();

  // pipe all available measurements to output buffer:
  if (new_gnss_data_.size() > 0) {
    gnss_data_buff.insert(
        gnss_data_buff.end(), new_gnss_data_.begin(), new_gnss_data_.end());
    new_gnss_data_.clear();
  }

  buff_mutex_.unlock();
}
}  // namespace lidar_localization