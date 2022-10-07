/*
 * @Description: Publish synced Lidar-IMU-GNSS measurement
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-22 23:16:44
 */
#ifndef LIDAR_LOCALIZATION_PUBLISHER_LIDAR_MEASUREMENT_PUBLISHER_H_
#define LIDAR_LOCALIZATION_PUBLISHER_LIDAR_MEASUREMENT_PUBLISHER_H_

#include <glog/logging.h>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "lidar_localization/LidarMeasurement.h"
#include "lidar_localization/sensor_data/cloud_data.h"
#include "lidar_localization/sensor_data/imu_data.h"
#include "lidar_localization/sensor_data/velocity_data.h"

namespace lidar_localization {

class LidarMeasurementPublisher {
 public:
  LidarMeasurementPublisher(const ros::NodeHandle& nh,
                            const std::string topic_name,
                            const std::string lidar_frame_id,
                            const std::string imu_frame_id,
                            const std::string gnss_odometry_pos_frame_id,
                            const std::string gnss_odometry_vel_frame_id,
                            const size_t buff_size);

  LidarMeasurementPublisher() = default;

  void Publish(const double time,
               const CloudData::CloudTypePtr& cloud_ptr_input,
               const IMUData& imu_data,
               const Eigen::Matrix4f& transform_matrix,
               const VelocityData& velocity_data);

  void Publish(const CloudData::CloudTypePtr& cloud_ptr_input,
               const IMUData& imu_data,
               const Eigen::Matrix4f& transform_matrix,
               const VelocityData& velocity_data);

  bool HasSubscribers() const { return publisher_.getNumSubscribers() != 0; }

 private:
  void SetPointCloud(const ros::Time& time,
                     const CloudData::CloudTypePtr& cloud_data_ptr,
                     sensor_msgs::PointCloud2& point_cloud) const;

  void SetIMU(const ros::Time& time,
              const IMUData& imu_data,
              sensor_msgs::Imu& imu) const;

  void SetGNSSOdometry(const ros::Time& time,
                       const Eigen::Matrix4f& transform_matrix,
                       const VelocityData& velocity_data,
                       nav_msgs::Odometry& gnss_odometry) const;

  void PublishData(const ros::Time& time,
                   const CloudData::CloudTypePtr& cloud_ptr_input,
                   const IMUData& imu_data,
                   const Eigen::Matrix4f& transform_matrix,
                   const VelocityData& velocity_data);

  ros::NodeHandle nh_;

  std::string lidar_frame_id_{};
  std::string imu_frame_id_{};
  std::string gnss_odometry_pos_frame_id_{};
  std::string gnss_odometry_vel_frame_id_{};

  LidarMeasurement lidar_measurement_;

  ros::Publisher publisher_;
};

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_PUBLISHER_LIDAR_MEASUREMENT_PUBLISHER_H_