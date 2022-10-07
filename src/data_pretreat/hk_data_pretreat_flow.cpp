/*
 * @Description: data preprocess for urbanNav dataset
 * @Author: ZiJieChen
 * @Date: 2022-10-03 14:28:23
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-10-03 19:10:35
 */
#include "lidar_localization/data_pretreat/hk_data_pretreat_flow.h"

namespace lidar_localization {
HKDataPretreatFlow::HKDataPretreatFlow(const ros::NodeHandle& nh,
                                       const std::string cloud_topic) {
  // subscribers:
  // a. velodyne measurement:
  cloud_sub_ptr_ =
      std::make_shared<CloudSubscriber>(nh, "/velodyne_points", 100000);
  // b. Xsens Mti 10 IMU:
  imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/imu/data", 1000000);
  // c. Novatel RTK:
  gnss_sub_ptr_ =
      std::make_shared<OdometrySubscriber>(nh, "/navsat/odom", 1000000);

  // publishers:
  cloud_pub_ptr_ =
      std::make_shared<CloudPublisher>(nh, cloud_topic, "/velo_link", 100);
  imu_pub_ptr_ =
      std::make_shared<IMUPublisher>(nh, "/synced_imu", "/imu_link", 100);
  pos_vel_pub_ptr_ = std::make_shared<PosVelPublisher>(
      nh, "/synced_pos_vel", "/map", "/imu_link", 100);
  gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(
      nh, "/synced_gnss", "/map", "/map", 100);

  // motion compensation for lidar measurement:
  distortion_adjust_ptr_ = std::make_shared<DistortionAdjust>();

  // loading config params
  const std::string config_file_path =
      WORK_SPACE_PATH + "/config/mapping/hk_data_pretreat.yaml";
  YAML::Node config_node = YAML::LoadFile(config_file_path);

  float array[16];
  for (size_t i = 0; i < 16; i++) {
    array[i] = config_node["T_imu_lidar"][i].as<float>();
  }
  T_imu_lidar_ = Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>>(array);

  is_mapping_ = config_node["is_mapping"].as<bool>();

  LOG(INFO) << std::endl
            << "-----------------Init Data Pretreat-------------------"
            << std::endl
            << "is_mapping: " << is_mapping_ << std::endl
            << "T_imu_lidar: " << std::endl
            << T_imu_lidar_ << std::endl;
}

bool HKDataPretreatFlow::Run() {
  if (!ReadData()) {
    return false;
  }

  while (HasData()) {
    if (!ValidData()) {
      continue;
    }

    TransformData();
    PublishData();
  }

  return true;
}

bool HKDataPretreatFlow::ReadData() {
  static std::deque<IMUData> unsynced_imu_;
  static std::deque<PoseData> unsynced_gnss_;

  // fetch lidar measurements from buffer:
  cloud_sub_ptr_->ParseData(cloud_data_buff_);
  imu_sub_ptr_->ParseData(unsynced_imu_);
  gnss_sub_ptr_->ParseData(unsynced_gnss_);

  if (cloud_data_buff_.size() == 0) {
    return false;
  }

  // use timestamp of lidar measurement as reference:
  const double cloud_time = cloud_data_buff_.front().time_;
  // sync IMU, velocity and GNSS with lidar measurement:
  // find the two closest measurement around lidar measurement time
  // then use linear interpolation to generate synced measurement:
  const bool valid_imu =
      IMUData::SyncData(cloud_time, unsynced_imu_, imu_data_buff_);
  const bool valid_gnss =
      PoseData::SyncData(cloud_time, unsynced_gnss_, gnss_data_buff_);

  if (!valid_imu || !valid_gnss) {
    if (unsynced_gnss_.front().time_ > cloud_time) {
      cloud_data_buff_.pop_front();
    }

    if (unsynced_imu_.front().time_ > cloud_time) {
      cloud_data_buff_.pop_front();
    }

    return false;
  }

  return true;
}

bool HKDataPretreatFlow::HasData() {
  if (cloud_data_buff_.size() == 0) {
    return false;
  }
  if (imu_data_buff_.size() == 0) {
    return false;
  }
  if (gnss_data_buff_.size() == 0) {
    return false;
  }
  return true;
}

bool HKDataPretreatFlow::ValidData() {
  current_cloud_data_ = cloud_data_buff_.front();
  current_imu_data_ = imu_data_buff_.front();
  current_gnss_data_ = gnss_data_buff_.front();

  const double diff_imu_time =
      current_cloud_data_.time_ - current_imu_data_.time_;
  const double diff_gnss_time =
      current_cloud_data_.time_ - current_gnss_data_.time_;
  //
  // this check assumes the frequency of lidar is 10Hz:
  //
  const double half_sampling_time = 0.05;
  if (diff_imu_time < -half_sampling_time ||
      diff_gnss_time < -half_sampling_time) {
    cloud_data_buff_.pop_front();
    return false;
  }

  if (diff_imu_time > half_sampling_time) {
    imu_data_buff_.pop_front();
    return false;
  }

  if (diff_gnss_time > half_sampling_time) {
    gnss_data_buff_.pop_front();
    return false;
  }

  cloud_data_buff_.pop_front();
  imu_data_buff_.pop_front();
  gnss_data_buff_.pop_front();

  return true;
}

bool HKDataPretreatFlow::TransformData() {
  static Eigen::Vector3f gnss_origin = Eigen::Vector3f::Zero();
  static bool gnss_init = false;

  // a. init gnss origin
  // to make sure the gnss origin is [0,0,0]
  if (!gnss_init) {
    gnss_origin = current_gnss_data_.pose_.block<3, 1>(0, 3);
    gnss_init = true;
  }
  current_gnss_data_.pose_.block<3, 1>(0, 3) =
      current_gnss_data_.pose_.block<3, 1>(0, 3) - gnss_origin;

  // b. motion compensation for lidar measurements:
  // convert the laser points to imu frame
  CloudData::CloudTypePtr temp_cloud(new CloudData::CloudType());
  pcl::transformPointCloud(
      *current_cloud_data_.cloud_ptr_, *temp_cloud, T_imu_lidar_.inverse());

  // convert the gnss velocity from ENU frame to imu frame
  Eigen::Vector3d v_b = (current_gnss_data_.pose_.block<3, 3>(0, 0).inverse() *
                         current_gnss_data_.vel_.v)
                            .cast<double>();

  current_velocity_data_.linear_velocity_.x = v_b.x();
  current_velocity_data_.linear_velocity_.y = v_b.y();
  current_velocity_data_.linear_velocity_.z = v_b.z();

  current_velocity_data_.angular_velocity_.x =
      current_imu_data_.angular_velocity_.x;
  current_velocity_data_.angular_velocity_.y =
      current_imu_data_.angular_velocity_.y;
  current_velocity_data_.angular_velocity_.z =
      current_imu_data_.angular_velocity_.z;

  distortion_adjust_ptr_->SetMotionInfo(current_velocity_data_, 0.1);
  CloudData::CloudTypePtr adjusted_cloud(new CloudData::CloudType());
  distortion_adjust_ptr_->AdjustCloud(temp_cloud, adjusted_cloud);
  *current_cloud_data_.cloud_ptr_ = *adjusted_cloud;

  return true;
}

bool HKDataPretreatFlow::PublishData() const {
  // take lidar measurement time as synced timestamp:
  const double timestamp_synced = current_cloud_data_.time_;

  cloud_pub_ptr_->Publish(current_cloud_data_.cloud_ptr_, timestamp_synced);

  imu_pub_ptr_->Publish(current_imu_data_, timestamp_synced);

  // a. gnss frame's in ENU
  // b. gnss frame's velocity in ENU
  gnss_pub_ptr_->Publish(
      current_gnss_data_.pose_, current_gnss_data_.vel_.v, timestamp_synced);

  return true;
}

}  // namespace lidar_localization