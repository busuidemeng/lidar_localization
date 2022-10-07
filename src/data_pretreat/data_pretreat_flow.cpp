/*
 * @Description: 数据预处理模块，包括时间同步、点云去畸变等
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:38:42
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-30 10:33:37
 */
#include "lidar_localization/data_pretreat/data_pretreat_flow.h"

namespace lidar_localization {
DataPretreatFlow::DataPretreatFlow(const ros::NodeHandle& nh,
                                   const std::string cloud_topic) {
  // subscribers:
  // a. velodyne measurement:
  cloud_sub_ptr_ =
      std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);
  // b. OXTS IMU:
  imu_sub_ptr_ =
      std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
  // c. OXTS velocity:
  velocity_sub_ptr_ =
      std::make_shared<VelocitySubscriber>(nh, "/kitti/oxts/gps/vel", 1000000);
  // d. OXTS GNSS:
  gnss_sub_ptr_ =
      std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);
  lidar_to_imu_ptr_ =
      std::make_shared<TFListener>(nh, "/imu_link", "/velo_link");

  // publishers:
  cloud_pub_ptr_ =
      std::make_shared<CloudPublisher>(nh, cloud_topic, "/velo_link", 100);
  imu_pub_ptr_ =
      std::make_shared<IMUPublisher>(nh, "/synced_imu", "/imu_link", 100);
  pos_vel_pub_ptr_ = std::make_shared<PosVelPublisher>(
      nh, "/synced_pos_vel", "/map", "/imu_link", 100);
  gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(
      nh, "/synced_gnss", "/map", "/map", 100);

  is_mapping_ = nh.param<bool>("mapping", true);

  // motion compensation for lidar measurement:
  distortion_adjust_ptr_ = std::make_shared<DistortionAdjust>();
}

bool DataPretreatFlow::Run() {
  if (!ReadData()) {
    return false;
  }

  if (!InitCalibration()) {
    return false;
  }

  if (!InitGNSS()) {
    return false;
  }

  while (HasData()) {
    if (!ValidData())
      continue;

    TransformData();
    PublishData();
  }

  return true;
}

bool DataPretreatFlow::ReadData() {
  static std::deque<IMUData> unsynced_imu_;
  static std::deque<VelocityData> unsynced_velocity_;
  static std::deque<GNSSData> unsynced_gnss_;

  // fetch lidar measurements from buffer:
  cloud_sub_ptr_->ParseData(cloud_data_buff_);
  imu_sub_ptr_->ParseData(unsynced_imu_);
  velocity_sub_ptr_->ParseData(unsynced_velocity_);
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
  const bool valid_velocity = VelocityData::SyncData(
      cloud_time, unsynced_velocity_, velocity_data_buff_);
  const bool valid_gnss =
      GNSSData::SyncData(cloud_time, unsynced_gnss_, gnss_data_buff_);

  // only mark lidar as 'inited' when all the three sensors are synced:
  static bool sensor_inited = false;
  if (!sensor_inited) {
    if (!valid_imu || !valid_velocity || !valid_gnss) {
      cloud_data_buff_.pop_front();
      return false;
    }
    sensor_inited = true;
  }

  return true;
}

bool DataPretreatFlow::InitCalibration() {
  // lookup imu pose in lidar frame:
  static bool calibration_received = false;
  if (!calibration_received) {
    if (lidar_to_imu_ptr_->LookupData(lidar_to_imu_)) {
      calibration_received = true;
    }
  }

  return calibration_received;
}

bool DataPretreatFlow::InitGNSS() {
  static bool gnss_inited = false;

  if (!gnss_inited) {
    // system is mapping mode
    if (is_mapping_) {
      gnss_init_data_ = gnss_data_buff_.front();
      gnss_init_data_.InitOriginPosition();
      gnss_inited = true;
      LOG(INFO) << "System is mapping mode, gnss origin inited." << std::endl;
    }
    // system is localization mode
    else {
      const std::string gnss_origin_path =
          WORK_SPACE_PATH + "/slam_data/map/gnss_origin.txt";
      if (!FileManager::IsValidDirectory(gnss_origin_path)) {
        LOG(ERROR)
            << "System is localization mode, can not find gnss origin point."
            << std::endl;
        return false;
      }

      std::ifstream ifs(gnss_origin_path);

      double lon = 0.0;
      double lat = 0.0;
      double alt = 0.0;

      ifs >> lat;
      ifs >> lon;
      ifs >> alt;

      gnss_init_data_.latitude_ = lat;
      gnss_init_data_.longitude_ = lon;
      gnss_init_data_.altitude_ = alt;
      gnss_init_data_.InitOriginPosition();

      gnss_inited = true;
    }
  }

  return gnss_inited;
}

bool DataPretreatFlow::SaveGnssOrigin() {
  const std::string gnss_origin_path =
      WORK_SPACE_PATH + "/slam_data/map/gnss_origin.txt";

  std::ofstream ofs(gnss_origin_path);

  ofs << std::setprecision(12) << gnss_init_data_.latitude_ << " "
      << gnss_init_data_.longitude_ << " " << gnss_init_data_.altitude_;
  ofs.close();

  LOG(INFO) << "save gnss_origin in: " << gnss_origin_path << std::endl
            << " lat: " << gnss_init_data_.latitude_
            << " lon: " << gnss_init_data_.longitude_
            << " alt: " << gnss_init_data_.altitude_ << std::endl;
  return true;
}

bool DataPretreatFlow::HasData() {
  if (cloud_data_buff_.size() == 0) {
    return false;
  }
  if (imu_data_buff_.size() == 0) {
    return false;
  }
  if (velocity_data_buff_.size() == 0) {
    return false;
  }
  if (gnss_data_buff_.size() == 0) {
    return false;
  }
  return true;
}

bool DataPretreatFlow::ValidData() {
  current_cloud_data_ = cloud_data_buff_.front();
  current_imu_data_ = imu_data_buff_.front();
  current_velocity_data_ = velocity_data_buff_.front();
  current_gnss_data_ = gnss_data_buff_.front();

  const double diff_imu_time =
      current_cloud_data_.time_ - current_imu_data_.time_;
  const double diff_velocity_time =
      current_cloud_data_.time_ - current_velocity_data_.time_;
  const double diff_gnss_time =
      current_cloud_data_.time_ - current_gnss_data_.time_;
  //
  // this check assumes the frequency of lidar is 10Hz:
  //
  const double half_sampling_time = 0.05;
  if (diff_imu_time < -half_sampling_time ||
      diff_velocity_time < -half_sampling_time ||
      diff_gnss_time < -half_sampling_time) {
    cloud_data_buff_.pop_front();
    return false;
  }

  if (diff_imu_time > half_sampling_time) {
    imu_data_buff_.pop_front();
    return false;
  }

  if (diff_velocity_time > half_sampling_time) {
    velocity_data_buff_.pop_front();
    return false;
  }

  if (diff_gnss_time > half_sampling_time) {
    gnss_data_buff_.pop_front();
    return false;
  }

  cloud_data_buff_.pop_front();
  imu_data_buff_.pop_front();
  velocity_data_buff_.pop_front();
  gnss_data_buff_.pop_front();

  return true;
}

bool DataPretreatFlow::TransformData() {
  // a. get reference pose:
  gnss_pose_ = Eigen::Matrix4f::Identity();
  // get position from GNSS
  current_gnss_data_.UpdateXYZ();
  gnss_pose_(0, 3) = current_gnss_data_.local_E_;
  gnss_pose_(1, 3) = current_gnss_data_.local_N_;
  gnss_pose_(2, 3) = current_gnss_data_.local_U_;
  // get orientation from IMU:
  gnss_pose_.block<3, 3>(0, 0) = current_imu_data_.GetOrientationMatrix();
  // this is lidar pose in GNSS/map frame:
  gnss_pose_ *= lidar_to_imu_;

  // b. set synced pos vel
  pos_vel_.pos_.x() = current_gnss_data_.local_E_;
  pos_vel_.pos_.y() = current_gnss_data_.local_N_;
  pos_vel_.pos_.z() = current_gnss_data_.local_U_;

  pos_vel_.vel_.x() = current_velocity_data_.linear_velocity_.x;
  pos_vel_.vel_.y() = current_velocity_data_.linear_velocity_.y;
  pos_vel_.vel_.z() = current_velocity_data_.linear_velocity_.z;

  // NOTE: disable motion compensation since Kitti dataset has been compensated
  // // c. motion compensation for lidar measurements:
  current_velocity_data_.TransformCoordinate(lidar_to_imu_);
  // distortion_adjust_ptr_->SetMotionInfo(0.1, current_velocity_data_);
  // distortion_adjust_ptr_->AdjustCloud(current_cloud_data_.cloud_ptr_,
  // current_cloud_data_.cloud_ptr_);

  return true;
}

bool DataPretreatFlow::PublishData() const {
  // take lidar measurement time as synced timestamp:
  const double timestamp_synced = current_cloud_data_.time_;

  cloud_pub_ptr_->Publish(current_cloud_data_.cloud_ptr_, timestamp_synced);
  imu_pub_ptr_->Publish(current_imu_data_, timestamp_synced);

  pos_vel_pub_ptr_->Publish(pos_vel_, timestamp_synced);

  //
  // this synced odometry has the following info:
  //
  // a. lidar frame's pose in map
  // b. lidar frame's velocity
  gnss_pub_ptr_->Publish(gnss_pose_, current_velocity_data_, timestamp_synced);

  return true;
}

}  // namespace lidar_localization