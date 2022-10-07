/*
 * @Description: LIO mapping backend, interface
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-28 16:25:11
 */
#ifndef LIDAR_LOCALIZATION_MAPPING_BACK_END_LIO_BACK_END_SIM_H_
#define LIDAR_LOCALIZATION_MAPPING_BACK_END_LIO_BACK_END_SIM_H_

#include <deque>
#include <fstream>
#include <string>

#include <glog/logging.h>

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>

#include <pcl/io/pcd_io.h>

// sensor data
#include "lidar_localization/sensor_data/cloud_data.h"
#include "lidar_localization/sensor_data/imu_data.h"
#include "lidar_localization/sensor_data/key_frame.h"
#include "lidar_localization/sensor_data/loop_pose.h"
#include "lidar_localization/sensor_data/pose_data.h"
#include "lidar_localization/sensor_data/velocity_data.h"
// models
#include "lidar_localization/models/graph_optimizer/g2o/g2o_graph_optimizer.h"
#include "lidar_localization/models/pre_integrator/imu_pre_integrator.h"
#include "lidar_localization/models/pre_integrator/odo_pre_integrator.h"
// tools
#include "lidar_localization/tools/file_manager.h"
// global
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {

class LIOBackEndSim {
 public:
  LIOBackEndSim();

  bool InsertLoopPose(const LoopPose& loop_pose);
  bool UpdateIMUPreIntegration(const IMUData& imu_data);
  bool UpdateOdoPreIntegration(const VelocityData& velocity_data);

  bool Update(const PoseData& laser_odom,
              const PoseData& gnss_pose,
              const IMUData& imu_data);

  bool ForceOptimize();
  bool SaveOptimizedPose();

  bool has_new_key_frame() const { return has_new_key_frame_; }
  bool has_new_optimized() const { return has_new_optimized_; }

  void GetOptimizedKeyFrames(std::deque<KeyFrame>& key_frames_deque);
  void GetLatestKeyScan(CloudData& key_scan);

  void GetLatestKeyFrame(KeyFrame& key_frame) const {
    key_frame = current_key_frame_;
  }

  void GetLatestKeyGNSS(KeyFrame& key_frame) const {
    key_frame = current_key_gnss_;
  }

 private:
  bool InitWithConfig();

  bool InitDataPath(const YAML::Node& config_node);
  bool InitParam(const YAML::Node& config_node);
  bool InitGraphOptimizer(const YAML::Node& config_node);
  bool InitIMUPreIntegrator(const YAML::Node& config_node);
  bool InitOdoPreIntegrator(const YAML::Node& config_node);

  void ResetParam();
  static bool SavePose(const Eigen::Matrix4f& pose, std::ofstream& ofs);

  bool MaybeNewKeyFrame(const PoseData& laser_odom,
                        const PoseData& gnss_pose,
                        const IMUData& imu_data);

  bool AddNodeAndEdge(const PoseData& gnss_data);

  bool MaybeOptimized();
  void ShowIMUPreIntegrationResidual(
      const PoseData& last_gnss_pose,
      const PoseData& curr_gnss_pose,
      const IMUPreIntegrator::IMUPreIntegration& imu_pre_integration);

  std::string key_frames_path_{};
  std::string scan_context_path_{};
  std::string trajectory_path_{};

  std::ofstream ground_truth_ofs_;
  std::ofstream laser_odom_ofs_;
  std::ofstream optimized_pose_ofs_;
  std::ofstream pure_integration_ofs_;

  float key_frame_distance_{2.0};  // m

  bool has_new_key_frame_{false};
  bool has_new_optimized_{false};

  CloudData current_key_scan_;
  KeyFrame current_key_frame_;
  KeyFrame current_key_gnss_;
  std::deque<KeyFrame> key_frames_deque_;
  std::deque<KeyFrame> key_gnss_deque_;
  std::deque<KeyFrame> optimized_key_frames_;

  std::deque<IMUPreIntegrator::IMUPreIntegration> pre_integration_deque_;

  // pre-integrator:
  std::shared_ptr<IMUPreIntegrator> imu_pre_integrator_ptr_{};
  IMUPreIntegrator::IMUPreIntegration imu_pre_integration_;
  std::shared_ptr<OdoPreIntegrator> odo_pre_integrator_ptr_{};
  OdoPreIntegrator::OdoPreIntegration odo_pre_integration_;

  // optimizer:
  std::shared_ptr<InterfaceGraphOptimizer> graph_optimizer_ptr_{};

  struct GraphOptimizerConfig {
    bool use_gnss{true};
    bool use_loop_close{false};
    bool use_imu_pre_integration{false};
    bool use_odo_pre_integration{false};

    struct {
      int key_frame{100};
      int loop_closure{10};
    } optimization_step_size;

    Eigen::VectorXd odom_edge_noise;
    Eigen::VectorXd close_loop_noise;
    Eigen::VectorXd gnss_noise;

    GraphOptimizerConfig() {
      odom_edge_noise.resize(6);
      close_loop_noise.resize(6);
      gnss_noise.resize(3);
    }
  };
  GraphOptimizerConfig graph_optimizer_config_;

  struct {
    int key_frame{0};
    int loop_closure{0};

    bool HasEnoughKeyFrames(const int num_key_frames_thresh) {
      if (key_frame >= num_key_frames_thresh) {
        key_frame = 0;
        return true;
      }

      return false;
    }

    bool HasEnoughLoopClosures(const int num_loop_closures_thresh) {
      if (loop_closure >= num_loop_closures_thresh) {
        loop_closure = 0;
        return true;
      }

      return false;
    }

  } counter_;
};

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_MAPPING_BACK_END_LIO_BACK_END_SIM_H_