/*
 * @Description: LIO mapping backend ROS node
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-10-02 21:29:58
 */
#include <glog/logging.h>

#include <ros/ros.h>
// service
#include "lidar_localization/optimizeMap.h"
// mapping
#include "lidar_localization/mapping/back_end/hk_lio_back_end_flow.h"
// global
#include "lidar_localization/global_defination/global_defination.h"

using namespace lidar_localization;

bool need_optimize_map_ = false;

bool OptimizeMapCb(optimizeMap::Request& request,
                   optimizeMap::Response& response) {
  need_optimize_map_ = true;
  response.succeed = true;
  return response.succeed;
}

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
  FLAGS_alsologtostderr = 1;

  ros::init(argc, argv, "lio_back_end_node");
  ros::NodeHandle nh;

  std::string cloud_topic, odom_topic;
  nh.param<std::string>("cloud_topic", cloud_topic, "/synced_cloud");
  nh.param<std::string>("odom_topic", odom_topic, "/laser_odom");

  //
  // subscribe to:
  //
  // a. undistorted Velodyne measurement:
  // b. lidar pose in map frame:
  // c. lidar odometry estimation:
  // d. loop close pose:
  // publish:
  // a. lidar odometry in map frame:
  // b. key frame pose and corresponding GNSS/IMU pose
  // c. optimized key frame sequence as trajectory
  std::shared_ptr<HKLIOBackEndFlow> hk_lio_back_end_flow_ptr =
      std::make_shared<HKLIOBackEndFlow>(nh, cloud_topic, odom_topic);
  ros::ServiceServer service =
      nh.advertiseService("optimize_map", OptimizeMapCb);

  ros::Rate rate(10);
  while (ros::ok()) {
    ros::spinOnce();

    hk_lio_back_end_flow_ptr->Run();

    if (need_optimize_map_) {
      hk_lio_back_end_flow_ptr->ForceOptimize();
      hk_lio_back_end_flow_ptr->SaveOptimizedOdometry();

      need_optimize_map_ = false;
    }

    rate.sleep();
  }

  return EXIT_SUCCESS;
}