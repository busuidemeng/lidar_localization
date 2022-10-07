/*
 * @Description:
 * @Autor: ZiJieChen
 * @Date: 2022-09-28 15:29:44
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-10-03 19:41:57
 */
#include <glog/logging.h>

#include <ros/ros.h>

#include "lidar_localization/mapping/back_end/lio_back_end_sim_flow.h"

#include "lidar_localization/global_defination/global_defination.h"

#include "lidar_localization/optimizeMap.h"

using namespace lidar_localization;

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
  FLAGS_alsologtostderr = 1;

  ros::init(argc, argv, "lio_back_end_sim_node");
  ros::NodeHandle nh;

  std::shared_ptr<LIOBackEndSimFlow> lio_back_end_sim_flow_ptr =
      std::make_shared<LIOBackEndSimFlow>(nh);

  ros::Rate rate(10);
  while (ros::ok()) {
    ros::spinOnce();

    lio_back_end_sim_flow_ptr->Run();

    rate.sleep();
  }

  LOG(INFO) << " optimize the path and save the path. " << std::endl;
  lio_back_end_sim_flow_ptr->ForceOptimize();
  lio_back_end_sim_flow_ptr->ForceOptimize();
  lio_back_end_sim_flow_ptr->SaveOptimizedOdometry();
  LOG(INFO) << " simulation is finshed. " << std::endl;

  return EXIT_SUCCESS;
}