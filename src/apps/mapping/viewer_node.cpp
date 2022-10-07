/*
 * @Description: viewer 的 node 文件
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:56:27
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-25 22:33:01
 */
#include <glog/logging.h>

#include <ros/ros.h>
// service
#include "lidar_localization/saveMap.h"
// mapping
#include "lidar_localization/mapping/viewer/viewer_flow.h"
// global
#include "lidar_localization/global_defination/global_defination.h"

using namespace lidar_localization;

bool need_save_map = false;

bool save_map_callback(saveMap::Request& request, saveMap::Response& response) {
  need_save_map = true;
  response.succeed = true;
  return response.succeed;
}

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
  FLAGS_alsologtostderr = 1;

  ros::init(argc, argv, "viewer_node");
  ros::NodeHandle nh;

  std::string cloud_topic{};
  nh.param<std::string>("cloud_topic", cloud_topic, "/synced_cloud");
  std::shared_ptr<ViewerFlow> viewer_flow_ptr =
      std::make_shared<ViewerFlow>(nh, cloud_topic);

  ros::ServiceServer service =
      nh.advertiseService("save_map", save_map_callback);

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();

    viewer_flow_ptr->Run();
    if (need_save_map) {
      need_save_map = false;
      viewer_flow_ptr->SaveMap();
    }

    rate.sleep();
  }

  return 0;
}