/*
 * @Description:
 * @Autor: ZiJieChen
 * @Date: 2022-09-29 15:16:23
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-10-03 22:18:02
 */
#include <fstream>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include "lidar_localization/simulator/activity.h"

using namespace lidar_localization;

int main(int argc, char** argv) {
  ros::init(argc, argv, "simulator_node");

  generator::Activity activity;

  activity.Init();

  // 100 Hz:
  ros::Rate loop_rate(100);
  while (ros::ok()) {
    ros::spinOnce();

    activity.Run();

    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}