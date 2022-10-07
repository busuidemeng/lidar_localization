/*
 * @Description:
 * @Author: Ren Qian
 * @Date: 2020-02-29 03:32:14
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-25 22:18:13
 */
#ifndef LIDAR_LOCALIZATION_MAPPING_LOOP_CLOSING_LOOP_CLOSING_FLOW_H_
#define LIDAR_LOCALIZATION_MAPPING_LOOP_CLOSING_LOOP_CLOSING_FLOW_H_

#include <deque>

#include <glog/logging.h>

#include <ros/ros.h>

// subscriber
#include "lidar_localization/subscriber/cloud_subscriber.h"
#include "lidar_localization/subscriber/key_frame_subscriber.h"
// publisher
#include "lidar_localization/publisher/loop_pose_publisher.h"
// loop closing
#include "lidar_localization/mapping/loop_closing/loop_closing.h"
// global
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {
class LoopClosingFlow {
 public:
  LoopClosingFlow(ros::NodeHandle& nh);

  bool Run();
  bool Save();

 private:
  bool ReadData();
  bool HasData() const;
  bool ValidData();
  bool PublishData();

  // subscriber
  std::shared_ptr<CloudSubscriber> key_scan_sub_ptr_{};
  std::shared_ptr<KeyFrameSubscriber> key_frame_sub_ptr_{};
  std::shared_ptr<KeyFrameSubscriber> key_gnss_sub_ptr_{};
  // publisher
  std::shared_ptr<LoopPosePublisher> loop_pose_pub_ptr_{};
  // loop closing
  std::shared_ptr<LoopClosing> loop_closing_ptr_{};

  std::deque<CloudData> key_scan_buff_;
  std::deque<KeyFrame> key_frame_buff_;
  std::deque<KeyFrame> key_gnss_buff_;

  CloudData current_key_scan_;
  KeyFrame current_key_frame_;
  KeyFrame current_key_gnss_;
};
}  // namespace lidar_localization

#endif