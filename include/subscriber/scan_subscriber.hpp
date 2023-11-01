#ifndef MROBOT_FRAME_CLOUD_SUBSCRIBER_HPP_
#define MROBOT_FRAME_CLOUD_SUBSCRIBER_HPP_
// STD LIB
#include <deque>
#include <mutex>
#include <thread>
#include <vector>
// ROS LIB
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
// THIRD LIB
#include "open_karto/Karto.h"
#include "sensor_data/laser_scan_data.hpp"

namespace mrobot_frame {
class ScanSubscriber {
public:
  ScanSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);
  ScanSubscriber() = default;
  ~ScanSubscriber();

  void ParseData(std::deque<LaserScanData> &ranges_data_buff);
  karto::LaserRangeFinder *getLaser(); //设备号可以想办法去除

private:
  void msg_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg_ptr);
  karto::LaserRangeFinder *
  getLaser(const sensor_msgs::LaserScan::ConstPtr &scan);

private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  std::deque<LaserScanData> new_scan_data_;
  std::mutex buff_mutex_;

  //想办法去除对设备号的依赖，从而减少下面的代码
  tf::TransformListener tf_;
  std::map<std::string, karto::LaserRangeFinder *> lasers_;
  std::map<std::string, bool> lasers_inverted_;
  karto::Dataset *dataset_;
  karto::LaserRangeFinder *laser_;

  std::string laser_frame_, odom_frame_;
};
} // namespace mrobot_frame

#endif