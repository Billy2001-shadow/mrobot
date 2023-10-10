#ifndef MROBOT_FRAME_CLOUD_SUBSCRIBER_HPP_
#define MROBOT_FRAME_CLOUD_SUBSCRIBER_HPP_

#include "data_pretreat/trans_data_methods/transcloud.hpp"
#include "sensor_data/cloud_data.hpp"

#include <deque>
#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <thread>

#include "message_filters/subscriber.h"
#include "open_karto/Karto.h"
#include "sensor_data/ranges_data.hpp"
#include "tf/message_filter.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

namespace mrobot_frame {
class CloudSubscriber {
public:
  CloudSubscriber(ros::NodeHandle &nh, std::string topic_name,
                  size_t buff_size);
  CloudSubscriber() = default;
  ~CloudSubscriber();
  void ParseData(std::deque<CloudData> &cloud_data_buff);
  void ParseData(std::deque<RangesData> &ranges_data_buff);
  karto::LaserRangeFinder *getLaser();

private:
  void msg_callback(const sensor_msgs::LaserScan::ConstPtr &cloud_msg_ptr);
  karto::LaserRangeFinder *
  getLaser(const sensor_msgs::LaserScan::ConstPtr &scan);

private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  std::deque<CloudData> new_cloud_data_;
  std::deque<RangesData> new_ranges_data_;
  std::mutex buff_mutex_;

  tf::TransformListener tf_;

  std::map<std::string, karto::LaserRangeFinder *> lasers_;
  std::map<std::string, bool> lasers_inverted_;
  karto::Dataset *dataset_;
  karto::LaserRangeFinder *laser_;

  std::string odom_frame_ = "odom"; //里程计坐标系的名字
  std::string laser_frame_ = "base_laser_link";
};
} // namespace mrobot_frame

#endif