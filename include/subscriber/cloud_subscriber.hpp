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

private:
  void msg_callback(const sensor_msgs::LaserScan::ConstPtr &cloud_msg_ptr);

private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  std::deque<CloudData> new_cloud_data_;
  std::mutex buff_mutex_;

  tf::TransformListener tf_; //以下三行组合使用
  message_filters::Subscriber<sensor_msgs::LaserScan> *scan_filter_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> *scan_filter_;

  std::string odom_frame_ = "odom"; //里程计坐标系的名字
};
} // namespace mrobot_frame

#endif