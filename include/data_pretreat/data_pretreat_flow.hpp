#ifndef MROBOT_FRAME_DATA_PRETREAT_DATA_PRETREAT_FLOW_HPP_
#define MROBOT_FRAME_DATA_PRETREAT_DATA_PRETREAT_FLOW_HPP_

#include "publisher/cloud_publisher.hpp"
#include "publisher/odometry_publisher.hpp"
#include "sensor_data/cloud_data.hpp"
#include "subscriber/cloud_subscriber.hpp"
#include "subscriber/cloud_subscriber2.hpp"
#include "subscriber/tf_listener.hpp"

#include "visualization_msgs/MarkerArray.h"
#include <memory>
#include <ros/ros.h>

namespace mrobot_frame {
class DataPretreatFlow {
public:
  DataPretreatFlow(ros::NodeHandle &nh, std::string scan_topic,
                   std::string cloud_topic);
  bool Run();

private:
  bool ReadData();
  bool HasData();
  bool ValidData();
  bool TransformDataToMap();
  bool PublishData();

  void publishWheelOdomVisualization();

private:
  // subscriber
  std::shared_ptr<CloudSubscriber2> cloud_sub_ptr_2;
  std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
  std::shared_ptr<TFListener> tf_pose_ptr_;

  // publisher
  ros::Publisher marker_publisher_;
  std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
  std::shared_ptr<OdometryPublisher> odom_pub_ptr_;

  std::deque<CloudData> cloud_data_buff_;

  CloudData current_cloud_data_;

  Eigen::Matrix4f tf_pose_ = Eigen::Matrix4f::Identity();
  std::vector<Eigen::Vector2f> tf_pose_vector_;

  unsigned marker_count_ = 0;
};
} // namespace mrobot_frame

#endif