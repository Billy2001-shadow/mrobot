#ifndef MROBOT_FRAME_MAPPING_FRONT_END_FRONT_END_FLOW_HPP_
#define MROBOT_FRAME_MAPPING_FRONT_END_FRONT_END_FLOW_HPP_

// STD LIB

// ROS LIB

// THIRD LIB

// My LIB
#include "mapping/front_end/front_end.hpp"
#include "publisher/key_frame_publisher.hpp"
#include "subscriber/scan_subscriber.hpp"

namespace mrobot_frame {
class FrontEndFlow {
public:
  FrontEndFlow(ros::NodeHandle &nh, std::string scan_topic);
  bool Run();

private:
  bool ReadData();
  bool HasData();
  bool ValidData();
  bool UpdateLaserOdometry();
  bool PublishData();

private:
  std::shared_ptr<ScanSubscriber> scan_sub_ptr_;
  std::shared_ptr<TFListener> laser2odom_tf_ptr_;
  std::shared_ptr<FrontEnd> front_end_ptr_;
  std::shared_ptr<KeyFramePublisher> keyframe_pub_ptr_;

  std::deque<LaserScanData> scan_data_buff_;
  LaserScanData current_scan_data_;
  karto::Pose2 karto_pose;
};
} // namespace mrobot_frame

#endif