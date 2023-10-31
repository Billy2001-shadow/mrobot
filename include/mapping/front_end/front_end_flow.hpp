#ifndef MROBOT_FRAME_MAPPING_FRONT_END_FRONT_END_FLOW_HPP_
#define MROBOT_FRAME_MAPPING_FRONT_END_FRONT_END_FLOW_HPP_

// STD LIB

// ROS LIB

// THIRD LIB

// My LIB
#include "mapping/front_end/front_end.hpp"
#include "publisher/key_frame_publisher.hpp"
#include "subscriber/cloud_subscriber.hpp"

namespace mrobot_frame {
class FrontEndFlow {
public:
  FrontEndFlow(ros::NodeHandle &nh, std::string cloud_topic);
  bool Run();

private:
  bool ReadData();
  bool HasData();
  bool ValidData();
  bool UpdateLaserOdometry();
  bool PublishData();

private:
  std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
  std::shared_ptr<TFListener> tf_pose_ptr_;
  std::shared_ptr<FrontEnd> front_end_ptr_;
  std::shared_ptr<KeyFramePublisher> keyframe_pub_ptr_;

  std::deque<RangesData> ranges_data_buff_;
  RangesData current_ranges_data_;
  karto::Pose2 karto_pose;
};
} // namespace mrobot_frame

#endif