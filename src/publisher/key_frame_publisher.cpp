#include "publisher/key_frame_publisher.hpp"

namespace mrobot_frame {
KeyFramePublisher::KeyFramePublisher(ros::NodeHandle &nh,
                                     std::string topic_name,
                                     std::string frame_id, int buff_size)
    : nh_(nh), frame_id_(frame_id) {

  publisher_ = nh_.advertise<mrobot_frame::keyframemsg>(topic_name, buff_size);
}

void KeyFramePublisher::Publish(KeyFrame &key_frame) {
  mrobot_frame::keyframemsg keyframe_stamped;

  keyframe_stamped.header.stamp = key_frame.ranges_data.time;
  keyframe_stamped.header.frame_id = frame_id_;
  keyframe_stamped.header.seq = 1;

  keyframe_stamped.Pose2d.at(0) = key_frame.corrected_pose.GetX();
  keyframe_stamped.Pose2d.at(1) = key_frame.corrected_pose.GetY();
  keyframe_stamped.Pose2d.at(2) = key_frame.corrected_pose.GetHeading();

  for (int i = 0; i < key_frame.ranges_data.angles_readings.size(); i++) {
    keyframe_stamped.angles.push_back(
        key_frame.ranges_data.angles_readings.at(i));
  }
  for (int i = 0; i < key_frame.ranges_data.range_readings.size(); i++) {
    keyframe_stamped.readings.push_back(
        key_frame.ranges_data.range_readings.at(i));
  }
  publisher_.publish(keyframe_stamped);
}

bool KeyFramePublisher::HasSubscribers() {
  return publisher_.getNumSubscribers() != 0;
}
} // namespace mrobot_frame