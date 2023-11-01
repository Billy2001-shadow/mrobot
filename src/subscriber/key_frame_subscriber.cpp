#include "subscriber/key_frame_subscriber.hpp"

namespace mrobot_frame {
KeyFrameSubscriber::KeyFrameSubscriber(ros::NodeHandle &nh,
                                       std::string topic_name, size_t buff_size)
    : nh_(nh) {
  subscriber_ = nh_.subscribe(topic_name, buff_size,
                              &KeyFrameSubscriber::msg_callback, this);
}

void KeyFrameSubscriber::msg_callback(
    const mrobot_frame::keyframemsgConstPtr &key_frame_msg_ptr) {
  KeyFrame key_frame;

  double x, y, theta;
  x = key_frame_msg_ptr->Pose2d.at(0);
  y = key_frame_msg_ptr->Pose2d.at(1);
  theta = key_frame_msg_ptr->Pose2d.at(2);

  key_frame.corrected_pose.SetX(x);
  key_frame.corrected_pose.SetY(y);
  key_frame.corrected_pose.SetHeading(theta);

  key_frame.ranges_data.time = key_frame_msg_ptr->header.stamp;
  for (int i = 0; i < key_frame_msg_ptr->angles.size(); i++) {
    key_frame.ranges_data.angles_readings.push_back(
        key_frame_msg_ptr->angles.at(i));
  }
  for (int i = 0; i < key_frame_msg_ptr->readings.size(); i++) {
    key_frame.ranges_data.range_readings.push_back(
        key_frame_msg_ptr->readings.at(i));
  }

  new_key_frame_.push_back(key_frame);
}

void KeyFrameSubscriber::ParseData(std::deque<KeyFrame> &key_frame_buff) {
  if (new_key_frame_.size() > 0) {
    key_frame_buff.insert(key_frame_buff.end(), new_key_frame_.begin(),
                          new_key_frame_.end());
    new_key_frame_.clear();
  }
}
} // namespace mrobot_frame