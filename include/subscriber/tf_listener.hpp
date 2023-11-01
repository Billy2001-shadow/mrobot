#ifndef MROBOT_FRAME_TF_LISTENER_HPP_
#define MROBOT_FRAME_TF_LISTENER_HPP_

// ROS LIB
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <string>

#include "open_karto/Karto.h"
#include <Eigen/Dense>

namespace mrobot_frame {
class TFListener {
public:
  TFListener(ros::NodeHandle &nh, std::string base_frame_id,
             std::string child_frame_id);
  TFListener() = default;

  // bool LookupData(Eigen::Matrix4f &transform_matrix);
  // 获取t时刻下 轮式里程计的位姿
  bool getRelativePose(karto::Pose2 &karto_pose, const ros::Time &t);
  bool getRelativePose(double relative_pose[3], const ros::Time &t);

private:
  ros::NodeHandle nh_;
  tf::TransformListener listener_;

  std::string base_frame_id_;
  std::string child_frame_id_;
};
} // namespace mrobot_frame

#endif