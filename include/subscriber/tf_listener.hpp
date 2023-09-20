#ifndef MROBOT_FRAME_TF_LISTENER_HPP_
#define MROBOT_FRAME_TF_LISTENER_HPP_

#include <string>

#include "open_karto/Karto.h"
#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace mrobot_frame {
class TFListener {
public:
  TFListener(ros::NodeHandle &nh, std::string base_frame_id,
             std::string child_frame_id);
  TFListener() = default;

  bool LookupData(Eigen::Matrix4f &transform_matrix);
  bool TransformToVector(Eigen::Vector3d &transform_vector);
  bool TransformToVector(std::vector<Eigen::Vector2f> &transform_pose_vector);
  bool getOdomPose(karto::Pose2 &karto_pose, const ros::Time &t);

private:
  bool TransformToMatrix(const tf::StampedTransform &transform,
                         Eigen::Matrix4f &transform_matrix);

private:
  ros::NodeHandle nh_;
  tf::TransformListener listener_;

  std::string base_frame_id_;
  std::string child_frame_id_;
};
} // namespace mrobot_frame

#endif