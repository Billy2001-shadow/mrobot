#include "subscriber/tf_listener.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "ros/time.h"

#include <Eigen/Geometry>

namespace mrobot_frame {
TFListener::TFListener(ros::NodeHandle &nh, std::string base_frame_id,
                       std::string child_frame_id)
    : nh_(nh), base_frame_id_(base_frame_id), child_frame_id_(child_frame_id) {}

bool TFListener::getOdomPose(karto::Pose2 &karto_pose, const ros::Time &t) {
  // Get the robot's pose
  tf::Stamped<tf::Pose> ident(
      tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0)),
      t, child_frame_id_);
  tf::Stamped<tf::Transform> odom_pose;
  try {
    listener_.transformPose(base_frame_id_, ident, odom_pose);
  } catch (tf::TransformException e) {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  double yaw = tf::getYaw(odom_pose.getRotation());

  karto_pose =
      karto::Pose2(odom_pose.getOrigin().x(), odom_pose.getOrigin().y(), yaw);
  return true;
}

bool TFListener::LookupData(Eigen::Matrix4f &transform_matrix) {
  try {
    tf::StampedTransform transform;
    listener_.lookupTransform(base_frame_id_, child_frame_id_, ros::Time(0),
                              transform);
    TransformToMatrix(transform, transform_matrix);
    return true;
  } catch (tf::TransformException &ex) {
    return false;
  }
}

bool TFListener::TransformToMatrix(const tf::StampedTransform &transform,
                                   Eigen::Matrix4f &transform_matrix) {
  Eigen::Translation3f tl_btol(transform.getOrigin().getX(),
                               transform.getOrigin().getY(),
                               transform.getOrigin().getZ());

  double roll, pitch, yaw;
  tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
  Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());

  // 此矩阵为 child_frame_id 到 base_frame_id 的转换矩阵
  transform_matrix = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

  return true;
}

bool TFListener::TransformToVector(
    std::vector<Eigen::Vector2f> &transform_pose_vector) {
  try {
    tf::StampedTransform transform;
    listener_.lookupTransform(base_frame_id_, child_frame_id_, ros::Time(0),
                              transform);

    Eigen::Vector2f pose_vec(transform.getOrigin().getX(),
                             transform.getOrigin().getY());
    transform_pose_vector.push_back(pose_vec);

    return true;
  } catch (tf::TransformException &ex) {
    return false;
  }
}

} // namespace mrobot_frame