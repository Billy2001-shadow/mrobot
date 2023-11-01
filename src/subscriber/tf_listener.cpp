#include "subscriber/tf_listener.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "ros/time.h"

#include <Eigen/Geometry>

namespace mrobot_frame {
TFListener::TFListener(ros::NodeHandle &nh, std::string base_frame_id,
                       std::string child_frame_id)
    : nh_(nh), base_frame_id_(base_frame_id), child_frame_id_(child_frame_id) {}

bool TFListener::getRelativePose(karto::Pose2 &karto_pose, const ros::Time &t) {
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

bool TFListener::getRelativePose(double relative_pose[3], const ros::Time &t) {
  //先为子体坐标系创建一个标识
  tf::Stamped<tf::Pose> ident(
      tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0)),
      t, child_frame_id_);
  //创建一个父坐标系位姿的变量
  tf::Stamped<tf::Transform> transform_pose;
  try {
    //将子体坐标系的标识转换到里程计坐标系下
    listener_.transformPose(base_frame_id_, ident, transform_pose);
  } catch (tf::TransformException e) {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  relative_pose[0] = transform_pose.getOrigin().x();
  relative_pose[1] = transform_pose.getOrigin().y();
  relative_pose[2] = tf::getYaw(transform_pose.getRotation());

  return true;
}

// bool TFListener::LookupData(Eigen::Matrix4f &transform_matrix) {
//   try {
//     tf::StampedTransform transform;
//     listener_.lookupTransform(base_frame_id_, child_frame_id_, ros::Time(0),
//                               transform);
//     TransformToMatrix(transform, transform_matrix);
//     return true;
//   } catch (tf::TransformException &ex) {
//     return false;
//   }
// }

} // namespace mrobot_frame