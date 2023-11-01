#include "publisher/odometry_publisher.hpp"

namespace mrobot_frame {
OdometryPublisher::OdometryPublisher(ros::NodeHandle &nh,
                                     std::string topic_name,
                                     std::string base_frame_id,
                                     std::string child_frame_id, int buff_size)
    : nh_(nh) {

  publisher_ = nh_.advertise<nav_msgs::Odometry>(topic_name, buff_size);
  odometry_.header.frame_id = base_frame_id;
  odometry_.child_frame_id = child_frame_id;
}

void OdometryPublisher::Publish(const double odom_pose[3], ros::Time ros_time) {
  // ros::Time ros_time((float)time);
  Eigen::Matrix4f transform_matrix;
  //先将2D的位姿odom_pose转换成转换矩阵形式transform_matrix
  Eigen::Matrix3f rotation_matrix;
  rotation_matrix << cos(odom_pose[2]), -sin(odom_pose[2]), 0,
      sin(odom_pose[2]), cos(odom_pose[2]), 0, 0, 0, 1;
  transform_matrix << rotation_matrix,
      Eigen::Vector3f(odom_pose[0], odom_pose[1], 0), 0, 0, 0, 1;

  PublishData(transform_matrix, ros_time);
}

void OdometryPublisher::Publish(const Eigen::Matrix4f &transform_matrix,
                                double time) {
  ros::Time ros_time((float)time);
  PublishData(transform_matrix, ros_time);
}

void OdometryPublisher::Publish(const Eigen::Matrix4f &transform_matrix) {
  PublishData(transform_matrix, ros::Time::now());
}

void OdometryPublisher::PublishData(const Eigen::Matrix4f &transform_matrix,
                                    ros::Time time) {
  odometry_.header.stamp = time;

  // set the position
  odometry_.pose.pose.position.x = transform_matrix(0, 3);
  odometry_.pose.pose.position.y = transform_matrix(1, 3);
  odometry_.pose.pose.position.z = transform_matrix(2, 3);

  Eigen::Quaternionf q;
  q = transform_matrix.block<3, 3>(0, 0);
  odometry_.pose.pose.orientation.x = q.x();
  odometry_.pose.pose.orientation.y = q.y();
  odometry_.pose.pose.orientation.z = q.z();
  odometry_.pose.pose.orientation.w = q.w();

  publisher_.publish(odometry_);
}

bool OdometryPublisher::HasSubscribers() {
  return publisher_.getNumSubscribers() != 0;
}
} // namespace mrobot_frame