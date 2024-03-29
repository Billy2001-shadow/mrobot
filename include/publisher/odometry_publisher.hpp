#ifndef MROBOT_FRAME_PUBLISHER_ODOMETRY_PUBLISHER_HPP_
#define MROBOT_FRAME_PUBLISHER_ODOMETRY_PUBLISHER_HPP_

#include <string>

#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

namespace mrobot_frame {
class OdometryPublisher {
public:
  OdometryPublisher(ros::NodeHandle &nh, std::string topic_name,
                    std::string base_frame_id, std::string child_frame_id,
                    int buff_size);
  OdometryPublisher() = default;

  void Publish(const double odom_pose[3], ros::Time ros_time);
  void Publish(const Eigen::Matrix4f &transform_matrix, double time);
  void Publish(const Eigen::Matrix4f &transform_matrix);

  bool HasSubscribers();

private:
  void PublishData(const Eigen::Matrix4f &transform_matrix, ros::Time time);

private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  nav_msgs::Odometry odometry_;
};
} // namespace mrobot_frame
#endif