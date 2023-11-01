#ifndef MROBOT_FRAME_MAPPING_FRONT_END_FRONT_END_HPP_
#define MROBOT_FRAME_MAPPING_FRONT_END_FRONT_END_HPP_

// STD LIB
#include <deque>

// ROS LIB
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

// THIRD LIB
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

// My LIB
#include "mapping/mapping/mapping.hpp"
#include "mapping/spa_solver/spa_solver.hpp"
#include "open_karto/Mapper.h"
#include "sensor_data/key_frame.hpp"
#include "sensor_data/laser_scan_data.hpp"
#include "subscriber/tf_listener.hpp"

namespace mrobot_frame {
class FrontEnd {

public:
  //这个结构体是否需要 考量一下
  struct Frame {
    karto::Pose2 karto_pose;
    LaserScanData ranges_data;
  };

public:
  FrontEnd();
  ~FrontEnd();
  //得到一帧点云，就返回一个位姿
  bool Update(karto::LaserRangeFinder *laser, const LaserScanData &scan_data,
              karto::Pose2 &karto_pose);
  void publishPoseVisualization();
  bool HasNewKeyFrame();
  void GetLatestKeyFrame(KeyFrame &key_frame);

private:
  bool InitWithConfig();
  bool InitParam(const YAML::Node &config_node);
  void ResetParam();

private:
  bool has_new_key_frame_;

  KeyFrame current_key_frame_;
  std::deque<KeyFrame> key_frames_deque_;
  karto::Mapper *mapper_;
  SpaSolver *solver_;
  std::vector<kt_double> pose_vector_;
  unsigned marker_count_ = 0;
  ros::Publisher marker_publisher_;
  std::shared_ptr<Mapping> mapping_ptr_;
};
} // namespace mrobot_frame

#endif