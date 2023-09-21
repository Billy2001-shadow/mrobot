#ifndef MROBOT_FRAME_MAPPING_FRONT_END_FRONT_END_HPP_
#define MROBOT_FRAME_MAPPING_FRONT_END_FRONT_END_HPP_

#include "mapping/mapping/mapping.hpp"
#include "mapping/spa_solver/spa_solver.hpp"
#include "open_karto/Mapper.h"
#include "sensor_data/ranges_data.hpp"
#include "subscriber/tf_listener.hpp"
#include "visualization_msgs/MarkerArray.h"
#include <Eigen/Dense>
#include <deque>
#include <yaml-cpp/yaml.h>

namespace mrobot_frame {
class FrontEnd {

public:
  struct Frame {
    karto::Pose2 karto_pose;
    RangesData ranges_data;
  };

public:
  FrontEnd();
  //得到一帧点云，就返回一个位姿
  bool Update(karto::LaserRangeFinder *laser, const RangesData &ranges_data,
              karto::Pose2 &karto_pose);
  bool SetInitPose();
  void publishPoseVisualization();
  bool GetRosMap(nav_msgs::OccupancyGrid &ros_map);

private:
  bool InitWithConfig();
  bool InitParam(const YAML::Node &config_node);
  bool UpdateWithNewFrame(KeyFrame &new_key_frame);
  bool GetPredictPose(Eigen::Matrix4f &MatrixPose,
                      double time); //获取成功就true

private:
  karto::Mapper *mapper_;
  SpaSolver *solver_;
  std::vector<kt_double> pose_vector_;
  unsigned marker_count_ = 0;
  ros::Publisher marker_publisher_;
  std::shared_ptr<Mapping> mapping_ptr_;
};
} // namespace mrobot_frame

#endif