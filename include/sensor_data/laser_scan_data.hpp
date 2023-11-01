#ifndef MROBOT_FRAME_SENSOR_DATA_RNAGES_DATA_HPP_
#define MROBOT_FRAME_SENSOR_DATA_RNAGES_DATA_HPP_

#include <ros/ros.h>
#include <vector>

namespace mrobot_frame {
class LaserScanData {

public:
  ros::Time time;
  std::vector<double> angles_readings;
  std::vector<double> range_readings;
  //后续可以加一个size来表示激光雷达数据的数量，在构造函数中初始化
};
} // namespace mrobot_frame

#endif