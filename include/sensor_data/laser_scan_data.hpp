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
};
} // namespace mrobot_frame

#endif