#ifndef MROBOT_FRAME_SENSOR_DATA_RNAGES_DATA_HPP_
#define MROBOT_FRAME_SENSOR_DATA_RNAGES_DATA_HPP_

#include <iostream>
#include <ros/ros.h>
#include <vector>

namespace mrobot_frame {
class RangesData {

public:
  ros::Time time;
  std::vector<double> angles;
  std::vector<double> readings;
};
} // namespace mrobot_frame

#endif