#ifndef MROBOT_FRAME_SENSOR_DATA_KEY_FRAME_HPP_
#define MROBOT_FRAME_SENSOR_DATA_KEY_FRAME_HPP_
// #include "open_karto/Mapper.h"
#include "sensor_data/laser_scan_data.hpp"

namespace mrobot_frame {
class KeyFrame {
public:
  unsigned int index = 0;
  double corrected_pose[3];
  // karto::Pose2 corrected_pose;
  LaserScanData scan_data;
};
} // namespace mrobot_frame

#endif