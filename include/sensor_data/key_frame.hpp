#ifndef MROBOT_FRAME_SENSOR_DATA_KEY_FRAME_HPP_
#define MROBOT_FRAME_SENSOR_DATA_KEY_FRAME_HPP_
#include "open_karto/Mapper.h"
#include "sensor_data/ranges_data.hpp"

namespace mrobot_frame {
class KeyFrame {
public:
  unsigned int index = 0;
  karto::Pose2 corrected_pose;
  RangesData ranges_data;
};
} // namespace mrobot_frame

#endif