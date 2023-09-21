#ifndef MROBOT_FRAME_MAPPING_MAPPING_FLOW_HPP_
#define MROBOT_FRAME_MAPPING_MAPPING_FLOW_HPP_

#include <deque>
#include <ros/ros.h>
// subscriber
#include "mapping/front_end/front_end.hpp"
#include "subscriber/cloud_subscriber.hpp"
#include "subscriber/cloud_subscriber2.hpp"

// #include "mrobot_frame/subscriber/key_frame_subscriber.hpp"

// publisher
// #include "publisher/odometry_publisher.hpp"
// #include "publisher/cloud_publisher.hpp"
#include "mapping/mapping/mapping.hpp"
#include "publisher/gridmap_publisher.hpp"

namespace mrobot_frame {
class MappingFlow {
public:
  MappingFlow(ros::NodeHandle &nh);
  bool Run();
  bool
  SaveGridmap(); //建立珊格地图(后面可以和viewer中保存点云地图的函数名区分开来)
                 //供node文件调用

private:
  bool ReadData();
  bool HasData();
  bool ValidData();
  bool PublishData();

private:
  std::shared_ptr<FrontEnd> front_end_ptr_;
};
} // namespace mrobot_frame

#endif