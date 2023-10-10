#ifndef MROBOT_FRAME_MAPPING_MAPPING_FLOW_HPP_
#define MROBOT_FRAME_MAPPING_MAPPING_FLOW_HPP_

#include <deque>
#include <memory>
#include <queue>
#include <ros/ros.h>
// subscriber
#include "mapping/front_end/front_end.hpp"
#include "mrobot_frame/keyframemsg.h"
#include "sensor_data/key_frame.hpp"
#include "subscriber/cloud_subscriber.hpp"
#include "subscriber/key_frame_subscriber.hpp"
// publisher
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
  std::shared_ptr<KeyFrameSubscriber> keyframe_sub_ptr_;
  ros::Subscriber keyframe_sub_;

  KeyFrame current_keyframe_;
  std::deque<KeyFrame> key_frame_buff_;

  std::shared_ptr<Mapping> mapping_ptr_;
  std::shared_ptr<GridmapPublisher> occupancygrid_pub_ptr_;

  ros::Duration map_update_interval_;
  bool got_map_ = false;
};
} // namespace mrobot_frame

#endif