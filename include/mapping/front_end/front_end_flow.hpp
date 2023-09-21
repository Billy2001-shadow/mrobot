#ifndef MROBOT_FRAME_MAPPING_FRONT_END_FRONT_END_FLOW_HPP_
#define MROBOT_FRAME_MAPPING_FRONT_END_FRONT_END_FLOW_HPP_

#include <ros/ros.h>

#include "mapping/front_end/front_end.hpp"
#include "publisher/cloud_publisher.hpp"
#include "publisher/gridmap_publisher.hpp"
#include "sensor_data/ranges_data.hpp"
#include "subscriber/cloud_subscriber.hpp"
#include "subscriber/tf_listener.hpp"
namespace mrobot_frame {
class FrontEndFlow {
public:
  FrontEndFlow(ros::NodeHandle &nh, std::string cloud_topic,
               std::string laser_odom_topic);
  bool Run();

private:
  bool ReadData();
  bool HasData();
  bool ValidData();
  bool UpdateLaserOdometry();
  bool PublishData();
  bool publishLaserOdomVisualization();
  bool updateMap();

private:
  std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
  std::shared_ptr<TFListener> tf_pose_ptr_;
  std::shared_ptr<FrontEnd> front_end_ptr_;

  std::deque<RangesData> cloud_data_buff_;
  RangesData current_ranges_data_;
  karto::Pose2 karto_pose;
  bool got_map_ = false;
  ros::Duration map_update_interval_;
  std::shared_ptr<GridmapPublisher> occupancygrid_pub_ptr_;
};
} // namespace mrobot_frame

#endif