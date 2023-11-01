#include "mapping/mapping/mapping_flow.hpp"

namespace mrobot_frame {
MappingFlow::MappingFlow(ros::NodeHandle &nh) {

  //订阅者：订阅关键帧
  keyframe_sub_ptr_ =
      std::make_shared<KeyFrameSubscriber>(nh, "KeyFrame", 10000);
  //发布者：占据栅格地图
  occupancygrid_pub_ptr_ = std::make_shared<GridmapPublisher>(
      nh, "occupancygrid", "robot_6/odom", 1);

  //根据历史关键帧建图
  mapping_ptr_ = std::make_shared<Mapping>(); //调用核心功能类

  map_update_interval_.fromSec(5.0);
}

bool MappingFlow::Run() {
  ros::Time last_map_update(0, 0);
  if (!ReadData())
    return false;

  while (HasData()) {
    if (ValidData()) {
      mapping_ptr_->OccupanyMapping(current_keyframe_);
      // if (!got_map_ || (current_ranges_data_.time - last_map_update) >
      //                      map_update_interval_) {
      //   if (updateMap()) {
      //     last_map_update = current_ranges_data_.time;
      //     got_map_ = true;
      //     ROS_DEBUG("Updated the map");
      //   }
      PublishData(); //每过一个关键帧更新一次地图
    }
  }

  return true;
}

bool MappingFlow::ReadData() {
  keyframe_sub_ptr_->ParseData(key_frame_buff_);
  return true;
}

bool MappingFlow::HasData() {
  if (key_frame_buff_.size() == 0)
    return false;

  return true;
}

bool MappingFlow::ValidData() {
  current_keyframe_ = key_frame_buff_.front();
  key_frame_buff_.pop_front();
  return true;
}

bool MappingFlow::PublishData() {
  occupancygrid_pub_ptr_->Publish(mapping_ptr_->GetCurrentMap());
  return true;
}

bool MappingFlow::SaveGridmap() { return true; }
} // namespace mrobot_frame