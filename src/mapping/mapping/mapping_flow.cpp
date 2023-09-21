#include "mapping/mapping/mapping_flow.hpp"
#include "global_defination/global_defination.h"
#include "glog/logging.h"

namespace mrobot_frame {
MappingFlow::MappingFlow(ros::NodeHandle &nh) {

  //这里只初始化一个前端的共享指针，
  front_end_ptr_ =
      std::make_shared<FrontEnd>(); //前端的核心算法模块,从前端获取关键帧
  //   occupancygrid_pub_ptr_ =
  //       std::make_shared<GridmapPublisher>(nh, "occupancygrid", "odom", 1);

  //   mapping_ptr_ = std::make_shared<Mapping>(); //调用核心功能类
}

bool MappingFlow::Run() {
  //   if (!ReadData())
  //     return false;

  //   while (HasData()) {
  //     if (ValidData()) {
  //       mapping_ptr_->OccupanyMapping(current_keyframe);
  //       PublishData(); //每过一个关键帧更新一次地图
  //     }
  //   }

  return true;
}

bool MappingFlow::ReadData() {

  //   key_frame_sub_ptr_->ParseData(key_frame_buff_);

  return true;
}

bool MappingFlow::HasData() {
  //   if (key_frame_buff_.size() == 0)
  //     return false;

  return true;
}

bool MappingFlow::ValidData() {
  //   current_keyframe = key_frame_buff_.front();
  //   key_frame_buff_.pop_front();

  return true;
}

bool MappingFlow::PublishData() {
  //   occupancygrid_pub_ptr_->Publish(mapping_ptr_->GetCurrentMap());
  return true;
}

bool MappingFlow::SaveGridmap() { return true; }
} // namespace mrobot_frame