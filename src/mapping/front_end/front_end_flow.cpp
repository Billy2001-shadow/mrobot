#include "mapping/front_end/front_end_flow.hpp"
#include "ros/node_handle.h"
#include <memory>

namespace mrobot_frame {

FrontEndFlow::FrontEndFlow(ros::NodeHandle &nh, std::string cloud_topic,
                           std::string laser_odom_topic) {
  std::string laser_frame, odom_frame;
  nh.param<std::string>("laser_frame", laser_frame, "base_laser");
  nh.param<std::string>("odom_frame", odom_frame, "odom");

  //订阅点云信息  tf_pose_ptr_可以优化为寻找所需时间的tf位姿
  cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, cloud_topic, 100000);
  tf_pose_ptr_ = std::make_shared<TFListener>(nh, odom_frame, laser_frame);
  //发布关键帧
  keyframe_pub_ptr_ =
      std::make_shared<KeyFramePublisher>(nh, "KeyFrame", "laser_link", 10000);
  //前端的核心算法模块
  front_end_ptr_ = std::make_shared<FrontEnd>();
}

bool FrontEndFlow::Run() {
  if (!ReadData())
    return false;
  while (HasData()) {
    if (!ValidData())
      continue;
    UpdateLaserOdometry();
    PublishData();
    // front_end_ptr_->publishPoseVisualization();
  }
  return true;
}

bool FrontEndFlow::ReadData() {
  cloud_sub_ptr_->ParseData(cloud_data_buff_);
  return true;
}

bool FrontEndFlow::HasData() { return cloud_data_buff_.size() > 0; }

bool FrontEndFlow::ValidData() {
  current_ranges_data_ = cloud_data_buff_.front();
  cloud_data_buff_.pop_front();
  return true;
}

bool FrontEndFlow::UpdateLaserOdometry() {
  //先获取当前激光帧对应的里程计信息
  if (!tf_pose_ptr_->getOdomPose(karto_pose, current_ranges_data_.time))
    return false;
  karto::LaserRangeFinder *laser = cloud_sub_ptr_->getLaser();
  //根据当前点云数据更新激光里程计计算得到的位姿
  // karto_pose为最终返回的机器人位姿
  return front_end_ptr_->Update(laser, current_ranges_data_, karto_pose);
}

bool FrontEndFlow::PublishData() {
  if (front_end_ptr_->HasNewKeyFrame()) {
    KeyFrame key_frame;
    front_end_ptr_->GetLatestKeyFrame(key_frame);
    keyframe_pub_ptr_->Publish(key_frame);
  }
  return true;
}

} // namespace mrobot_frame
