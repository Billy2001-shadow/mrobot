#include "mapping/front_end/front_end_flow.hpp"
#include "ros/node_handle.h"

namespace mrobot_frame {

FrontEndFlow::FrontEndFlow(ros::NodeHandle &nh, std::string cloud_topic,
                           std::string laser_odom_topic) {
  std::string laser_frame, odom_frame;
  nh.param<std::string>("laser_frame", laser_frame, "base_laser_link");
  nh.param<std::string>("odom_frame", odom_frame, "odom");

  std::cout << "cloud_topic = " << cloud_topic << std::endl;
  //订阅点云信息  tf_pose_ptr_可以优化为寻找所需时间的tf位姿
  cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, cloud_topic, 100000);
  tf_pose_ptr_ = std::make_shared<TFListener>(nh, odom_frame, laser_frame);

  front_end_ptr_ = std::make_shared<FrontEnd>(); //前端的核心算法模块
  occupancygrid_pub_ptr_ =
      std::make_shared<GridmapPublisher>(nh, "occupancygrid", "odom", 1);
  // tf_pose_ptr_->getOdomPose(karto::Pose2 &karto_pose, const ros::Time &t)
  // 获取某一个时刻的机器人位姿 karto_pose的形式
  map_update_interval_.fromSec(5.0);
}

bool FrontEndFlow::Run() {
  // std::cout << "run run run run run run " << std::endl;
  ros::Time last_map_update(0, 0);
  if (!ReadData())
    return false;
  while (HasData()) {

    if (!ValidData()) {

      continue;
    }

    if (UpdateLaserOdometry()) {
      std::cout << "可视化 可视化 可视化 可视化 可视化" << std::endl;
      front_end_ptr_->publishPoseVisualization();
      //在这个地方建图试试看
      if (!got_map_ || (current_ranges_data_.time - last_map_update) >
                           map_update_interval_) {
        if (updateMap()) {
          last_map_update = current_ranges_data_.time;
          got_map_ = true;
          ROS_DEBUG("Updated the map");
        }
      }
    }
  }
  // std::cout << "No No Run Run Run Run Run Run Run!" << std::endl;

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
  // std::cout << "Update Update Update Update Update Update" << std::endl;
  // current_ranges_data_ 比较新？
  if (!tf_pose_ptr_->getOdomPose(karto_pose, current_ranges_data_.time))
    return false;
  karto::LaserRangeFinder *laser = cloud_sub_ptr_->getLaser();
  //根据当前点云数据更新激光里程计计算得到的位姿
  // karto_pose为最终返回的机器人位姿
  return front_end_ptr_->Update(laser, current_ranges_data_, karto_pose);
}

bool FrontEndFlow::publishLaserOdomVisualization() {
  //在这个地方可视化激光里程计的位姿(只可视化出关键帧的)

  return true;
}

bool FrontEndFlow::updateMap() {
  //在这里直接取数组，然后转换成ROS格式发布出去
  nav_msgs::OccupancyGrid ros_map;
  front_end_ptr_->GetRosMap(ros_map);
  occupancygrid_pub_ptr_->Publish(ros_map);

  return true;
}

} // namespace mrobot_frame
