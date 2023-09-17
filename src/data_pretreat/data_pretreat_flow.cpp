/*
 * @Description: 数据预处理模块：包括时间同步、点云去畸变等
 * @Author: Chen Wu
 * @Date: 2023-09-17 12:38:42
 */
#include "global_defination/global_defination.h"
#include "glog/logging.h"

#include "data_pretreat/data_pretreat_flow.hpp"
#include <cstdint>
#include <pcl/common/transforms.h>

namespace mrobot_frame {
DataPretreatFlow::DataPretreatFlow(ros::NodeHandle &nh, std::string scan_topic,
                                   std::string cloud_topic) {

  std::string laser_frame, odom_frame;
  nh.param<std::string>("laser_frame", laser_frame, "laser_link");
  nh.param<std::string>("odom_frame", odom_frame, "odom");
  // subscriber
  cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, scan_topic, 100000);
  tf_pose_ptr_ = std::make_shared<TFListener>(nh, odom_frame, laser_frame);

  // publisher
  cloud_pub_ptr_ =
      std::make_shared<CloudPublisher>(nh, cloud_topic, laser_frame, 100);
  odom_pub_ptr_ = std::make_shared<OdometryPublisher>(
      nh, "/odom_pose", odom_frame, laser_frame, 100);
  marker_publisher_ = nh.advertise<visualization_msgs::MarkerArray>(
      "visualization_marker_array", 1);
}

bool DataPretreatFlow::Run() {
  if (!ReadData())
    return false;

  while (HasData()) {
    if (!ValidData())
      continue;
    TransformDataToMap();
    PublishData();
    // publishWheelOdomVisualization();
  }

  return true;
}

bool DataPretreatFlow::ReadData() {
  cloud_sub_ptr_->ParseData(cloud_data_buff_);

  if (cloud_data_buff_.size() == 0)
    return false;

  return true;
}

bool DataPretreatFlow::HasData() { return cloud_data_buff_.size() > 0; }

bool DataPretreatFlow::ValidData() {
  current_cloud_data_ = cloud_data_buff_.front();

  cloud_data_buff_.pop_front();

  return true;
}

bool DataPretreatFlow::TransformDataToMap() {
  tf_pose_ptr_->LookupData(tf_pose_);
  tf_pose_ptr_->TransformToVector(tf_pose_vector_);
  // pcl::transformPointCloud(*current_cloud_data_.cloud_ptr,
  //                          *current_cloud_data_.cloud_ptr,
  //                          tf_pose_); //将点云转换到odom坐标系下

  //在这里加上一个调整点云的函数(去畸变)

  return true;
}

bool DataPretreatFlow::PublishData() {
  cloud_pub_ptr_->Publish(current_cloud_data_.cloud_ptr,
                          current_cloud_data_.time);
  odom_pub_ptr_->Publish(tf_pose_, current_cloud_data_.time); //轮式里程计
  // std::cout << "tf_pose_ = " << tf_pose_ << std::endl;

  return true;
}

void DataPretreatFlow::publishWheelOdomVisualization() {
  visualization_msgs::MarkerArray marray;
  //顶点
  visualization_msgs::Marker m;
  m.header.frame_id = "odom";
  m.header.stamp = ros::Time::now();
  m.id = 0;
  m.ns = "mrobot_frame";
  m.type = visualization_msgs::Marker::SPHERE;
  m.pose.position.x = 0.0;
  m.pose.position.y = 0.0;
  m.pose.position.z = 0.0;
  m.scale.x = 0.1;
  m.scale.y = 0.1;
  m.scale.z = 0.1;
  m.color.r = 1.0;
  m.color.g = 0;
  m.color.b = 0.0;
  m.color.a = 1.0;
  m.lifetime = ros::Duration(0);
  //边长
  visualization_msgs::Marker edge;
  edge.header.frame_id = "odom";
  edge.header.stamp = ros::Time::now();
  edge.action = visualization_msgs::Marker::ADD;
  edge.ns = "mrobot_frame";
  edge.id = 0;
  edge.type = visualization_msgs::Marker::LINE_STRIP;
  edge.scale.x = 0.1;
  edge.scale.y = 0.1;
  edge.scale.z = 0.1;
  edge.color.a = 1.0;
  edge.color.r = 0.0;
  edge.color.g = 0.0;
  edge.color.b = 1.0;

  m.action = visualization_msgs::Marker::ADD;
  uint32_t id = 0;
  for (uint32_t i = 0; i < tf_pose_vector_.size(); i++) {
    m.id = id;
    m.pose.position.x = tf_pose_vector_[i].x();
    m.pose.position.y = tf_pose_vector_[i].y();
    marray.markers.push_back(visualization_msgs::Marker(m));
    id++;

    m.pose.position.x = tf_pose_vector_[i + 1].x();
    m.pose.position.y = tf_pose_vector_[i + 1].y();
    marray.markers.push_back(visualization_msgs::Marker(m));
    id++;

    edge.points.clear();

    if (i != tf_pose_vector_.size() - 1) {
      geometry_msgs::Point p;
      p.x = tf_pose_vector_[i].x();
      p.y = tf_pose_vector_[i].y();
      edge.points.push_back(p);
      p.x = tf_pose_vector_[i + 1].x();
      p.y = tf_pose_vector_[i + 1].y();
      edge.points.push_back(p);
      edge.id = id;

      marray.markers.push_back(visualization_msgs::Marker(edge));
      id++;
    }
  }

  m.action = visualization_msgs::Marker::DELETE;
  for (; id < marker_count_; id++) {
    m.id = id;
    marray.markers.push_back(visualization_msgs::Marker(m));
  }

  marker_count_ = marray.markers.size();
  marker_publisher_.publish(marray);
}

} // namespace mrobot_frame