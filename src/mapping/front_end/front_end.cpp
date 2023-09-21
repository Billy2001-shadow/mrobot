#include "mapping/front_end/front_end.hpp"
#include "global_defination/global_defination.h"
#include "glog/logging.h"
#include "mapping/mapping/mapping.hpp"

namespace mrobot_frame {
FrontEnd::FrontEnd() {
  mapper_ = new karto::Mapper();
  mapping_ptr_ = std::make_shared<Mapping>();
  InitWithConfig();
}

//初始化参数，匹配方法，滤波
bool FrontEnd::InitWithConfig() {
  ros::NodeHandle private_nh_("~");
  bool use_scan_matching;
  if (private_nh_.getParam("use_scan_matching", use_scan_matching))
    mapper_->setParamUseScanMatching(use_scan_matching);

  bool use_scan_barycenter;
  if (private_nh_.getParam("use_scan_barycenter", use_scan_barycenter))
    mapper_->setParamUseScanBarycenter(use_scan_barycenter);

  double minimum_time_interval;
  if (private_nh_.getParam("minimum_time_interval", minimum_time_interval))
    mapper_->setParamMinimumTimeInterval(minimum_time_interval);

  double minimum_travel_distance;
  if (private_nh_.getParam("minimum_travel_distance", minimum_travel_distance))
    mapper_->setParamMinimumTravelDistance(minimum_travel_distance);

  double minimum_travel_heading;
  if (private_nh_.getParam("minimum_travel_heading", minimum_travel_heading))
    mapper_->setParamMinimumTravelHeading(minimum_travel_heading);

  int scan_buffer_size;
  if (private_nh_.getParam("scan_buffer_size", scan_buffer_size))
    mapper_->setParamScanBufferSize(scan_buffer_size);

  double scan_buffer_maximum_scan_distance;
  if (private_nh_.getParam("scan_buffer_maximum_scan_distance",
                           scan_buffer_maximum_scan_distance))
    mapper_->setParamScanBufferMaximumScanDistance(
        scan_buffer_maximum_scan_distance);

  double link_match_minimum_response_fine;
  if (private_nh_.getParam("link_match_minimum_response_fine",
                           link_match_minimum_response_fine))
    mapper_->setParamLinkMatchMinimumResponseFine(
        link_match_minimum_response_fine);

  double link_scan_maximum_distance;
  if (private_nh_.getParam("link_scan_maximum_distance",
                           link_scan_maximum_distance))
    mapper_->setParamLinkScanMaximumDistance(link_scan_maximum_distance);

  double loop_search_maximum_distance;
  if (private_nh_.getParam("loop_search_maximum_distance",
                           loop_search_maximum_distance))
    mapper_->setParamLoopSearchMaximumDistance(loop_search_maximum_distance);

  bool do_loop_closing;
  if (private_nh_.getParam("do_loop_closing", do_loop_closing))
    mapper_->setParamDoLoopClosing(do_loop_closing);

  int loop_match_minimum_chain_size;
  if (private_nh_.getParam("loop_match_minimum_chain_size",
                           loop_match_minimum_chain_size))
    mapper_->setParamLoopMatchMinimumChainSize(loop_match_minimum_chain_size);

  double loop_match_maximum_variance_coarse;
  if (private_nh_.getParam("loop_match_maximum_variance_coarse",
                           loop_match_maximum_variance_coarse))
    mapper_->setParamLoopMatchMaximumVarianceCoarse(
        loop_match_maximum_variance_coarse);

  double loop_match_minimum_response_coarse;
  if (private_nh_.getParam("loop_match_minimum_response_coarse",
                           loop_match_minimum_response_coarse))
    mapper_->setParamLoopMatchMinimumResponseCoarse(
        loop_match_minimum_response_coarse);

  double loop_match_minimum_response_fine;
  if (private_nh_.getParam("loop_match_minimum_response_fine",
                           loop_match_minimum_response_fine))
    mapper_->setParamLoopMatchMinimumResponseFine(
        loop_match_minimum_response_fine);

  // Setting Correlation Parameters from the Parameter Server

  double correlation_search_space_dimension;
  if (private_nh_.getParam("correlation_search_space_dimension",
                           correlation_search_space_dimension))
    mapper_->setParamCorrelationSearchSpaceDimension(
        correlation_search_space_dimension);

  double correlation_search_space_resolution;
  if (private_nh_.getParam("correlation_search_space_resolution",
                           correlation_search_space_resolution))
    mapper_->setParamCorrelationSearchSpaceResolution(
        correlation_search_space_resolution);

  double correlation_search_space_smear_deviation;
  if (private_nh_.getParam("correlation_search_space_smear_deviation",
                           correlation_search_space_smear_deviation))
    mapper_->setParamCorrelationSearchSpaceSmearDeviation(
        correlation_search_space_smear_deviation);

  // Setting Correlation Parameters, Loop Closure Parameters from the Parameter
  // Server

  double loop_search_space_dimension;
  if (private_nh_.getParam("loop_search_space_dimension",
                           loop_search_space_dimension))
    mapper_->setParamLoopSearchSpaceDimension(loop_search_space_dimension);

  double loop_search_space_resolution;
  if (private_nh_.getParam("loop_search_space_resolution",
                           loop_search_space_resolution))
    mapper_->setParamLoopSearchSpaceResolution(loop_search_space_resolution);

  double loop_search_space_smear_deviation;
  if (private_nh_.getParam("loop_search_space_smear_deviation",
                           loop_search_space_smear_deviation))
    mapper_->setParamLoopSearchSpaceSmearDeviation(
        loop_search_space_smear_deviation);

  // Setting Scan Matcher Parameters from the Parameter Server

  double distance_variance_penalty;
  if (private_nh_.getParam("distance_variance_penalty",
                           distance_variance_penalty))
    mapper_->setParamDistanceVariancePenalty(distance_variance_penalty);

  double angle_variance_penalty;
  if (private_nh_.getParam("angle_variance_penalty", angle_variance_penalty))
    mapper_->setParamAngleVariancePenalty(angle_variance_penalty);

  double fine_search_angle_offset;
  if (private_nh_.getParam("fine_search_angle_offset",
                           fine_search_angle_offset))
    mapper_->setParamFineSearchAngleOffset(fine_search_angle_offset);

  double coarse_search_angle_offset;
  if (private_nh_.getParam("coarse_search_angle_offset",
                           coarse_search_angle_offset))
    mapper_->setParamCoarseSearchAngleOffset(coarse_search_angle_offset);

  double coarse_angle_resolution;
  if (private_nh_.getParam("coarse_angle_resolution", coarse_angle_resolution))
    mapper_->setParamCoarseAngleResolution(coarse_angle_resolution);

  double minimum_angle_penalty;
  if (private_nh_.getParam("minimum_angle_penalty", minimum_angle_penalty))
    mapper_->setParamMinimumAnglePenalty(minimum_angle_penalty);

  double minimum_distance_penalty;
  if (private_nh_.getParam("minimum_distance_penalty",
                           minimum_distance_penalty))
    mapper_->setParamMinimumDistancePenalty(minimum_distance_penalty);

  bool use_response_expansion;
  if (private_nh_.getParam("use_response_expansion", use_response_expansion))
    mapper_->setParamUseResponseExpansion(use_response_expansion);

  marker_publisher_ = private_nh_.advertise<visualization_msgs::MarkerArray>(
      "visualization_marker_array", 1);
  // Set solver to be used in loop closure
  solver_ = new SpaSolver();
  mapper_->SetScanSolver(solver_);
  return true;
}

//得到一帧点云，就返回一个位姿
/*
ranges_data : 激光雷达坐标系下的点云数组
karto_pose : 最终激光里程计优化后的位姿
bag的发布频率过慢？ TF的频率跟不上scan的频率
*/
bool FrontEnd::Update(karto::LaserRangeFinder *laser,
                      const RangesData &ranges_data, karto::Pose2 &karto_pose) {

  karto::LocalizedRangeScan *range_scan =
      new karto::LocalizedRangeScan(laser->GetName(), ranges_data.readings);
  range_scan->SetOdometricPose(karto_pose);
  range_scan->SetCorrectedPose(karto_pose);
  // std::cout << "LaserRangeScan.size() = "
  //           << range_scan->GetNumberOfRangeReadings();
  // Add the localized range scan to the mapper
  bool processed;
  if ((processed = mapper_->Process(range_scan))) {
    // std::cout << "Pose: " << range_scan->GetOdometricPose() << " Corrected
    // Pose: " << range_scan->GetCorrectedPose() << std::endl;
    //矫正后的机器人位姿，这个位姿经过后端优化了吗？
    karto::Pose2 corrected_pose = range_scan->GetCorrectedPose();
    // corrected_pose.GetX(),corrected_pose.GetY(),corrected_pose.GetHeading();

    pose_vector_.push_back(corrected_pose.GetX());
    pose_vector_.push_back(corrected_pose.GetY());
    // std::cout << "size = " << ranges_data.angles.size() << std::endl;
    // for (int i = 0; i < ranges_data.angles.size(); i++) {
    //   std::cout << "," << ranges_data.angles[i] << std::endl;
    // }
    // LOG(INFO) << "x = " << corrected_pose.GetX()
    //           << "y = " << corrected_pose.GetY();
    KeyFrame new_key_frame;
    new_key_frame.corrected_pose = corrected_pose;
    new_key_frame.ranges_data = ranges_data;
    //直接处理新的关键帧
    UpdateWithNewFrame(new_key_frame);
  }
  std::cout << "processed = " << processed << std::endl;
  return processed;
}

/*
根据新的关键帧更新局部地图，并设置SetInputTarget(local_map_ptr_);
从这里返回关键帧，设置为外部可调用的函数
*/
bool FrontEnd::UpdateWithNewFrame(KeyFrame &new_key_frame) {
  //更新rosmap试试看
  mapping_ptr_->OccupanyMapping(new_key_frame);
  return true;
}
bool FrontEnd::GetRosMap(nav_msgs::OccupancyGrid &ros_map) {
  ros_map = mapping_ptr_->GetCurrentMap();

  return true;
}

void FrontEnd::publishPoseVisualization() {

  visualization_msgs::MarkerArray marray;

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
  for (uint32_t i = 0; i < pose_vector_.size() / 2; i += 2) {
    m.id = id;
    m.pose.position.x = pose_vector_[2 * i];
    m.pose.position.y = pose_vector_[2 * i + 1];
    marray.markers.push_back(visualization_msgs::Marker(m));
    id++;

    m.pose.position.x = pose_vector_[2 * (i + 1)];
    m.pose.position.y = pose_vector_[2 * (i + 1) + 1];
    marray.markers.push_back(visualization_msgs::Marker(m));
    id++;

    edge.points.clear();

    geometry_msgs::Point p;
    p.x = pose_vector_[2 * i];
    p.y = pose_vector_[2 * i + 1];
    edge.points.push_back(p);
    p.x = pose_vector_[2 * (i + 1)];
    p.y = pose_vector_[2 * (i + 1) + 1];
    edge.points.push_back(p);
    edge.id = id;

    marray.markers.push_back(visualization_msgs::Marker(edge));
    id++;
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