#include "mapping/front_end/front_end.hpp"
#include "global_defination/global_defination.h"

namespace mrobot_frame {

FrontEnd::FrontEnd() {
  // marker_publisher_ = private_nh_.advertise<visualization_msgs::MarkerArray>(
  //     "visualization_marker_array", 1);
  mapper_ = new karto::Mapper(); //能不能用共享指针 最好对Karto进行解耦
                                 //去除对Karto的依赖
  InitWithConfig();
}

FrontEnd::~FrontEnd() {
  if (mapper_)
    delete mapper_;
}
//初始化参数，匹配方法
bool FrontEnd::InitWithConfig() {

  std::string config_file_path = WORK_SPACE_PATH + "/config/front_end.yaml";
  YAML::Node config_node = YAML::LoadFile(config_file_path);

  std::cout << "-----------------前端匹配初始化-------------------"
            << std::endl;
  InitParam(config_node);

  // Set solver to be used in loop closure
  solver_ = new SpaSolver();
  mapper_->SetScanSolver(solver_);
  return true;
}

bool FrontEnd::InitParam(const YAML::Node &config_node) {
  // General Parameters
  mapper_->setParamUseScanMatching(config_node["use_scan_matching"].as<bool>());
  mapper_->setParamUseScanBarycenter(
      config_node["use_scan_barycenter"].as<bool>());
  mapper_->setParamMinimumTimeInterval(
      config_node["minimum_time_interval"].as<double>());
  mapper_->setParamMinimumTravelDistance(
      config_node["minimum_travel_distance"].as<double>());
  mapper_->setParamMinimumTravelHeading(
      config_node["minimum_travel_heading"].as<double>());
  mapper_->setParamScanBufferSize(config_node["scan_buffer_size"].as<int>());
  mapper_->setParamScanBufferMaximumScanDistance(
      config_node["scan_buffer_maximum_scan_distance"].as<double>());
  mapper_->setParamLinkMatchMinimumResponseFine(
      config_node["link_match_minimum_response_fine"].as<double>());
  mapper_->setParamLinkScanMaximumDistance(
      config_node["link_scan_maximum_distance"].as<double>());
  mapper_->setParamLoopSearchMaximumDistance(
      config_node["loop_search_maximum_distance"].as<double>());
  mapper_->setParamDoLoopClosing(config_node["do_loop_closing"].as<bool>());
  mapper_->setParamLoopMatchMinimumChainSize(
      config_node["loop_match_minimum_chain_size"].as<int>());
  mapper_->setParamLoopMatchMaximumVarianceCoarse(
      config_node["loop_match_maximum_variance_coarse"].as<double>());
  mapper_->setParamLoopMatchMinimumResponseCoarse(
      config_node["loop_match_minimum_response_coarse"].as<double>());
  mapper_->setParamLoopMatchMinimumResponseFine(
      config_node["loop_match_minimum_response_fine"].as<double>());
  // Setting Correlation Parameters from the Parameter Server
  mapper_->setParamCorrelationSearchSpaceDimension(
      config_node["correlation_search_space_dimension"].as<double>());
  mapper_->setParamCorrelationSearchSpaceResolution(
      config_node["correlation_search_space_resolution"].as<double>());
  mapper_->setParamCorrelationSearchSpaceSmearDeviation(
      config_node["correlation_search_space_smear_deviation"].as<double>());
  // Setting Correlation Parameters, Loop Closure Parameters from the Parameter
  // Server
  mapper_->setParamLoopSearchSpaceDimension(
      config_node["loop_search_space_dimension"].as<double>());
  mapper_->setParamLoopSearchSpaceResolution(
      config_node["loop_search_space_resolution"].as<double>());
  mapper_->setParamLoopSearchSpaceSmearDeviation(
      config_node["loop_search_space_smear_deviation"].as<double>());

  // Setting Scan Matcher Parameters from the Parameter Server
  mapper_->setParamDistanceVariancePenalty(
      config_node["distance_variance_penalty"].as<double>());
  mapper_->setParamAngleVariancePenalty(
      config_node["angle_variance_penalty"].as<double>());
  mapper_->setParamFineSearchAngleOffset(
      config_node["fine_search_angle_offset"].as<double>());
  mapper_->setParamCoarseSearchAngleOffset(
      config_node["coarse_search_angle_offset"].as<double>());
  mapper_->setParamCoarseAngleResolution(
      config_node["coarse_angle_resolution"].as<double>());
  mapper_->setParamMinimumAnglePenalty(
      config_node["minimum_angle_penalty"].as<double>());
  mapper_->setParamMinimumDistancePenalty(
      config_node["minimum_distance_penalty"].as<double>());
  mapper_->setParamUseResponseExpansion(
      config_node["use_response_expansion"].as<bool>());

  return true;
}

//得到一帧点云，就返回一个位姿
/*
ranges_data : 激光雷达坐标系下的点云数组
karto_pose : 最终激光里程计优化后的位姿
bag的发布频率过慢？ TF的频率跟不上scan的频率
*/
bool FrontEnd::Update(karto::LaserRangeFinder *laser,
                      const LaserScanData &ranges_data,
                      karto::Pose2 &karto_pose) {
  // ResetParam(); //重置关键帧

  karto::LocalizedRangeScan *range_scan = new karto::LocalizedRangeScan(
      laser->GetName(), ranges_data.range_readings);
  range_scan->SetOdometricPose(karto_pose);
  range_scan->SetCorrectedPose(karto_pose);

  // Add the localized range scan to the mapper
  bool processed;
  if ((processed = mapper_->Process(range_scan))) {
    // has_new_key_frame_ = true; //关键帧
    // std::cout << "Pose: " << range_scan->GetOdometricPose() << " Corrected
    // Pose: " << range_scan->GetCorrectedPose() << std::endl;
    karto::Pose2 corrected_pose = range_scan->GetCorrectedPose();

    pose_vector_.push_back(corrected_pose.GetX());
    pose_vector_.push_back(corrected_pose.GetY());

    KeyFrame new_key_frame;
    new_key_frame.index = (unsigned int)key_frames_deque_.size();
    new_key_frame.corrected_pose = corrected_pose;
    new_key_frame.ranges_data = ranges_data;
    // key_frames_deque_.push_back(new_key_frame); 目前不需要所有的关键帧
    current_key_frame_ = new_key_frame;
  }
  return processed;
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

void FrontEnd::ResetParam() { has_new_key_frame_ = false; }

bool FrontEnd::HasNewKeyFrame() { return has_new_key_frame_; }

void FrontEnd::GetLatestKeyFrame(KeyFrame &key_frame) {
  key_frame = current_key_frame_;
}

} // namespace mrobot_frame