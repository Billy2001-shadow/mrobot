#include "mapping/mapping/mapping_flow.hpp"
#include <ros/ros.h>

using namespace mrobot_frame;
int main(int argc, char *argv[]) {

  ros::init(argc, argv, "mrobot_frame_mapping_node");
  ros::NodeHandle nh;

  std::shared_ptr<MappingFlow> mapping_flow_ptr =
      std::make_shared<MappingFlow>(nh);

  ros::Rate rate(100);

  while (ros::ok()) {
    ros::spinOnce();

    mapping_flow_ptr->Run();

    rate.sleep();
  }

  return 0;
}