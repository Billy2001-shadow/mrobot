#include "glog/logging.h"
#include <ros/ros.h>

#include "global_defination/global_defination.h"
#include "mapping/mapping/mapping_flow.hpp"

using namespace mrobot_frame;

int main(int argc, char *argv[]) {

  google::InitGoogleLogging(argv[0]);
  FLAGS_log_dir = WORK_SPACE_PATH + "/log";
  FLAGS_alsologtostderr = 1;
  FLAGS_colorlogtostderr = true; //是否启用不同颜色显示

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