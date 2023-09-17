#include "glog/logging.h"
#include <ros/ros.h>

#include "data_pretreat/data_pretreat_flow.hpp"
#include "global_defination//global_defination.h"

using namespace mrobot_frame;

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
  FLAGS_alsologtostderr = 1;

  ros::init(argc, argv, "mrobot_frame_data_pretreat_node"); // 节点的名字
  ros::NodeHandle nh;

  std::string scan_topic, cloud_topic;
  nh.param<std::string>("scan_topic", scan_topic, "base_scan");
  nh.param<std::string>("cloud_topic", cloud_topic, "pretreat_cloud");
  std::shared_ptr<DataPretreatFlow> data_pretreat_flow_ptr =
      std::make_shared<DataPretreatFlow>(nh, scan_topic, cloud_topic);

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();

    data_pretreat_flow_ptr->Run(); //订阅程序和数据输出程序分开

    rate.sleep();
  }

  return 0;
}
