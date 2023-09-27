#include "global_defination//global_defination.h"
#include "glog/logging.h"
#include "ros/init.h"
#include <ros/ros.h>

#include "mapping/front_end/front_end_flow.hpp"

using namespace mrobot_frame;

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_log_dir = WORK_SPACE_PATH + "/log";
  FLAGS_alsologtostderr = 1;
  FLAGS_colorlogtostderr = true; //是否启用不同颜色显示

  ros::init(argc, argv, "mrobot_frame_front_end_node");
  ros::NodeHandle nh;

  std::string cloud_topic, laser_odom_topic;
  nh.param<std::string>("cloud_topic", cloud_topic, "scan");
  nh.param<std::string>("laser_odom_topic", laser_odom_topic,
                        "laser_odom_pose");

  std::shared_ptr<FrontEndFlow> front_end_flow_ptr =
      std::make_shared<FrontEndFlow>(nh, cloud_topic, laser_odom_topic);

  ros::Rate rate(10); // 10 hz
  while (ros::ok()) {
    //不会一直等待回调函数的到来
    ros::spinOnce();
    front_end_flow_ptr->Run();

    rate.sleep(); //控制循环的频率
  }

  return 0;
}