#include "mapping/front_end/front_end_flow.hpp"

using namespace mrobot_frame;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "mrobot_frame_front_end_node");
  ros::NodeHandle nh;

  // std::string cloud_topic; //订阅的点云话题
  // nh.param<std::string>("cloud_topic", cloud_topic, "scan");
  std::string scan_topic; //订阅的点云话题
  nh.param<std::string>("scan_topic", scan_topic, "scan");

  std::shared_ptr<FrontEndFlow> front_end_flow_ptr =
      std::make_shared<FrontEndFlow>(nh, scan_topic);

  ros::Rate rate(10); // 10 hz
  while (ros::ok()) {
    //不会一直等待回调函数的到来
    ros::spinOnce();
    front_end_flow_ptr->Run();

    rate.sleep(); //控制循环的频率
  }

  return 0;
}