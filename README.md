# mrobot
2D激光里程计框架

该激光SLAM框架仅支持2D激光雷达数据



数据预处理模块的需求：

- 订阅原始激光雷达的消息 sensor_msgs::
- 发布激光雷达点云消息(预处理之后的)

不能通过ros的标准格式来进行前端和建图模块的通信了，那前端得到的优化后的位姿和range_readings该如何喂给建图模块呢？

把readings写到txt文件中？



```
visualization_msgs::MarkerArray marray; 的用法
```

![image-20230917171139204](/home/cw/test/test_mrobot_ws/src/mrobot/README.assets/image-20230917171139204.png)
