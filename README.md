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





### 目前框架的缺陷

#### 前端

- 

#### 后端

- 后端还未从Karto slam中解耦出来

#### 建图

- 建图部分没有根据点云的范围自适应调整地图大小，现在是初始化一个很大的空白地图
- 建图部分加一个保存栅格地图的操作(pgm格式)



未发布map到odom下的坐标转换关系，目前只是前端+建图

没有加入回环

障碍物边界不清晰，有散射的可通行区域





不能适应不同的数据集

- basic_localization_stage_indexed.bag报错
  - LaserRangeScan contains 1081 range readings, expected 1081
  - 这个数据集一共有1081个点





```
Mapper.cpp中出现的
kt_bool LaserRangeFinder::Validate(SensorData *pSensorData) {
  LaserRangeScan *pLaserRangeScan = dynamic_cast<LaserRangeScan *>(pSensorData);

  // verify number of range readings in LaserRangeScan matches the number of
  // expected range readings
  if (pLaserRangeScan->GetNumberOfRangeReadings() !=
      GetNumberOfRangeReadings() - 1) {
    std::cout << "LaserRangeScan contains "
              << pLaserRangeScan->GetNumberOfRangeReadings()
              << " range readings, expected " << GetNumberOfRangeReadings()
              << std::endl;
    return false;
  }

  return true;
}
```

