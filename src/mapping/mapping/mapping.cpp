#include "global_defination/global_defination.h"
#include "glog/logging.h"
#include "mapping/mapping/mapping_flow.hpp"

namespace mrobot_frame {
Mapping::Mapping() {
  InitWithConfig(); //载入栅格地图参数
}

bool Mapping::InitWithConfig() {
  std::string config_file_path = WORK_SPACE_PATH + "/config/mapping.yaml";
  YAML::Node config_node = YAML::LoadFile(config_file_path);

  std::cout << "-----------------建图初始化-------------------" << std::endl;
  InitParam(config_node);

  return true;
}

bool Mapping::InitParam(const YAML::Node &config_node) {

  mapParams.width = config_node["gridmap_width"].as<int>();
  mapParams.height = config_node["gridmap_height"].as<int>();
  mapParams.resolution = config_node["gridmap_resolution"].as<double>();

  //每次被击中的log变化值，覆盖栅格建图算法需要的参数
  mapParams.log_free = config_node["gridmap_log_free"].as<double>();
  mapParams.log_occ = config_node["gridmap_log_occ"].as<double>();

  //每个栅格的最大最小值．
  mapParams.log_max = config_node["gridmap_log_max"].as<double>();
  mapParams.log_min = config_node["gridmap_log_min"].as<double>();

  mapParams.origin_x = config_node["gridmap_origin_x"].as<double>();
  mapParams.origin_y = config_node["gridmap_origin_y"].as<double>();

  //地图的原点，在地图的正中间
  mapParams.offset_x = config_node["gridmap_offset_x"].as<int>(); // 500
  mapParams.offset_y = config_node["gridmap_offset_y"].as<int>(); // 500

  //选择建图的方法
  mapping_method_ = config_node["mapping_method"].as<std::string>();
  std::cout << "建图选择方法为：" << mapping_method_ << std::endl;

  pMap = new unsigned char
      [mapParams.width *
       mapParams.height]; // pMap为指向数组首个元素的指针(后面主要维护pMap)

  //计数建图算法需要的参数
  //每个栅格被激光击中的次数
  pMapHits = new unsigned long[mapParams.width * mapParams.height];
  //每个栅格被激光通过的次数
  pMapMisses = new unsigned long[mapParams.width * mapParams.height];

  // TSDF建图算法需要的参数
  pMapW = new unsigned long[mapParams.width * mapParams.height];
  pMapTSDF = new double[mapParams.width * mapParams.height];

  //初始化
  for (int i = 0; i < mapParams.width * mapParams.height; i++) {
    pMap[i] = 50;
    pMapHits[i] = 0;
    pMapMisses[i] = 0;
    pMapW[i] = 0;
    pMapTSDF[i] = -1;
  }
  return true;
}

void Mapping::OccupanyMapping(KeyFrame &current_keyframe) {

  //机器人位姿角归一化
  if (current_keyframe.corrected_pose[2] > 3.1415926) {
    current_keyframe.corrected_pose[2] -= 2 * 3.1415926;
  }

  else if (current_keyframe.corrected_pose[2] < -3.1415926) {
    current_keyframe.corrected_pose[2] += 2 * 3.1415926;
  }

  Eigen::Vector3d robotPose(current_keyframe.corrected_pose[0],
                            current_keyframe.corrected_pose[1],
                            current_keyframe.corrected_pose[2]);

  //激光雷达的栅格地图坐标
  GridIndex robotIndex = ConvertWorld2GridIndex(robotPose(0), robotPose(1));

  //每一个激光束
  for (int id = 0; id < current_keyframe.scan_data.range_readings.size();
       id++) {

    double dist = current_keyframe.scan_data.range_readings[id];
    double angle = current_keyframe.scan_data.angles_readings[id];

    if (std::isinf(dist) || std::isnan(dist) || (std::abs(dist) >= 20))
      continue;

    double theta = robotPose(2);
    double laser_x = dist * cos(angle);
    double laser_y = dist * sin(angle);

    double world_x = cos(theta) * laser_x - sin(theta) * laser_y + robotPose(0);

    double world_y = sin(theta) * laser_x + cos(theta) * laser_y + robotPose(1);

    GridIndex beamPointIndex = ConvertWorld2GridIndex(world_x, world_y);
    std::vector<GridIndex> beamTraceindexes = TraceLine(
        robotIndex.x, robotIndex.y, beamPointIndex.x, beamPointIndex.y);

    if (mapping_method_ == "cover_grid") {
      for (auto index : beamTraceindexes) {
        if (isValidGridIndex(index)) {
          int tmpLinearIndex =
              GridIndexToLinearIndex(index); // tmpLinearIndex一直为1？
          if (pMap[tmpLinearIndex] == 0)
            continue;
          pMap[tmpLinearIndex] += mapParams.log_free;
        } else {
          // std::cerr << "index if invalid!!!" << std::endl;
        }
      } // for

      if (isValidGridIndex(beamPointIndex)) {
        int tmpLinearIndex = GridIndexToLinearIndex(beamPointIndex);
        pMap[tmpLinearIndex] += mapParams.log_occ;
        if (pMap[tmpLinearIndex] >= 100)
          pMap[tmpLinearIndex] = 100;
      } else {
        // std::cerr << "beamPointIndex if invalid!!!" << std::endl;
      }
    } // if

    else if (mapping_method_ == "count_mapping") {
      for (auto index : beamTraceindexes) {
        if (isValidGridIndex(index)) {
          int tmpLinearIndex = GridIndexToLinearIndex(index);
          ++pMapMisses[tmpLinearIndex];
        } else {
          // std::cerr << "index if invalid!!!" << std::endl;
        }
      } // for
      if (isValidGridIndex(beamPointIndex)) {
        int tmpLinearIndex = GridIndexToLinearIndex(beamPointIndex);
        ++pMapHits[tmpLinearIndex];
      } else {
        // std::cerr << "beamPointIndex if invalid!!!" << std::endl;
      }

    } // else if

    else {
      ROS_ERROR("choose correct mapping method");
    }

  } //一束激光雷达数据中的数据点遍历
}

/**
 * Increments all the grid cells from (x0, y0) to (x1, y1);
 * //不包含(x1,y1)
 * 2D画线算法　来进行计算两个点之间的grid cell
 * @param x0
 * @param y0
 * @param x1
 * @param y1 robotIndex.x,robotIndex.y,beamPointIndex.x,beamPointIndex.y
 */
std::vector<GridIndex> Mapping::TraceLine(int x0, int y0, int x1, int y1) {
  GridIndex tmpIndex;
  std::vector<GridIndex> gridIndexVector;

  bool steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) // k>1或k<-1
  {
    std::swap(x0, y0);
    std::swap(x1, y1);
  }
  if (x0 > x1) {
    std::swap(x0, x1);
    std::swap(y0, y1);
  }

  int deltaX = x1 - x0;
  int deltaY = abs(y1 - y0);
  int error = 0;
  int ystep;
  int y = y0;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;
  }

  int pointX;
  int pointY;
  for (int x = x0; x <= x1; x++) {
    if (steep) {
      pointX = y;
      pointY = x;
    } else {
      pointX = x;
      pointY = y;
    }

    error += deltaY;

    if (2 * error >= deltaX) {
      y += ystep;
      error -= deltaX;
    }

    //不包含最后一个点．
    if (pointX == x1 && pointY == y1)
      continue;

    //保存所有的点
    tmpIndex.SetIndex(pointX, pointY);

    gridIndexVector.push_back(tmpIndex);
  }

  return gridIndexVector;
}

//从世界坐标系转换到栅格坐标系
GridIndex Mapping::ConvertWorld2GridIndex(double x, double y) {
  GridIndex index;
  //地图的正中间为栅格坐标系的中心，（mapParams.offset_x，mapParams.offset_y）
  index.x = std::ceil((x - mapParams.origin_x) / mapParams.resolution) +
            mapParams.offset_x;
  index.y = std::ceil((y - mapParams.origin_y) / mapParams.resolution) +
            mapParams.offset_y;

  return index;
}

int Mapping::GridIndexToLinearIndex(GridIndex index) {
  int linear_index;
  linear_index = index.x + index.y * mapParams.width;
  return linear_index;
}

//判断index是否有效
bool Mapping::isValidGridIndex(GridIndex index) {
  if (index.x >= 0 && index.x < mapParams.width && index.y >= 0 &&
      index.y < mapParams.height)
    return true;

  return false;
}

void Mapping::DestoryMap() {
  if (pMap != NULL)
    delete pMap;
}

nav_msgs::OccupancyGrid Mapping::GetCurrentMap() {
  // std::cout << "开始建图，请稍后..." << std::endl;
  nav_msgs::OccupancyGrid Gridmap;

  Gridmap.info.resolution =
      mapParams.resolution; //这个通过yaml文件给mapParams.resolution置位
  Gridmap.info.origin.position.x = 0.0;
  Gridmap.info.origin.position.y = 0.0;
  Gridmap.info.origin.position.z = 0.0;
  Gridmap.info.origin.orientation.x = 0.0;
  Gridmap.info.origin.orientation.y = 0.0;
  Gridmap.info.origin.orientation.z = 0.0;
  Gridmap.info.origin.orientation.w = 1.0;

  Gridmap.info.origin.position.x =
      mapParams.origin_x; //这个通过yaml文件给mapParams.resolution置位
  Gridmap.info.origin.position.y =
      mapParams.origin_y; //这些参数应该在mapping类下搞定的
  Gridmap.info.width = mapParams.width;
  Gridmap.info.height = mapParams.height;
  Gridmap.data.resize(Gridmap.info.width * Gridmap.info.height);

  // 0~100
  // int cnt0, cnt1, cnt2;
  // cnt0 = cnt1 = cnt2 = 100;
  // map_mutex_.lock();
  if (mapping_method_ == "cover_grid") {
    for (int i = 0; i < Gridmap.info.width * Gridmap.info.height; i++) {
      if (pMap[i] == 50) //？
      {
        Gridmap.data[i] = -1.0; // Unknown is -1.
      } else {
        Gridmap.data[i] = pMap[i]; // unsigned char *pMap; //指向unsigned
        // char类型元素的指针
      }
    }
  }

  else if (mapping_method_ == "count_mapping") {
    for (int i = 0; i < Gridmap.info.width * Gridmap.info.height; i++) {
      if (pMapHits[i] + pMapMisses[i] != 0) {
        Gridmap.data[i] =
            (double)pMapHits[i] / (pMapHits[i] + pMapMisses[i]) * 100;
        if (Gridmap.data[i] >= 40)
          Gridmap.data[i] = 100;
        if (Gridmap.data[i] <= 20)
          Gridmap.data[i] = 0;
      } else {
        Gridmap.data[i] = -1;
      }

      // if ((pMapHits[i] + pMapMisses[i]) != 0) {
      //   Gridmap.data[i] =
      //       (double)pMapHits[i] / (pMapHits[i] + pMapMisses[i]) * 100;
      //   if (Gridmap.data[i] >= 35) {
      //     Gridmap.data[i] = 100;
      //   } else {
      //     Gridmap.data[i] = -1;
      //   }
      // }
    }
  }
  // map_mutex_.unlock();
  return Gridmap;
}

//保存地图的服务
bool Mapping::SaveMap() {

  std::cout << "地图保存完成，地址是：" << std::endl;

  return true;
}

} // namespace mrobot_frame