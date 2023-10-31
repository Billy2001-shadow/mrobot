cmake_minimum_required(VERSION 2.8.3)
project(mrobot_frame)

SET(CMAKE_BUILD_TYPE "Release")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++14)
add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  
  tf #后面是否需要换成TF2？ tf2 tf2_ros tf2_geometry_msgs
  
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation 
  
  open_karto
  sparse_bundle_adjustment
  # eigen_conversions
  # std_srvs         #暂时不需要
)

add_message_files(
  FILES
  keyframemsg.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
 )


set(ALL_TARGET_LIBRARIES "")

include(cmake/glog.cmake)
include(cmake/YAML.cmake)
include(cmake/g2o.cmake)

include_directories(include 
    ${catkin_INCLUDE_DIRS}
    "/usr/include/eigen3"
)
include(cmake/global_defination.cmake)
catkin_package(
  CATKIN_DEPENDS message_runtime
)



file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/apps/*_node.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})


# add_executable(${PROJECT_NAME}_data_pretreat_node src/apps/data_pretreat_node.cpp ${ALL_SRCS})
# add_dependencies(${PROJECT_NAME}_data_pretreat_node ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME}_data_pretreat_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(${PROJECT_NAME}_front_end_node src/apps/front_end_node.cpp ${ALL_SRCS})
add_dependencies(${PROJECT_NAME}_front_end_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_front_end_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(${PROJECT_NAME}_mapping_node src/apps/mapping_node.cpp ${ALL_SRCS})
add_dependencies(${PROJECT_NAME}_mapping_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_mapping_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})