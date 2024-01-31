cmake_minimum_required(VERSION 2.8.3)
project(control_logic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  behaviortree_cpp_v3
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs std_srvs behaviortree_cpp_v3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#imu_subscriber_node executable
add_executable(behavior_tree_node src/main3.cpp)
target_link_libraries(behavior_tree_node ${catkin_LIBRARIES})
