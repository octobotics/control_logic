cmake_minimum_required(VERSION 2.8.3)
project(control_logic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  behaviortree_cpp_v3
  tf2_geometry_msgs
  geometry_msgs
  behaviortree_cpp
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs std_srvs behaviortree_cpp_v3 geometry_msgs tf2_geometry_msgs behaviortree_cpp
)




include_directories(
  ${catkin_INCLUDE_DIRS}
)

#imu_subscriber_node executable
add_executable(behavior_tree_node src/main3.cpp)
target_link_libraries(behavior_tree_node ${catkin_LIBRARIES})
add_executable(imu_subscriber_node src/main.cpp)
target_link_libraries(imu_subscriber_node ${catkin_LIBRARIES})
