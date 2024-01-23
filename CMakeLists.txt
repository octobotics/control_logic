cmake_minimum_required(VERSION 2.8.3)
project(control_logic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs std_srvs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#imu_subscriber_node executable
add_executable(imu_subscriber_node src/main.cpp)
target_link_libraries(imu_subscriber_node ${catkin_LIBRARIES})