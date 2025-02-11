cmake_minimum_required(VERSION 3.0.2)
project(ufs_state_machine)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  costmap_2d
  geometry_msgs
  tf2_ros
)

catkin_package(
	#  INCLUDE_DIRS include
  LIBRARIES ufs_state_machine
  CATKIN_DEPENDS roscpp std_msgs nav_msgs costmap_2d geometry_msgs tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(state_machine_node src/state_machine_node.cpp)

target_link_libraries(state_machine_node
  ${catkin_LIBRARIES}
)
