cmake_minimum_required(VERSION 3.0.2)
project(crawler_teleop)


find_package(catkin REQUIRED COMPONENTS
  rospy
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES crawler_teleop
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

