cmake_minimum_required(VERSION 2.8.3)
project(dynamic_obstacle_avoidance)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
  geometry_msgs
  duckietown_msgs
)

add_message_files(
 	FILES
 	dynamic_obstacle.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  duckietown_msgs
)

catkin_package(
#  INCLUDE_DIRS include
   LIBRARIES duckietown_msgs
   CATKIN_DEPENDS
   geometry_msgs
   roscpp rospy
   std_msgs
   message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
