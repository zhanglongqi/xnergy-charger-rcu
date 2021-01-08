cmake_minimum_required(VERSION 3.0.2)
project(xnergy_charger_rcu)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  diagnostic_msgs
  diagnostic_updater
)

catkin_python_setup()

add_message_files(
  DIRECTORY msg
  FILES
  ChargerState.msg
)

add_action_files(
  DIRECTORY action
  FILES
  Charge.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs sensor_msgs
)
catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs rospy sensor_msgs std_msgs message_runtime diagnostic_updater std_srvs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)