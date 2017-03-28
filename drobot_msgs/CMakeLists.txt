cmake_minimum_required(VERSION 2.8.3)
project(drobot_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  DrobotStatus.msg
  DrobotControl.msg
  DrobotOdom.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS std_msgs message_runtime)
