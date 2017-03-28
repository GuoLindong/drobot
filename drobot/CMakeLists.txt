cmake_minimum_required(VERSION 2.8.3)
project(drobot)

find_package(catkin REQUIRED COMPONENTS drobot_msgs
  diagnostic_updater roslaunch roslint roscpp sensor_msgs)
find_package(Boost REQUIRED COMPONENTS chrono)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES legacy_wrapper
  CATKIN_DEPENDS drobot_msgs roscpp sensor_msgs
  DEPENDS Boost
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(legacy_wrapper
  src/legacy_wrapper/crc.cpp
  src/legacy_wrapper/Logger.cpp
  src/legacy_wrapper/Message.cpp
  src/legacy_wrapper/Message_data.cpp
  src/legacy_wrapper/Message_request.cpp
  src/legacy_wrapper/Message_cmd.cpp
  src/legacy_wrapper/Transport.cpp
  src/legacy_wrapper/Number.cpp
  src/legacy_wrapper/linux_serial.cpp
)

add_executable(drobot_node
  src/drobot_base.cpp src/drobot_hardware.cpp src/legacy_wrapper.cpp
)
target_link_libraries(drobot_node legacy_wrapper ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(drobot_node drobot_msgs_gencpp)

roslint_cpp(
  src/drobot_base.cpp
  src/drobot_hardware.cpp 
  include/drobot_base/drobot_hardware.h
)

roslaunch_add_file_check(launch)

install(TARGETS legacy_wrapper drobot_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
