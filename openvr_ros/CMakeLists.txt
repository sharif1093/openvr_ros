cmake_minimum_required(VERSION 2.8.3)
project(openvr_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  genmsg
  std_msgs
  geometry_msgs
  message_generation
)

find_library(OPENVR_LIBRARY openvr_api HINTS /usr/local/lib/)
## Alternatively, the following instructions could be used:
# set(OPENVR "~/openvr")
# include_directories("${OPENVR}/headers")
# link_directories("${OPENVR}/lib/linux64")

add_message_files(
  FILES
  TrackedDeviceClass.msg
  TrackedDevicePose.msg
  TrackedDeviceResult.msg
  TrackedDeviceRole.msg
  TrackedDeviceHeader.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES openvr_ros
 CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(track_publisher_node src/track_publisher.cpp)
add_dependencies(track_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(track_publisher_node ${catkin_LIBRARIES} ${OPENVR_LIBRARY} ${CMAKE_DL_LIBS})
