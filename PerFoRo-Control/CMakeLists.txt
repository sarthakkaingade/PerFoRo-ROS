cmake_minimum_required(VERSION 2.8.3)
project(PerFoRoControl)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg message_generation)

## Add message files
add_message_files(
  FILES
  MODE.msg
  NavigatePerFoRo.msg
)

## Generate message files
generate_messages(
   DEPENDENCIES
   std_msgs
)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(PerFoRoControl src/PerFoRoControl.cpp)
target_link_libraries(PerFoRoControl ${catkin_LIBRARIES})
add_dependencies(PerFoRoControl PerFoRoControl_generate_messages_cpp)