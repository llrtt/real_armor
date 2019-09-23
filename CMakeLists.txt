cmake_minimum_required(VERSION 2.8.3)
project(send_msg)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)
find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(include ${catkin_INCLUDE_DIRS})
 
## Declare ROS messages and services
add_message_files(DIRECTORY msg FILES rt.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)
add_executable(send_msg src/send_msg.cpp)
target_link_libraries(send_msg ${catkin_LIBRARIES})
target_link_libraries( send_msg ${OpenCV_LIBS} )








