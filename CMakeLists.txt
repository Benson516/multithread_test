cmake_minimum_required(VERSION 2.8.3)
project(multithread_test)
set (CMAKE_CXX_STANDARD 14) 

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
#
  msgs # ITRI package: "msgs"
#
  pcl_conversions
  pcl_ros
#
  image_transport
  cv_bridge
)




################################################
## Declare ROS messages, services and actions ##
################################################



# file(GLOB SOURCES_MESSAGE *.msg)
# add_message_files(
#   ${SOURCES_MESSAGE}
# )

generate_messages(
  DEPENDENCIES
  std_msgs
  msgs
)


find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  DEPENDS Eigen OpenCV
)


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################



###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES multithread_test
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime image_transport sensor_msgs cv_bridge message_runtime 
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  include/async_buffer/
  include/ROS_interface/
  include/ROS_interface/include
  .
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})





# Libraries
add_library (async_buffer STATIC include/async_buffer/async_buffer.cpp) 
# target_link_libraries(async_buffer ${ALL_LIBS}  ) 

add_library (ROS_interface STATIC include/ROS_interface/ROS_interface.cpp) 
target_link_libraries(ROS_interface async_buffer) 


add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(listener_async_spinner src/listener_async_spinner.cpp)
target_link_libraries(listener_async_spinner ${catkin_LIBRARIES})

add_executable(listener_async_spinner_async_buffer src/listener_async_spinner_async_buffer.cpp )
target_link_libraries(listener_async_spinner_async_buffer ${catkin_LIBRARIES} async_buffer)

#
add_executable(listener_ROS_interface src/listener_ROS_interface.cpp )
target_link_libraries(listener_ROS_interface ROS_interface ${OpenCV_LIBS} ${catkin_LIBRARIES}) # Note: the order is important
#
add_executable(talker_ROS_interface src/talker_ROS_interface.cpp )
target_link_libraries(talker_ROS_interface ROS_interface ${OpenCV_LIBS} ${catkin_LIBRARIES}) # Note: the order is important



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_multithread_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)



