cmake_minimum_required(VERSION 2.8.3)
project(pick_objects)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp move_base_msgs actionlib visualization_msgs geometry_msgs)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp move_base_msgs actionlib visualization_msgs geometry_msgs
  DEPENDS
)

###########
## Build ##
###########

add_executable(pick_objects src/pick_objects.cpp)
target_link_libraries(pick_objects ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS pick_objects
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark all other useful stuff for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)