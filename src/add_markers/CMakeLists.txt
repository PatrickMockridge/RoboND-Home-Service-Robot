cmake_minimum_required(VERSION 2.8.3)
project(add_markers)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp visualization_msgs geometry_msgs tf)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp visualization_msgs geometry_msgs tf
  DEPENDS
)

###########
## Build ##
###########

add_executable(add_markers src/add_markers.cpp)
target_link_libraries(add_markers ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS add_markers
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark all other useful stuff for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)