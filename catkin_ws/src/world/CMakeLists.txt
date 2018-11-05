cmake_minimum_required(VERSION 2.8.3)
project(world)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
  DEPENDS
)

###########
## Build ##
###########


#############
## Install ##
#############

## Mark executables and/or libraries for installation

## Mark all other useful stuff for installation