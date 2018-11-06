#!/bin/sh
export TURTLEBOT_GAZEBO_WORLD_FILE=$(rospack find world)/room.world
export TURTLEBOT_GAZEBO_MAP_FILE=$(rospack find world)/room_map.yaml

terminator -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
terminator -e "roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
terminator -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
terminator -e "rosrun pick_objects pick_objects" &
sleep 5
terminator -e "rosrun add_markers add_markers"