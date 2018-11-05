#!/bin/sh
export TURTLEBOT_GAZEBO_WORLD_FILE=$(rospack find world)/room.world

terminator -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
terminator -e "roslaunch turtlebot_gazebo gmapping_demo.launch; rosparam set /slam_gmapping 'linear_update:0.1 angular_update:.1 particles:200 minimum_score:1000'" &
sleep 5
terminator -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
terminator -e "rosrun wall_follower wall_follower"
#terminator -e "roslaunch turtlebot_teleop keyboard_teleop.launch"