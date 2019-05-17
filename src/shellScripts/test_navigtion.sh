#!/bin/sh
#Launch turtlebot in U world
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/worlds/UWorld_SG.world " &
sleep 5
#Launch MC Localization - AMCL
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/catkin_ws/src/worlds/uworld.yaml " & 
sleep 5
#Launch rviz
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch "
