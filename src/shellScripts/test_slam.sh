#!/bin/sh
#Launch turtlebot in U world
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/worlds/UWorld_SG.world " &
sleep 5
#Launch gmapping demo
xterm  -e  " roslaunch turtlebot_gazebo gmapping_demo.launch" & ## rosrun gmapping slam_gmapping " & 
sleep 5
#Launch rviz
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5
#Launch teleop
xterm  -e  " roslaunch turtlebot_teleop keyboard_teleop.launch" 