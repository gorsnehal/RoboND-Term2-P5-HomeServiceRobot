#!/bin/sh
#Launch turtlebot in U world
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/worlds/UWorld_SG.world " &
sleep 5
#Launch gmapping with custom xml
xterm  -e  " roslaunch turtlebot_gazebo gmapping_demo.launch custom_gmapping_launch_file:=/home/workspace/catkin_ws/src/wall_follower/launch/asus_xtion_pro_gmapping.launch.xml " &
sleep 5
#Launch rviz
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5
#Launch wall follower node
xterm  -e  " rosrun wall_follower wall_follower" 