# RoboND-Term2-P5-HomeServiceRobot
Udacity Robotics ND - Term 2 - Project 5 : Home Service Robot

Build a Home Service Robot in ROS.

## Installation, Build & Running Scripts ##

* Download the repository folder 'src' to catkin workspace folder (Assumed that Linux 16.04 and ROS Kinetic installation with catkin_ws folder is already there)
Use following command to clone entire repository, and then copy content of 'src' folder to your catkin_ws/src folder.
```
$ git clone https://github.com/gorsnehal/RoboND-Term2-P5-HomeServiceRobot.git
```

* Perform following steps for dependencies
```
$ cd ~/catkin_ws
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-navigation
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
$ cd ~/catkin_ws/src/shellScripts
$ chmod +x test_slam.sh
$ chmod +x add_markers.sh
$ chmod +x test_navigation.sh
$ chmod +x pick_objects.sh
$ chmod +x wall_follower.sh
$ chmod +x home_service.sh
```

* Compile & Build the catkin workspace
```
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
```

* To manually test SLAM (Optional)
```
$ cd ~/catkin_ws/src/shellScripts
$ ./test_slam.sh
```

* Map the environment with wall following algorithm (Optional)
```
$ cd ~/catkin_ws/src/shellScripts
$ ./wall_follower.sh
```

Once you map the U shape environment, save the map to src/worlds folder (I have already provided U world there).
```
rosrun map_server map_saver -f ~/catkin_ws/src/worlds/my_map
```

* Testing Navigation (Optional)
```
$ cd ~/catkin_ws/src/shellScripts
$ ./test_navigation.sh
```

* Home Service Robot auto navigation and marker displays on the built map
```
$ cd ~/catkin_ws/src/shellScripts
$ ./home_service.sh
```

If you are building a map of your own world and deploy Home service robot in it, don't forget to modify script files.