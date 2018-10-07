# RoboND_HomeServiceRobot
Udacity Robot Sodtware Engineer Nanodegree Term 2 Project

## Project folder structure
```
catkin_ws/src
    |-- add_markers
    |-- Map
        |-- generated map files (pgm and yaml)
    |-- pick_objects
    |-- RvizConfig
    |-- ShellScripts
        |-- scripts to run each exercises
    |-- slam_gmapping
    |-- turtlebot
    |-- turtlebot_apps
    |-- turtlebot_interactions
    |-- turtlebot_simulator
    |-- wall_follower
    |-- World
        |-- world file crate in Building Editor
```
## Exercises to run
After importing a world environement crate by Building Editor in Gazebo, manually teleoperate robot movement and SLAM .
### Test SLAM
```
/ShellScripts/test_slam.sh
```
### Wall follower
Autonomously drive the robot to map the environment.
```
/ShellScripts/wall_follower.sh
```
### Test navigation
Manually command robot by using the 2D Nav Goal arrow in rviz to move the robot.
```
/ShellScripts/test_navigation.sh
```
### Pick objects
Command robot to move to desired pick and drop off locations.
```
/ShellScripts/pick_objects.sh
```
### Add markers
Testing showing/hiding objects in the environment to replicate object picked/dropped off by the robot.
```
/ShellScripts/add_markers.sh
```
### Home service robot
Combination of pick_objects and add_markers to simualtion robot movement and object show/hide.
```
/ShellScripts/home_service.sh
```
