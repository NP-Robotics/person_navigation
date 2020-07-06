# person_navigation
ROS package to translate from the output of person_tracking node to cmd_vel movement.

Robot moves to target when target is selected. When target is not selected robot spins in the last movement direction.
# Setup
## Hardware
- Intel Realsense 2 
## Requirements
- Python3
- rospy
- OpenCV
## ROS
- ROS Melodic
- [cv_bridge](https://github.com/ros-perception/vision_opencv)
- [realsense2_camera](https://github.com/IntelRealSense/realsense-ros)
- [person_tracking](https://github.com/NP-Robotics/person_tracking_ros)
## Installation
Setup ROS Melodic and your catkin_ws if you have not done so.

Install the required python 3 modules using pip3 or a virtualenv. Do not use Anaconda. Git clone this repository into src of your catkin_ws.
 ```
cd catkin_ws/src
git clone https://github.com/NP-Robotics/person_navigation.git
cd person_navigation
```
`catkin_make` the package
```
cd ..
catkin_make
```

# Running
## Initialization
Start `roscore`
```
roscore
```
Start realsense node for depth and color image aligned
```
roslaunch realsense2_camera rs_aligned_depth.launch
```
Start person_tracking node in service mode.
```
roslaunch person_tracking person_tracker_srv.launch
```
Start person_navigation node.
```
roslaunch person_navigation person_navigation.launch
```
person_navigation should be working and publishing movement values to cmd_vel.
