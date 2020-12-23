# UAV ROS Tracker

| Ubuntu 18.04  | Ubuntu 20.04|
|---------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------|
 [![Melodic](https://github.com/lmark1/uav_ros_tracker/workflows/Melodic/badge.svg)](https://github.com/lmark1/uav_ros_tracker/actions) | [![Noetic](https://github.com/lmark1/uav_ros_tracker/workflows/Noetic/badge.svg)](https://github.com/lmark1/uav_ros_tracker/actions) |

## Summary

The main trajectory reference generator package of the [uav_ros_stack](https://github.com/lmark1/uav_ros_stack).  

## TOPP Tracker

This reference generator uses [topp_ros](https://github.com/larics/topp_ros) and [toppra](https://github.com/hungpham2511/toppra) to generate trajectory points based on:  
* the desired input trajectory - ```trajectory_msgs/MultiDOFTrajectory```
* a single point - ```geometry_msgs/PoseStamped```