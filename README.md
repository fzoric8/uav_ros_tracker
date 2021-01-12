# UAV ROS Tracker

| Ubuntu 18.04  | Ubuntu 20.04|
|---------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------|
 [![Melodic](https://github.com/lmark1/uav_ros_tracker/workflows/Melodic/badge.svg)](https://github.com/lmark1/uav_ros_tracker/actions) | [![Noetic](https://github.com/lmark1/uav_ros_tracker/workflows/Noetic/badge.svg)](https://github.com/lmark1/uav_ros_tracker/actions) |

## Summary

The main trajectory reference generator package of the [uav_ros_stack](https://github.com/lmark1/uav_ros_stack).  

## TOPP Tracker

This reference generator uses [topp_ros](https://github.com/larics/topp_ros) and [toppra](https://github.com/hungpham2511/toppra) to generate trajectory points based on:  
* *topp/input_trajectory* 
  * ```trajectory_msgs/MultiDOFTrajectory```
  * Points from which a trajectory is generated

* *topp/input_pose*
  * ```geometry_msgs/PoseStamped```
  * A trajectory from current reference to the given pose is generated

Launch TOPP Tracker with a default configuration as follows:
```bash
export UAV_NAMESPACE=red; roslaunch uav_ros_tracker topp_tracker.launchp
```