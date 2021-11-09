#ifndef WAYPOINT_INTERFACE_HPP
#define WAYPOINT_INTERFACE_HPP

#include <ros/ros.h>
#include <uav_ros_msgs/Waypoint.h>
#include <uav_ros_msgs/Waypoints.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <uav_ros_msgs/WaypointStatus.h>

namespace uav_ros_tracker {

/**
 * @brief Interface used for implementing waypoint mission planning.
 *
 */
class waypoint_interface
{
public:
  virtual void addWaypoint(const uav_ros_msgs::Waypoint& waypoint)   = 0;
  virtual void addWaypoints(const uav_ros_msgs::Waypoints& waypoint) = 0;
  virtual void clearWaypoints()                                      = 0;
  virtual geometry_msgs::PoseArray     getWaypointArray()            = 0;
  virtual uav_ros_msgs::WaypointStatus getWaypointStatus(
    const nav_msgs::Odometry& odom)                                         = 0;
  virtual bool initialize(ros::NodeHandle& nh, ros::NodeHandle& nh_private) = 0;

protected:
  waypoint_interface() { ROS_INFO("[waypoint_interface] Constructor"); }
};

}// namespace uav_ros_tracker

#endif /* WAYPOINT_INTERFACE_HPP */