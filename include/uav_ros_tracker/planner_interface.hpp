#ifndef PLANNER_INTERFACE_HPP
#define PLANNER_INTERFACE_HPP

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
class planner_interface
{
public:
  /**
   * @brief Add waypoints
   *
   * @param waypoint
   */
  virtual void addWaypoint(const uav_ros_msgs::Waypoint& waypoint) = 0;

  /**
   * @brief Add waypoints
   *
   * @param waypoint
   */
  virtual void addWaypoints(const uav_ros_msgs::Waypoints& waypoint) = 0;

  /**
   * @brief Clear the waypoints
   *
   */
  virtual void clearWaypoints() = 0;

  /**
   * @brief Get the Waypoint Array object
   *
   * @return geometry_msgs::PoseArray
   */
  virtual geometry_msgs::PoseArray getWaypointArray() = 0;

  /**
   * @brief Get the Waypoint Status object
   *
   * @param odom
   * @return uav_ros_msgs::WaypointStatus
   */
  virtual uav_ros_msgs::WaypointStatus getWaypointStatus(
    const nav_msgs::Odometry& odom) = 0;

  /**
   * @brief Initialize the planner
   *
   * @param nh
   * @param nh_private
   */
  virtual bool initialize(ros::NodeHandle& nh, ros::NodeHandle& nh_private) = 0;

  /**
   * @brief Check if a new waypoint should be published
   *
   * @param current_odometry
   * @param tracking_enabled True if trajectory tracking is enabled and ready
   * @param control_enabled True if UAV control is enabled and ready
   * @return std::tuple<bool, std::string, uav_ros_msgs::WaypointPtr>
   *  bool - True if waypoint is successfully published
   *  string - message
   *  WaypointPtr - currently active waypoint
   */
  virtual std::tuple<bool, std::string, uav_ros_msgs::WaypointPtr> publishWaypoint(
    const nav_msgs::Odometry& current_odometry,
    bool                      tracking_enabled = true,
    bool                      control_enabled  = true) = 0;

protected:
  planner_interface() { ROS_INFO("[planner_interface] Constructor"); }
};

}// namespace uav_ros_tracker

#endif /* PLANNER_INTERFACE_HPP */