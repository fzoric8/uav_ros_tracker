#ifndef PLANNER_INTERFACE_HPP
#define PLANNER_INTERFACE_HPP

#include <ros/ros.h>
#include <uav_ros_msgs/Waypoint.h>
#include <uav_ros_msgs/Waypoints.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <uav_ros_msgs/WaypointStatus.h>
#include <geometry_msgs/TransformStamped.h>
#include <unordered_map>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
  virtual bool initialize(
    ros::NodeHandle&                                                 nh,
    ros::NodeHandle&                                                 nh_private,
    std::unordered_map<std::string, geometry_msgs::TransformStamped> transform_map,
    std::string                                                      tracking_frame) = 0;

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

  /**
   * @brief Transform the given waypoint using the transform map
   *
   * @param waypoint
   * @return uav_ros_msgs::Waypoint
   */
  static uav_ros_msgs::Waypoint transform_waypoint(
    const uav_ros_msgs::Waypoint&                                           waypoint,
    const std::unordered_map<std::string, geometry_msgs::TransformStamped>& transform_map,
    const std::string& tracking_frame)
  {
    auto waypoint_frame = waypoint.pose.header.frame_id;
    if (transform_map.find(waypoint_frame) == transform_map.end()) {
      ROS_WARN("[PlannerInterface] Waypoint frame %s unrecognized, setting to %s.",
               waypoint_frame.c_str(),
               tracking_frame.c_str());
      waypoint_frame = tracking_frame;
    }

    geometry_msgs::PoseStamped transformed_pose;
    tf2::doTransform(waypoint.pose, transformed_pose, transform_map.at(waypoint_frame));

    uav_ros_msgs::Waypoint new_wp;
    new_wp.pose                 = transformed_pose;
    new_wp.pose.pose.position.z = waypoint.pose.pose.position.z;
    new_wp.waiting_time         = waypoint.waiting_time;
    return new_wp;
  }

  static double calc_distance(const nav_msgs::Odometry&     odom,
                              const uav_ros_msgs::Waypoint& waypoint)
  {
    return sqrt(pow(odom.pose.pose.position.x - waypoint.pose.pose.position.x, 2)
                + pow(odom.pose.pose.position.y - waypoint.pose.pose.position.y, 2)
                + pow(odom.pose.pose.position.z - waypoint.pose.pose.position.z, 2));
  }

protected:
  planner_interface() { ROS_INFO("[planner_interface] Constructor"); }
};

}// namespace uav_ros_tracker

#endif /* PLANNER_INTERFACE_HPP */