#ifndef WAYPOINT_FLIER_HPP
#define WAYPOINT_FLIER_HPP

#include <mutex>
#include <optional>
#include <deque>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <uav_ros_msgs/Waypoint.h>
#include <uav_ros_msgs/Waypoints.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <uav_ros_msgs/WaypointStatus.h>
#include <uav_ros_tracker/planner_interface.hpp>

namespace uav_ros_tracker {
class WaypointPublisher : public planner_interface
{
public:
  void addWaypoint(const uav_ros_msgs::Waypoint& waypoint) override;
  void addWaypoints(const uav_ros_msgs::Waypoints& waypoints) override;
  void clearWaypoints() override;

  geometry_msgs::PoseArray     getWaypointArray() override;
  uav_ros_msgs::WaypointStatus getWaypointStatus(const nav_msgs::Odometry& odom) override;
  void updateTransformMap(std::unordered_map<std::string, geometry_msgs::TransformStamped>
                            transform_map) override;

  bool initialize(
    ros::NodeHandle&                                                 nh,
    ros::NodeHandle&                                                 nh_private,
    std::unordered_map<std::string, geometry_msgs::TransformStamped> transform_map,
    std::string tracking_frame) override;

  std::tuple<bool, std::string, uav_ros_msgs::WaypointPtr> publishWaypoint(
    const nav_msgs::Odometry& current_odometry,
    bool                      tracking_enabled = true,
    bool                      control_enabled  = true) override;

private:
  std::optional<uav_ros_msgs::WaypointPtr> getCurrentWaypoint();
  double                distanceToCurrentWp(const nav_msgs::Odometry& odom);
  static constexpr auto WAYPOINT_RATE = 10;
  static constexpr auto THROTTLE_TIME = 5;
  static constexpr auto DISTANCE_TOL  = 0.3;
  static constexpr auto NAME          = "WaypointPublisher";

  ros::NodeHandle m_nh;
  bool            m_flying_to_wp = false;

  void reset();

  ros::Timer                                                       m_waiting_timer;
  bool                                                             m_is_waiting = false;
  std::string                                                      m_tracking_frame;
  std::unordered_map<std::string, geometry_msgs::TransformStamped> m_transform_map;

  ros::Publisher                        m_tracker_pose_pub;
  std::mutex                            m_waypoint_buffer_mutex;
  std::deque<uav_ros_msgs::WaypointPtr> m_waypoint_buffer;
};
}// namespace uav_ros_tracker

#endif /* WAYPOINT_FLIER_HPP */
