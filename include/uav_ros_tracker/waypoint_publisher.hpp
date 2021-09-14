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

namespace uav_ros_tracker {
class WaypointPublisher
{
public:
  WaypointPublisher(ros::NodeHandle& nh, const std::string& pose_in);
  void addWaypoint(uav_ros_msgs::WaypointPtr waypoint);
  void addWaypoint(uav_ros_msgs::Waypoint waypoint);
  void addWaypoints(uav_ros_msgs::WaypointsPtr waypoints);
  std::tuple<bool, std::string, uav_ros_msgs::WaypointPtr> publishWaypoint(
    const geometry_msgs::PoseStamped& current_carrot_pose,
    bool                              tracking_enabled = true,
    bool                              control_enabled  = true);

private:
  static constexpr auto WAYPOINT_RATE = 10;
  static constexpr auto THROTTLE_TIME = 5;
  static constexpr auto DISTANCE_TOL  = 0.1;
  static constexpr auto NAME          = "WaypointPublisher";

  ros::NodeHandle m_nh;
  bool            m_flying_to_wp = false;

  void   reset();
  double calc_distance(const geometry_msgs::PoseStamped& odom,
                       const uav_ros_msgs::Waypoint&     waypoint);

  ros::Timer m_waiting_timer;
  bool       m_is_waiting = false;

  ros::Publisher                        m_tracker_pose_pub;
  std::mutex                            m_waypoint_buffer_mutex;
  std::deque<uav_ros_msgs::WaypointPtr> m_waypoint_buffer;
};
}// namespace uav_ros_tracker

#endif /* WAYPOINT_FLIER_HPP */