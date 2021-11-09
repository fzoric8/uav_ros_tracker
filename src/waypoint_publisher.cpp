#include "geometry_msgs/PoseArray.h"
#include "ros/forwards.h"
#include <mutex>
#include <uav_ros_tracker/waypoint_publisher.hpp>

using namespace uav_ros_tracker;

void WaypointPublisher::initialize(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
{
  m_nh = nh;
  m_tracker_pose_pub = m_nh.advertise<geometry_msgs::PoseStamped>("pose_in", 1);
}

void WaypointPublisher::reset()
{
  m_flying_to_wp = false;
  m_is_waiting   = false;
  m_waiting_timer.stop();
}

std::tuple<bool, std::string, uav_ros_msgs::WaypointPtr>
  WaypointPublisher::publishWaypoint(const nav_msgs::Odometry& current_odometry,
                                     bool                      tracking_enabled,
                                     bool                      control_enabled)
{
  if (!control_enabled) {
    reset();
    return std::make_tuple<bool, std::string, uav_ros_msgs::WaypointPtr>(
      false, "UAV off-board control is disabled.", {});
  }

  // Get the latest waypoint
  uav_ros_msgs::WaypointPtr current_waypoint_ptr;
  {
    std::lock_guard<std::mutex> lock(m_waypoint_buffer_mutex);

    if (m_waypoint_buffer.empty()) {
      reset();
      return std::make_tuple<bool, std::string, uav_ros_msgs::WaypointPtr>(
        false, "No waypoints available.", {});
    }

    current_waypoint_ptr = m_waypoint_buffer.front();
  }

  if (current_waypoint_ptr == nullptr) {
    return std::make_tuple<bool, std::string, uav_ros_msgs::WaypointPtr>(
      false, "No waypoints available.", {});
  }

  auto distance_to_wp = this->calc_distance(current_odometry, *current_waypoint_ptr);
  // If we are are flying and still haven't reached the distance
  if (m_flying_to_wp && distance_to_wp >= DISTANCE_TOL) {
    return std::make_tuple(false, "Flying to current waypoint!", current_waypoint_ptr);
  }

  // We have reached the desired waypoint distance, start the waiting
  if (m_flying_to_wp && !m_is_waiting && distance_to_wp < DISTANCE_TOL) {
    m_is_waiting    = true;
    m_flying_to_wp  = false;
    m_waiting_timer = m_nh.createTimer(
      ros::Duration(current_waypoint_ptr->waiting_time),
      [&](const ros::TimerEvent& /*unused*/) {
        ROS_INFO("[%s] Waiting at waypoint finished!", NAME);
        m_is_waiting = false;

        {
          std::lock_guard<std::mutex> lock(m_waypoint_buffer_mutex);
          m_waypoint_buffer.pop_front();
        }
      },
      true /* oneshot */,
      false /*autostart */);
    m_waiting_timer.start();
    return std::make_tuple(
      false, "Waiting at the current waypoint!", current_waypoint_ptr);
  }

  // If we are not flying but rather waiting than
  if (m_is_waiting) {
    return std::make_tuple(
      false, "Waiting at the current waypoint!", current_waypoint_ptr);
  }

  // Check if tracker is available
  if (!tracking_enabled) {
    reset();
    return std::make_tuple<bool, std::string, uav_ros_msgs::WaypointPtr>(
      false, "Tracker is busy!", {});
  }

  // We are not flying or waiting, therefore publish the current waypoint.
  m_flying_to_wp        = true;
  auto pose_ref         = current_waypoint_ptr->pose;
  pose_ref.header.stamp = ros::Time::now();
  m_tracker_pose_pub.publish(pose_ref);

  return std::make_tuple(true, "Hello From publish_waypoint", current_waypoint_ptr);
}

uav_ros_msgs::WaypointStatus WaypointPublisher::getWaypointStatus(
  const nav_msgs::Odometry& odom)
{
  uav_ros_msgs::WaypointStatus wp_status;
  auto                         current_wp = getCurrentWaypoint();
  wp_status.current_wp =
    current_wp.has_value() ? *current_wp.value() : uav_ros_msgs::Waypoint{};
  wp_status.distance_to_wp = distanceToCurrentWp(odom);
  wp_status.flying_to_wp   = m_flying_to_wp;
  wp_status.waiting_at_wp  = m_is_waiting;
  return wp_status;
}

void WaypointPublisher::addWaypoint(const uav_ros_msgs::Waypoint& waypoint)
{
  std::lock_guard<std::mutex> lock(m_waypoint_buffer_mutex);
  m_waypoint_buffer.emplace_back(
    boost::make_shared<uav_ros_msgs::Waypoint>(std::move(waypoint)));

  ROS_INFO("[%s] Waypoint Added [%.2f, %.2f, %.2f]",
           NAME,
           waypoint.pose.pose.position.x,
           waypoint.pose.pose.position.y,
           waypoint.pose.pose.position.z);
}

void WaypointPublisher::addWaypoints(const uav_ros_msgs::Waypoints& waypoints)
{
  for (const auto& waypoint : waypoints.waypoints) { addWaypoint(waypoint); }
}

void WaypointPublisher::clearWaypoints()
{
  std::lock_guard<std::mutex> lock(m_waypoint_buffer_mutex);
  reset();
  m_waypoint_buffer.clear();
}

double WaypointPublisher::distanceToCurrentWp(const nav_msgs::Odometry& odom)
{
  auto optional_waypoint = getCurrentWaypoint();
  if (!optional_waypoint.has_value() || !optional_waypoint.value()) { return -1; }

  return calc_distance(odom, *optional_waypoint.value());
}

std::optional<uav_ros_msgs::WaypointPtr> WaypointPublisher::getCurrentWaypoint()
{
  std::optional<uav_ros_msgs::WaypointPtr> current_waypoint = std::nullopt;

  {
    std::lock_guard<std::mutex> lock(m_waypoint_buffer_mutex);

    if (m_waypoint_buffer.empty()) { return current_waypoint; }

    current_waypoint =
      std::make_optional<uav_ros_msgs::WaypointPtr>(m_waypoint_buffer.front());
  }

  return current_waypoint;
}

geometry_msgs::PoseArray WaypointPublisher::getWaypointArray()
{
  std::lock_guard<std::mutex> lock(m_waypoint_buffer_mutex);
  geometry_msgs::PoseArray    waypoint_array;

  for (const auto& waypoint : m_waypoint_buffer) {
    waypoint_array.poses.push_back(waypoint->pose.pose);
  }

  return waypoint_array;
}

double WaypointPublisher::calc_distance(const nav_msgs::Odometry&     odom,
                                        const uav_ros_msgs::Waypoint& waypoint)
{
  return sqrt(pow(odom.pose.pose.position.x - waypoint.pose.pose.position.x, 2)
              + pow(odom.pose.pose.position.y - waypoint.pose.pose.position.y, 2)
              + pow(odom.pose.pose.position.z - waypoint.pose.pose.position.z, 2));
}
