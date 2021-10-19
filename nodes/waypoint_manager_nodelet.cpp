#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/duration.h"
#include "ros/publisher.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/buffer_core.h"
#include "tf2/convert.h"
#include "tf2/exceptions.h"
#include "tf2/transform_storage.h"
#include "tf2_ros/buffer.h"
#include "uav_ros_msgs/Waypoint.h"
#include <mutex>
#include <uav_ros_tracker/waypoint_publisher.hpp>
#include <nodelet/nodelet.h>
#include <std_srvs/SetBool.h>
#include <uav_ros_msgs/WaypointStatus.h>
#include <tf2_ros/transform_listener.h>
#include <vector>
#include <unordered_map>
#include <uav_ros_lib/param_util.hpp>
#include <uav_ros_lib/ros_convert.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace uav_ros_tracker {
class WaypointManager : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  static constexpr auto TRACKER_READY  = "ACCEPT";
  static constexpr auto TRACKER_ACTIVE = "ACTIVE";
  static constexpr auto HOLD           = "HOLD";
  static constexpr auto THROTTLE_S     = 5;

  bool                               m_is_initialized = false;
  std::unique_ptr<WaypointPublisher> m_waypoint_ptr;
  std::string                        m_tracking_frame;
  std::vector<std::string>           m_waypoint_frames;
  std::unordered_map<std::string, geometry_msgs::TransformStamped> m_transform_map;

  ros::Subscriber m_waypoint_sub;
  void            waypoint_cb(uav_ros_msgs::WaypointPtr msg);

  ros::Subscriber m_waypoints_sub;
  void            waypoints_cb(uav_ros_msgs::WaypointsPtr msg);

  ros::Subscriber m_tracker_status_sub;
  std::string     m_tracker_status = "N/A";
  std::mutex      m_tracker_status_mutex;
  void            tracker_status_cb(const std_msgs::StringConstPtr& msg);

  ros::Subscriber m_carrot_status_sub;
  std::string     m_carrot_status = "N/A";
  std::mutex      m_carrot_status_mutex;
  void            carrot_status_cb(const std_msgs::StringConstPtr& msg);

  ros::Subscriber            m_odom_sub;
  nav_msgs::OdometryConstPtr m_odom;
  bool                       m_odom_received = false;
  std::mutex                 m_odom_mutex;
  void                       odom_sub(const nav_msgs::OdometryConstPtr& msg);

  ros::ServiceServer m_clear_waypoints_srv;
  bool               clear_waypoints_cb(std_srvs::SetBool::Request&  req,
                                        std_srvs::SetBool::Response& resp);

  ros::Timer m_waypoint_timer;
  void       waypoint_loop(const ros::TimerEvent& /* unused */);

  ros::Publisher m_waypoint_status_pub;
  ros::Publisher m_waypoint_array_pub;

  uav_ros_msgs::Waypoint transform_waypoint(const uav_ros_msgs::Waypoint& waypoint);
};
}// namespace uav_ros_tracker

void uav_ros_tracker::WaypointManager::onInit()
{
  auto& nh         = getMTNodeHandle();
  auto& nh_private = getMTPrivateNodeHandle();

  param_util::getParamOrThrow(nh_private, "tracking_frame", m_tracking_frame);
  param_util::getParamOrThrow(nh_private, "waypoint_frames", m_waypoint_frames);

  // Initialize transform helpers
  tf2_ros::Buffer            buffer;
  tf2_ros::TransformListener transform_listener{ buffer };

  // Collect samples to get the average frame transform
  std::unordered_map<std::string, std::vector<geometry_msgs::TransformStamped>>
            multitransform_map;
  const int SAMPLE_COUNT = 10;
  while (ros::ok()) {
    ros::Duration(0.1).sleep();
    ros::spinOnce();

    bool samples_collected = true;
    for (const auto& frame : m_waypoint_frames) {
      if (multitransform_map[frame].size() == SAMPLE_COUNT) {
        ROS_INFO("[%s] Got all samples for frame %s", getName().c_str(), frame.c_str());
        continue;
      }

      samples_collected = false;
      try {
        auto trans = buffer.lookupTransform(m_tracking_frame, frame, ros::Time(0));
        multitransform_map[frame].push_back(trans);
        ROS_INFO("[%s] Got sample %ld for frame %s",
                 getName().c_str(),
                 multitransform_map[frame].size(),
                 frame.c_str());
      } catch (tf2::TransformException& ex) {
        ROS_FATAL_THROTTLE(2.0,
                           "[%s] Unable to find transform to %s with message: %s",
                           getName().c_str(),
                           frame.c_str(),
                           ex.what());
        continue;
      }
    }

    if (samples_collected) { break; }
  }

  // Average out the samples
  for (const auto& [frame, transform_samples] : multitransform_map) {
    double       total_heading = 0;
    tf2::Vector3 total_translation;
    for (const auto& transform_sample : transform_samples) {
      auto heading = ros_convert::calculateYaw(transform_sample.transform.rotation);
      tf2::Vector3 translation(transform_sample.transform.translation.x,
                               transform_sample.transform.translation.y,
                               transform_sample.transform.translation.z);
      total_heading += heading;
      total_translation += translation;
    }

    total_heading /= SAMPLE_COUNT;
    total_translation /= SAMPLE_COUNT;

    geometry_msgs::TransformStamped final_transform;
    final_transform.transform.translation.x = total_translation.x();
    final_transform.transform.translation.y = total_translation.y();
    final_transform.transform.translation.z = total_translation.z();
    final_transform.transform.rotation = ros_convert::calculate_quaternion(total_heading);

    ROS_INFO_STREAM(getName() << " Translation for frame " << frame
                              << " is: " << final_transform);
    m_transform_map[frame] = final_transform;
  }

  // Add Identity tracking frame transform
  m_transform_map[m_tracking_frame] = geometry_msgs::TransformStamped{};
  m_transform_map[m_tracking_frame].transform.rotation.w = 1;

  ROS_INFO_STREAM(getName() << "Translation for frame " << m_tracking_frame
                            << " is: " << m_transform_map[m_tracking_frame]);

  m_tracker_status_sub =
    nh.subscribe("tracker/status", 1, &WaypointManager::tracker_status_cb, this);
  m_carrot_status_sub =
    nh.subscribe("carrot/status", 1, &WaypointManager::carrot_status_cb, this);
  m_waypoint_ptr   = std::make_unique<WaypointPublisher>(nh, "pose_in");
  m_waypoint_sub   = nh.subscribe("waypoint", 1, &WaypointManager::waypoint_cb, this);
  m_waypoints_sub  = nh.subscribe("waypoints", 1, &WaypointManager::waypoints_cb, this);
  m_odom_sub       = nh.subscribe("odometry", 1, &WaypointManager::odom_sub, this);
  m_waypoint_timer = nh.createTimer(ros::Rate(50), &WaypointManager::waypoint_loop, this);
  m_clear_waypoints_srv =
    nh.advertiseService("clear_waypoints", &WaypointManager::clear_waypoints_cb, this);
  m_waypoint_status_pub =
    nh.advertise<uav_ros_msgs::WaypointStatus>("waypoint_status", 1);
  m_waypoint_array_pub = nh.advertise<geometry_msgs::PoseArray>("waypoint_array", 1);

  m_is_initialized = true;
  ROS_INFO("[%s] Initialized.", this->getName().c_str());
}

void uav_ros_tracker::WaypointManager::waypoint_loop(const ros::TimerEvent& /* unused */)
{
  if (!m_is_initialized) {
    ROS_INFO_THROTTLE(THROTTLE_S, "[%s] Nodelet not initialized.", getName().c_str());
    return;
  }

  if (!m_odom_received) {
    ROS_INFO_THROTTLE(THROTTLE_S, "[%s] Odometry not recieved.", getName().c_str());
    return;
  }

  nav_msgs::Odometry current_odometry;
  {
    std::lock_guard<std::mutex> lock(m_odom_mutex);
    current_odometry = *m_odom;
  }

  bool control_enabled = false;
  {
    std::lock_guard<std::mutex> lock(m_carrot_status_mutex);
    control_enabled = m_carrot_status == HOLD;
  }

  bool tracking_enabled = false;
  {
    std::lock_guard<std::mutex> lock(m_tracker_status_mutex);
    tracking_enabled = m_tracker_status == TRACKER_READY;
  }

  auto [wp_published, message, current_waypoint] =
    m_waypoint_ptr->publishWaypoint(current_odometry, tracking_enabled, control_enabled);

  auto waypoint_array_msg            = m_waypoint_ptr->getWaypointArray();
  waypoint_array_msg.header.frame_id = m_tracking_frame;
  waypoint_array_msg.header.stamp    = ros::Time::now();
  m_waypoint_array_pub.publish(waypoint_array_msg);

  // Create a status message
  uav_ros_msgs::WaypointStatus wp_status;
  auto                         current_wp = m_waypoint_ptr->getCurrentWaypoint();
  wp_status.current_wp =
    current_wp.has_value() ? *current_wp.value() : uav_ros_msgs::Waypoint{};
  wp_status.distance_to_wp = m_waypoint_ptr->distanceToCurrentWp(current_odometry);
  wp_status.flying_to_wp   = m_waypoint_ptr->isFlying();
  wp_status.waiting_at_wp  = m_waypoint_ptr->isWaiting();
  m_waypoint_status_pub.publish(wp_status);

  // No waypoint is published
  if (!wp_published || current_waypoint == nullptr) {
    ROS_WARN_THROTTLE(THROTTLE_S, "[%s] %s", getName().c_str(), message.c_str());
    return;
  }

  ROS_INFO_THROTTLE(THROTTLE_S,
                    "[%s] Published waypoint is: [%.2f, %.2f, %.2f]]",
                    getName().c_str(),
                    current_waypoint->pose.pose.position.x,
                    current_waypoint->pose.pose.position.y,
                    current_waypoint->pose.pose.position.z);
}

void uav_ros_tracker::WaypointManager::odom_sub(const nav_msgs::OdometryConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(m_odom_mutex);
  m_odom          = msg;
  m_odom_received = true;
}

void uav_ros_tracker::WaypointManager::waypoint_cb(const uav_ros_msgs::WaypointPtr msg)
{
  if (!m_is_initialized) {
    ROS_INFO_THROTTLE(THROTTLE_S, "[%s] Nodelet not initialized.", getName().c_str());
    return;
  }
  auto transformed_wp = transform_waypoint(*msg);
  m_waypoint_ptr->addWaypoint(transformed_wp);
}

void uav_ros_tracker::WaypointManager::waypoints_cb(const uav_ros_msgs::WaypointsPtr msg)
{
  if (!m_is_initialized) {
    ROS_INFO_THROTTLE(THROTTLE_S, "[%s] Nodelet not initialized.", getName().c_str());
    return;
  }

  uav_ros_msgs::Waypoints transformed_wps;
  for (const auto& wp : msg->waypoints) {
    transformed_wps.waypoints.push_back(transform_waypoint(wp));
  }
  m_waypoint_ptr->addWaypoints(transformed_wps);
}

void uav_ros_tracker::WaypointManager::tracker_status_cb(
  const std_msgs::StringConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(m_tracker_status_mutex);
  m_tracker_status = msg->data;
}

void uav_ros_tracker::WaypointManager::carrot_status_cb(
  const std_msgs::StringConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(m_carrot_status_mutex);
  m_carrot_status = msg->data;
}

bool uav_ros_tracker::WaypointManager::clear_waypoints_cb(
  std_srvs::SetBool::Request&  req,
  std_srvs::SetBool::Response& resp)
{
  if (!req.data) {
    resp.success = false;
    resp.message = "Clear not requested!";
    return true;
  }

  m_waypoint_ptr->clearWaypoints();
  resp.success = true;
  resp.message = "Waypoints successfully cleared";
  return true;
}

uav_ros_msgs::Waypoint uav_ros_tracker::WaypointManager::transform_waypoint(
  const uav_ros_msgs::Waypoint& waypoint)
{
  auto waypoint_frame = waypoint.pose.header.frame_id;
  if (m_transform_map.find(waypoint_frame) == m_transform_map.end()) {
    ROS_WARN("[%s] Waypoint frame %s unrecognized, setting to %s.",
             getName().c_str(),
             waypoint_frame.c_str(),
             m_tracking_frame.c_str());
    waypoint_frame = m_tracking_frame;
  }

  geometry_msgs::PoseStamped transformed_pose;
  tf2::doTransform(waypoint.pose, transformed_pose, m_transform_map[waypoint_frame]);

  uav_ros_msgs::Waypoint new_wp;
  new_wp.pose         = transformed_pose;
  new_wp.waiting_time = waypoint.waiting_time;
  return new_wp;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uav_ros_tracker::WaypointManager, nodelet::Nodelet)
