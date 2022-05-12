#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include <std_msgs/String.h>
#include "ros/duration.h"
#include "ros/publisher.h"
#include <pluginlib/class_loader.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/buffer_core.h"
#include "tf2/convert.h"
#include "tf2/exceptions.h"
#include "tf2/transform_storage.h"
#include "tf2_ros/buffer.h"
#include "uav_ros_msgs/Waypoint.h"
#include <mutex>
#include <uav_ros_tracker/planner_interface.hpp>
#include <nodelet/nodelet.h>
#include <std_srvs/SetBool.h>
#include <uav_ros_msgs/WaypointStatus.h>
#include <tf2_ros/transform_listener.h>
#include <vector>
#include <unordered_map>
#include <uav_ros_lib/param_util.hpp>
#include <uav_ros_lib/ros_convert.hpp>
#include <geometry_msgs/TransformStamped.h>

using planner_loader_t = pluginlib::ClassLoader<uav_ros_tracker::planner_interface>;

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

  // Planner plugin
  std::mutex                                            m_planner_ptr_mutex;
  std::unique_ptr<planner_loader_t>                     m_planner_loader_ptr;
  boost::shared_ptr<uav_ros_tracker::planner_interface> m_planner_ptr;

  bool                     m_is_initialized = false;
  std::string              m_tracking_frame;
  bool                     m_override_waypoints;
  bool                     m_disable_when_grounded;
  std::vector<std::string> m_waypoint_frames;

  std::unordered_map<std::string, geometry_msgs::TransformStamped> m_transform_map;
  void initialize_transform_map();

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

  ros::Timer m_update_transform_map;
  void       update_transform_map_loop(const ros::TimerEvent& /* unused */);

  ros::Publisher m_waypoint_status_pub;
  ros::Publisher m_waypoint_array_pub;

  // Initialize transform helpers
  std::unique_ptr<tf2_ros::Buffer>            m_buffer_ptr;
  std::unique_ptr<tf2_ros::TransformListener> m_transform_listener_ptr;
};
}// namespace uav_ros_tracker

void uav_ros_tracker::WaypointManager::onInit()
{
  auto& nh         = getMTNodeHandle();
  auto& nh_private = getMTPrivateNodeHandle();

  param_util::getParamOrThrow(nh_private, "tracking_frame", m_tracking_frame);
  param_util::getParamOrThrow(nh_private, "override_waypoints", m_override_waypoints);
  param_util::getParamOrThrow(
    nh_private, "disable_when_grounded", m_disable_when_grounded);
  param_util::getParamOrThrow(nh_private, "waypoint_frames", m_waypoint_frames);

  m_buffer_ptr             = std::make_unique<tf2_ros::Buffer>();
  m_transform_listener_ptr = std::make_unique<tf2_ros::TransformListener>(*m_buffer_ptr);

  initialize_transform_map();

  // Get the planner name
  std::string planner_name;
  param_util::getParamOrThrow(nh_private, "planner_name", planner_name);

  // Load planner plugin
  m_planner_loader_ptr = std::make_unique<planner_loader_t>(
    "uav_ros_tracker", "uav_ros_tracker::planner_interface");
  m_planner_ptr = m_planner_loader_ptr->createUniqueInstance(planner_name);

  if (m_planner_ptr == nullptr) {
    ROS_ERROR("[WaypointManager] Unable to load plugin %s", planner_name.c_str());
    ros::shutdown();
  }

  // Initialize the planner
  auto init_success =
    m_planner_ptr->initialize(nh, nh_private, m_transform_map, m_tracking_frame);
  if (!init_success) {
    ROS_ERROR("[WaypointManager] planner initialization unsucessful!");
    ros::shutdown();
  }

  m_tracker_status_sub =
    nh.subscribe("tracker/status", 1, &WaypointManager::tracker_status_cb, this);
  m_carrot_status_sub =
    nh.subscribe("carrot/status", 1, &WaypointManager::carrot_status_cb, this);
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

  m_update_transform_map =
    nh.createTimer(ros::Rate(10), &WaypointManager::update_transform_map_loop, this);
}

void uav_ros_tracker::WaypointManager::update_transform_map_loop(
  const ros::TimerEvent& /* unused */)
{
  initialize_transform_map();
  {
    std::lock_guard<std::mutex> lock(m_planner_ptr_mutex);
    m_planner_ptr->updateTransformMap(m_transform_map);
  }
}

void uav_ros_tracker::WaypointManager::initialize_transform_map()
{
  m_transform_map.clear();

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
        ROS_DEBUG("[%s] Got all samples for frame %s", getName().c_str(), frame.c_str());
        continue;
      }

      samples_collected = false;
      try {
        auto trans = m_buffer_ptr->lookupTransform(
          m_tracking_frame, frame, ros::Time(0), ros::Duration(1));
        multitransform_map[frame].push_back(trans);
        ROS_DEBUG("[%s] Got sample %ld for frame %s",
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

    ROS_DEBUG_STREAM(getName() << " Translation for frame " << frame
                               << " is: " << final_transform);
    m_transform_map[frame] = final_transform;
  }

  // Add Identity tracking frame transform
  m_transform_map[m_tracking_frame] = geometry_msgs::TransformStamped{};
  m_transform_map[m_tracking_frame].transform.rotation.w = 1;

  ROS_DEBUG_STREAM(getName() << "Translation for frame " << m_tracking_frame
                             << " is: " << m_transform_map[m_tracking_frame]);
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

  geometry_msgs::PoseArray     waypoint_array_msg;
  uav_ros_msgs::WaypointStatus wp_status;
  bool                         wp_published;
  std::string                  message;
  uav_ros_msgs::WaypointPtr    current_waypoint;

  {
    std::lock_guard<std::mutex> lock(m_planner_ptr_mutex);
    auto [ret_wp_published, ret_message, ret_current_waypoint] =
      m_planner_ptr->publishWaypoint(current_odometry, tracking_enabled, control_enabled);
    wp_published     = ret_wp_published;
    message          = ret_message;
    current_waypoint = ret_current_waypoint;

    // Publish waypoint array
    waypoint_array_msg = m_planner_ptr->getWaypointArray();

    // Create a status message
    wp_status = m_planner_ptr->getWaypointStatus(current_odometry);
  }

  if (waypoint_array_msg.header.frame_id == "") {
    waypoint_array_msg.header.frame_id = m_waypoint_frames.front();
  }
  m_waypoint_array_pub.publish(waypoint_array_msg);
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
  m_planner_ptr->addWaypoint(*msg);
}

void uav_ros_tracker::WaypointManager::waypoints_cb(const uav_ros_msgs::WaypointsPtr msg)
{
  if (!m_is_initialized) {
    ROS_INFO_THROTTLE(THROTTLE_S, "[%s] Nodelet not initialized.", getName().c_str());
    return;
  }

  // If override flag is set then clear all existing waypoints before adding new ones
  if (m_override_waypoints) {
    std_srvs::SetBool::Request req;
    req.data = true;
    std_srvs::SetBool::Response resp;

    clear_waypoints_cb(req, resp);

    if (!resp.success) {
      ROS_FATAL("[Waypointmanager::waypoints_cb] - Unable to clear waypoints!");
      return;
    }
  }

  // Disable waypoint manager when grounded
  if (m_disable_when_grounded) {
    bool control_enabled = false;
    {
      std::lock_guard<std::mutex> lock(m_carrot_status_mutex);
      control_enabled = m_carrot_status == HOLD;
    }

    if (!control_enabled) {
      ROS_FATAL(
        "[WaypointManager::waypoints_cb] - Sending waypoints is disabled when UAV is "
        "grounded.");
      return;
    }
  }

  m_planner_ptr->addWaypoints(*msg);
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

  m_planner_ptr->clearWaypoints();
  resp.success = true;
  resp.message = "Waypoints successfully cleared";
  return true;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uav_ros_tracker::WaypointManager, nodelet::Nodelet)
