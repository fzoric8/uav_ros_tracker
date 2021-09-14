#include <uav_ros_tracker/waypoint_publisher.hpp>
#include <nodelet/nodelet.h>

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

  ros::Subscriber                    m_carrot_sub;
  geometry_msgs::PoseStampedConstPtr m_carrot_pose;
  bool                               m_carrot_pose_received = false;
  std::mutex                         m_carrot_pose_mutex;
  void carrot_pose_sub(const geometry_msgs::PoseStampedConstPtr& msg);

  ros::Timer m_waypoint_timer;
  void       waypoint_loop(const ros::TimerEvent& /* unused */);
};
}// namespace uav_ros_tracker

void uav_ros_tracker::WaypointManager::onInit()
{
  auto& nh             = getMTNodeHandle();
  m_tracker_status_sub = nh.subscribe<std_msgs::String>(
    "tracker/status", 1, &WaypointManager::tracker_status_cb, this);
  m_carrot_status_sub = nh.subscribe<std_msgs::String>(
    "carrot/status", 1, &WaypointManager::carrot_status_cb, this);
  m_waypoint_ptr = std::make_unique<WaypointPublisher>(nh, "pose_in");
  m_waypoint_sub = nh.subscribe<uav_ros_msgs::WaypointPtr>(
    "waypoint", 1, &WaypointManager::waypoint_cb, this);
  m_waypoints_sub = nh.subscribe<uav_ros_msgs::WaypointsPtr>(
    "waypoints", 1, &WaypointManager::waypoints_cb, this);
  m_carrot_sub = nh.subscribe<geometry_msgs::PoseStamped>(
    "carrot/pose", 1, &WaypointManager::carrot_pose_sub, this);
  m_waypoint_timer = nh.createTimer(ros::Rate(50), &WaypointManager::waypoint_loop, this);
  m_is_initialized = true;
  ROS_INFO("[%s] Initialized.", this->getName().c_str());
}

void uav_ros_tracker::WaypointManager::waypoint_loop(const ros::TimerEvent& /* unused */)
{
  if (!m_is_initialized) {
    ROS_INFO_THROTTLE(THROTTLE_S, "[%s] Nodelet not initialized.", getName().c_str());
    return;
  }

  if (!m_carrot_pose_received) {
    ROS_INFO_THROTTLE(THROTTLE_S, "[%s] Odometry not recieved.", getName().c_str());
    return;
  }

  geometry_msgs::PoseStamped current_carrot_pose;
  {
    std::lock_guard<std::mutex> lock(m_carrot_pose_mutex);
    current_carrot_pose = *m_carrot_pose;
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

  auto [wp_published, message, current_waypoint] = m_waypoint_ptr->publishWaypoint(
    current_carrot_pose, tracking_enabled, control_enabled);

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

void uav_ros_tracker::WaypointManager::carrot_pose_sub(
  const geometry_msgs::PoseStampedConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(m_carrot_pose_mutex);
  m_carrot_pose          = msg;
  m_carrot_pose_received = true;
}

void uav_ros_tracker::WaypointManager::waypoint_cb(const uav_ros_msgs::WaypointPtr msg)
{
  m_waypoint_ptr->addWaypoint(msg);
}

void uav_ros_tracker::WaypointManager::waypoints_cb(const uav_ros_msgs::WaypointsPtr msg)
{
  m_waypoint_ptr->addWaypoints(msg);
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

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uav_ros_tracker::WaypointManager, nodelet::Nodelet)