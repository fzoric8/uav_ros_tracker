#include <uav_ros_tracker/mpc_tracker.hpp>
#include <uav_ros_lib/topic_handler.hpp>

#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>

std::unique_ptr<uav_ros_tracker::MPCTracker> mpc_tracker;
std::string carrot_status = "OFF";

bool enable_srv_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
  if (req.data && carrot_status == "HOLD") {
    mpc_tracker->activate();
    resp.message = "MPC Tracker activated";
    resp.success = true;
  } else if (req.data && carrot_status != "HOLD") {
    mpc_tracker->deactivate();
    resp.message = "MPC Tracker deactivated - position hold is disabled";
    resp.success = false;
  } else {
    mpc_tracker->deactivate();
    resp.message = "MPC Tracker deactivated";
    resp.success = false;
  }
  return true;
}

bool reset_srv_cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
  mpc_tracker->reset();
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mpc_tracker");
  ros::NodeHandle nh;
  mpc_tracker = std::make_unique<uav_ros_tracker::MPCTracker>(nh);
  auto enable_srv = nh.advertiseService("tracker/enable", enable_srv_cb);
  auto reset_srv = nh.advertiseService("tracker/reset", reset_srv_cb);
  auto status_pub = nh.advertise<std_msgs::String>("tracker/status", 1);

  auto carrot_status_sub = nh.subscribe<std_msgs::String>(
    "carrot/status", 1, [&](const std_msgs::StringConstPtr &msg) {
      carrot_status = msg->data;
      if (mpc_tracker->is_active() && (carrot_status != "HOLD")) {
        mpc_tracker->deactivate();
      }
    });

  auto status_timer =
    nh.createTimer(ros::Duration(ros::Rate(10)), [&](const ros::TimerEvent &e) {
      std_msgs::String status_msg;

      if (mpc_tracker->is_tracking()) {
        status_msg.data = "ACTIVE";
        status_pub.publish(status_msg);
        return;
      }

      status_msg.data = "ACCEPT";
      status_pub.publish(status_msg);
    });
  ros::spin();
  return 0;
}