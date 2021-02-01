#include <uav_ros_tracker/mpc_tracker.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mpc_tracker");
  uav_ros_tracker::MPCTracker mpc_tracker;
  ros::spin();
  return 0;
}