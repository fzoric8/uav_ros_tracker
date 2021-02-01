#include <uav_ros_tracker/mpc_tracker.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mpc_tracker");
  ros::NodeHandle nh;
  uav_ros_tracker::MPCTracker mpc_tracker(nh);
  ros::spin();
  return 0;
}