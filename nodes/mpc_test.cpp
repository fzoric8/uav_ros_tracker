#include <memory>
#include <uav_ros_tracker/cvx_wrapper.h>

int main()
{
  auto solver_x = std::make_unique<uav_ros_trackers::cvx_wrapper::CvxWrapper>(
    true, 25, std::vector<double>{ 5000, 0, 0, 0 }, 0.01, 0.2, 0);
  auto solver_y = std::make_unique<uav_ros_trackers::cvx_wrapper::CvxWrapper>(
    true, 25, std::vector<double>{ 5000, 0, 0, 0 }, 0.01, 0.2, 1);
  auto solver_z = std::make_unique<uav_ros_trackers::cvx_wrapper::CvxWrapper>(
    true, 25, std::vector<double>{ 5000, 0, 0, 0 }, 0.01, 0.2, 2);
  auto solver_heading = std::make_unique<uav_ros_trackers::cvx_wrapper::CvxWrapper>(
    true, 25, std::vector<double>{ 5000, 0, 0, 0 }, 0.01, 0.2, 0);

  
  return 0;
}