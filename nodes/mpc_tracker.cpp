#include <memory>

#include <ros/ros.h>
#include <uav_ros_tracker/cvx_wrapper.h>
#include <uav_ros_lib/param_util.hpp>

class MPCTracker
{
public:
  MPCTracker()
  {
    m_solver_x = std::make_unique<uav_ros_trackers::cvx_wrapper::CvxWrapper>(
      true, 25, std::vector<double>{ 5000, 0, 0, 0 }, 0.01, 0.2, 0);
    m_solver_y = std::make_unique<uav_ros_trackers::cvx_wrapper::CvxWrapper>(
      true, 25, std::vector<double>{ 5000, 0, 0, 0 }, 0.01, 0.2, 1);
    m_solver_z = std::make_unique<uav_ros_trackers::cvx_wrapper::CvxWrapper>(
      true, 25, std::vector<double>{ 5000, 0, 0, 0 }, 0.01, 0.2, 2);
    m_solver_heading = std::make_unique<uav_ros_trackers::cvx_wrapper::CvxWrapper>(
      true, 25, std::vector<double>{ 5000, 0, 0, 0 }, 0.01, 0.2, 0);

    initialize_parameters();
  }

private:
  void initialize_parameters()
  {
    ros::NodeHandle nh_private("~");
    param_util::getParamOrThrow(nh_private, "rate", m_tracker_rate);

    // Load translation parameters
    param_util::getParamOrThrow(
      nh_private, "model/translation/n_states", m_trans_state_count);
    param_util::getParamOrThrow(
      nh_private, "model/translation/n_inputs", m_trans_input_count);
    m_A = param_util::loadMatrixOrThrow(
      nh_private, "model/translation/A", m_trans_state_count, m_trans_state_count);
    m_B = param_util::loadMatrixOrThrow(
      nh_private, "model/translation/B", m_trans_state_count, m_trans_input_count);

    // Load heading parameters
    param_util::getParamOrThrow(
      nh_private, "model/heading/n_states", m_heading_state_count);
    param_util::getParamOrThrow(
      nh_private, "model/heading/n_inputs", m_heading_input_count);
    m_A_heading = param_util::loadMatrixOrThrow(
      nh_private, "model/heading/A", m_heading_state_count, m_heading_state_count);
    m_B_heading = param_util::loadMatrixOrThrow(
      nh_private, "model/heading/B", m_heading_state_count, m_heading_input_count);
  }

  std::unique_ptr<uav_ros_trackers::cvx_wrapper::CvxWrapper> m_solver_x;
  std::unique_ptr<uav_ros_trackers::cvx_wrapper::CvxWrapper> m_solver_y;
  std::unique_ptr<uav_ros_trackers::cvx_wrapper::CvxWrapper> m_solver_z;
  std::unique_ptr<uav_ros_trackers::cvx_wrapper::CvxWrapper> m_solver_heading;

  double m_tracker_rate;
  int m_trans_state_count;
  int m_trans_input_count;
  int m_heading_state_count;
  int m_heading_input_count;

  Eigen::MatrixXd m_A;
  Eigen::MatrixXd m_B;
  Eigen::MatrixXd m_A_heading;
  Eigen::MatrixXd m_B_heading;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "mpc_tracker");
  MPCTracker mpc_tracker;
  return 0;
}