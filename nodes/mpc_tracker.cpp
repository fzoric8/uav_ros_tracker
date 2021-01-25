#include <memory>

#include <ros/ros.h>
#include <uav_ros_tracker/cvx_wrapper.h>
#include <uav_ros_lib/param_util.hpp>

/**
 * @brief UAV MPC Tracker implementation as found at
 * https://github.com/ctu-mrs/mrs_uav_trackers.
 *
 */
class MPCTracker
{
public:
  MPCTracker()
  {
    initialize_parameters();

    m_solver_x = std::make_unique<uav_ros_trackers::cvx_wrapper::CvxWrapper>(
      true, m_max_iter_xy, m_Q_xy, m_dt1, m_dt2, 0);
    m_solver_y = std::make_unique<uav_ros_trackers::cvx_wrapper::CvxWrapper>(
      true, m_max_iter_xy, m_Q_xy, m_dt1, m_dt2, 1);
    m_solver_z = std::make_unique<uav_ros_trackers::cvx_wrapper::CvxWrapper>(
      true, m_max_iter_xy, m_Q_xy, m_dt1, m_dt2, 2);
    m_solver_heading = std::make_unique<uav_ros_trackers::cvx_wrapper::CvxWrapper>(
      true, m_max_iter_z, m_Q_z, m_dt1, m_dt2, 0);

    // Initialize mpc states
    m_mpc_state = Eigen::MatrixXd::Zero(m_trans_state_count, 1);
    m_mpc_heading_state = Eigen::MatrixXd::Zero(m_heading_state_count, 1);
    m_mpc_u = Eigen::VectorXd::Zero(m_trans_input_count);

    // Initialize desired trajectory
    m_desired_traj_x = Eigen::MatrixXd::Zero(m_horizon_len, 1);
    m_desired_traj_y = Eigen::MatrixXd::Zero(m_horizon_len, 1);
    m_desired_traj_z = Eigen::MatrixXd::Zero(m_horizon_len, 1);
    m_desired_traj_heading = Eigen::MatrixXd::Zero(m_horizon_len, 1);

    // Initialize predicted trajectory
    m_predicted_trajectory =
      Eigen::MatrixXd::Zero(m_horizon_len * m_trans_state_count, 1);
    m_predicted_heading_trajectory =
      Eigen::MatrixXd::Zero(m_horizon_len * m_heading_state_count, 1);

    // Initialize timers
    m_mpc_iteration_timer =
      m_nh.createTimer(ros::Rate(m_tracker_rate), &MPCTracker::mpc_iteration, this);
  }

private:

  void mpc_iteration(const ros::TimerEvent& /* unused */)
  {
    ROS_INFO_THROTTLE(2.0, "MPCTracker::mpc_iterations");
  }

  void initialize_parameters()
  {
    ros::NodeHandle nh_private("~");
    param_util::getParamOrThrow(nh_private, "rate", m_tracker_rate);
    m_dt1 = 1.0 / m_tracker_rate;

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

    // Load solver parameters
    param_util::getParamOrThrow(nh_private, "solver/horizon_len", m_horizon_len);
    param_util::getParamOrThrow(nh_private, "solver/xy/max_iterations", m_max_iter_xy);
    param_util::getParamOrThrow(nh_private, "solver/z/max_iterations", m_max_iter_z);
    param_util::getParamOrThrow(nh_private, "solver/xy/Q", m_Q_xy);
    param_util::getParamOrThrow(nh_private, "solver/z/Q", m_Q_z);
    param_util::getParamOrThrow(nh_private, "solver/dt2", m_dt2);
  }

  std::unique_ptr<uav_ros_trackers::cvx_wrapper::CvxWrapper> m_solver_x;
  std::unique_ptr<uav_ros_trackers::cvx_wrapper::CvxWrapper> m_solver_y;
  std::unique_ptr<uav_ros_trackers::cvx_wrapper::CvxWrapper> m_solver_z;
  std::unique_ptr<uav_ros_trackers::cvx_wrapper::CvxWrapper> m_solver_heading;

  ros::NodeHandle m_nh;
  double m_tracker_rate;

  /* Solver parameters */
  int m_horizon_len;
  int m_max_iter_xy;
  int m_max_iter_z;
  std::vector<double> m_Q_xy;
  std::vector<double> m_Q_z;
  double m_dt1;
  double m_dt2;

  /* Number of states */
  int m_trans_state_count;
  int m_trans_input_count;
  int m_heading_state_count;
  int m_heading_input_count;

  /* Virtual UAV translation and heading model */
  Eigen::MatrixXd m_A;
  Eigen::MatrixXd m_B;
  Eigen::MatrixXd m_A_heading;
  Eigen::MatrixXd m_B_heading;

  /* Current state of the Virtual UAV */
  Eigen::MatrixXd m_mpc_state;
  Eigen::MatrixXd m_mpc_heading_state;
  Eigen::MatrixXd m_mpc_u;

  /* Current desired trajectory */
  Eigen::MatrixXd m_desired_traj_x;
  Eigen::MatrixXd m_desired_traj_y;
  Eigen::MatrixXd m_desired_traj_z;
  Eigen::MatrixXd m_desired_traj_heading;

  /** Current predicted trajectory */
  Eigen::MatrixXd m_predicted_trajectory;
  Eigen::MatrixXd m_predicted_heading_trajectory;

  /** Timers **/
  ros::Timer m_mpc_iteration_timer;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "mpc_tracker");
  MPCTracker mpc_tracker;
  ros::spin();
  return 0;
}