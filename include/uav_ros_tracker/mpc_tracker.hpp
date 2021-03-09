#ifndef MPC_TRACKER_HPP
#define MPC_TRACKER_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

#include <uav_ros_tracker/cvx_wrapper.h>
#include <uav_ros_lib/reconfigure_handler.hpp>
#include <uav_ros_tracker/MPCTrackerParametersConfig.h>

namespace uav_ros_tracker {

/**
 * @brief UAV MPC Tracker implementation as found at
 * https://github.com/ctu-mrs/mrs_uav_trackers.
 *
 * This class generates UAV position command based on input trajectory or input pose.
 * Internally, MPCTracker controls a virtual UAV whose current state is interpreted as a
 * position command for the actual UAV. The virtual UAV is controlled via snap commands,
 * generated from the MPC solver. Inputs to the MPC solver are desired trajectory,
 * current virtual UAV state and state constraints.
 *
 */
class MPCTracker
{

public:
  /**
   * @brief Construct a new MPCTracker object. Initialize the MPC tracker;
   * 
   * @param nh 
   */
  explicit MPCTracker(ros::NodeHandle& nh);

  /**
   * @brief Activate the MPCTracker.
   * 
   */
  std::tuple<bool, std::string> activate();

  /**
   * @brief Deactivate the MPCTracker
   * 
   */
  void deactivate();

  /**
   * @brief Reset the MPCTracker
   * 
   */
  void reset();

  /**
   * @brief Check if MPCTracker is active
   * 
   * @return true if active
   * @return false otherwise
   */
  bool is_active();

  /**
   * @brief Check if MPCTracker is tracking
   * 
   * @return true if tracking
   * @return false otherwise
   */
  bool is_tracking();
  
private:
  void initialize_parameters();

  /* timer callbacks */
  void tracking_timer(const ros::TimerEvent &event);
  void mpc_timer(const ros::TimerEvent &event);

  /* topic callbacks */
  void trajectory_callback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr &msg);
  void odom_callback(const nav_msgs::OdometryConstPtr &msg);
  void pose_callback(const geometry_msgs::PoseStampedConstPtr &msg);
  void csv_traj_callback(const std_msgs::StringConstPtr &msg);
  
  void publish_command_reference();
  void publish_predicted_trajectory();

  void calculate_mpc();
  void iterate_virtual_uav_model();
  void set_virtual_uav_state(const nav_msgs::Odometry &msg);
  void set_single_ref_point(double x, double y, double z, double heading);

  /* trajectory helper methods */
  void load_trajectory(const trajectory_msgs::MultiDOFJointTrajectory &traj_msg);
  void interpolate_desired_trajectory();
  std::tuple<Eigen::MatrixXd, Eigen::MatrixXd> filter_desired_traj_xy(
    const Eigen::VectorXd &des_x_trajectory,
    const Eigen::VectorXd &des_y_trajectory,
    double max_speed_x,
    double max_speed_y);
  Eigen::MatrixXd filter_desired_traj_z(const Eigen::VectorXd &des_z_trajectory,
    double max_ascending_speed,
    double max_descending_speed);

  /* Solver objects */
  std::unique_ptr<uav_ros_tracker::cvx_wrapper::CvxWrapper> m_solver_x;
  std::unique_ptr<uav_ros_tracker::cvx_wrapper::CvxWrapper> m_solver_y;
  std::unique_ptr<uav_ros_tracker::cvx_wrapper::CvxWrapper> m_solver_z;
  std::unique_ptr<uav_ros_tracker::cvx_wrapper::CvxWrapper> m_solver_heading;

  ros::NodeHandle m_nh;
  double m_tracker_rate;
  std::string m_frame_id;
  
  /* MPCTracker flags */
  bool m_is_initialized = false;
  bool m_is_trajectory_tracking = false;
  bool m_is_active = true;
  bool m_request_permission = true;
  bool m_goto_trajectory_start;

  /* Solver parameters */
  int m_horizon_len;
  int m_max_iter_xy;
  int m_max_iter_z;
  int m_max_iter_heading;
  std::vector<double> m_Q_xy;
  std::vector<double> m_Q_z;
  std::vector<double> m_Q_heading;
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

  Eigen::MatrixXd m_A_orig;
  Eigen::MatrixXd m_B_orig;
  Eigen::MatrixXd m_A_heading_orig;
  Eigen::MatrixXd m_B_heading_orig;

  ros::Time m_model_iterations_last_time;
  bool m_is_model_first_iter = true;

  /* Current state of the Virtual UAV */
  Eigen::MatrixXd m_mpc_state;
  Eigen::MatrixXd m_mpc_heading_state;
  Eigen::MatrixXd m_mpc_u;
  double m_mpc_u_heading;

  /* Current desired trajectory */
  Eigen::MatrixXd m_desired_traj_x;
  Eigen::MatrixXd m_desired_traj_y;
  Eigen::MatrixXd m_desired_traj_z;
  Eigen::MatrixXd m_desired_traj_heading;

  Eigen::MatrixXd m_desired_traj_filtered_x;
  Eigen::MatrixXd m_desired_traj_filtered_y;
  Eigen::MatrixXd m_desired_traj_filtered_z;
  Eigen::MatrixXd m_desired_traj_filtered_heading;

  Eigen::VectorXd m_desired_traj_whole_x;
  Eigen::VectorXd m_desired_traj_whole_y;
  Eigen::VectorXd m_desired_traj_whole_z;
  Eigen::VectorXd m_desired_traj_whole_heading;

  /** Current predicted trajectory */
  Eigen::MatrixXd m_predicted_trajectory;
  Eigen::MatrixXd m_predicted_heading_trajectory;

  /** Trajectory parameters **/
  int m_trajectory_size;
  int m_trajectory_idx;
  double m_trajectory_dt;

  /** Timers **/
  ros::Timer m_mpc_timer_timer;
  ros::Timer m_tracking_timer;

  /** Subscribers **/
  nav_msgs::Odometry m_curr_odom;
  ros::Subscriber m_odom_sub;
  ros::Subscriber m_pose_sub;
  ros::Subscriber m_traj_sub;
  ros::Subscriber m_csv_traj_sub;

  /** Publishers **/
  ros::Publisher m_traj_point_pub;
  ros::Publisher m_mpc_desired_traj_pub;
  ros::Publisher m_mpc_desired_filtered_traj_pub;
  ros::Publisher m_mpc_predicted_traj_pub;
  ros::Publisher m_processed_trajectory_pub;
  ros::Publisher m_attitude_target_debug_pub;
  ros::Publisher m_current_pose_debug_pub;
  ros::Publisher m_curr_traj_point_debug_pub;

  /* Dynamic Reconfigre */
  std::unique_ptr<
    ros_util::ReconfigureHandler<uav_ros_tracker::MPCTrackerParametersConfig>>
    m_reconfigure_handler;
};

}// namespace uav_ros_tracker

#endif /* MPC_TRACKER_HPP */