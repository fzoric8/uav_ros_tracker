#include <memory>

#include <geometry_msgs/PoseArray.h>
#include <mavros_msgs/AttitudeTarget.h>

#include <uav_ros_tracker/mpc_tracker.hpp>
#include <uav_ros_lib/param_util.hpp>
#include <uav_ros_lib/ros_convert.hpp>
#include <uav_ros_lib/nonlinear_filters.hpp>
#include <uav_ros_lib/trajectory/trajectory_helper.hpp>

uav_ros_tracker::MPCTracker::MPCTracker(ros::NodeHandle &nh) : m_nh(nh)
{
  initialize_parameters();

  m_solver_x = std::make_unique<uav_ros_tracker::cvx_wrapper::CvxWrapper>(
    false, m_max_iter_xy, m_Q_xy, m_dt1, m_dt2, 0);
  m_solver_y = std::make_unique<uav_ros_tracker::cvx_wrapper::CvxWrapper>(
    false, m_max_iter_xy, m_Q_xy, m_dt1, m_dt2, 1);
  m_solver_z = std::make_unique<uav_ros_tracker::cvx_wrapper::CvxWrapper>(
    false, m_max_iter_z, m_Q_z, m_dt1, m_dt2, 2);
  m_solver_heading = std::make_unique<uav_ros_tracker::cvx_wrapper::CvxWrapper>(
    false, m_max_iter_heading, m_Q_heading, m_dt1, m_dt2, 0);

  // Initialize mpc states
  m_mpc_state = Eigen::MatrixXd::Zero(m_trans_state_count, 1);
  m_mpc_heading_state = Eigen::MatrixXd::Zero(m_heading_state_count, 1);
  m_mpc_u = Eigen::VectorXd::Zero(m_trans_input_count);
  m_mpc_u_heading = 0;

  // Initialize desired trajectory
  m_desired_traj_x = Eigen::MatrixXd::Zero(m_horizon_len, 1);
  m_desired_traj_y = Eigen::MatrixXd::Zero(m_horizon_len, 1);
  m_desired_traj_z = Eigen::MatrixXd::Zero(m_horizon_len, 1);
  m_desired_traj_heading = Eigen::MatrixXd::Zero(m_horizon_len, 1);

  // Initialize predicted trajectory
  m_predicted_trajectory = Eigen::MatrixXd::Zero(m_horizon_len * m_trans_state_count, 1);
  m_predicted_heading_trajectory =
    Eigen::MatrixXd::Zero(m_horizon_len * m_trans_state_count, 1);

  m_is_trajectory_tracking = false;
  m_goto_trajectory_start = false;
  m_is_initialized = false;

  // Initialize subscribers
  m_odom_sub = m_nh.subscribe("odometry", 1, &MPCTracker::odom_callback, this);
  m_pose_sub = m_nh.subscribe("tracker/input_pose", 1, &MPCTracker::pose_callback, this);
  m_traj_sub =
    m_nh.subscribe("tracker/input_trajectory", 1, &MPCTracker::trajectory_callback, this);
  m_csv_traj_sub =
    m_nh.subscribe("tracker/csv_input", 1, &MPCTracker::csv_traj_callback, this);

  // Initialize publishers
  m_traj_point_pub =
    m_nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("position_command", 1);
  m_mpc_desired_traj_pub =
    m_nh.advertise<geometry_msgs::PoseArray>("debug/mpc_desired_trajectory", 1);
  m_mpc_desired_filtered_traj_pub =
    m_nh.advertise<geometry_msgs::PoseArray>("debug/mpc_desired_filtered_trajectory", 1);
  m_mpc_predicted_traj_pub =
    m_nh.advertise<geometry_msgs::PoseArray>("debug/mpc_predicted_trajectory", 1);
  m_processed_trajectory_pub =
    m_nh.advertise<geometry_msgs::PoseArray>("debug/processed_trajectory", 1);
  m_attitude_target_debug_pub =
    m_nh.advertise<mavros_msgs::AttitudeTarget>("debug/mpc_attitude_target", 1);
  m_current_pose_debug_pub =
    m_nh.advertise<geometry_msgs::PoseStamped>("debug/virtual_uav_pose", 1);
  m_curr_traj_point_debug_pub =
    m_nh.advertise<geometry_msgs::PoseStamped>("debug/curr_traj_point", 1);

  // Initialize timers
  m_mpc_timer_timer =
    m_nh.createTimer(ros::Rate(m_tracker_rate), &MPCTracker::mpc_timer, this);
  m_tracking_timer =
    m_nh.createTimer(ros::Rate(1.0), &MPCTracker::tracking_timer, this, false, false);
}

std::tuple<bool, std::string> uav_ros_tracker::MPCTracker::activate()
{
  ROS_INFO("MPCTracker::activate");

  if (!m_is_initialized) {
    return { false, "MPCTracker::activate - odometry callback missing!" };
  }

  // If a lot of time passed from last odom measurement, something is wrong and don't
  // activate
  if ((ros::Time::now() - m_curr_odom.header.stamp).toSec() > 0.1) {
    return { false, "MPCTracker::activate - last odometry message is too old." };
  }

  m_is_active = true;

  set_virtual_uav_state(m_curr_odom);
  return { true, "MPCTracker::activate - success" };
}
void uav_ros_tracker::MPCTracker::deactivate()
{
  ROS_INFO("MPCTracker::deactivate");
  m_is_active = false;
  m_is_trajectory_tracking = false;
  m_trajectory_idx = 0;
  m_tracking_timer.stop();

  set_virtual_uav_state(m_curr_odom);
  set_single_ref_point(
    m_mpc_state(0, 0), m_mpc_state(4, 0), m_mpc_state(8, 0), m_mpc_heading_state(0, 0));
}
void uav_ros_tracker::MPCTracker::reset() { ROS_INFO("MPCTracker::reset"); }

bool uav_ros_tracker::MPCTracker::is_active() { return m_is_active; }
bool uav_ros_tracker::MPCTracker::is_tracking() { return m_is_trajectory_tracking; }

void uav_ros_tracker::MPCTracker::tracking_timer(const ros::TimerEvent & /* unused */)
{
  if (!m_is_active) {
    ROS_WARN_THROTTLE(
      1.0, "MPCTracker::tracking_timer - tracker not active, but trajectory recieved");
    return;
  }

  nav_msgs::Odometry curr_virtual_odom;
  curr_virtual_odom.pose.pose.position.x = m_mpc_state(0, 0);
  curr_virtual_odom.pose.pose.position.y = m_mpc_state(4, 0);
  curr_virtual_odom.pose.pose.position.z = m_mpc_state(8, 0);

  trajectory_msgs::MultiDOFJointTrajectoryPoint first_traj_point;
  first_traj_point.transforms.push_back(geometry_msgs::Transform());
  first_traj_point.transforms.front().translation.x =
    m_desired_traj_whole_x(m_trajectory_idx);
  first_traj_point.transforms.front().translation.y =
    m_desired_traj_whole_y(m_trajectory_idx);
  first_traj_point.transforms.front().translation.z =
    m_desired_traj_whole_z(m_trajectory_idx);

  if (m_goto_trajectory_start
      && trajectory_helper::is_close_to_reference(
           first_traj_point, curr_virtual_odom, 0.25)) {

    ROS_INFO("MPCTracker::tracking_timer - first point reached");
    m_goto_trajectory_start = false;
    m_is_trajectory_tracking = true;
  } else if (m_is_trajectory_tracking) {
    m_trajectory_idx++;
  }

  ROS_INFO_STREAM_THROTTLE(1.0,
    "MPCTracker::tracking_timer - [" << m_trajectory_idx << "/" << m_trajectory_size
                                     << "]");
  if (m_trajectory_idx == m_trajectory_size) {
    m_is_trajectory_tracking = false;
    m_trajectory_idx = m_trajectory_size - 1;
    m_tracking_timer.stop();
    ROS_INFO("MPCTracker::tracking_timer - trajectory tracking done");
  }

  geometry_msgs::PoseStamped debug_traj_ref;
  debug_traj_ref.header.frame_id = m_frame_id;
  debug_traj_ref.header.stamp = ros::Time::now();
  debug_traj_ref.pose.position.x = m_desired_traj_whole_x(m_trajectory_idx);
  debug_traj_ref.pose.position.y = m_desired_traj_whole_y(m_trajectory_idx);
  debug_traj_ref.pose.position.z = m_desired_traj_whole_z(m_trajectory_idx);
  debug_traj_ref.pose.orientation =
    ros_convert::calculate_quaternion(m_desired_traj_whole_heading(m_trajectory_idx));
  m_curr_traj_point_debug_pub.publish(debug_traj_ref);
}

void uav_ros_tracker::MPCTracker::mpc_timer(const ros::TimerEvent & /* unused */)
{
  if (!m_is_initialized) {
    ROS_INFO_THROTTLE(1.0, "MPCTracker::mpc_timers - not initialized");
    return;
  }

  if (!m_is_active) {
    ROS_INFO_THROTTLE(1.0, "MPCTracker::mpc_timers - not active");
    return;
  }

  ROS_INFO_THROTTLE(10.0, "MPCTracker::mpc_timers");


  if (m_is_trajectory_tracking) { interpolate_desired_trajectory(); }

  // If trajectory tracking is done and request permission is enabled
  if (!m_is_trajectory_tracking && m_request_permission) {
    nav_msgs::Odometry curr_virtual_odom;
    curr_virtual_odom.pose.pose.position.x = m_mpc_state(0, 0);
    curr_virtual_odom.pose.pose.position.y = m_mpc_state(4, 0);
    curr_virtual_odom.pose.pose.position.z = m_mpc_state(8, 0);

    trajectory_msgs::MultiDOFJointTrajectoryPoint traj_point;
    traj_point.transforms.push_back(geometry_msgs::Transform());
    traj_point.transforms.front().translation.x = m_desired_traj_x(m_horizon_len - 1);
    traj_point.transforms.front().translation.y = m_desired_traj_y(m_horizon_len - 1);
    traj_point.transforms.front().translation.z = m_desired_traj_z(m_horizon_len - 1);

    if (trajectory_helper::is_close_to_reference(traj_point, curr_virtual_odom, 0.01)) {
      ROS_INFO("MPCTracker::mpc_timers - tracking is done!");
      deactivate();
      return;
    }
  }

  calculate_mpc();
  iterate_virtual_uav_model();
  publish_command_reference();
}

void uav_ros_tracker::MPCTracker::publish_command_reference()
{
  // Check if the states are finite
  bool is_state_finite = true;
  for (int i = 0; i < m_trans_state_count; i++) {
    if (!std::isfinite(m_mpc_state(i, 0))) { is_state_finite = false; }
  }

  // Construct the point command
  trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_ref =
    ros_convert::to_trajectory_point(m_curr_odom.pose.pose.position.x,
      m_curr_odom.pose.pose.position.y,
      m_curr_odom.pose.pose.position.z,
      m_curr_odom.pose.pose.orientation.x,
      m_curr_odom.pose.pose.orientation.y,
      m_curr_odom.pose.pose.orientation.z,
      m_curr_odom.pose.pose.orientation.w);

  // Concstruct the pose command
  geometry_msgs::PoseStamped virtual_uav_pose;
  virtual_uav_pose.header.frame_id = m_frame_id;
  virtual_uav_pose.header.stamp = ros::Time::now();
  virtual_uav_pose.pose.position.x = m_curr_odom.pose.pose.position.x;
  virtual_uav_pose.pose.position.y = m_curr_odom.pose.pose.position.y;
  virtual_uav_pose.pose.position.z = m_curr_odom.pose.pose.position.z;
  virtual_uav_pose.pose.orientation = m_curr_odom.pose.pose.orientation;

  if (is_state_finite) {
    // TODO: Make a msg to accomodate jerk commands
    trajectory_point_ref.transforms.front().translation.x = m_mpc_state(0, 0);
    trajectory_point_ref.velocities.front().linear.x = m_mpc_state(1, 0);
    trajectory_point_ref.accelerations.front().linear.x = m_mpc_state(2, 0);

    trajectory_point_ref.transforms.front().translation.y = m_mpc_state(4, 0);
    trajectory_point_ref.velocities.front().linear.y = m_mpc_state(5, 0);
    trajectory_point_ref.accelerations.front().linear.y = m_mpc_state(6, 0);

    trajectory_point_ref.transforms.front().translation.z = m_mpc_state(8, 0);
    trajectory_point_ref.velocities.front().linear.z = m_mpc_state(9, 0);
    trajectory_point_ref.accelerations.front().linear.z = m_mpc_state(10, 0);

    virtual_uav_pose.pose.position.x = m_mpc_state(0, 0);
    virtual_uav_pose.pose.position.y = m_mpc_state(4, 0);
    virtual_uav_pose.pose.position.z = m_mpc_state(8, 0);

  } else {
    ROS_ERROR_THROTTLE(
      1.0, "MPCTracker::publish_command_reference - MPC state is not finite.");
    trajectory_point_ref.velocities.front().linear.x = 0;
    trajectory_point_ref.accelerations.front().linear.x = 0;

    trajectory_point_ref.velocities.front().linear.y = 0;
    trajectory_point_ref.accelerations.front().linear.y = 0;

    trajectory_point_ref.velocities.front().linear.z = 0;
    trajectory_point_ref.accelerations.front().linear.z = 0;
  }

  // Check if the heading states are finite
  bool is_heading_finite = true;
  for (int i = 0; i < m_heading_state_count; i++) {
    if (!std::isfinite(m_mpc_heading_state(i, 0))) { is_heading_finite = false; }
  }

  if (is_heading_finite) {
    // TODO: Make a message to accomodate heading command
    // TODO: Make a message to accomodate angular jerk commands
    trajectory_point_ref.transforms.front().rotation =
      ros_convert::calculate_quaternion(m_mpc_heading_state(0, 0));
    trajectory_point_ref.velocities.front().angular.z = m_mpc_heading_state(1, 0);
    trajectory_point_ref.accelerations.front().angular.z = m_mpc_heading_state(2, 0);

    virtual_uav_pose.pose.orientation = trajectory_point_ref.transforms.front().rotation;
  } else {
    trajectory_point_ref.velocities.front().angular.z = 0;
    trajectory_point_ref.accelerations.front().angular.z = 0;
  }

  // TODO: This message doesn't even have a timestamp WTF?!
  // TODO: Make a trajectory point message with a timestamp in the header...

  m_traj_point_pub.publish(trajectory_point_ref);
  m_current_pose_debug_pub.publish(virtual_uav_pose);
}

void uav_ros_tracker::MPCTracker::calculate_mpc()
{
  double iters_z = 0;
  double iters_x = 0;
  double iters_y = 0;
  double iters_heading = 0;

  // Get newest tracker parasm
  auto constraints = m_reconfigure_handler->getData();

  /* Do the z-axis MPC */

  // Calculated filtered z-axis reference
  Eigen::MatrixXd des_z_filtered = filter_desired_traj_z(
    m_desired_traj_z, constraints.z_velocity, constraints.z_velocity);

  // Set the z-axis initial state
  Eigen::MatrixXd initial_z = Eigen::MatrixXd::Zero(m_trans_state_count, 1);
  initial_z(0, 0) = m_mpc_state(8, 0);
  initial_z(1, 0) = m_mpc_state(9, 0);
  initial_z(2, 0) = m_mpc_state(10, 0);
  initial_z(3, 0) = m_mpc_state(11, 0);

  // Set z solver inputs
  m_solver_z->setVelQ(0);
  m_solver_z->setInitialState(initial_z);
  m_solver_z->loadReference(des_z_filtered);
  m_solver_z->setLimits(constraints.z_velocity,
    constraints.z_velocity,
    constraints.z_acceleration,
    constraints.z_acceleration,
    constraints.z_jerk,
    constraints.z_jerk,
    constraints.z_snap,
    constraints.z_snap);

  // Run z solver
  iters_z += m_solver_z->solveCvx();
  m_solver_z->getStates(m_predicted_trajectory);
  m_mpc_u(2) = m_solver_z->getFirstControlInput();

  /* Do the x-axis MPC */

  // Calculate filterd xy-axis reference
  auto [des_x_filtered, des_y_filtered] = filter_desired_traj_xy(
    m_desired_traj_x, m_desired_traj_y, constraints.xy_velocity, constraints.xy_velocity);

  // Set x-axis initial state
  Eigen::MatrixXd initial_x = Eigen::MatrixXd::Zero(m_trans_state_count, 1);
  initial_x(0, 0) = m_mpc_state(0, 0);
  initial_x(1, 0) = m_mpc_state(1, 0);
  initial_x(2, 0) = m_mpc_state(2, 0);
  initial_x(3, 0) = m_mpc_state(3, 0);

  // Set x solver inputs
  m_solver_x->setVelQ(0);
  m_solver_x->setInitialState(initial_x);
  m_solver_x->loadReference(des_x_filtered);
  m_solver_x->setLimits(constraints.xy_velocity,
    constraints.xy_velocity,
    constraints.xy_acceleration,
    constraints.xy_acceleration,
    constraints.xy_jerk,
    constraints.xy_jerk,
    constraints.xy_snap,
    constraints.xy_snap);

  // Run x solver
  iters_x += m_solver_x->solveCvx();
  m_solver_x->getStates(m_predicted_trajectory);
  m_mpc_u(0) = m_solver_x->getFirstControlInput();

  /* Do the y-axis MPC */

  // Set y-axis initial state
  Eigen::MatrixXd initial_y = Eigen::MatrixXd::Zero(m_trans_state_count, 1);
  initial_y(0, 0) = m_mpc_state(4, 0);
  initial_y(1, 0) = m_mpc_state(5, 0);
  initial_y(2, 0) = m_mpc_state(6, 0);
  initial_y(3, 0) = m_mpc_state(7, 0);

  // Set y solver inputs
  m_solver_y->setVelQ(0);
  m_solver_y->setInitialState(initial_y);
  m_solver_y->loadReference(des_y_filtered);
  m_solver_y->setLimits(constraints.xy_velocity,
    constraints.xy_velocity,
    constraints.xy_acceleration,
    constraints.xy_acceleration,
    constraints.xy_jerk,
    constraints.xy_jerk,
    constraints.xy_snap,
    constraints.xy_snap);

  // Run x solver
  iters_y += m_solver_y->solveCvx();
  m_solver_y->getStates(m_predicted_trajectory);
  m_mpc_u(1) = m_solver_y->getFirstControlInput();

  /* Do the heading MPC */

  // unwrap the heading reference
  m_desired_traj_heading(0, 0) =
    ros_convert::unwrap(m_desired_traj_heading(0, 0), m_mpc_heading_state(0));
  for (int i = 1; i < m_horizon_len; i++) {
    m_desired_traj_heading(i, 0) =
      ros_convert::unwrap(m_desired_traj_heading(i, 0), m_desired_traj_heading(i - 1, 0));
  }

  // Set heading solver inputs
  m_solver_heading->setVelQ(0);
  m_solver_heading->setInitialState(m_mpc_heading_state);
  m_solver_heading->loadReference(m_desired_traj_heading);
  m_solver_heading->setLimits(constraints.heading_velocity,
    constraints.heading_velocity,
    constraints.heading_acceleration,
    constraints.heading_acceleration,
    constraints.heading_jerk,
    constraints.heading_jerk,
    constraints.heading_snap,
    constraints.heading_snap);

  // Run heading solver
  iters_heading += m_solver_heading->solveCvx();
  m_solver_heading->getStates(m_predicted_heading_trajectory);
  m_mpc_u_heading = m_solver_heading->getFirstControlInput();

  // Saturate heading
  m_mpc_u(0) = nonlinear_filters::saturation(
    m_mpc_u(0), -1.01 * constraints.xy_snap, 1.01 * constraints.xy_snap);
  m_mpc_u(1) = nonlinear_filters::saturation(
    m_mpc_u(1), -1.01 * constraints.xy_snap, 1.01 * constraints.xy_snap);
  m_mpc_u(2) = nonlinear_filters::saturation(
    m_mpc_u(2), -1.01 * constraints.z_snap, 1.01 * constraints.z_snap);
  m_mpc_u_heading = nonlinear_filters::saturation(
    m_mpc_u_heading, -1.01 * constraints.heading_snap, 1.01 * constraints.heading_snap);

  mavros_msgs::AttitudeTarget target;
  target.header.stamp = ros::Time::now();
  target.body_rate.x = m_mpc_u(0);
  target.body_rate.y = m_mpc_u(1);
  target.body_rate.z = m_mpc_u_heading;
  target.thrust = m_mpc_u(2);
  m_attitude_target_debug_pub.publish(target);

  // Publish desired trajectory
  geometry_msgs::PoseArray debug_des_filtered;
  debug_des_filtered.header.stamp = ros::Time::now();
  debug_des_filtered.header.frame_id = m_frame_id;

  geometry_msgs::PoseArray debug_des;
  debug_des.header.stamp = ros::Time::now();
  debug_des.header.frame_id = m_frame_id;

  for (int i = 0; i < m_horizon_len; i++) {

    geometry_msgs::Pose filt_des_pose;
    geometry_msgs::Pose des_pose;

    filt_des_pose.position.x = des_x_filtered(i, 0);
    filt_des_pose.position.y = des_y_filtered(i, 0);
    filt_des_pose.position.z = des_z_filtered(i, 0);

    des_pose.position.x = m_desired_traj_x(i, 0);
    des_pose.position.y = m_desired_traj_y(i, 0);
    des_pose.position.z = m_desired_traj_z(i, 0);

    filt_des_pose.orientation =
      ros_convert::calculate_quaternion(m_desired_traj_heading(i));
    des_pose.orientation = ros_convert::calculate_quaternion(m_desired_traj_heading(i));

    debug_des_filtered.poses.push_back(filt_des_pose);
    debug_des.poses.push_back(des_pose);
  }

  m_mpc_desired_filtered_traj_pub.publish(debug_des_filtered);
  m_mpc_desired_traj_pub.publish(debug_des);

  publish_predicted_trajectory();

  // Assign filtered variables
  m_desired_traj_filtered_x = des_x_filtered;
  m_desired_traj_filtered_y = des_y_filtered;
  m_desired_traj_filtered_z = des_z_filtered;
  m_desired_traj_filtered_heading = m_desired_traj_heading;
}

void uav_ros_tracker::MPCTracker::publish_predicted_trajectory()
{
  // Publish predicted trajectory
  geometry_msgs::PoseArray debug_trajectory_out;
  debug_trajectory_out.header.stamp = ros::Time::now();
  debug_trajectory_out.header.frame_id = m_frame_id;

  for (int i = 0; i < m_horizon_len; i++) {

    geometry_msgs::Pose newPose;

    newPose.position.x = m_predicted_trajectory(i * m_trans_state_count);
    newPose.position.y = m_predicted_trajectory(i * m_trans_state_count + 4);
    newPose.position.z = m_predicted_trajectory(i * m_trans_state_count + 8);

    newPose.orientation = ros_convert::calculate_quaternion(
      m_predicted_heading_trajectory(i * m_trans_state_count));

    debug_trajectory_out.poses.push_back(newPose);
  }

  m_mpc_predicted_traj_pub.publish(debug_trajectory_out);
}

void uav_ros_tracker::MPCTracker::iterate_virtual_uav_model()
{

  if (m_is_model_first_iter) {

    m_model_iterations_last_time = ros::Time::now();
    m_is_model_first_iter = false;

  } else {

    double dt = (ros::Time::now() - m_model_iterations_last_time).toSec();

    if (dt > 0.001 && dt < 2.0) {

      // clang-format off
        m_A  << 1, dt, 0.5*dt*dt, 0,         0, 0,  0,         0,         0, 0,  0,         0,
              0, 1,  dt,        0.5*dt*dt, 0, 0,  0,         0,         0, 0,  0,         0,
              0, 0,  1,         dt,        0, 0,  0,         0,         0, 0,  0,         0,
              0, 0,  0,         1,         0, 0,  0,         0,         0, 0,  0,         0,
              0, 0,  0,         0,         1, dt, 0.5*dt*dt, 0,         0, 0,  0,         0,
              0, 0,  0,         0,         0, 1,  dt,        0.5*dt*dt, 0, 0,  0,         0,
              0, 0,  0,         0,         0, 0,  1,         dt,        0, 0,  0,         0,
              0, 0,  0,         0,         0, 0,  0,         1,         0, 0,  0,         0,
              0, 0,  0,         0,         0, 0,  0,         0,         1, dt, 0.5*dt*dt, 0,
              0, 0,  0,         0,         0, 0,  0,         0,         0, 1,  dt,        0.5*dt*dt,
              0, 0,  0,         0,         0, 0,  0,         0,         0, 0,  1,         dt,
              0, 0,  0,         0,         0, 0,  0,         0,         0, 0,  0,         1;

        m_B  << 0,  0,  0,
              0,  0,  0,
              0,  0,  0,
              dt, 0,  0,
              0,  0,  0,
              0,  0,  0,
              0,  0,  0,
              0,  dt, 0,
              0,  0,  0,
              0,  0,  0,
              0,  0,  0,
              0,  0,  dt;

        m_A_heading  << 1, dt, 0.5*dt*dt, 0,
                      0, 1,  dt,        0.5*dt*dt,
                      0, 0,  1,         dt,
                      0, 0,  0,         1;

        m_B_heading  << 0,
                      0,
                      0,
                      dt;

      // clang-format on
    } else {
      ROS_WARN_STREAM_THROTTLE(
        1.0, "MPCTracker::iterate_virtual_uav_model - weird dt: " << dt);

      m_A = m_A_orig;
      m_B = m_B_orig;

      m_A_heading = m_A_heading_orig;
      m_B_heading = m_B_heading_orig;
    }

    m_model_iterations_last_time = ros::Time::now();
  }


  m_mpc_state = m_A * m_mpc_state + m_B * m_mpc_u;
  m_mpc_heading_state = m_A_heading * m_mpc_heading_state + m_B_heading * m_mpc_u_heading;

  m_mpc_heading_state(0) = ros_convert::wrapMinMax(m_mpc_heading_state(0), -M_PI, M_PI);
}

void uav_ros_tracker::MPCTracker::interpolate_desired_trajectory()
{
  // interpolate the trajectory points and fill in the desired_trajectory vector
  for (int i = 0; i < m_horizon_len; i++) {

    double first_time = m_dt1 + i * m_dt2;

    int first_idx = m_trajectory_idx + floor(first_time / m_trajectory_dt);
    int second_idx = first_idx + 1;

    double interp_coeff = std::fmod(first_time / m_trajectory_dt, 1.0);

    if (second_idx >= m_trajectory_size) { second_idx = m_trajectory_size - 1; }
    if (first_idx >= m_trajectory_size) { first_idx = m_trajectory_size - 1; }


    m_desired_traj_x(i, 0) = m_desired_traj_whole_x(m_trajectory_idx + i);
    m_desired_traj_y(i, 0) = m_desired_traj_whole_y(m_trajectory_idx + i);
    m_desired_traj_z(i, 0) = m_desired_traj_whole_z(m_trajectory_idx + i);

    m_desired_traj_heading(i, 0) = m_desired_traj_whole_heading(m_trajectory_idx + i);
  }
}

void uav_ros_tracker::MPCTracker::csv_traj_callback(const std_msgs::StringConstPtr &msg)
{
  trajectory_msgs::MultiDOFJointTrajectory csv_trajectory;

  try {
    csv_trajectory = trajectory_helper::trajectory_from_csv(msg->data, m_frame_id, false);
  } catch (std::runtime_error &e) {
    ROS_ERROR_STREAM(
      "MPCTracker::csv_traj_callback - unable to read trajectory: " << msg->data);
    return;
  }

  ROS_INFO("MPCTracker::csv_traj_callback - loading csv trajectory");
  m_is_trajectory_tracking = false;
  m_tracking_timer.stop();
  set_virtual_uav_state(m_curr_odom);
  load_trajectory(csv_trajectory);
}

void uav_ros_tracker::MPCTracker::trajectory_callback(
  const trajectory_msgs::MultiDOFJointTrajectoryConstPtr &msg)
{
  m_is_trajectory_tracking = false;
  m_tracking_timer.stop();
  set_virtual_uav_state(m_curr_odom);
  load_trajectory(*msg);
}

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd>
  uav_ros_tracker::MPCTracker::filter_desired_traj_xy(
    const Eigen::VectorXd &des_x_trajectory,
    const Eigen::VectorXd &des_y_trajectory,
    double max_speed_x,
    double max_speed_y)
{
  Eigen::MatrixXd filtered_x_trajectory = Eigen::MatrixXd::Zero(m_horizon_len, 1);
  Eigen::MatrixXd filtered_y_trajectory = Eigen::MatrixXd::Zero(m_horizon_len, 1);

  double difference_x;
  double difference_y;
  double max_sample_x;
  double max_sample_y;

  for (int i = 0; i < m_horizon_len; i++) {

    if (i == 0) {
      max_sample_x = max_speed_x * m_dt1;
      max_sample_y = max_speed_y * m_dt1;
      difference_x = des_x_trajectory(i, 0) - m_mpc_state(0, 0);
      difference_y = des_y_trajectory(i, 0) - m_mpc_state(4, 0);
    } else {
      max_sample_x = max_speed_x * m_dt2;
      max_sample_y = max_speed_y * m_dt2;
      difference_x = des_x_trajectory(i, 0) - filtered_x_trajectory(i - 1, 0);
      difference_y = des_y_trajectory(i, 0) - filtered_y_trajectory(i - 1, 0);
    }

    double direction_angle = atan2(difference_y, difference_x);
    double max_dir_sample_x = abs(max_sample_x * cos(direction_angle));
    double max_dir_sample_y = abs(max_sample_y * sin(direction_angle));

    if (max_sample_x > max_dir_sample_x) { max_sample_x = max_dir_sample_x; }
    if (max_sample_y > max_dir_sample_y) { max_sample_y = max_dir_sample_y; }

    // saturate the difference
    difference_x =
      nonlinear_filters::saturation(difference_x, -max_sample_x, max_sample_x);
    difference_y =
      nonlinear_filters::saturation(difference_y, -max_sample_y, max_sample_y);

    if (i == 0) {
      filtered_x_trajectory(i, 0) = m_mpc_state(0, 0) + difference_x;
      filtered_y_trajectory(i, 0) = m_mpc_state(4, 0) + difference_y;
    } else {
      filtered_x_trajectory(i, 0) = filtered_x_trajectory(i - 1, 0) + difference_x;
      filtered_y_trajectory(i, 0) = filtered_y_trajectory(i - 1, 0) + difference_y;
    }
  }

  return std::make_tuple(filtered_x_trajectory, filtered_y_trajectory);
}


Eigen::MatrixXd uav_ros_tracker::MPCTracker::filter_desired_traj_z(
  const Eigen::VectorXd &des_z_trajectory,
  const double max_ascending_speed,
  const double max_descending_speed)
{

  double difference_z;
  double max_sample_z;

  Eigen::MatrixXd filtered_trajectory = Eigen::MatrixXd::Zero(m_horizon_len, 1);

  double current_z = m_mpc_state(8, 0);

  for (int i = 0; i < m_horizon_len; i++) {

    if (i == 0) {

      difference_z = des_z_trajectory(i, 0) - current_z;

      if (difference_z > 0) {
        max_sample_z = max_ascending_speed * m_dt1;
      } else {
        max_sample_z = max_descending_speed * m_dt1;
      }

    } else {

      difference_z = des_z_trajectory(i, 0) - filtered_trajectory(i - 1, 0);

      if (difference_z > 0) {
        max_sample_z = max_ascending_speed * m_dt2;
      } else {
        max_sample_z = max_descending_speed * m_dt2;
      }
    }

    // saturate the difference
    difference_z =
      nonlinear_filters::saturation(difference_z, -max_sample_z, max_sample_z);

    if (i == 0) {
      filtered_trajectory(i, 0) = current_z + difference_z;
    } else {
      filtered_trajectory(i, 0) = filtered_trajectory(i - 1, 0) + difference_z;
    }
  }

  return filtered_trajectory;
}

void uav_ros_tracker::MPCTracker::odom_callback(const nav_msgs::OdometryConstPtr &msg)
{
  if (!m_is_initialized) {
    set_virtual_uav_state(*msg);
    set_single_ref_point(
      m_mpc_state(0, 0), m_mpc_state(4, 0), m_mpc_state(8, 0), m_mpc_heading_state(0, 0));
  }
  m_curr_odom = *msg;
}

void uav_ros_tracker::MPCTracker::pose_callback(
  const geometry_msgs::PoseStampedConstPtr &msg)
{
  m_tracking_timer.stop();
  m_is_trajectory_tracking = false;

  set_virtual_uav_state(m_curr_odom);
  set_single_ref_point(msg->pose.position.x,
    msg->pose.position.y,
    msg->pose.position.z,
    ros_convert::calculateYaw(msg->pose.orientation));
}

void uav_ros_tracker::MPCTracker::load_trajectory(
  const trajectory_msgs::MultiDOFJointTrajectory &traj_msg)
{
  if (!m_is_initialized) {
    ROS_WARN("MPCTracker::load_trajectory - not initialized");
    return;
  }

  if (traj_msg.points.empty()) {
    ROS_WARN("MPCTracker::load_trajectory - no points given");
    deactivate();
    return;
  }

  // TODO: trajectory_msgs::MultiDOFJointTrajectory does not have delta
  double trajectory_dt = 0.2;

  // Interpolate trajectory according to constraints and step size
  auto constraints = m_reconfigure_handler->getData();

  // Trajectory speed - dont generate a max speed trajectory

  // if (max_speed > constraints.xy_velocity) {
  //   max_speed = constraints.xy_velocity;
  // } else if (max_speed > constraints.z_velocity) {
  //   max_speed = constraints.z_velocity;
  // }

  auto interpolated_msg = trajectory_helper::interpolate_points(traj_msg,
    0.8 * sqrt(2.) * constraints.xy_velocity * trajectory_dt,
    0.8 * constraints.z_velocity * trajectory_dt);

  int trajectory_size = interpolated_msg.points.size();

  ROS_INFO_STREAM("MPCTracker::load_trajectory - recieve trajectory with "
                  << trajectory_size << " points.");

  // Allocate the desired trajectory
  m_desired_traj_whole_x = Eigen::VectorXd::Zero(trajectory_size + m_horizon_len, 1);
  m_desired_traj_whole_y = Eigen::VectorXd::Zero(trajectory_size + m_horizon_len, 1);
  m_desired_traj_whole_z = Eigen::VectorXd::Zero(trajectory_size + m_horizon_len, 1);
  m_desired_traj_whole_heading =
    Eigen::VectorXd::Zero(trajectory_size + m_horizon_len, 1);

  // Add all trajectory points
  for (int i = 0; i < trajectory_size; i++) {
    auto point = interpolated_msg.points.at(i);
    m_desired_traj_whole_x(i) = point.transforms.front().translation.x;
    m_desired_traj_whole_y(i) = point.transforms.front().translation.y;
    m_desired_traj_whole_z(i) = point.transforms.front().translation.z;
    m_desired_traj_whole_heading(i) =
      ros_convert::calculateYaw(point.transforms.front().rotation);
  }

  // extend it so it has smooth ending
  for (int i = 0; i < m_horizon_len; i++) {
    m_desired_traj_whole_x(i + trajectory_size) =
      m_desired_traj_whole_x(i + trajectory_size - 1);
    m_desired_traj_whole_y(i + trajectory_size) =
      m_desired_traj_whole_y(i + trajectory_size - 1);
    m_desired_traj_whole_z(i + trajectory_size) =
      m_desired_traj_whole_z(i + trajectory_size - 1);
    m_desired_traj_whole_heading(i + trajectory_size) =
      m_desired_traj_whole_heading(i + trajectory_size - 1);
  }

  if (m_is_trajectory_tracking) {
    m_tracking_timer.stop();
    m_is_trajectory_tracking = false;
  }

  set_single_ref_point(m_desired_traj_whole_x(0),
    m_desired_traj_whole_y(0),
    m_desired_traj_whole_z(0),
    m_desired_traj_whole_heading(0));

  // Start the tracking timer
  m_is_trajectory_tracking = false;
  m_goto_trajectory_start = true;
  m_trajectory_size = trajectory_size;
  m_trajectory_idx = 0;
  m_trajectory_dt = trajectory_dt;
  m_tracking_timer.setPeriod(ros::Duration(trajectory_dt));
  m_tracking_timer.start();


  geometry_msgs::PoseArray debug_trajectory_out;
  debug_trajectory_out.header.stamp = ros::Time::now();
  debug_trajectory_out.header.frame_id = m_frame_id;

  for (int i = 0; i < trajectory_size; i++) {

    geometry_msgs::Pose new_pose;

    new_pose.position.x = m_desired_traj_whole_x(i);
    new_pose.position.y = m_desired_traj_whole_y(i);
    new_pose.position.z = m_desired_traj_whole_z(i);

    new_pose.orientation =
      ros_convert::calculate_quaternion(m_desired_traj_whole_heading(i));

    debug_trajectory_out.poses.push_back(new_pose);
  }
  m_processed_trajectory_pub.publish(debug_trajectory_out);

  ROS_INFO("MPCTracker::load_trajectory - start trajectory tracking");
}

void uav_ros_tracker::MPCTracker::set_virtual_uav_state(const nav_msgs::Odometry &msg)
{
  m_mpc_state(0, 0) = msg.pose.pose.position.x;
  m_mpc_state(1, 0) = msg.twist.twist.linear.x;
  m_mpc_state(2, 0) = 0;
  m_mpc_state(3, 0) = 0;

  m_mpc_state(4, 0) = msg.pose.pose.position.y;
  m_mpc_state(5, 0) = msg.twist.twist.linear.y;
  m_mpc_state(6, 0) = 0;
  m_mpc_state(7, 0) = 0;

  m_mpc_state(8, 0) = msg.pose.pose.position.z;
  m_mpc_state(9, 0) = msg.twist.twist.linear.z;
  m_mpc_state(10, 0) = 0;
  m_mpc_state(11, 0) = 0;

  auto yaw = ros_convert::calculateYaw(msg.pose.pose.orientation);
  m_mpc_heading_state(0, 0) = yaw;
  m_mpc_heading_state(1, 0) = msg.twist.twist.linear.z;
  m_mpc_heading_state(2, 0) = 0;
  m_mpc_heading_state(3, 0) = 0;

  m_is_initialized = true;
}

void uav_ros_tracker::MPCTracker::set_single_ref_point(double x,
  double y,
  double z,
  double heading)
{
  m_desired_traj_x.fill(x);
  m_desired_traj_y.fill(y);
  m_desired_traj_z.fill(z);
  m_desired_traj_heading.fill(heading);
}

void uav_ros_tracker::MPCTracker::initialize_parameters()
{
  ros::NodeHandle nh_private("~");
  param_util::getParamOrThrow(nh_private, "frame_id", m_frame_id);
  param_util::getParamOrThrow(nh_private, "rate", m_tracker_rate);
  param_util::getParamOrThrow(nh_private, "request_permission", m_request_permission);
  if (m_request_permission) { m_is_active = false; }
  m_dt1 = 1.0 / m_tracker_rate;

  // Load translation parameters
  param_util::getParamOrThrow(
    nh_private, "model/translation/n_states", m_trans_state_count);
  param_util::getParamOrThrow(
    nh_private, "model/translation/n_inputs", m_trans_input_count);
  m_A_orig = param_util::loadMatrixOrThrow(
    nh_private, "model/translation/A", m_trans_state_count, m_trans_state_count);
  m_B_orig = param_util::loadMatrixOrThrow(
    nh_private, "model/translation/B", m_trans_state_count, m_trans_input_count);
  m_A = m_A_orig;
  m_B = m_B_orig;

  // Load heading parameters
  param_util::getParamOrThrow(
    nh_private, "model/heading/n_states", m_heading_state_count);
  param_util::getParamOrThrow(
    nh_private, "model/heading/n_inputs", m_heading_input_count);
  m_A_heading_orig = param_util::loadMatrixOrThrow(
    nh_private, "model/heading/A", m_heading_state_count, m_heading_state_count);
  m_B_heading_orig = param_util::loadMatrixOrThrow(
    nh_private, "model/heading/B", m_heading_state_count, m_heading_input_count);
  m_A_heading = m_A_heading_orig;
  m_B_heading = m_B_heading_orig;

  // Load solver parameters
  param_util::getParamOrThrow(nh_private, "solver/horizon_len", m_horizon_len);
  param_util::getParamOrThrow(nh_private, "solver/xy/max_iterations", m_max_iter_xy);
  param_util::getParamOrThrow(nh_private, "solver/z/max_iterations", m_max_iter_z);
  param_util::getParamOrThrow(
    nh_private, "solver/heading/max_iterations", m_max_iter_heading);
  param_util::getParamOrThrow(nh_private, "solver/xy/Q", m_Q_xy);
  param_util::getParamOrThrow(nh_private, "solver/z/Q", m_Q_z);
  param_util::getParamOrThrow(nh_private, "solver/heading/Q", m_Q_heading);
  param_util::getParamOrThrow(nh_private, "solver/dt2", m_dt2);

  // Load solver constraints
  uav_ros_tracker::MPCTrackerParametersConfig default_config;

  param_util::getParamOrThrow(
    nh_private, "constraints/xy/velocity", default_config.xy_velocity);
  param_util::getParamOrThrow(
    nh_private, "constraints/xy/acceleration", default_config.xy_acceleration);
  param_util::getParamOrThrow(nh_private, "constraints/xy/jerk", default_config.xy_jerk);
  param_util::getParamOrThrow(nh_private, "constraints/xy/snap", default_config.xy_snap);

  param_util::getParamOrThrow(
    nh_private, "constraints/z/velocity", default_config.z_velocity);
  param_util::getParamOrThrow(
    nh_private, "constraints/z/acceleration", default_config.z_acceleration);
  param_util::getParamOrThrow(nh_private, "constraints/z/jerk", default_config.z_jerk);
  param_util::getParamOrThrow(nh_private, "constraints/z/snap", default_config.z_snap);

  param_util::getParamOrThrow(
    nh_private, "constraints/heading/velocity", default_config.heading_velocity);
  param_util::getParamOrThrow(
    nh_private, "constraints/heading/acceleration", default_config.heading_acceleration);
  param_util::getParamOrThrow(
    nh_private, "constraints/heading/jerk", default_config.heading_jerk);
  param_util::getParamOrThrow(
    nh_private, "constraints/heading/snap", default_config.heading_snap);

  m_reconfigure_handler = std::make_unique<
    ros_util::ReconfigureHandler<uav_ros_tracker::MPCTrackerParametersConfig>>(
    default_config, "mpc_tracker");
}