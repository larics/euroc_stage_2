/*
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 *
 * This code can be used only for academic, non-commercial use.
 * This code cannot be redistributed under any license, open source or otherwise.
 *
 * CVXGEN license: http://cvxgen.com/docs/license.html
 * FORCES license: http://forces.ethz.ch
 *
 */

#include <mav_linear_mpc/PD_attitude_controller.h>

namespace mav_control {

PDAttitudeController::PDAttitudeController()
    : gravity_(9.81),
      initialized_params_(false)
{
}

PDAttitudeController::~PDAttitudeController()
{
}

void PDAttitudeController::InitializeParams()
{

  //Get parameters from RosParam server
  std::vector<double> temporary_Q, temporary_R, temporary_P, temporary_R_omega;
  std::vector<double> temporary_inertia_matrix, temporary_allocation_matrix;

  ros::NodeHandle pnh("~");

  double rotor_force_constant;  //F_i = k_n * rotor_velocity_i^2
  double rotor_moment_constant;  // M_i = k_m * F_i
  double arm_length;

  if (!pnh.getParam("rotor_force_constant", rotor_force_constant))
    ROS_ERROR(
        "rotor_force_constant in MPC position controller is not loaded from ros parameter server");

  if (!pnh.getParam("rotor_moment_constant", rotor_moment_constant))
    ROS_ERROR(
        "rotor_moment_constant in MPC position controller is not loaded from ros parameter server");

  if (!pnh.getParam("arm_length", arm_length))
    ROS_ERROR("arm_length in MPC position controller is not loaded from ros parameter server");

  if (!pnh.getParam("inertia", temporary_inertia_matrix))
    ROS_ERROR("arm_length in MPC position controller is not loaded from ros parameter server");

  if (!pnh.getParam("allocation_matrix", temporary_allocation_matrix))
    ROS_ERROR(
        "allocation matrix in MPC position controller is not loaded from ros parameter server");

  Eigen::Map<Eigen::MatrixXd> allocation_matrix_map(temporary_allocation_matrix.data(), 4, 6);
  Eigen::Map<Eigen::MatrixXd> inertia_matrix_map(temporary_inertia_matrix.data(), 3, 3);

  Eigen::MatrixXd allocation_matrix(4, 6);
  allocation_matrix = allocation_matrix_map;

  inertia_ = inertia_matrix_map;

  Eigen::Matrix4d I;
  I.setZero();
  I.block<3, 3>(0, 0) = inertia_;
  I(3, 3) = 1;

  Eigen::Matrix4d K;
  K.setZero();
  K(0, 0) = arm_length * rotor_force_constant;
  K(1, 1) = arm_length * rotor_force_constant;
  K(2, 2) = rotor_force_constant * rotor_moment_constant;
  K(3, 3) = rotor_force_constant;

  angular_acc_to_rotor_velocities_ = allocation_matrix.transpose()
      * (allocation_matrix * allocation_matrix.transpose()).inverse() * K.inverse() * I;

  attitude_thrust_reference_.setZero();

  initialized_params_ = true;
}

void PDAttitudeController::SetOdometry(const mav_msgs::EigenOdometry& odometry)
{
  odometry_ = odometry;
}

void PDAttitudeController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const
{
  assert(rotor_velocities);
  assert(initialized_params_);

  rotor_velocities->resize(6);

  Eigen::Vector3d angular_acceleration;
  ComputeDesiredAngularAcc(&angular_acceleration);

  Eigen::Vector4d angular_acceleration_thrust;
  angular_acceleration_thrust.block<3, 1>(0, 0) = angular_acceleration;
  angular_acceleration_thrust(3) = attitude_thrust_reference_(3);

  Eigen::Vector4d cross_term;
  cross_term
      << inertia_.inverse()
          * (odometry_.angular_velocity_B.cross(inertia_ * odometry_.angular_velocity_B)), 0;

  *rotor_velocities = angular_acc_to_rotor_velocities_ * (angular_acceleration_thrust - cross_term);
  *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
  *rotor_velocities = rotor_velocities->cwiseSqrt();
}

void PDAttitudeController::ComputeDesiredAngularAcc(Eigen::Vector3d* angular_acc) const
{
  assert(angular_acc);

  Eigen::Vector3d current_rpy;
  quat2rpy(odometry_.orientation_W_B, &current_rpy);

  double error_roll = attitude_thrust_reference_(0) - current_rpy(0);
  double error_pitch = attitude_thrust_reference_(1) - current_rpy(1);

  //desired omega = [0;0;0]
  double error_p = 0 - odometry_.angular_velocity_B(0);
  double error_q = 0 - odometry_.angular_velocity_B(1);
  double error_r = attitude_thrust_reference_(2) - odometry_.angular_velocity_B(2);

  *angular_acc << roll_gain_ * error_roll + p_gain_ * error_p, pitch_gain_ * error_pitch
      + q_gain_ * error_q, r_gain_ * error_r;

}

}
