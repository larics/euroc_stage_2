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

#include <mav_linear_mpc/steady_state_calculation.h>

namespace mav_control {
SteadyStateCalculation::SteadyStateCalculation(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      controller_nh_(private_nh, "controller")
{
  this->Initialize();
}

SteadyStateCalculation::~SteadyStateCalculation()
{
}

void SteadyStateCalculation::Initialize()
{
  std::vector<double> temporary_A, temporary_B, temporary_Bd, temporary_C;

  if (!controller_nh_.getParam("n_state", state_size_))
    ROS_ERROR("n_state in target state calc is not loaded from ros parameter server");
  if (!controller_nh_.getParam("n_input", input_size_))
    ROS_ERROR("n_input in target state calc is not loaded from ros parameter server");
  if (!controller_nh_.getParam("n_measurement", reference_size_))
    ROS_ERROR("n_measurement in target state calc is not loaded from ros parameter server");
  if (!controller_nh_.getParam("n_disturbance", disturbance_size_))
    ROS_ERROR("n_disturbance in target state calc is not loaded from ros parameter server");
  if (!controller_nh_.getParam("A", temporary_A))
    ROS_ERROR("A in target state calc is not loaded from ros parameter server");
  if (!controller_nh_.getParam("B", temporary_B))
    ROS_ERROR("B in target state calc is not loaded from ros parameter server");
  if (!controller_nh_.getParam("Bd", temporary_Bd))
    ROS_ERROR("Bd in target state calc is not loaded from ros parameter server");
  if (!controller_nh_.getParam("C", temporary_C))
    ROS_ERROR("C in target state calc is not loaded from ros parameter server");

  Eigen::MatrixXd left_hand_side;
  left_hand_side.resize(state_size_ + reference_size_, state_size_ + input_size_);

  Eigen::Map<Eigen::MatrixXd> A_map(temporary_A.data(), state_size_, state_size_);
  Eigen::Map<Eigen::MatrixXd> B_map(temporary_B.data(), state_size_, input_size_);
  Eigen::Map<Eigen::MatrixXd> Bd_map(temporary_Bd.data(), state_size_, disturbance_size_);
  Eigen::Map<Eigen::MatrixXd> C_map(temporary_C.data(), reference_size_, state_size_);

  Eigen::MatrixXd A;
  A = A_map;
  Eigen::MatrixXd B;
  B = B_map;
  Bd_ = Bd_map;
  Eigen::MatrixXd C;
  C = C_map;

  left_hand_side << A - Eigen::MatrixXd::Identity(state_size_, state_size_), B, C, Eigen::MatrixXd::Zero(
      reference_size_, input_size_);

  //TODO(mina) use a more robust way
  pseudo_inverse_left_hand_side_ = (left_hand_side.transpose() * left_hand_side).inverse()
      * left_hand_side.transpose();

  initialized_params_ = true;
  ROS_INFO("Linear MPC: Steady State calculation is initialized correctly");
}

void SteadyStateCalculation::ComputeSteadyState(Eigen::VectorXd &estimated_disturbance,
                                                Eigen::VectorXd &reference,
                                                Eigen::VectorXd* steady_state_state,
                                                Eigen::VectorXd* steady_state_input)
{
  assert(steady_state_state);
  assert(steady_state_input);
  assert(initialized_params_);

  Eigen::VectorXd target_state_and_input(state_size_ + input_size_);
  Eigen::VectorXd right_hand_side(state_size_ + reference_size_);

  right_hand_side << -Bd_ * estimated_disturbance, reference;

  target_state_and_input = pseudo_inverse_left_hand_side_ * right_hand_side;

  steady_state_state->resize(state_size_);
  steady_state_input->resize(input_size_);

  *steady_state_state = target_state_and_input.segment(0, state_size_);
  *steady_state_input = target_state_and_input.segment(state_size_, input_size_);
}

}
