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

#ifndef INCLUDE_MAV_CONTROL_STEADYSTATECALCULATION_H_
#define INCLUDE_MAV_CONTROL_STEADYSTATECALCULATION_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <mav_linear_mpc/common.h>
#include <ros/ros.h>

namespace mav_control {

class SteadyStateCalculation
{
 public:
  SteadyStateCalculation(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
  ~SteadyStateCalculation();

  void ComputeSteadyState(Eigen::VectorXd &estimated_state, Eigen::VectorXd &reference,
                          Eigen::VectorXd* steadystate_state, Eigen::VectorXd* steady_state_input);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  ros::NodeHandle nh_, private_nh_, controller_nh_;
  int state_size_;
  int disturbance_size_;
  int reference_size_;
  int input_size_;
  bool initialized_params_;
  Eigen::MatrixXd Bd_;
  Eigen::MatrixXd pseudo_inverse_left_hand_side_;

  void Initialize();
};

}

#endif /* INCLUDE_MAV_CONTROL_STEADYSTATECALCULATION_H_ */
