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

#ifndef MPC_POSITION_CONTROLLER_H
#define MPC_POSITION_CONTROLLER_H

#include "common.h"
#include <memory>

#include <mav_linear_mpc/KF_disturbance_observer.h>
#include <mav_linear_mpc/steady_state_calculation.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include <stdio.h>
#include <memory.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <deque>
#include <Eigen/Eigen>
#include <iostream>

#ifdef UseForcesSolver
  #include "FireFlyOffsetFreeMPC.h"
#endif

#ifdef UseCVXGENSolver
    #include "solver.h"
#endif

#define PREDICTION_HORIZON 20

namespace mav_control{

class Integrator;

class LinearModelPredictiveController{
 public:

   LinearModelPredictiveController(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
  ~LinearModelPredictiveController();
  void InitializeParams();
  void ApplyParameters();

  void SetStatePenalityMatrix(const Eigen::MatrixXd& Q){
    Q_ = Q;
  }

  void SetCommandPenalityMatrix(const Eigen::MatrixXd& R){
    R_ = R;
  }

  void SetDeltaCommandPenalityMatrix(const Eigen::MatrixXd & R_delta){
    R_delta_ = R_delta;
  }

  void SetMaximumCommand(double max_roll, double max_pitch, double max_thrust);

  void SetUseXyOffsetFree(bool use_xy_offset_free){
    use_xy_offset_free_ = use_xy_offset_free;
  }

  void SetUseHeightErrorIntegrator(bool use_height_error_integrator){
    use_height_error_integrator_ = use_height_error_integrator;
  }

  void SetUseKfEstimatedStates(bool use_KF_estimated_state){
    use_KF_estimated_state_ = use_KF_estimated_state;
  }

  void SetYawGain(double K_yaw){
    K_yaw_ = K_yaw;
  }

  void SetHeightIntratorGain(double Ki_height){
    Ki_height_ = Ki_height;
  }

  void SetUseCommandFilter(bool filter_position, bool filter_velocity){
    use_command_position_filter_ = filter_position;
    use_command_velocity_filter_ = filter_velocity;
  }
  
  double GetMass() const
  {
    return mass_;
  }

  bool GetCurrentReference(mav_msgs::EigenTrajectoryPoint* reference) const;

  void SetOdometry(const mav_msgs::EigenOdometry& odometry);
  void SetCommandTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory);
  void SetCommandTrajectory(const mav_msgs::EigenTrajectoryPointDeque& command_trajectory);

  void CalculateAttitudeThrust(Eigen::Vector4d* ref_attitude_thrust);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  typedef std::deque<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > Vector3dDeque;

  void UpdateQueue(Eigen::VectorXd estimated_disturbances);

  ros::NodeHandle nh_, private_nh_, controller_nh_;

  double mass_;
  const double gravity_;
  const double sampling_time_;

  int state_size_;
  int input_size_;
  int disturbance_size_;
  int measurement_size_;

  bool verbose_;

  double K_yaw_;
  double Ki_height_;
  double height_error_integrator_;
  double height_error_integrator_limit_;

  bool initialized_params_;

  bool use_KF_estimated_state_;
  bool use_xy_offset_free_;
  bool use_height_error_integrator_;

  bool use_command_position_filter_;
  bool use_command_velocity_filter_;

  double command_position_filter_time_constant_;
  double command_velocity_filter_time_constant_;

  Eigen::Vector4d command_roll_pitch_yaw_thrust_;  //actual roll, pitch, yaw, thrust command
  Eigen::Vector3d linearized_command_roll_pitch_thrust_;

  double SolveTime_avg;

  Vector3dDeque position_command_queue_;
  Vector3dDeque velocity_command_queue_;
  std::deque<double> yaw_command_queue_;

  double yaw_command_;
  mav_msgs::EigenTrajectoryPoint command_trajectory_;
  mav_msgs::EigenOdometry odometry_;

  std::unique_ptr<KFDisturbanceObserver> state_observer_;
  std::unique_ptr<SteadyStateCalculation> steady_state_calculation_;

  //LQR if solver fails

  /* MPC Matrices */
    //Model: A, B, Bd
  Eigen::MatrixXd model_A_;   //dynamics matrix
  Eigen::MatrixXd model_B_;   //transfer matrix
  Eigen::MatrixXd model_Bd_;  //Disturbance transfer matrix

  //Cost matrices
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd Q_final_;
  Eigen::MatrixXd R_;
  Eigen::MatrixXd R_delta_;

  Eigen::Vector3d u_max_;
  Eigen::Vector3d u_min_;

  Eigen::MatrixXd LQR_K_;

#ifdef UseForcesSolver
  Eigen::MatrixXd cost_Hessian_;
  Eigen::MatrixXd cost_Hessian_final_;

  std::deque<Eigen::VectorXd> FORCES_queue_;

  //FORCES parameters struct
  FireFlyOffsetFreeMPC_params FORCES_params_;
#endif

#ifdef UseCVXGENSolver
  std::deque<Eigen::VectorXd> CVXGEN_queue_;
#endif
};

} // end namespace mav_control

#endif
