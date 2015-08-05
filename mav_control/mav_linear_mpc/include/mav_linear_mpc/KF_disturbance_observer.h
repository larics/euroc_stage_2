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

#ifndef KFDisturbanceObserver_H_
#define KFDisturbanceObserver_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <ros/ros.h>
#include <mav_linear_mpc/common.h>
#include <mav_linear_mpc/ObserverState.h>
#include <std_srvs/Empty.h>
#include <iostream>

namespace mav_control {
class KFDisturbanceObserver
{
 public:

  KFDisturbanceObserver(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
  void Reset(const Eigen::Vector3d& initial_position, const Eigen::Vector3d& initial_velocity,
             const Eigen::Vector3d& initial_attitude, const Eigen::Vector3d& initial_angular_rate,
             const Eigen::Vector3d& initial_external_forces,
             const Eigen::Vector3d& initial_external_moments);

  //Getters
  Eigen::Vector3d GetEstimatedPosition() const
  {
    if (initialized_)
      return state_.segment(0, 3);
    else
      return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d GetEstimatedVelocity() const
  {
    if (initialized_)
      return state_.segment(3, 3);
    else
      return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d GetEstimatedAttitude() const
  {
    if (initialized_)
      return state_.segment(6, 3);
    else
      return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d GetEstimatedAngularVelocity() const
  {
    if (initialized_)
      return state_.segment(9, 3);
    else
      return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d GetEstimatedExternalForces() const
  {
    if (initialized_ == true && calibrate_ == false)
      return state_.segment(12, 3) - forces_offset_;
    else
      return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d GetEstimatedExternalMoments() const
  {
    if (initialized_ && calibrate_ == false)
      return state_.segment(15, 3) - moments_offset_;
    else
      return Eigen::Vector3d::Zero();
  }

  void GetEstimatedState(Eigen::VectorXd* estimated_state) const;

  //Feeding
  void FeedPositionMeasurement(const Eigen::Vector3d& position);
  void FeedVelocityMeasurement(const Eigen::Vector3d& velocity);
  void FeedRotationMatrix(const Eigen::Matrix3d& rotation_matrix);
  void FeedAttitudeCommand(const Eigen::Vector4d& roll_pitch_yaw_thrust_cmd);

  bool UpdateEstimator();

  virtual ~KFDisturbanceObserver();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  static constexpr int kStateSize = 18;
  static constexpr int kMeasurementSize = 9;
  typedef Eigen::Matrix<double, kStateSize, 1> StateVector;

  ros::NodeHandle nh_, private_nh_, observer_nh_;
  bool initialized_;
  Eigen::Matrix<double, kStateSize, 1> state_;  // [pos, vel, rpy, omega, external_forces, external_moments]
  Eigen::Matrix<double, kStateSize, 1> predicted_state_;
  Eigen::Matrix<double, kMeasurementSize, 1> measurements_;  // [pos, vel, rpy]
  Eigen::Matrix3d rotation_matrix_;
  Eigen::Vector4d roll_pitch_yaw_thrust_cmd_;
  Eigen::Matrix<double, kStateSize, 1> process_noise_covariance_; // We express it as diag() later.
  Eigen::Matrix<double, kStateSize, kStateSize> state_covariance_;
  Eigen::Matrix<double, kMeasurementSize, kMeasurementSize> measurement_covariance_;
  Eigen::Matrix3d drag_coefficients_matrix_;

  Eigen::SparseMatrix<double> F_; // System dynamics matrix.
//  Eigen::Matrix<double, kStateSize, kStateSize> F_; // System dynamics matrix.
  Eigen::Matrix<double, kStateSize, kMeasurementSize> K_; // Kalman gain matrix.
  Eigen::SparseMatrix<double> H_; // Measurement matrix.
//  Eigen::Matrix<double, kMeasurementSize, kStateSize> H_; // Measurement matrix.

  Eigen::Vector3d external_forces_limit_;
  Eigen::Vector3d external_moments_limit_;
  Eigen::Vector3d omega_limit_;

  // Parameters
  double roll_damping_;
  double roll_omega_;
  double roll_gain_;

  double pitch_damping_;
  double pitch_omega_;
  double pitch_gain_;

  double yaw_damping_;
  double yaw_omega_;
  double yaw_gain_;

  double gravity_;

  ros::ServiceServer service_;
  ros::Publisher observer_state_pub_;

  bool calibrate_;         // true if calibrating
  ros::Time start_calibration_time_;   // t0 calibration
  ros::Duration calibration_duration_;     // calibration duration
  Eigen::Vector3d forces_offset_;
  Eigen::Vector3d moments_offset_;
  int calibration_counter_;

  void Initialize();
  void SystemDynamics(double dt);
  bool StartCalibrationCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

};
}
#endif /* SRC_KFDisturbanceObserver_H_ */
